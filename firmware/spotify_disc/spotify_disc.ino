/*
 * Spotify Album Art as a Spinning Disc — 64x64 HUB75 LED Matrix
 * -------------------------------------------------------------
 * Hardware : ESP32 DevKit V1 (30-pin, WROOM) + P2.5 64x64 HUB75E panel
 *            (Chipone ICN2037BP column drivers, 74HC04 input buffer)
 *            + 5V/4A supply
 * Libraries: ESP32-HUB75-MatrixPanel-I2S-DMA (mrfaptastic)
 *            TJpg_Decoder (Bodmer)
 *            ArduinoJson v6 (bblanchon)
 *            (WiFiManager is no longer needed)
 *
 * WIFI
 *   Every network the display connects to is remembered (up to 8, most
 *   recent first); the default network below is always kept.
 *   At startup the "SpotifyDisc-Setup" network appears straight away while
 *   the saved networks are tried for 60 s. Join it with your phone any time
 *   to add a network (a page opens, or browse to 192.168.4.1). After 60 s
 *   without a connection the panel shows the join screen.
 *   Normal use:
 *     Open http://<HOSTNAME>.local on the same WiFi (spotify-disc.local by default).
 *     On iPhone: Share -> Add to Home Screen, so it opens like an app.
 *
 * TASK LAYOUT
 *   Core 0 — networkTask: Spotify polling, image download + decode
 *            webTask:     phone dashboard
 *            ledTask:     status LED
 *   Core 1 — loop():      disc rendering only
 *
 * Wiring (DevKit V1 silkscreen labels) — unchanged from the P3 panel:
 *   R1 D25   G1 D26   B1 D27   R2 D14   G2 D33   B2 D13
 *   A  D23   B  D22   C  D5    D  TX2   E  D32
 *   LAT D4   OE D15   CLK RX2  GND GND  5V -> VIN
 *
 * Status LED (onboard blue):
 *   Fast blink = setup mode, slow blink = offline, off = all good
 */

#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <Preferences.h>
#include <DNSServer.h>
#include <esp_wifi.h>
#include <base64.h>
#include <ArduinoJson.h>
#include <TJpg_Decoder.h>
#include <ESP32-HUB75-MatrixPanel-I2S-DMA.h>
#include <Adafruit_GFX.h>
#include <Fonts/FreeSansBold12pt7b.h>
#include <time.h>
#include <sys/time.h>

// ============================ CONFIG ============================

// Your home network. Tried first on every boot, and never overwritten.
// Credentials live in secrets.h (git-ignored). Copy secrets.example.h
// to secrets.h and fill it in; see README.md.
#include "secrets.h"

const char *DEFAULT_WIFI_SSID = SECRET_WIFI_SSID;
const char *DEFAULT_WIFI_PASS = SECRET_WIFI_PASS;
const uint32_t BOOT_CONNECT_WINDOW_MS = 60000;    // Try saved networks this long, then show setup
const uint32_t LOST_SETUP_AFTER_MS    = 120000;   // WiFi gone this long in use: open setup too

const char *SPOTIFY_CLIENT_ID     = SECRET_SPOTIFY_CLIENT_ID;
const char *SPOTIFY_CLIENT_SECRET = SECRET_SPOTIFY_CLIENT_SECRET;
const char *SPOTIFY_REFRESH_TOKEN = SECRET_SPOTIFY_REFRESH_TOKEN;



// Dashboard address becomes http://<HOSTNAME>.local
// Lowercase letters, digits and hyphens only; no spaces or dots.
const char *HOSTNAME       = "spotify-disc";
const char *SETUP_AP_NAME  = "SpotifyDisc-Setup";

const uint32_t POLL_INTERVAL_MS = 4000;
const uint16_t TARGET_FPS       = 30;

// Bilinear sampling while rotating: smooth edges instead of stair-step
// shimmer. Costs under 1 ms per frame.
const bool     SMOOTH_ROTATION  = true;

// ---- Panel settings ----
// P2.5 64x64 panel with Chipone ICN2037BP column drivers. The ICN2037 is a
// plain constant-current shift register (no startup sequence), so SHIFTREG
// is the right driver mode.
#define PANEL_DRIVER HUB75_I2S_CFG::SHIFTREG

// Bump this whenever the panel is swapped. On the first boot with a new
// number, everything that was tuned for the old panel and saved in flash
// (clock phase, latch blanking, image and colour settings) is reset to the
// defaults below, once. 1 = old P3 (FM6124EJ), 2-3 = P2.5 (ICN2037BP).
const uint8_t  PANEL_PROFILE       = 3;   // 3 = P2.5, defaults matched to the old look

// Flip on the dashboard if the image is shifted by one column or colours
// glitch in patterns.
const bool     PANEL_CLKPHASE      = true;

// ICN2037 outputs switch off slowly, so a little of each row bleeds into
// the next one: dark areas never go fully black and edges look smeared.
// A wider blanking window around each latch hides that. 1 = library
// default, up to 4 (more = slightly dimmer).
const uint8_t  PANEL_LATCH_DEFAULT = 2;

// White balance, % per channel. The small LEDs on P2.5 panels have far
// stronger blue and green than red, so white looks cold and warm tones
// (skin, wood, brown) drift towards cyan. Tune live on the dashboard with
// Panel test -> Greys.
const uint8_t  WB_R_DEFAULT = 95;
const uint8_t  WB_G_DEFAULT = 97;     // Tuned by eye on this panel
const uint8_t  WB_B_DEFAULT = 97;

// Hardware brightness floor (0-255). The library dims by shortening how
// long the LEDs are on for each colour bit. Below about 30% the shortest
// pulses get too short for the ICN2037 to switch cleanly, the faint bits
// drop out unevenly per channel, and colours shift green or pink. So the
// panel never goes below this; lower brightness settings are done in
// software by scaling the pixel values instead.
const uint8_t  HW_BRIGHTNESS_MIN = 100;

const uint8_t  STATUS_LED_PIN   = 2;
const uint8_t  BOOT_BUTTON_PIN  = 0;      // The "BOOT" button on the DevKit
const uint32_t SETUP_HOLD_MS    = 3000;   // Hold this long to enter WiFi setup

// Physical remote button (optional). Wire a push button between this pin
// and GND; -1 = not fitted yet. D21 (GPIO 21) is free on the DevKit V1
// (D33 is now used for G2).
// Avoid D34/D35/VP/VN: they have no internal pull-up.
const int8_t   REMOTE_BUTTON_PIN = -1;
const uint16_t LONG_PRESS_MS     = 700;    // Hold at least this long = long press

// ---- Home mode: clock and weather ----
const char    *TIME_ZONE          = "CET-1CEST,M3.5.0,M10.5.0/3";  // Berlin incl. summer time
const float    WEATHER_LAT        = 52.52f;                        // Berlin
const float    WEATHER_LON        = 13.41f;
const uint32_t WEATHER_REFRESH_MS = 10UL * 60 * 1000;
const uint32_t NET_TASK_STACK   = 12288;   // Raise to 16384 on "Stack canary" resets

// ===================== MATRIX PIN MAPPING =======================

#define R1_PIN 25
#define G1_PIN 26
#define B1_PIN 27
#define R2_PIN 14
#define G2_PIN 33   // Was 12: GPIO 12 is a boot strapping pin and dropped green bits
#define B2_PIN 13
#define A_PIN  23
#define B_PIN  22
#define C_PIN   5
#define D_PIN  17
#define E_PIN  32
#define LAT_PIN 4
#define OE_PIN 15
#define CLK_PIN 16

#define PANEL_W 64
#define PANEL_H 64

#define MAX_IMAGE_BYTES 16384
#define CENTER_FP 2064384          // 31.5 in 16.16 fixed point

// ==================== SETTINGS (saved to flash) =================
// Changed from the phone, read by the renderer.

volatile uint8_t  cfgBrightness   = 100;
volatile float    cfgSpinSpeed    = 45.0f;
volatile bool     cfgSpindle      = true;
volatile bool     cfgPaused       = false;
volatile uint8_t  cfgMode         = 0;       // Music view: 0 = disc, 1 = lyrics, 2 = cover
volatile bool     cfgHome         = false;   // Home mode (clock + weather) on top of that
volatile int16_t  cfgLyricOffset  = 0;       // ms, positive = lyrics earlier
volatile uint8_t  cfgBlackLevel   = 20;      // 0-60: faint noise below this becomes black
volatile uint8_t  cfgContrast     = 110;     // Tone curve exponent x100 (100 = off)
volatile uint8_t  cfgSaturation   = 100;     // Colour strength in % (100 = unchanged)
volatile uint8_t  cfgShadow       = 60;      // 0-100: how much colour is removed from dark tones
volatile uint8_t  cfgWbR          = WB_R_DEFAULT;   // White balance, % per channel
volatile uint8_t  cfgWbG          = WB_G_DEFAULT;
volatile uint8_t  cfgWbB          = WB_B_DEFAULT;
volatile bool     wbDirty         = true;    // Rebuild the white-balance table on the render core
volatile bool     redrawAll       = false;   // Output setting changed: redraw the current view
volatile bool     artReload       = false;   // Image settings changed: fetch the cover again
TaskHandle_t      netTaskHandle   = nullptr;

// Panel signal settings. Applied at boot only (the matrix has to be
// re-initialised), so the dashboard saves them and restarts.
bool     cfgClkPhase   = PANEL_CLKPHASE;
uint8_t  cfgLatchBlank = PANEL_LATCH_DEFAULT;   // 0 = library default, else 1-4
uint32_t activeClkHz   = 0;                // What the panel actually runs at
uint8_t  activeLatch   = 0;
volatile bool     brightnessDirty = false;
volatile uint32_t settingsDirtyAt = 0;      // 0 = nothing to save

Preferences prefs;

// ======================= SHARED STATE ===========================

MatrixPanel_I2S_DMA *dma_display = nullptr;
WebServer   server(80);

uint16_t displayBuf[PANEL_W * PANEL_H];   // What the renderer draws from
uint16_t *decodeBuf = nullptr;             // Only allocated while decoding
SemaphoreHandle_t bufMutex;

volatile bool haveArt       = false;
volatile bool newArt        = false;
volatile bool isPlaying     = false;
volatile bool netOk         = false;
volatile bool inSetupPortal = false;

// ============================= WIFI =============================

// Connection screens, drawn over everything else by the render loop
enum : uint8_t { NV_NONE, NV_CONNECTING, NV_SETUP, NV_CONNECTED, NV_LOST };
volatile uint8_t  netView      = NV_CONNECTING;
volatile uint32_t netViewSince = 0;
volatile int8_t   netBars      = 3;            // Signal strength on the green screen
char netLine[48] = "";                          // Second text line (network being tried...)
portMUX_TYPE netMux = portMUX_INITIALIZER_UNLOCKED;

// Saved networks, most recently used first
#define MAX_KNOWN_NETS 8
struct KnownNet { char ssid[33]; char pass[65]; };
KnownNet knownNets[MAX_KNOWN_NETS];
uint8_t  knownCount = 0;
SemaphoreHandle_t wifiMutex;                    // Guards knownNets and scanList

// Last scan, for the setup page
#define MAX_SCAN 16
struct ScanEntry { char ssid[33]; int8_t rssi; bool open; };
ScanEntry scanList[MAX_SCAN];
uint8_t   scanCount = 0;
uint32_t  scanAt    = 0;
volatile bool scanRequested = false;

// Setup hotspot and "join this network" requests from its page
volatile bool apActive   = false;
volatile bool joinPending = false;
char joinSsid[33] = "", joinPass[65] = "";
enum : uint8_t { JOIN_IDLE, JOIN_TRYING, JOIN_OK, JOIN_FAILED };
volatile uint8_t joinResult = JOIN_IDLE;

volatile bool bootIdle = true;                  // No cover since boot: show READY + IP
volatile uint32_t coverVersion = 0;   // Bumped on every new cover; the dashboard polls it
volatile bool maskDirty = true;       // Rebuild the disc edge mask on the render core
volatile uint8_t testPattern = 0;     // 0 off, 1 white, 2 red, 3 green, 4 blue, 5 bars, 6 stripes, 7 greys
volatile bool testDirty = false;

// Spotify status for the dashboard. Guarded by statusMutex.
struct SpotifyStatus {
  bool     tokenOk;
  int      lastHttp;
  uint32_t lastOkMs;               // 0 = never
  char     state[12];              // "playing", "paused", "idle", "error"
  char     track[96];
  char     artist[64];
};
SpotifyStatus spStatus = { false, 0, 0, "starting", "", "" };
SemaphoreHandle_t statusMutex;

// Result of one currently-playing request. Defined up here because the
// Arduino IDE auto-inserts function prototypes above the first function,
// and any type they use must already exist at that point.
struct NowPlaying {
  bool     playing = false;
  bool     hasItem = false;
  String   id, name, artists, artist0, imageUrl;
  uint32_t progressMs = 0;        // Playback position when Spotify answered
  uint32_t durationMs = 0;
  uint32_t sampledAt  = 0;        // millis() matching progressMs
};

// ============================ LYRICS ============================

enum : uint8_t { MODE_DISC = 0, MODE_LYRICS = 1, MODE_COVER = 2 };
enum : uint8_t { LYR_NONE = 0, LYR_LOADING, LYR_READY, LYR_NOTFOUND, LYR_INSTRUMENTAL, LYR_ERROR };

volatile uint8_t lyricsState = LYR_NONE;
SemaphoreHandle_t lyricsMutex;

// Synced lyrics of the current track: start times plus NUL-separated UTF-8
// text. Written only by the network task while LOADING, read only while
// READY; the mutex guards the switch between the two.
#define MAX_LYRIC_LINES  144
#define LYRIC_TEXT_BYTES 4096
uint32_t lyricTime[MAX_LYRIC_LINES];
uint16_t lyricOff[MAX_LYRIC_LINES];
uint16_t lyricCount    = 0;
char     lyricText[LYRIC_TEXT_BYTES];
uint16_t lyricTextUsed = 0;

// Where in the song we are: Spotify's position at the last poll + when.
portMUX_TYPE posMux = portMUX_INITIALIZER_UNLOCKED;
uint32_t posBaseMs     = 0;
uint32_t posBaseMillis = 0;

// ============================= HOME =============================

struct Weather {
  bool    valid;
  float   tempC;
  uint8_t code;                   // WMO weather code
  bool    isDay;
};
Weather weatherNow = { false, 0.0f, 0, true };
portMUX_TYPE wxMux = portMUX_INITIALIZER_UNLOCKED;
uint32_t weatherFetchedAt = 0;    // Network task only

char panelToast[16] = "";         // Short message under the clock
volatile uint32_t panelToastUntil = 0;

// ============== NETWORK-TASK-ONLY STATE (core 0) ================

String   accessToken    = "";
uint32_t tokenExpiresAt = 0;
String   currentTrackId = "";

// ================ RENDER-LOOP-ONLY STATE (core 1) ===============

uint32_t lastFrame    = 0;
float    spinAngle    = 0.0f;

// ====================== SETTINGS PERSISTENCE ====================

// Connection screen state, set by the network task
void setNetView(uint8_t v) {
  if (netView != v) { netViewSince = millis(); netView = v; }
}
void setNetLine(const char *txt) {
  portENTER_CRITICAL(&netMux);
  strlcpy(netLine, txt, sizeof(netLine));
  portEXIT_CRITICAL(&netMux);
}

// First boot on a new panel: drop the settings tuned for the old one.
// Brightness, spin, mode, lyrics offset and WiFi are kept.
void applyPanelProfile() {
  prefs.begin("disc", false);
  uint8_t saved = prefs.getUChar("panel", 1);     // Older firmware = P3 panel
  if (saved != PANEL_PROFILE) {
    prefs.putUChar("panel", PANEL_PROFILE);
    prefs.putBool("clkph", PANEL_CLKPHASE);
    prefs.putUChar("latch", PANEL_LATCH_DEFAULT);
    prefs.putUChar("wbr", WB_R_DEFAULT);
    prefs.putUChar("wbg", WB_G_DEFAULT);
    prefs.putUChar("wbb", WB_B_DEFAULT);
    prefs.putUChar("blk", 20);
    prefs.putUChar("ctr", 110);
    prefs.putUChar("sat2", 100);
    prefs.putUChar("shd", 60);
    Serial.printf("[panel] new panel profile %u (was %u): panel settings reset\n",
                  PANEL_PROFILE, saved);
  }
  prefs.end();
}

void loadSettings() {
  prefs.begin("disc", true);
  cfgBrightness = prefs.getUChar("bright", 100);
  cfgSpinSpeed  = prefs.getFloat("spin", 45.0f);
  cfgSpindle    = prefs.getBool("spindle", true);
  cfgPaused     = prefs.getBool("paused", false);
  cfgClkPhase   = prefs.getBool("clkph", PANEL_CLKPHASE);
  cfgLatchBlank = prefs.getUChar("latch", PANEL_LATCH_DEFAULT);
  cfgMode        = prefs.getUChar("mode", 0);
  cfgHome        = prefs.getBool("home", false);
  cfgLyricOffset = prefs.getShort("lyroff", 0);
  cfgBlackLevel  = prefs.getUChar("blk", 20);
  cfgContrast    = prefs.getUChar("ctr", 110);
  cfgSaturation  = prefs.getUChar("sat2", 100);
  cfgShadow      = prefs.getUChar("shd", 60);
  cfgWbR         = prefs.getUChar("wbr", WB_R_DEFAULT);
  cfgWbG         = prefs.getUChar("wbg", WB_G_DEFAULT);
  cfgWbB         = prefs.getUChar("wbb", WB_B_DEFAULT);
  prefs.end();
}

void saveSettings() {
  prefs.begin("disc", false);
  prefs.putUChar("bright", cfgBrightness);
  prefs.putFloat("spin", cfgSpinSpeed);
  prefs.putBool("spindle", cfgSpindle);
  prefs.putBool("paused", cfgPaused);
  prefs.putUChar("mode", cfgMode);
  prefs.putBool("home", cfgHome);
  prefs.putShort("lyroff", cfgLyricOffset);
  prefs.putUChar("blk", cfgBlackLevel);
  prefs.putUChar("ctr", cfgContrast);
  prefs.putUChar("sat2", cfgSaturation);
  prefs.putUChar("shd", cfgShadow);
  prefs.putUChar("wbr", cfgWbR);
  prefs.putUChar("wbg", cfgWbG);
  prefs.putUChar("wbb", cfgWbB);
  prefs.end();
  Serial.println("[settings] saved");
}

// ======================== STATUS HELPERS ========================

void setSpotifyStatus(const char *state, const char *track, const char *artist, int http, bool ok) {
  xSemaphoreTake(statusMutex, portMAX_DELAY);
  strlcpy(spStatus.state, state, sizeof(spStatus.state));
  if (track)  strlcpy(spStatus.track, track, sizeof(spStatus.track));
  if (artist) strlcpy(spStatus.artist, artist, sizeof(spStatus.artist));
  spStatus.lastHttp = http;
  if (ok) spStatus.lastOkMs = millis();
  xSemaphoreGive(statusMutex);
}

void setTokenOk(bool ok) {
  xSemaphoreTake(statusMutex, portMAX_DELAY);
  spStatus.tokenOk = ok;
  xSemaphoreGive(statusMutex);
}

// ========================= REMOTE BUTTON =========================
// One set of actions for every way of pressing "the button": the dashboard
// remote now, a physical button on REMOTE_BUTTON_PIN later.
//   Tap:  cycle Lyrics -> Disc -> Cover (lyrics skipped if the song has none)
//   Hold: Home (clock + weather). Hold again: back to music, if something
//         is playing.

void showPanelToast(const char *msg) {
  strlcpy(panelToast, msg, sizeof(panelToast));
  panelToastUntil = millis() + 1800;
}

const char *remoteTap() {
  if (cfgHome) {
    showPanelToast("HOLD = MUSIC");
    return "Home mode is on. Hold the button to go back to music.";
  }
  // Lyrics mode without lyrics shows the disc, so treat it as the disc
  uint8_t cur = (cfgMode == MODE_LYRICS && lyricsState != LYR_READY) ? MODE_DISC : cfgMode;
  uint8_t next;
  switch (cur) {
    case MODE_LYRICS: next = MODE_DISC;   break;
    case MODE_DISC:   next = MODE_COVER;  break;
    default:          next = MODE_LYRICS; break;
  }
  const char *msg = "";
  if (next == MODE_LYRICS && lyricsState != LYR_READY) {
    next = MODE_DISC;
    msg = "No synced lyrics for this song, so it went straight to the disc.";
  }
  cfgMode = next;
  settingsDirtyAt = millis() | 1;
  return msg;
}

const char *remoteHold() {
  if (!cfgHome) {
    cfgHome = true;
    settingsDirtyAt = millis() | 1;
    if (netTaskHandle) xTaskNotifyGive(netTaskHandle);   // Get the weather now if needed
    return "";
  }
  if (!isPlaying) {
    showPanelToast("NO MUSIC");
    return "Nothing is playing on Spotify, so home mode stays on.";
  }
  cfgHome = false;
  settingsDirtyAt = millis() | 1;
  return "";
}

// ===================== REBOOT INTO SETUP ========================

void rebootIntoSetup() {
  prefs.begin("disc", false);
  prefs.putBool("portal", true);    // Next boot skips straight to the setup page
  prefs.end();
  Serial.println("[wifi] rebooting into setup mode");
  delay(300);
  ESP.restart();
}

// ======================= STATUS LED TASK ========================
// Its own task so it keeps blinking while the setup portal blocks.
// Also watches the BOOT button: hold it 3 s to reboot into WiFi setup.

void ledTask(void *param) {
  bool     on         = false;
  uint32_t lastToggle = 0;
  uint32_t pressStart = 0;
  uint32_t rbStart    = 0;      // Remote button
  bool     rbHeld     = false;

  for (;;) {
    uint32_t now = millis();

    // Remote button: tap on release, hold fires as soon as it's long enough
    if (REMOTE_BUTTON_PIN >= 0) {
      bool down = digitalRead(REMOTE_BUTTON_PIN) == LOW;
      if (down) {
        if (!rbStart) { rbStart = now; rbHeld = false; }
        else if (!rbHeld && now - rbStart >= LONG_PRESS_MS) { rbHeld = true; remoteHold(); }
      } else if (rbStart) {
        if (!rbHeld && now - rbStart >= 40) remoteTap();   // 40 ms: ignore contact bounce
        rbStart = 0;
      }
    }

    if (digitalRead(BOOT_BUTTON_PIN) == LOW) {
      if (!pressStart) pressStart = now;
      digitalWrite(STATUS_LED_PIN, HIGH);          // Solid while held
      if (now - pressStart >= SETUP_HOLD_MS) rebootIntoSetup();
      vTaskDelay(pdMS_TO_TICKS(50));
      continue;
    }
    pressStart = 0;

    if (!inSetupPortal && netOk && WiFi.status() == WL_CONNECTED) {
      if (on) { digitalWrite(STATUS_LED_PIN, LOW); on = false; }
    } else {
      uint32_t interval = inSetupPortal ? 100 : 300;
      if (now - lastToggle >= interval) {
        lastToggle = now;
        on = !on;
        digitalWrite(STATUS_LED_PIN, on ? HIGH : LOW);
      }
    }
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

// ================= JPEG DECODE INTO BACK BUFFER =================

bool jpegToBuffer(int16_t x, int16_t y, uint16_t w, uint16_t h, uint16_t *bitmap) {
  if (y >= PANEL_H || !decodeBuf) return false;
  for (uint16_t row = 0; row < h; row++) {
    int16_t py = y + row;
    if (py < 0 || py >= PANEL_H) continue;
    for (uint16_t col = 0; col < w; col++) {
      int16_t px = x + col;
      if (px < 0 || px >= PANEL_W) continue;
      decodeBuf[py * PANEL_W + px] = bitmap[row * w + col];
    }
  }
  return true;
}

// ====================== PANEL OUTPUT LAYER ======================
// Every view sends its pixels through here, so the white balance applies
// everywhere: covers, lyrics, home, connection screens and test patterns.

uint8_t wbLut[3][256];

// Brightness actually sent to the panel hardware
uint8_t hwBrightness() {
  return cfgBrightness < HW_BRIGHTNESS_MIN ? HW_BRIGHTNESS_MIN : cfgBrightness;
}

void buildWbLut() {
  const float gain[3] = { cfgWbR / 100.0f, cfgWbG / 100.0f, cfgWbB / 100.0f };

  // Software dimming below the hardware floor. The library's brightness
  // table makes light roughly follow value^2.2, so the value is scaled by
  // the 1/2.2 power of the light ratio: the slider then dims as smoothly
  // below the floor as above it.
  float dim = 1.0f;
  if (cfgBrightness < HW_BRIGHTNESS_MIN)
    dim = powf((float)cfgBrightness / HW_BRIGHTNESS_MIN, 1.0f / 2.2f);

  for (int c = 0; c < 3; c++)
    for (int v = 0; v < 256; v++)
      wbLut[c][v] = (uint8_t)lroundf(v * gain[c] * dim);
}

static inline void outRGB(int x, int y, uint8_t r, uint8_t g, uint8_t b) {
  dma_display->drawPixelRGB888(x, y, wbLut[0][r], wbLut[1][g], wbLut[2][b]);
}

static inline void out565(int x, int y, uint16_t c) {
  uint8_t r5 = c >> 11, g6 = (c >> 5) & 63, b5 = c & 31;
  outRGB(x, y, (r5 << 3) | (r5 >> 2), (g6 << 2) | (g6 >> 4), (b5 << 3) | (b5 >> 2));
}

// ==================== ROTATED DISC RENDERER =====================

// ---- Panel test patterns ----
// Top half of the panel is fed by the R1/G1/B1 wires, bottom half by
// R2/G2/B2, so a fault in only one half points straight at those wires.
void drawTestPattern(uint8_t p) {
  // Greys: eight bands of neutral grey, darkest at the top, sent as equal
  // R, G and B (after white balance). Adjust the white balance sliders
  // until none of the bands has a tint.
  if (p == 7) {
    static const uint8_t levels[8] = { 8, 16, 24, 32, 48, 64, 96, 160 };
    for (int y = 0; y < PANEL_H; y++)
      for (int x = 0; x < PANEL_W; x++) {
        uint8_t v = levels[y / 8];
        outRGB(x, y, v, v, v);
      }
    return;
  }

  static const uint16_t bars[8] = { 0xFFFF, 0xFFE0, 0x07FF, 0x07E0,    // white yellow cyan green
                                    0xF81F, 0xF800, 0x001F, 0x0000 };  // magenta red blue black
  for (int y = 0; y < PANEL_H; y++) {
    for (int x = 0; x < PANEL_W; x++) {
      uint16_t c = 0;
      switch (p) {
        case 1: c = 0xFFFF; break;
        case 2: c = 0xF800; break;
        case 3: c = 0x07E0; break;
        case 4: c = 0x001F; break;
        case 6:                            // White, every other column:
          c = (x & 1) ? 0 : 0xFFFF;        // half the current of solid white
          break;
        case 5:
          if (y < 32) {
            c = bars[x / 8];
          } else {
            uint8_t v = x * 255 / 63;      // Grey ramp, black to white
            c = dma_display->color565(v, v, v);
          }
          break;
      }
      out565(x, y, c);
    }
  }
  dma_display->flipDMABuffer();
}

// ---- Anti-aliased disc mask ----
// Each pixel stores how much of it lies inside the disc (0-255), measured
// with 4x4 sub-samples. Edge pixels blend smoothly into black instead of
// forming a jagged stair-step.
uint8_t discMask[PANEL_W * PANEL_H];

const float DISC_RADIUS    = 31.75f;   // Just inside the panel edge
const float SPINDLE_RADIUS = 4.0f;
const uint8_t MASK_FLOOR   = 48;       // Faintest edge pixels -> off (see below)

void buildDiscMask(bool spindle) {
  const float ro2 = DISC_RADIUS * DISC_RADIUS;
  const float rh2 = SPINDLE_RADIUS * SPINDLE_RADIUS;

  for (int y = 0; y < PANEL_H; y++) {
    for (int x = 0; x < PANEL_W; x++) {
      int inside = 0;
      for (int sy = 0; sy < 4; sy++) {
        for (int sx = 0; sx < 4; sx++) {
          float fx = x + (sx + 0.5f) * 0.25f - 32.0f;
          float fy = y + (sy + 0.5f) * 0.25f - 32.0f;
          float d2 = fx * fx + fy * fy;
          if (d2 <= ro2 && (!spindle || d2 >= rh2)) inside++;
        }
      }
      uint16_t a = inside * 255 / 16;
      // Pixels only a sliver inside the edge would end up very dim, and
      // very dim levels are exactly what shimmers on a HUB75 panel.
      if (a < MASK_FLOOR) a = 0;
      discMask[y * PANEL_W + x] = a;
    }
  }
}

static inline uint16_t scale565(uint16_t c, uint16_t a) {
  uint16_t r = (c >> 11) & 31, g = (c >> 5) & 63, b = c & 31;
  r = (r * a + 127) / 255;
  g = (g * a + 127) / 255;
  b = (b * a + 127) / 255;
  return (r << 11) | (g << 5) | b;
}

// Bilinear sample. fx, fy are 16.16 fixed-point source coordinates where
// integers land on pixel centres.
static inline uint16_t sampleBilinear(const uint16_t *src, int32_t fx, int32_t fy) {
  int x0 = fx >> 16, y0 = fy >> 16;
  uint32_t wx = (fx >> 8) & 0xFF, wy = (fy >> 8) & 0xFF;

  int x1 = x0 + 1, y1 = y0 + 1;
  x0 = constrain(x0, 0, PANEL_W - 1); x1 = constrain(x1, 0, PANEL_W - 1);
  y0 = constrain(y0, 0, PANEL_H - 1); y1 = constrain(y1, 0, PANEL_H - 1);

  uint16_t c00 = src[y0 * PANEL_W + x0], c10 = src[y0 * PANEL_W + x1];
  uint16_t c01 = src[y1 * PANEL_W + x0], c11 = src[y1 * PANEL_W + x1];

  uint32_t ix = 256 - wx, iy = 256 - wy;

  #define CH(c, sh, m) (((c) >> (sh)) & (m))
  #define LERP2(sh, m) (( (CH(c00,sh,m) * ix + CH(c10,sh,m) * wx) * iy + \
                          (CH(c01,sh,m) * ix + CH(c11,sh,m) * wx) * wy + 32768) >> 16)
  uint16_t r = LERP2(11, 31), g = LERP2(5, 63), b = LERP2(0, 31);
  #undef LERP2
  #undef CH

  return (r << 11) | (g << 5) | b;
}

// Sends one RGB565 pixel to the panel, keeping near-greys exactly grey.
// RGB565 has 6 bits of green but only 5 of red and blue, so a grey comes
// back with green a step off. The panel's brightness curve is steep at dim
// levels, where one step can mean 20% more green: dark greys glow green (or
// magenta when the rounding goes the other way). Colours within a hair of
// neutral are therefore sent as equal R, G and B at 8 bits; real colours,
// even subtle dark tints, are too far from grey to be touched.
// White balance is applied after this, in outRGB().
static inline void putPixel(int x, int y, uint16_t c) {
  uint8_t r5 = c >> 11, g6 = (c >> 5) & 63, b5 = c & 31;
  uint8_t r = (r5 << 3) | (r5 >> 2);
  uint8_t g = (g6 << 2) | (g6 >> 4);
  uint8_t b = (b5 << 3) | (b5 >> 2);
  uint8_t hi = max(r, max(g, b)), lo = min(r, min(g, b));

  // Shadow cleanup: the panel only has a few brightness steps for dark
  // tones, and each channel lands on its own step, so a faint colour cast
  // in a dark area (common in JPEGs) turns into an obvious green or
  // magenta. Dark pixels are pulled towards grey: full strength up to
  // 48/255, fading out towards 96, above which colours are left alone.
  if (hi < 96 && cfgShadow) {
    int k = (int)cfgShadow * 256 / 100;                       // 0-256
    if (hi > 48) k = k * (96 - hi) / 48;
    int y = (r + g + b) / 3;
    r = r + (((y - r) * k) >> 8);
    g = g + (((y - g) * k) >> 8);
    b = b + (((y - b) * k) >> 8);
    hi = max(r, max(g, b));
    lo = min(r, min(g, b));
  }
  if (hi - lo <= 6 + hi / 32) r = g = b = (r + g + b + 1) / 3;
  outRGB(x, y, r, g, b);
}

void renderDisc(const uint16_t *src, float angleDeg) {
  float rad = angleDeg * 0.0174532925f;
  int32_t cosF = (int32_t)(cosf(rad) * 256.0f);
  int32_t sinF = (int32_t)(sinf(rad) * 256.0f);

  for (int y = 0; y < PANEL_H; y++) {
    int32_t dy = ((int32_t)y << 16) - CENTER_FP;

    for (int x = 0; x < PANEL_W; x++) {
      uint8_t a = discMask[y * PANEL_W + x];
      if (!a) { putPixel(x, y, 0); continue; }

      int32_t dx  = ((int32_t)x << 16) - CENTER_FP;
      int32_t sxF = ((dx * cosF + dy * sinF) >> 8) + CENTER_FP;
      int32_t syF = ((dy * cosF - dx * sinF) >> 8) + CENTER_FP;

      uint16_t c;
      if (SMOOTH_ROTATION) {
        c = sampleBilinear(src, sxF, syF);
      } else {
        int sx = constrain(sxF >> 16, 0, PANEL_W - 1);
        int sy = constrain(syF >> 16, 0, PANEL_H - 1);
        c = src[sy * PANEL_W + sx];
      }

      if (a < 255) c = scale565(c, a);
      putPixel(x, y, c);
    }
  }
  dma_display->flipDMABuffer();
}

// ======================== LYRICS: DISPLAY ========================

const uint8_t  LYRICS_BG_LEVEL = 70;    // Cover brightness behind the text (0-255)
const uint16_t LYRICS_FADE_MS  = 250;   // Fade-in of each new line

GFXcanvas1 lyricMask(PANEL_W, PANEL_H);  // Current line, 1 bit per pixel
GFXcanvas1 nextMask(PANEL_W, PANEL_H);   // Upcoming line, drawn dim

int      shownLyricIdx   = -2;
uint32_t lyricFadeStart  = 0;
uint16_t lastDrawnFade   = 0xFFFF;
uint32_t shownCoverVer   = 0;

// Current playback position including the user's timing offset.
uint32_t playbackPosMs() {
  portENTER_CRITICAL(&posMux);
  uint32_t base = posBaseMs, at = posBaseMillis;
  portEXIT_CRITICAL(&posMux);
  int32_t pos = (int32_t)(base + (isPlaying ? (millis() - at) : 0)) + cfgLyricOffset;
  return pos < 0 ? 0 : (uint32_t)pos;
}

// Last line that has started at pos, or -1 during the intro.
// Caller holds lyricsMutex with lyricsState == LYR_READY.
int findLyricIndex(uint32_t pos) {
  int lo = 0, hi = (int)lyricCount - 1, ans = -1;
  while (lo <= hi) {
    int mid = (lo + hi) / 2;
    if (lyricTime[mid] <= pos) { ans = mid; lo = mid + 1; }
    else hi = mid - 1;
  }
  return ans;
}

// UTF-8 to the panel font's code page (CP437). Accented letters it has get
// their real glyph (é, ü, ç, ñ, ß...), the rest fall back to the plain
// letter, typographic quotes and dashes become ASCII, and scripts the font
// can't draw are left out.
size_t utf8ToCp437(const char *in, char *out, size_t cap) {
  static const uint16_t glyphCp[] = {
    0xC7,0xFC,0xE9,0xE2,0xE4,0xE0,0xE5,0xE7,0xEA,0xEB,0xE8,0xEF,0xEE,0xEC,0xC4,0xC5,
    0xC9,0xE6,0xC6,0xF4,0xF6,0xF2,0xFB,0xF9,0xFF,0xD6,0xDC,0xE1,0xED,0xF3,0xFA,0xF1,
    0xD1,0xBF,0xA1,0xDF };
  static const uint8_t glyphCh[] = {
    0x80,0x81,0x82,0x83,0x84,0x85,0x86,0x87,0x88,0x89,0x8A,0x8B,0x8C,0x8D,0x8E,0x8F,
    0x90,0x91,0x92,0x93,0x94,0x95,0x96,0x97,0x98,0x99,0x9A,0xA0,0xA1,0xA2,0xA3,0xA4,
    0xA5,0xA8,0xAD,0xE1 };

  size_t n = 0;
  auto putc_ = [&](uint8_t c) { if (n + 1 < cap) out[n++] = (char)c; };
  auto puts_ = [&](const char *t) { while (*t) putc_((uint8_t)*t++); };

  const uint8_t *p = (const uint8_t *)in;
  while (*p) {
    uint32_t cp;
    int extra;
    if      (*p < 0x80)           { cp = *p++;        extra = 0; }
    else if ((*p & 0xE0) == 0xC0) { cp = *p++ & 0x1F; extra = 1; }
    else if ((*p & 0xF0) == 0xE0) { cp = *p++ & 0x0F; extra = 2; }
    else if ((*p & 0xF8) == 0xF0) { cp = *p++ & 0x07; extra = 3; }
    else { p++; continue; }

    bool bad = false;
    for (int i = 0; i < extra; i++) {
      if ((*p & 0xC0) != 0x80) { bad = true; break; }
      cp = (cp << 6) | (*p++ & 0x3F);
    }
    if (bad) continue;

    if (cp < 0x80) { putc_(cp); continue; }

    bool done = false;
    for (size_t i = 0; i < sizeof(glyphCp) / sizeof(glyphCp[0]); i++) {
      if (glyphCp[i] == cp) { putc_(glyphCh[i]); done = true; break; }
    }
    if (done) continue;

    if (cp == 0x2018 || cp == 0x2019 || cp == 0x2032) putc_('\'');
    else if (cp == 0x201C || cp == 0x201D)            putc_('"');
    else if (cp == 0x2013 || cp == 0x2014)            putc_('-');
    else if (cp == 0x2026)                            puts_("...");
    else if (cp == 0xA0)                              putc_(' ');
    else if (cp == 0x152)                             puts_("OE");
    else if (cp == 0x153)                             puts_("oe");
    else if (cp >= 0xC0 && cp <= 0xC6)                putc_('A');
    else if (cp >= 0xC8 && cp <= 0xCB)                putc_('E');
    else if (cp >= 0xCC && cp <= 0xCF)                putc_('I');
    else if ((cp >= 0xD2 && cp <= 0xD6) || cp == 0xD8) putc_('O');
    else if (cp >= 0xD9 && cp <= 0xDC)                putc_('U');
    else if (cp == 0xDD)                              putc_('Y');
    else if (cp >= 0xE0 && cp <= 0xE6)                putc_('a');
    else if (cp >= 0xE8 && cp <= 0xEB)                putc_('e');
    else if (cp >= 0xEC && cp <= 0xEF)                putc_('i');
    else if ((cp >= 0xF2 && cp <= 0xF6) || cp == 0xF8) putc_('o');
    else if (cp >= 0xF9 && cp <= 0xFC)                putc_('u');
    else if (cp == 0xFD || cp == 0xFF)                putc_('y');
    // Anything else (CJK, emoji...) is left out
  }
  out[n] = 0;
  return n;
}

// Greedy word wrap into rows of at most 10 characters (the 64 px width).
int wrapRows(const char *s, char rows[][11], int maxRows) {
  const int maxChars = 10;
  int len = strlen(s), i = 0, r = 0;
  while (i < len && r < maxRows) {
    while (s[i] == ' ') i++;
    if (i >= len) break;
    int take = len - i;
    if (take > maxChars) {
      int brk = -1;
      for (int k = maxChars; k > 0; k--) if (s[i + k] == ' ') { brk = k; break; }
      take = brk > 0 ? brk : maxChars;          // No space: split the word
    }
    memcpy(rows[r], s + i, take);
    rows[r][take] = 0;
    for (int t = take; t > 0 && rows[r][t - 1] == ' '; t--) rows[r][t - 1] = 0;
    r++;
    i += take;
  }
  return r;
}

void printCentered(GFXcanvas1 &cv, const char *text, int y) {
  int w = strlen(text) * 6 - 1;
  cv.setCursor((PANEL_W - w) / 2, y);
  cv.print(text);
}

void buildLyricMasks(const char *cur, const char *next) {
  char buf[160];
  char rows[5][11];

  lyricMask.fillScreen(0);
  nextMask.fillScreen(0);
  for (GFXcanvas1 *cv : { &lyricMask, &nextMask }) {
    cv->setTextWrap(false);
    cv->cp437(true);
    cv->setTextSize(1);
    cv->setTextColor(1);
  }

  // Current line: up to 5 rows, centred in the top 50 px.
  // Empty (intro, instrumental break): a music note instead.
  utf8ToCp437(cur, buf, sizeof(buf));
  int n = wrapRows(buf, rows, 5);
  if (!n) { rows[0][0] = 0x0E; rows[0][1] = 0; n = 1; }
  int y0 = (50 - (n * 10 - 2)) / 2;
  for (int r = 0; r < n; r++) printCentered(lyricMask, rows[r], y0 + r * 10);

  // Next line: first row only, dim, along the bottom edge.
  utf8ToCp437(next, buf, sizeof(buf));
  if (wrapRows(buf, rows, 1)) printCentered(nextMask, rows[0], 55);
}

static inline uint16_t lerp565(uint16_t a, uint16_t b, uint16_t t) {   // t: 0-256
  uint16_t ar = a >> 11, ag = (a >> 5) & 63, ab = a & 31;
  uint16_t br = b >> 11, bg = (b >> 5) & 63, bb = b & 31;
  uint16_t r = (ar * (256 - t) + br * t) >> 8;
  uint16_t g = (ag * (256 - t) + bg * t) >> 8;
  uint16_t bl = (ab * (256 - t) + bb * t) >> 8;
  return (r << 11) | (g << 5) | bl;
}

static inline bool nearMask(int x, int y) {
  for (int dy = -1; dy <= 1; dy++)
    for (int dx = -1; dx <= 1; dx++)
      if ((dx || dy) && lyricMask.getPixel(x + dx, y + dy)) return true;
  return false;
}

// Dimmed cover behind, current line in white with a dark halo so it reads
// over bright artwork, next line in grey at the bottom.
void drawLyricsFrame(uint16_t fade) {
  const uint16_t nextColor = dma_display->color565(110, 110, 110);

  xSemaphoreTake(bufMutex, portMAX_DELAY);
  for (int y = 0; y < PANEL_H; y++) {
    for (int x = 0; x < PANEL_W; x++) {
      uint16_t art = displayBuf[y * PANEL_W + x];
      uint16_t bg  = scale565(art, LYRICS_BG_LEVEL);
      uint16_t c;
      if (lyricMask.getPixel(x, y))      c = lerp565(bg, 0xFFFF, fade);
      else if (nearMask(x, y))           c = scale565(art, LYRICS_BG_LEVEL / 4);
      else if (nextMask.getPixel(x, y))  c = nextColor;
      else                               c = bg;
      putPixel(x, y, c);
    }
  }
  xSemaphoreGive(bufMutex);
}

// Called every frame in lyrics mode. Only redraws when the line changes,
// during its fade-in, or when forced (entering the mode, new cover).
void updateLyricsView(bool force) {
  char cur[160] = "", next[160] = "";
  int  idx;

  xSemaphoreTake(lyricsMutex, portMAX_DELAY);
  if (lyricsState != LYR_READY) { xSemaphoreGive(lyricsMutex); return; }
  idx = findLyricIndex(playbackPosMs());
  bool changed = (idx != shownLyricIdx);
  if (changed || force) {
    if (idx >= 0)             strlcpy(cur,  lyricText + lyricOff[idx],     sizeof(cur));
    if (idx + 1 < lyricCount) strlcpy(next, lyricText + lyricOff[idx + 1], sizeof(next));
  }
  xSemaphoreGive(lyricsMutex);

  if (changed || force) {
    if (changed) lyricFadeStart = millis();
    shownLyricIdx = idx;
    buildLyricMasks(cur, next);
    lastDrawnFade = 0xFFFF;                  // Make sure the next frame draws
  }

  uint32_t el = millis() - lyricFadeStart;
  uint16_t fade = el >= LYRICS_FADE_MS ? 256 : (uint16_t)(el * 256 / LYRICS_FADE_MS);
  if (fade == lastDrawnFade) return;         // Static line: nothing to do
  drawLyricsFrame(fade);
  lastDrawnFade = fade;
}

// ========================= HOME: DISPLAY =========================
// Big clock, animated weather icon, temperature, date, a thin seconds line,
// over a sky gradient that follows the time of day.

enum : uint8_t { WX_NONE, WX_SUN, WX_MOON, WX_PARTLY_SUN, WX_PARTLY_MOON,
                 WX_CLOUD, WX_FOG, WX_RAIN, WX_SNOW, WX_THUNDER };

uint8_t weatherIconKind(const Weather &w) {
  if (!w.valid) return WX_NONE;
  uint8_t c = w.code;
  if (c == 0)                                    return w.isDay ? WX_SUN : WX_MOON;
  if (c <= 2)                                    return w.isDay ? WX_PARTLY_SUN : WX_PARTLY_MOON;
  if (c == 3)                                    return WX_CLOUD;
  if (c == 45 || c == 48)                        return WX_FOG;
  if ((c >= 71 && c <= 77) || c == 85 || c == 86) return WX_SNOW;
  if (c >= 95)                                   return WX_THUNDER;
  if (c >= 51)                                   return WX_RAIN;
  return WX_CLOUD;
}

static inline uint16_t rgb565(float r, float g, float b) {
  uint8_t R = constrain((int)r, 0, 255), G = constrain((int)g, 0, 255), B = constrain((int)b, 0, 255);
  return ((R >> 3) << 11) | ((G >> 2) << 5) | (B >> 3);
}

static bool inCloud(float x, float y) {
  auto in = [](float px, float py, float cx, float cy, float r) {
    float dx = px - cx, dy = py - cy;
    return dx * dx + dy * dy <= r * r;
  };
  return in(x, y, 5.5f, 9.0f, 3.2f) || in(x, y, 9.5f, 7.0f, 4.0f) || in(x, y, 12.5f, 9.5f, 2.8f) ||
         (x >= 4.0f && x <= 13.5f && y >= 9.0f && y <= 12.2f);
}

// Colour of pixel (x, y) of a 16x16 weather icon, false if empty.
// t is a running time in seconds for the animations.
bool iconPixel(uint8_t kind, int x, int y, float t, uint16_t &col) {
  float px = x + 0.5f, py = y + 0.5f;

  auto sun = [&](float cx, float cy, float r) -> bool {
    float dx = px - cx, dy = py - cy, d = sqrtf(dx * dx + dy * dy);
    if (d <= r) { col = rgb565(255, 196, 40); return true; }
    if (d >= r + 1.6f && d <= r + 3.6f) {
      float a = atan2f(dy, dx) + t * 0.5f;               // Rays turn slowly
      if (fabsf(sinf(a * 4.0f)) < 0.28f) { col = rgb565(255, 140, 20); return true; }
    }
    return false;
  };
  auto moon = [&](float cx, float cy, float r) -> bool {
    float dx = px - cx, dy = py - cy;
    float ox = px - (cx + r * 0.55f), oy = py - (cy - r * 0.45f);
    if (dx * dx + dy * dy <= r * r && ox * ox + oy * oy > r * r * 0.8f) {
      col = rgb565(235, 230, 190);
      return true;
    }
    return false;
  };
  auto cloud = [&](float ox, float oy) -> bool {
    if (!inCloud(px - ox, py - oy)) return false;
    col = (py - oy > 11.0f) ? rgb565(150, 156, 172) : rgb565(205, 210, 222);
    return true;
  };

  switch (kind) {
    case WX_SUN:
      return sun(7.5f, 7.5f, 3.4f);
    case WX_MOON:
      if (moon(7.5f, 8.0f, 5.0f)) return true;
      if ((x == 13 && y == 3) || (x == 2 && y == 13)) {            // Twinkling stars
        float v = 120 + 120 * (0.5f + 0.5f * sinf(t * 3.0f + x));
        col = rgb565(v, v, v);
        return true;
      }
      return false;
    case WX_PARTLY_SUN:
      if (cloud(1.5f, 2.5f)) return true;
      return sun(5.0f, 5.0f, 2.8f);
    case WX_PARTLY_MOON:
      if (cloud(1.5f, 2.5f)) return true;
      return moon(5.5f, 5.5f, 4.0f);
    case WX_CLOUD:
      return cloud(0.0f, 1.0f);
    case WX_FOG: {
      static const int rows[] = { 4, 7, 10, 13 };
      for (int i = 0; i < 4; i++) {
        if (y != rows[i]) continue;
        float shift = sinf(t * 0.8f + i) * 2.0f;                  // Drifting banks
        if (px > 1.5f + shift + (i & 1) * 2 && px < 14.5f + shift - (i & 1)) {
          col = rgb565(150, 150, 162);
          return true;
        }
      }
      return false;
    }
    case WX_RAIN:
    case WX_SNOW:
    case WX_THUNDER: {
      if (cloud(0.0f, -2.0f)) return true;
      if (kind == WX_THUNDER) {
        static const int8_t bolt[][2] = { {8,10}, {7,11}, {8,11}, {7,12}, {6,13}, {7,13}, {6,14} };
        bool flash = fmodf(t, 2.4f) < 0.25f;
        for (auto &b : bolt) {
          if (x == b[0] && y == b[1]) {
            col = flash ? rgb565(255, 255, 210) : rgb565(255, 196, 40);
            return true;
          }
        }
        return false;
      }
      float speed = (kind == WX_RAIN) ? 7.0f : 2.5f;              // Rain falls, snow drifts
      static const int cols[] = { 4, 8, 12 };
      for (int i = 0; i < 3; i++) {
        if (x != cols[i]) continue;
        int fy = 11 + (int)fmodf(t * speed + i * 1.7f, 5.0f);
        if (kind == WX_RAIN && (y == fy || y == fy + 1) && y <= 15) { col = rgb565(80, 150, 255);  return true; }
        if (kind == WX_SNOW && y == fy && y <= 15)                   { col = rgb565(230, 235, 245); return true; }
      }
      return false;
    }
  }
  return false;
}

// Sky gradient keyframes: hour, top colour, bottom colour
struct SkyKey { float h; uint8_t top[3], bot[3]; };
static const SkyKey SKY[] = {
  {  0.0f, {  6,  8, 30 }, {  0,  0,  6 } },   // Night
  {  5.0f, {  6,  8, 30 }, {  0,  0,  6 } },
  {  7.0f, { 90, 40, 60 }, { 40, 14, 20 } },   // Dawn
  {  9.0f, { 20, 60,120 }, {  4, 16, 40 } },   // Day
  { 17.0f, { 24, 60,115 }, {  6, 16, 40 } },
  { 19.5f, { 95, 32, 70 }, { 30,  8, 30 } },   // Dusk
  { 21.5f, {  6,  8, 30 }, {  0,  0,  6 } },   // Night
  { 24.0f, {  6,  8, 30 }, {  0,  0,  6 } },
};

void skyColors(float hour, float top[3], float bot[3]) {
  int i = 0;
  while (i < 6 && hour >= SKY[i + 1].h) i++;
  float f = (hour - SKY[i].h) / (SKY[i + 1].h - SKY[i].h);
  for (int c = 0; c < 3; c++) {
    top[c] = SKY[i].top[c] + (SKY[i + 1].top[c] - SKY[i].top[c]) * f;
    bot[c] = SKY[i].bot[c] + (SKY[i + 1].bot[c] - SKY[i].bot[c]) * f;
  }
}

bool localNow(struct tm &t, float &secFrac) {
  struct timeval tv;
  gettimeofday(&tv, nullptr);
  if (tv.tv_sec < 1700000000) return false;        // Not synced with NTP yet
  localtime_r(&tv.tv_sec, &t);
  secFrac = tv.tv_usec / 1e6f;
  return true;
}

void renderHome() {
  struct tm t;
  float sf = 0.0f;
  bool haveTime = localNow(t, sf);

  Weather w;
  portENTER_CRITICAL(&wxMux);
  w = weatherNow;
  portEXIT_CRITICAL(&wxMux);

  float anim = millis() / 1000.0f;

  // --- Big time into lyricMask (the lyrics view rebuilds it on return) ---
  char hh[3] = "--", mm[3] = "--";
  if (haveTime) {
    snprintf(hh, sizeof(hh), "%02d", t.tm_hour);
    snprintf(mm, sizeof(mm), "%02d", t.tm_min);
  }
  auto adv = [](char c) { return c == '1' ? 14 : (c == '-' ? 8 : 13); };   // Glyph advances
  int total = adv(hh[0]) + adv(hh[1]) + 6 + adv(mm[0]) + adv(mm[1]);
  const int base = 40;                             // Baseline of the digits

  lyricMask.fillScreen(0);
  lyricMask.setFont(&FreeSansBold12pt7b);
  lyricMask.setTextSize(1);
  lyricMask.setTextWrap(false);
  lyricMask.setTextColor(1);
  lyricMask.setCursor((PANEL_W - total) / 2 + 1, base);
  lyricMask.print(hh);
  int colonX = lyricMask.getCursorX() + 2;
  lyricMask.setCursor(lyricMask.getCursorX() + 6, base);
  lyricMask.print(mm);
  lyricMask.setFont(nullptr);

  // --- Temperature and date into nextMask ---
  nextMask.fillScreen(0);
  nextMask.setFont(nullptr);
  nextMask.cp437(true);
  nextMask.setTextSize(1);
  nextMask.setTextWrap(false);
  nextMask.setTextColor(1);

  char temp[8];
  if (w.valid) snprintf(temp, sizeof(temp), "%d\xF8", (int)lroundf(w.tempC));   // \xF8 = degree sign
  else         strcpy(temp, "--\xF8");
  nextMask.setCursor(PANEL_W - 3 - ((int)strlen(temp) * 6 - 1), 6);
  nextMask.print(temp);

  char line[16];
  if ((int32_t)(millis() - panelToastUntil) < 0) {
    strlcpy(line, panelToast, sizeof(line));
  } else if (haveTime) {
    strftime(line, sizeof(line), "%a %d %b", &t);
    for (char *c = line; *c; c++) *c = toupper((unsigned char)*c);
  } else {
    strcpy(line, "SYNCING");
  }
  nextMask.setCursor((PANEL_W - ((int)strlen(line) * 6 - 1)) / 2, 48);
  nextMask.print(line);

  // --- Compose ---
  float top[3], bot[3];
  skyColors(haveTime ? t.tm_hour + t.tm_min / 60.0f : 12.0f, top, bot);
  uint16_t rowCol[PANEL_H];
  for (int y = 0; y < PANEL_H; y++) {
    float f = y / 63.0f;
    rowCol[y] = rgb565(top[0] + (bot[0] - top[0]) * f,
                       top[1] + (bot[1] - top[1]) * f,
                       top[2] + (bot[2] - top[2]) * f);
  }

  const uint16_t timeCol  = rgb565(255, 244, 228);   // Warm white
  const uint16_t smallCol = rgb565(170, 176, 192);
  const uint16_t barCol   = rgb565(255, 150, 40);
  const uint16_t barDim   = rgb565(34, 34, 44);
  float pulse = haveTime ? 0.3f + 0.7f * (0.5f + 0.5f * cosf(sf * 6.2832f)) : 1.0f;   // Colon breathes
  uint16_t colonCol = lerp565(rowCol[base - 8], timeCol, (uint16_t)(pulse * 256));
  uint8_t kind = weatherIconKind(w);
  int progW = haveTime ? (int)((t.tm_sec + sf) / 60.0f * 56) : 0;                    // Seconds line

  for (int y = 0; y < PANEL_H; y++) {
    for (int x = 0; x < PANEL_W; x++) {
      uint16_t c = rowCol[y], ic;
      if (x >= 3 && x < 19 && y >= 1 && y < 17 && iconPixel(kind, x - 3, y - 1, anim, ic)) c = ic;
      if (nextMask.getPixel(x, y))  c = smallCol;
      if (lyricMask.getPixel(x, y)) c = timeCol;
      if ((x == colonX || x == colonX + 1) &&
          (y == base - 12 || y == base - 11 || y == base - 5 || y == base - 4)) c = colonCol;
      if (y == 59 && x >= 4 && x < 60) c = (x - 4 < progW) ? barCol : barDim;
      out565(x, y, c);
    }
  }
}

// Plain album cover, square and still.
void drawCoverStatic() {
  xSemaphoreTake(bufMutex, portMAX_DELAY);
  for (int y = 0; y < PANEL_H; y++)
    for (int x = 0; x < PANEL_W; x++)
      putPixel(x, y, displayBuf[y * PANEL_W + x]);
  xSemaphoreGive(bufMutex);
}

// ======================= CONNECTION SCREENS =======================
// WiFi symbol (dot + three arcs), drawn with soft edges and animated per
// screen: cyan wave while connecting, blue for setup, green on success,
// red with a pulsing slash when the connection is lost.

// How much of pixel (px, py) the symbol covers (0-1); part 0 = dot, 1-3 = arcs.
static float wifiCoverage(float px, float py, int &part) {
  const float cx = 32.0f, cy = 35.0f;
  float dx = px - cx, dy = py - cy, r = sqrtf(dx * dx + dy * dy);
  if (r < 3.2f) { part = 0; return constrain(3.2f - r, 0.0f, 1.0f); }
  if (dy > 0.0f) return 0.0f;
  float side = constrain((-dy - fabsf(dx)) * 0.7f + 0.5f, 0.0f, 1.0f);   // Soft 45-degree sides
  if (side <= 0.0f) return 0.0f;
  static const float R[3] = { 9.0f, 15.0f, 21.0f };
  for (int k = 0; k < 3; k++) {
    float d = fabsf(r - R[k]);
    if (d < 2.0f) { part = k + 1; return constrain(1.9f - d, 0.0f, 1.0f) * side; }
  }
  return 0.0f;
}

static float distToSegment(float px, float py, float ax, float ay, float bx, float by) {
  float vx = bx - ax, vy = by - ay, wx = px - ax, wy = py - ay;
  float t = constrain((wx * vx + wy * vy) / (vx * vx + vy * vy), 0.0f, 1.0f);
  float dx = px - (ax + t * vx), dy = py - (ay + t * vy);
  return sqrtf(dx * dx + dy * dy);
}

// Centred if it fits, otherwise scrolls smoothly.
void printMarquee(GFXcanvas1 &cv, const char *text, int y, float t) {
  int w = strlen(text) * 6 - 1;
  if (w <= PANEL_W - 2) {
    cv.setCursor((PANEL_W - w) / 2, y);
    cv.print(text);
    return;
  }
  int span = w + 18;
  int off = (int)(t * 16.0f) % span;
  cv.setCursor(2 - off, y);        cv.print(text);
  cv.setCursor(2 - off + span, y); cv.print(text);
}

static void prepTextMask() {
  nextMask.fillScreen(0);
  nextMask.setFont(nullptr);
  nextMask.cp437(true);
  nextMask.setTextSize(1);
  nextMask.setTextWrap(false);
  nextMask.setTextColor(1);
}

void renderNetView() {
  uint8_t v = netView;
  float t = (int32_t)(millis() - netViewSince) / 1000.0f;
  if (t < 0) t = 0;

  char raw[48], line2[64];
  portENTER_CRITICAL(&netMux);
  strlcpy(raw, netLine, sizeof(raw));
  portEXIT_CRITICAL(&netMux);
  utf8ToCp437(raw, line2, sizeof(line2));

  float col[3] = { 60, 190, 255 };
  float lvl[4] = { 1, 1, 1, 1 };
  float t1[3]  = { 150, 156, 172 }, t2[3] = { 60, 150, 200 };
  const char *line1 = "";
  bool  slash = false;
  float slashLvl = 1.0f, fade = 1.0f;

  switch (v) {
    case NV_CONNECTING:                  // A wave travels outward through the arcs
      line1 = "CONNECTING";
      for (int k = 1; k <= 3; k++) {
        float w = cosf(6.2832f * (t * 0.8f - (k - 1) * 0.16f));
        lvl[k] = 0.12f + 0.88f * powf(max(0.0f, w), 3.0f);
      }
      break;

    case NV_SETUP: {                     // Calm blue, breathing
      line1 = "JOIN WIFI";
      col[0] = 70; col[1] = 120; col[2] = 255;
      t2[0] = 70;  t2[1] = 120;  t2[2] = 255;
      float br = 0.55f + 0.45f * sinf(t * 2.0f);
      lvl[1] = lvl[2] = lvl[3] = 0.25f + 0.5f * br;
      lvl[0] = 0.5f + 0.5f * br;
      snprintf(line2, sizeof(line2), "%s  \xFA  192.168.4.1", SETUP_AP_NAME);   // \xFA = middle dot
      break;
    }

    case NV_CONNECTED:                   // Arcs pop in one by one, bars = signal strength
      line1 = "CONNECTED";
      col[0] = 40;  col[1] = 235; col[2] = 110;
      t1[0] = 40;   t1[1] = 235;  t1[2] = 110;
      t2[0] = 110;  t2[1] = 150;  t2[2] = 125;
      for (int k = 1; k <= 3; k++) {
        float on = t - (0.1f + (k - 1) * 0.12f);
        lvl[k] = k <= netBars ? constrain(on * 6.0f, 0.0f, 1.0f) : 0.15f;
      }
      if (t > 2.6f) fade = max(0.0f, 1.0f - (t - 2.6f) / 0.4f);
      break;

    case NV_LOST:                        // Dim red arcs searching, slash pulsing
      line1 = "WIFI LOST";
      col[0] = 255; col[1] = 45;  col[2] = 45;
      t1[0] = 255;  t1[1] = 70;   t1[2] = 70;
      t2[0] = 150;  t2[1] = 80;   t2[2] = 80;
      for (int k = 1; k <= 3; k++) {
        float w = cosf(6.2832f * (t * 0.5f - (k - 1) * 0.18f));
        lvl[k] = 0.10f + 0.25f * powf(max(0.0f, w), 2.0f);
      }
      lvl[0] = 0.35f;
      slash = true;
      slashLvl = 0.6f + 0.4f * (0.5f + 0.5f * sinf(t * 4.0f));
      break;
  }

  prepTextMask();
  int w1 = strlen(line1) * 6 - 1;
  nextMask.setCursor((PANEL_W - w1) / 2, 44);
  nextMask.print(line1);
  printMarquee(nextMask, line2, 54, t);

  for (int y = 0; y < PANEL_H; y++) {
    for (int x = 0; x < PANEL_W; x++) {
      float r = 0, g = 0, b = 0;
      int part = 0;
      float cov = wifiCoverage(x + 0.5f, y + 0.5f, part);
      if (cov > 0.0f) {
        float L = cov * lvl[part];
        r = col[0] * L; g = col[1] * L; b = col[2] * L;
      }
      if (slash) {
        float d = distToSegment(x + 0.5f, y + 0.5f, 19.0f, 13.0f, 45.0f, 39.0f);
        if (d < 1.3f)      { r = 255 * slashLvl; g = 30 * slashLvl; b = 30 * slashLvl; }
        else if (d < 2.6f) { r = g = b = 0; }                         // Gap around the slash
      }
      if (nextMask.getPixel(x, y)) {
        const float *tc = y < 52 ? t1 : t2;
        r = tc[0]; g = tc[1]; b = tc[2];
      }
      out565(x, y, rgb565(r * fade, g * fade, b * fade));
    }
  }
}

// Shown after connecting until the first cover arrives.
void renderIdle() {
  float t = millis() / 1000.0f;
  char ip[20];
  strlcpy(ip, WiFi.localIP().toString().c_str(), sizeof(ip));

  prepTextMask();
  printCentered(nextMask, "READY", 16);
  printMarquee(nextMask, ip, 30, t);
  printCentered(nextMask, "PLAY MUSIC", 46);

  for (int y = 0; y < PANEL_H; y++) {
    for (int x = 0; x < PANEL_W; x++) {
      uint16_t c = 0;
      if (nextMask.getPixel(x, y)) {
        if (y < 26)      c = rgb565(200, 205, 215);
        else if (y < 40) c = rgb565(60, 150, 200);
        else             c = rgb565(80, 84, 96);
      }
      out565(x, y, c);
    }
  }
}

// ========================= MATRIX SETUP =========================

void setupMatrix() {
  HUB75_I2S_CFG::i2s_pins pins = {
    R1_PIN, G1_PIN, B1_PIN, R2_PIN, G2_PIN, B2_PIN,
    A_PIN, B_PIN, C_PIN, D_PIN, E_PIN,
    LAT_PIN, OE_PIN, CLK_PIN
  };

  HUB75_I2S_CFG mxconfig(PANEL_W, PANEL_H, 1, pins);
  // 8-bit colour MUST match the library's compile-time brightness table
  // (also 8-bit). At any other runtime depth the library keeps only the low
  // bits of each value: dark tones glow, near-whites turn into random colours.
  //
  // Single buffering keeps memory below what 5-bit double buffering used
  // (about 32 KB vs 40 KB). Frames are drawn straight to the visible buffer;
  // at these spin speeds any tearing is sub-pixel.
  mxconfig.double_buff = false;
  mxconfig.setPixelColorDepthBits(8);
  mxconfig.driver   = PANEL_DRIVER;
  mxconfig.clkphase = cfgClkPhase;

  // Clock speed stays at the library default (8 MHz) on purpose. Faster
  // clocks make the library build a much larger DMA table: 56 KB at 16 MHz,
  // 80 KB at 20 MHz, versus 44 KB at 8 MHz. That starves TLS of memory and
  // every Spotify request fails. 8 MHz is also the safest speed for the
  // ICN2037 through the panel's 74HC04 buffer.
  if (cfgLatchBlank >= 1 && cfgLatchBlank <= 4) {
    mxconfig.latch_blanking = cfgLatchBlank;
  }

  activeClkHz = (uint32_t)mxconfig.i2sspeed;
  activeLatch = mxconfig.latch_blanking;
  Serial.printf("[panel] clock %lu Hz, phase %s, latch blanking %u\n",
                (unsigned long)activeClkHz, cfgClkPhase ? "normal" : "alternate", activeLatch);

  dma_display = new MatrixPanel_I2S_DMA(mxconfig);
  dma_display->begin();
  Serial.printf("[mem] after panel init: free heap %u, largest block %u\n",
                ESP.getFreeHeap(), heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
  dma_display->setBrightness8(hwBrightness());
  dma_display->clearScreen();
}

// Three lines, used to show the IP address on the Ready screen.
void showStatus3(const char *l1, const char *l2, const char *l3, uint16_t color) {
  dma_display->clearScreen();
  dma_display->setTextColor(color);
  dma_display->setTextSize(1);
  dma_display->setCursor(2, 14); dma_display->print(l1);
  dma_display->setCursor(2, 28); dma_display->print(l2);
  dma_display->setCursor(2, 40); dma_display->print(l3);
  dma_display->flipDMABuffer();
}

// Only called from setup(), before the tasks start drawing.
void showStatus(const char *line1, const char *line2, uint16_t color) {
  dma_display->clearScreen();
  dma_display->setTextColor(color);
  dma_display->setTextSize(1);
  dma_display->setCursor(2, 22);
  dma_display->print(line1);
  if (line2) {
    dma_display->setCursor(2, 34);
    dma_display->print(line2);
  }
  dma_display->flipDMABuffer();
}

// ========================== SPOTIFY AUTH ========================

void logNetError(const char *tag, int status);

bool refreshAccessToken() {
  WiFiClientSecure client;
  client.setInsecure();

  HTTPClient https;
  if (!https.begin(client, "https://accounts.spotify.com/api/token")) {
    netOk = false;
    return false;
  }

  String credentials = base64::encode(String(SPOTIFY_CLIENT_ID) + ":" + String(SPOTIFY_CLIENT_SECRET));
  https.addHeader("Authorization", "Basic " + credentials);
  https.addHeader("Content-Type", "application/x-www-form-urlencoded");

  String body = "grant_type=refresh_token&refresh_token=" + String(SPOTIFY_REFRESH_TOKEN);
  int status = https.POST(body);
  netOk = (status > 0);

  if (status <= 0) {
    https.end();
    logNetError("auth", status);
    return false;
  }
  if (status != 200) {
    Serial.printf("[auth] HTTP %d: %s\n", status, https.getString().c_str());
    https.end();
    if (status > 0) setTokenOk(false);   // Server said no — a real auth problem
    return false;
  }

  StaticJsonDocument<768> doc;
  DeserializationError err = deserializeJson(doc, https.getString());
  https.end();
  if (err) return false;

  accessToken = doc["access_token"].as<String>();
  uint32_t lifetime = doc["expires_in"] | 3600;
  tokenExpiresAt = millis() + (lifetime - 300) * 1000UL;

  setTokenOk(true);
  Serial.println("[auth] access token refreshed");
  return true;
}

bool ensureToken() {
  if (accessToken.length() == 0 || (int32_t)(millis() - tokenExpiresAt) >= 0) {
    return refreshAccessToken();
  }
  return true;
}

// ====================== NOW PLAYING LOOKUP ======================

bool loadAlbumArt(const String &url);
int  fetchLyrics(const NowPlaying &np);
String lyricsTrackId = "";          // Track whose lyrics we already asked for

void logNetError(const char *tag, int status) {
  Serial.printf("[%s] failed: %s (%d) | free heap %u, largest block %u\n",
                tag, HTTPClient::errorToString(status).c_str(), status,
                ESP.getFreeHeap(), heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
}

// Does the request and parsing in its own stack frame. The TLS client and
// JSON buffers are all released when this returns, before any image download.
// Returns the HTTP status; <= 0 means the request never reached Spotify.
int fetchNowPlaying(NowPlaying &np) {
  WiFiClientSecure client;
  client.setInsecure();

  HTTPClient https;
  if (!https.begin(client, "https://api.spotify.com/v1/me/player/currently-playing")) {
    return -1;
  }
  https.addHeader("Authorization", "Bearer " + accessToken);

  int status = https.GET();
  // Headers just arrived; Spotify read the position a moment before that.
  np.sampledAt = millis() - 60;
  if (status != 200) {
    https.end();
    return status;
  }

  // Heap-allocated so they don't eat into the task stack
  DynamicJsonDocument filter(384);
  filter["is_playing"] = true;
  filter["progress_ms"] = true;
  filter["item"]["duration_ms"] = true;
  filter["item"]["id"] = true;
  filter["item"]["name"] = true;
  filter["item"]["artists"][0]["name"] = true;
  filter["item"]["album"]["images"][0]["url"] = true;
  filter["item"]["album"]["images"][0]["width"] = true;

  DynamicJsonDocument doc(2048);
  DeserializationError err = deserializeJson(doc, https.getStream(),
                                             DeserializationOption::Filter(filter));
  https.end();

  if (err) {
    Serial.printf("[spotify] JSON error: %s\n", err.c_str());
    return -2;
  }

  np.playing    = doc["is_playing"] | false;
  np.progressMs = doc["progress_ms"] | 0;

  JsonObject item = doc["item"];
  if (item.isNull()) return status;          // Podcast, ad, or nothing loaded
  np.hasItem = true;

  np.id         = item["id"].as<String>();
  np.name       = item["name"].as<String>();
  np.durationMs = item["duration_ms"] | 0;
  np.artist0    = item["artists"][0]["name"] | "";

  for (JsonObject a : item["artists"].as<JsonArray>()) {
    if (np.artists.length()) np.artists += ", ";
    np.artists += a["name"].as<const char *>();
  }

  int smallest = 100000;
  for (JsonObject img : item["album"]["images"].as<JsonArray>()) {
    int w = img["width"] | 0;
    if (w == 64) { np.imageUrl = img["url"].as<String>(); break; }
    if (w > 0 && w < smallest) { smallest = w; np.imageUrl = img["url"].as<String>(); }
  }

  return status;
}

void pollSpotify() {
  if (!ensureToken()) {
    setSpotifyStatus("error", nullptr, nullptr, 0, false);
    return;
  }

  NowPlaying np;
  int status = fetchNowPlaying(np);

  if (status <= 0 && status != -2) {         // -2 = reached Spotify, bad JSON
    netOk = false;
    logNetError("spotify", status);
    setSpotifyStatus("error", nullptr, nullptr, status, false);
    return;
  }
  netOk = true;

  if (status == 401) {                       // Token died early
    accessToken = "";
    return;
  }

  if (status == 204 || (status == 200 && !np.hasItem)) {
    isPlaying = false;
    setSpotifyStatus("idle", "", "", status, true);
    if (currentTrackId != "") {
      currentTrackId = "";
      haveArt = false;
    }
    return;
  }

  if (status != 200) {
    Serial.printf("[spotify] HTTP %d\n", status);
    setSpotifyStatus("error", nullptr, nullptr, status, false);
    return;
  }

  portENTER_CRITICAL(&posMux);
  posBaseMs     = np.progressMs;
  posBaseMillis = np.sampledAt;
  portEXIT_CRITICAL(&posMux);
  isPlaying = np.playing;

  setSpotifyStatus(np.playing ? "playing" : "paused",
                   np.name.c_str(), np.artists.c_str(), status, true);

  // New song: hide the old song's lyrics straight away
  bool needLyrics = (np.id != lyricsTrackId);
  if (needLyrics && lyricsState != LYR_LOADING) {
    xSemaphoreTake(lyricsMutex, portMAX_DELAY);
    lyricsState = LYR_LOADING;
    xSemaphoreGive(lyricsMutex);
  }

  if (artReload) {                     // Image settings changed on the dashboard
    artReload = false;
    currentTrackId = "";
  }

  if (np.id != currentTrackId && np.imageUrl.length()) {
    Serial.println("[spotify] new track: " + np.name + " - " + np.artists);
    if (loadAlbumArt(np.imageUrl)) {
      currentTrackId = np.id;
    } else {
      Serial.println("[image] failed to load cover");
    }
  }

  if (needLyrics) {
    int r = fetchLyrics(np);
    if (r > 0) lyricsTrackId = np.id;   // Got an answer, found or not: don't ask again
  }
}

// ==================== DOWNLOAD, DECODE, SWAP ====================

// ==================== COVER IMAGE PROCESSING =====================
// Runs once per cover, right after decoding. Tuned for an LED panel in a
// dim room rather than a monitor:
//   1. Black level: JPEG noise in dark areas (values around 5-15) would light
//      single LEDs, so everything below the level becomes true black and the
//      rest is stretched back to full range.
//   2. Saturation: very bright LEDs make colours look washed out, so they're
//      pushed slightly away from grey.
//   3. Contrast: a tone curve pulls mid-tones down so they don't bleach
//      towards white.
//   4. Lone dim pixels with no lit neighbours (speckle around edges) are
//      removed, which cleans up the outline of shapes.

static inline uint8_t to8(uint16_t v, uint16_t max) { return (v * 255 + max / 2) / max; }

void processCover(uint16_t *px) {
  const int B = cfgBlackLevel;
  const int S = cfgSaturation;
  const float G = cfgContrast / 100.0f;

  uint8_t curve[256];
  for (int i = 0; i < 256; i++) curve[i] = (uint8_t)lroundf(255.0f * powf(i / 255.0f, G));

  static uint8_t lit[PANEL_W * PANEL_H / 8];
  memset(lit, 0, sizeof(lit));

  for (int i = 0; i < PANEL_W * PANEL_H; i++) {
    uint16_t c = px[i];
    int r = to8((c >> 11) & 31, 31), g = to8((c >> 5) & 63, 63), b = to8(c & 31, 31);

    if (max(r, max(g, b)) <= B) { px[i] = 0; continue; }

    r = max(0, r - B) * 255 / (255 - B);
    g = max(0, g - B) * 255 / (255 - B);
    b = max(0, b - B) * 255 / (255 - B);

    int y = (77 * r + 150 * g + 29 * b) >> 8;
    r = constrain(y + (r - y) * S / 100, 0, 255);
    g = constrain(y + (g - y) * S / 100, 0, 255);
    b = constrain(y + (b - y) * S / 100, 0, 255);

    r = curve[r]; g = curve[g]; b = curve[b];
    px[i] = ((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3);
    if (px[i]) lit[i >> 3] |= 1 << (i & 7);
  }

  auto isLit = [&](int x, int y) -> bool {
    if (x < 0 || y < 0 || x >= PANEL_W || y >= PANEL_H) return false;
    int i = y * PANEL_W + x;
    return ((lit[i >> 3] >> (i & 7)) & 1) != 0;
  };

  for (int y = 0; y < PANEL_H; y++) {
    for (int x = 0; x < PANEL_W; x++) {
      if (!isLit(x, y)) continue;
      uint16_t c = px[y * PANEL_W + x];
      int bright = max((c >> 11) << 3, max(((c >> 5) & 63) << 2, (c & 31) << 3));
      if (bright >= 80) continue;                    // Real detail, keep it
      int neighbours = 0;
      for (int dy = -1; dy <= 1; dy++)
        for (int dx = -1; dx <= 1; dx++)
          if ((dx || dy) && isLit(x + dx, y + dy)) neighbours++;
      if (neighbours <= 1) px[y * PANEL_W + x] = 0;  // Lone speck
    }
  }
}

bool loadAlbumArt(const String &url) {
  // Try plain HTTP first: no TLS handshake, far less memory. Covers are
  // public images, so there's nothing sensitive to protect.
  String plainUrl = url;
  plainUrl.replace("https://", "http://");

  WiFiClient       plainClient;
  WiFiClientSecure tlsClient;
  tlsClient.setInsecure();

  HTTPClient https;
  https.setFollowRedirects(HTTPC_DISABLE_FOLLOW_REDIRECTS);

  int status = -1;
  if (https.begin(plainClient, plainUrl)) {
    status = https.GET();
    if (status != 200) https.end();
  }

  if (status != 200) {
    Serial.printf("[image] plain HTTP gave %d, falling back to HTTPS\n", status);
    if (!https.begin(tlsClient, url)) return false;
    status = https.GET();
    if (status != 200) {
      https.end();
      if (status <= 0) logNetError("image", status);
      else Serial.printf("[image] HTTP %d\n", status);
      return false;
    }
  }

  int len = https.getSize();
  if (len <= 0 || len > MAX_IMAGE_BYTES) { https.end(); return false; }

  uint8_t *jpeg = (uint8_t *)malloc(len);
  if (!jpeg) { https.end(); return false; }

  WiFiClient *stream = https.getStreamPtr();
  int read = 0;
  uint32_t deadline = millis() + 8000;
  while (read < len && millis() < deadline) {
    int avail = stream->available();
    if (avail > 0) read += stream->readBytes(jpeg + read, min(avail, len - read));
    else vTaskDelay(1);
  }
  https.end();

  if (read != len) { free(jpeg); return false; }

  decodeBuf = (uint16_t *)calloc(PANEL_W * PANEL_H, sizeof(uint16_t));
  if (!decodeBuf) { free(jpeg); return false; }

  TJpgDec.drawJpg(0, 0, jpeg, len);
  processCover(decodeBuf);
  free(jpeg);

  // Copy in between frames: 8 KB takes microseconds
  xSemaphoreTake(bufMutex, portMAX_DELAY);
  memcpy(displayBuf, decodeBuf, sizeof(displayBuf));
  coverVersion++;
  xSemaphoreGive(bufMutex);

  free(decodeBuf);
  decodeBuf = nullptr;

  haveArt  = true;
  bootIdle = false;
  newArt  = true;
  return true;
}

// ===================== LYRICS: FETCH & PARSE =====================

String urlEncode(const String &in) {
  static const char hex[] = "0123456789ABCDEF";
  String out;
  out.reserve(in.length() * 3);
  for (size_t i = 0; i < in.length(); i++) {
    uint8_t c = in[i];
    bool plain = (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
                 (c >= '0' && c <= '9') || c == '-' || c == '_' || c == '.' || c == '~';
    if (plain) { out += (char)c; }
    else { out += '%'; out += hex[c >> 4]; out += hex[c & 15]; }
  }
  return out;
}

// One LRC line such as "[01:23.45][02:10.00] Some words". Metadata tags
// like "[ar:Artist]" are skipped. A line with several stamps is stored once
// and referenced from each start time.
static void addLrcLine(char *line) {
  uint32_t times[4];
  int nTimes = 0;
  char *p = line;

  while (*p == '[') {
    char *close = strchr(p, ']');
    if (!close) return;
    char *q = p + 1;
    if (!isdigit((unsigned char)*q)) return;

    uint32_t mm = 0, ss = 0, frac = 0;
    int fracDigits = 0;
    while (isdigit((unsigned char)*q)) mm = mm * 10 + (*q++ - '0');
    if (*q++ != ':') return;
    while (isdigit((unsigned char)*q)) ss = ss * 10 + (*q++ - '0');
    if (*q == '.' || *q == ':') {
      q++;
      while (isdigit((unsigned char)*q) && fracDigits < 3) { frac = frac * 10 + (*q++ - '0'); fracDigits++; }
    }
    while (fracDigits < 3) { frac *= 10; fracDigits++; }

    if (nTimes < 4) times[nTimes++] = (mm * 60 + ss) * 1000UL + frac;
    p = close + 1;
  }
  if (!nTimes) return;

  while (*p == ' ') p++;
  size_t len = strlen(p);
  while (len && (p[len - 1] == ' ' || p[len - 1] == '\r')) p[--len] = 0;
  if (lyricTextUsed + len + 1 > LYRIC_TEXT_BYTES) return;

  uint16_t off = lyricTextUsed;
  memcpy(lyricText + off, p, len + 1);
  lyricTextUsed += len + 1;

  for (int i = 0; i < nTimes && lyricCount < MAX_LYRIC_LINES; i++) {
    lyricTime[lyricCount] = times[i];
    lyricOff[lyricCount]  = off;
    lyricCount++;
  }
}

static void sortLyrics() {
  for (int i = 1; i < lyricCount; i++) {
    uint32_t t = lyricTime[i];
    uint16_t o = lyricOff[i];
    int j = i - 1;
    while (j >= 0 && lyricTime[j] > t) {
      lyricTime[j + 1] = lyricTime[j];
      lyricOff[j + 1]  = lyricOff[j];
      j--;
    }
    lyricTime[j + 1] = t;
    lyricOff[j + 1]  = o;
  }
}

// Streams LRCLIB's JSON and pulls out only "syncedLyrics", parsing it line by
// line as it arrives. The response also carries the plain lyrics, so never
// holding the whole body in memory keeps this to a few hundred bytes.
int requestLyrics(const String &url, bool &instrumental) {
  WiFiClientSecure client;
  client.setInsecure();

  HTTPClient https;
  https.useHTTP10(true);                 // No chunked encoding in the raw stream
  https.setTimeout(8000);
  https.setUserAgent("SpotifyDisc/1.0 (ESP32 LED matrix)");
  if (!https.begin(client, url)) return -1;

  int status = https.GET();
  if (status != 200) {
    https.end();
    return status;
  }

  static const char KEY_SYNC[]  = "\"syncedLyrics\":";
  static const char KEY_INSTR[] = "\"instrumental\":true";
  int  mSync = 0, mInstr = 0, phase = 0;   // 0 find key, 1 value start, 2 in string, 3 done
  bool esc = false;
  char line[160];
  int  lineLen = 0;

  auto feed = [&](char c) {
    if (c == '\n') { line[lineLen] = 0; addLrcLine(line); lineLen = 0; }
    else if (lineLen < (int)sizeof(line) - 1) line[lineLen++] = c;
  };

  WiFiClient *st = https.getStreamPtr();
  uint32_t deadline = millis() + 10000;

  auto nextByte = [&]() -> int {
    while (!st->available()) {
      if (!st->connected() || millis() > deadline) return -1;
      vTaskDelay(1);
    }
    return st->read();
  };

  while (phase < 3) {
    int b = nextByte();
    if (b < 0) break;
    char c = (char)b;

    if (phase == 0) {
      mSync  = (c == KEY_SYNC[mSync])   ? mSync + 1  : (c == KEY_SYNC[0]  ? 1 : 0);
      mInstr = (c == KEY_INSTR[mInstr]) ? mInstr + 1 : (c == KEY_INSTR[0] ? 1 : 0);
      if (!KEY_INSTR[mInstr]) { instrumental = true; mInstr = 0; }
      if (!KEY_SYNC[mSync]) phase = 1;
    } else if (phase == 1) {
      if (c == ' ') continue;
      phase = (c == '"') ? 2 : 3;                    // null: no synced lyrics
    } else if (esc) {
      esc = false;
      switch (c) {
        case 'n': feed('\n'); break;
        case 't': feed(' ');  break;
        case 'r': case 'b': case 'f': break;
        case 'u': {
          char h[5] = { 0 };
          for (int i = 0; i < 4; i++) { int x = nextByte(); h[i] = x < 0 ? '0' : (char)x; }
          uint32_t cp = strtoul(h, nullptr, 16);
          if (cp < 0x80)                     { feed((char)cp); }
          else if (cp < 0x800)               { feed(0xC0 | (cp >> 6)); feed(0x80 | (cp & 0x3F)); }
          else if (cp < 0xD800 || cp > 0xDFFF) {
            feed(0xE0 | (cp >> 12)); feed(0x80 | ((cp >> 6) & 0x3F)); feed(0x80 | (cp & 0x3F));
          }                                   // Surrogates (emoji): dropped
          break;
        }
        default: feed(c);                     // \" \\ \/
      }
    } else if (c == '\\') {
      esc = true;
    } else if (c == '"') {
      if (lineLen) { line[lineLen] = 0; addLrcLine(line); lineLen = 0; }
      phase = 3;
    } else {
      feed(c);
    }
  }

  https.end();
  return status;
}

int fetchLyrics(const NowPlaying &np) {
  xSemaphoreTake(lyricsMutex, portMAX_DELAY);
  lyricsState = LYR_LOADING;
  xSemaphoreGive(lyricsMutex);
  lyricCount = 0;
  lyricTextUsed = 0;

  String base = "https://lrclib.net/api/get?track_name=" + urlEncode(np.name) +
                "&artist_name=" + urlEncode(np.artist0);
  bool instrumental = false;

  // The duration picks the right version (live, remaster, radio edit...)
  int status = requestLyrics(base + "&duration=" + String((np.durationMs + 500) / 1000), instrumental);
  if (status == 404) status = requestLyrics(base, instrumental);

  if (status <= 0) {
    logNetError("lyrics", status);
    lyricsState = LYR_ERROR;
    return status;
  }

  sortLyrics();
  uint8_t result = lyricCount ? LYR_READY : (instrumental ? LYR_INSTRUMENTAL : LYR_NOTFOUND);

  xSemaphoreTake(lyricsMutex, portMAX_DELAY);
  lyricsState = result;
  xSemaphoreGive(lyricsMutex);

  Serial.printf("[lyrics] %s: %u synced lines (HTTP %d)\n",
                result == LYR_READY ? "found" : result == LYR_INSTRUMENTAL ? "instrumental" : "none",
                lyricCount, status);
  return status;
}

// ============================ WEATHER ============================
// Open-Meteo: free, no key needed. Plain HTTP first (no TLS memory),
// HTTPS as a fallback.

bool fetchWeather() {
  String path = "/v1/forecast?latitude=" + String(WEATHER_LAT, 3) +
                "&longitude=" + String(WEATHER_LON, 3) +
                "&current=temperature_2m,weather_code,is_day&timezone=auto";
  String body;
  {
    WiFiClient plain;
    HTTPClient http;
    if (http.begin(plain, "http://api.open-meteo.com" + path)) {
      if (http.GET() == 200) body = http.getString();
      http.end();
    }
  }
  if (!body.length()) {
    WiFiClientSecure tls;
    tls.setInsecure();
    HTTPClient https;
    if (https.begin(tls, "https://api.open-meteo.com" + path)) {
      int st = https.GET();
      if (st == 200) body = https.getString();
      else if (st <= 0) logNetError("weather", st);
      https.end();
    }
  }
  if (!body.length()) return false;

  StaticJsonDocument<256> filter;
  filter["current"]["temperature_2m"] = true;
  filter["current"]["weather_code"]   = true;
  filter["current"]["is_day"]         = true;
  DynamicJsonDocument doc(512);
  if (deserializeJson(doc, body, DeserializationOption::Filter(filter))) return false;
  JsonObject cur = doc["current"];
  if (cur.isNull()) return false;

  Weather w;
  w.valid = true;
  w.tempC = cur["temperature_2m"] | 0.0f;
  w.code  = cur["weather_code"] | 0;
  w.isDay = (cur["is_day"] | 1) != 0;

  portENTER_CRITICAL(&wxMux);
  weatherNow = w;
  portEXIT_CRITICAL(&wxMux);
  Serial.printf("[weather] %.1f C, code %u, %s\n", w.tempC, w.code, w.isDay ? "day" : "night");
  return true;
}

// ===================== WIFI: SAVED NETWORKS ======================
// Stored in their own flash namespace, separate from the settings.

static int findKnownLocked(const char *ssid) {
  for (int i = 0; i < knownCount; i++) if (!strcmp(knownNets[i].ssid, ssid)) return i;
  return -1;
}

// Index of the oldest network that may be dropped (never the default)
static int dropIndexLocked() {
  for (int i = knownCount - 1; i >= 0; i--)
    if (strcmp(knownNets[i].ssid, DEFAULT_WIFI_SSID)) return i;
  return -1;
}

static void removeKnownLocked(int i) {
  for (int j = i; j < knownCount - 1; j++) knownNets[j] = knownNets[j + 1];
  knownCount--;
}

void loadKnownNets() {
  Preferences p;
  p.begin("wifi", true);
  knownCount = min((int)p.getUChar("n", 0), MAX_KNOWN_NETS);
  for (int i = 0; i < knownCount; i++) {
    char k[4];
    snprintf(k, sizeof(k), "s%d", i); p.getString(k, knownNets[i].ssid, sizeof(knownNets[i].ssid));
    snprintf(k, sizeof(k), "p%d", i); p.getString(k, knownNets[i].pass, sizeof(knownNets[i].pass));
  }
  p.end();
}

void saveKnownNets() {
  KnownNet copy[MAX_KNOWN_NETS];
  xSemaphoreTake(wifiMutex, portMAX_DELAY);
  uint8_t n = knownCount;
  memcpy(copy, knownNets, sizeof(copy));
  xSemaphoreGive(wifiMutex);

  Preferences p;
  p.begin("wifi", false);
  p.putUChar("n", n);
  for (int i = 0; i < n; i++) {
    char k[4];
    snprintf(k, sizeof(k), "s%d", i); p.putString(k, copy[i].ssid);
    snprintf(k, sizeof(k), "p%d", i); p.putString(k, copy[i].pass);
  }
  p.end();
}

// Moves a network to the front (adding it if new). Called on every
// successful connection, so the list stays in most-recently-used order.
void rememberNetwork(const char *ssid, const char *pass) {
  if (!ssid || !ssid[0]) return;
  bool changed = true;
  xSemaphoreTake(wifiMutex, portMAX_DELAY);
  int i = findKnownLocked(ssid);
  if (i == 0 && !strcmp(knownNets[0].pass, pass)) {
    changed = false;                                  // Already first: no flash write
  } else {
    KnownNet n;
    strlcpy(n.ssid, ssid, sizeof(n.ssid));
    strlcpy(n.pass, pass, sizeof(n.pass));
    if (i >= 0) removeKnownLocked(i);
    else if (knownCount == MAX_KNOWN_NETS) { int d = dropIndexLocked(); if (d >= 0) removeKnownLocked(d); }
    if (knownCount < MAX_KNOWN_NETS) {
      for (int j = knownCount; j > 0; j--) knownNets[j] = knownNets[j - 1];
      knownNets[0] = n;
      knownCount++;
    }
  }
  xSemaphoreGive(wifiMutex);
  if (changed) saveKnownNets();
}

// Adds a network at the end of the list without connecting (dashboard).
void addNetwork(const char *ssid, const char *pass) {
  if (!ssid || !ssid[0]) return;
  xSemaphoreTake(wifiMutex, portMAX_DELAY);
  int i = findKnownLocked(ssid);
  if (i >= 0) {
    strlcpy(knownNets[i].pass, pass, sizeof(knownNets[i].pass));
  } else {
    if (knownCount == MAX_KNOWN_NETS) { int d = dropIndexLocked(); if (d >= 0) removeKnownLocked(d); }
    if (knownCount < MAX_KNOWN_NETS) {
      strlcpy(knownNets[knownCount].ssid, ssid, sizeof(knownNets[0].ssid));
      strlcpy(knownNets[knownCount].pass, pass, sizeof(knownNets[0].pass));
      knownCount++;
    }
  }
  xSemaphoreGive(wifiMutex);
  saveKnownNets();
}

bool forgetNetwork(const char *ssid) {
  if (!strcmp(ssid, DEFAULT_WIFI_SSID)) return false;   // The default always stays
  xSemaphoreTake(wifiMutex, portMAX_DELAY);
  int i = findKnownLocked(ssid);
  if (i >= 0) removeKnownLocked(i);
  xSemaphoreGive(wifiMutex);
  if (i >= 0) saveKnownNets();
  return i >= 0;
}

// The default network from the code is always in the list, with the
// password from the code.
void ensureDefaultNet() {
  if (!DEFAULT_WIFI_SSID[0]) return;
  xSemaphoreTake(wifiMutex, portMAX_DELAY);
  int i = findKnownLocked(DEFAULT_WIFI_SSID);
  bool changed = false;
  if (i < 0) {
    if (knownCount == MAX_KNOWN_NETS) removeKnownLocked(knownCount - 1);
    for (int j = knownCount; j > 0; j--) knownNets[j] = knownNets[j - 1];
    strlcpy(knownNets[0].ssid, DEFAULT_WIFI_SSID, sizeof(knownNets[0].ssid));
    strlcpy(knownNets[0].pass, DEFAULT_WIFI_PASS, sizeof(knownNets[0].pass));
    knownCount++;
    changed = true;
  } else if (strcmp(knownNets[i].pass, DEFAULT_WIFI_PASS)) {
    strlcpy(knownNets[i].pass, DEFAULT_WIFI_PASS, sizeof(knownNets[i].pass));
    changed = true;
  }
  xSemaphoreGive(wifiMutex);
  if (changed) saveKnownNets();
}

// One-time import of the network the old setup page (WiFiManager) saved.
void importLegacyNetwork() {
  Preferences p;
  p.begin("wifi", false);
  bool done = p.getBool("imported", false);
  if (!done) p.putBool("imported", true);
  p.end();
  if (done) return;

  wifi_config_t conf;
  if (esp_wifi_get_config(WIFI_IF_STA, &conf) == ESP_OK && conf.sta.ssid[0]) {
    char ssid[33], pass[65];
    memcpy(ssid, conf.sta.ssid, 32);     ssid[32] = 0;
    memcpy(pass, conf.sta.password, 64); pass[64] = 0;
    addNetwork(ssid, pass);
    Serial.printf("[wifi] imported saved network \"%s\"\n", ssid);
  }
}

// ===================== WIFI: CONNECTION MANAGER ====================

uint32_t apStopAt      = 0;     // Close the setup hotspot at this time (0 = not scheduled)
bool     everConnected = false; // First connection: start mDNS and the clock

void startSetupAP() {
  if (apActive) return;
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP(SETUP_AP_NAME);
  apActive = true;                // webTask starts the DNS redirect for the setup page
  apStopAt = 0;
  Serial.printf("[wifi] setup network \"%s\" open (192.168.4.1)\n", SETUP_AP_NAME);
}

void stopSetupAP() {
  if (!apActive) return;
  apActive = false;
  vTaskDelay(pdMS_TO_TICKS(50));  // Let webTask stop the DNS server first
  WiFi.softAPdisconnect(true);
  WiFi.mode(WIFI_STA);
  Serial.println("[wifi] setup network closed");
}

// Scans and keeps the strongest 16 (deduplicated) for the setup page.
// Leaves the raw results in place for the caller; call WiFi.scanDelete().
int runScan() {
  int n = WiFi.scanNetworks();
  if (n < 0) n = 0;
  xSemaphoreTake(wifiMutex, portMAX_DELAY);
  scanCount = 0;
  for (int i = 0; i < n; i++) {
    String name = WiFi.SSID(i);
    if (!name.length()) continue;
    int8_t r = WiFi.RSSI(i);
    int j;
    for (j = 0; j < scanCount; j++) if (name == scanList[j].ssid) break;
    if (j < scanCount) { if (r > scanList[j].rssi) scanList[j].rssi = r; continue; }
    if (scanCount == MAX_SCAN) {                         // Full: replace the weakest if stronger
      int w = 0;
      for (int k = 1; k < scanCount; k++) if (scanList[k].rssi < scanList[w].rssi) w = k;
      if (r <= scanList[w].rssi) continue;
      j = w;
    } else {
      j = scanCount++;
    }
    strlcpy(scanList[j].ssid, name.c_str(), sizeof(scanList[j].ssid));
    scanList[j].rssi = r;
    scanList[j].open = WiFi.encryptionType(i) == WIFI_AUTH_OPEN;
  }
  for (int a = 1; a < scanCount; a++) {                  // Strongest first
    ScanEntry e = scanList[a];
    int b = a - 1;
    while (b >= 0 && scanList[b].rssi < e.rssi) { scanList[b + 1] = scanList[b]; b--; }
    scanList[b + 1] = e;
  }
  scanAt = millis();
  xSemaphoreGive(wifiMutex);
  return n;
}

bool tryConnect(const char *ssid, const char *pass, uint32_t timeoutMs, bool isJoin) {
  Serial.printf("[wifi] trying \"%s\"\n", ssid);
  WiFi.disconnect(false, false);
  vTaskDelay(pdMS_TO_TICKS(50));
  WiFi.begin(ssid, pass[0] ? pass : nullptr);
  uint32_t start = millis();
  while (millis() - start < timeoutMs) {
    if (WiFi.status() == WL_CONNECTED) return true;
    if (!isJoin && joinPending) break;       // A network was just entered on the setup page
    vTaskDelay(pdMS_TO_TICKS(100));
  }
  WiFi.disconnect(false, false);
  return false;
}

// One pass: scan, then try each saved network that's in range, most
// recently used first.
bool connectKnownOnce() {
  int n = runScan();
  KnownNet cand[MAX_KNOWN_NETS];
  int nc = 0;
  xSemaphoreTake(wifiMutex, portMAX_DELAY);
  for (int k = 0; k < knownCount; k++)
    for (int i = 0; i < n; i++)
      if (WiFi.SSID(i) == knownNets[k].ssid) { cand[nc++] = knownNets[k]; break; }
  xSemaphoreGive(wifiMutex);
  WiFi.scanDelete();

  if (!nc) {
    setNetLine("SEARCHING");
    return false;
  }
  for (int i = 0; i < nc; i++) {
    setNetLine(cand[i].ssid);
    if (tryConnect(cand[i].ssid, cand[i].pass, 10000, false)) return true;
    if (joinPending) return false;
  }
  return false;
}

// Blocks until connected. boot = first connection after power-up.
void connectLoop(bool boot, bool forcePortal) {
  uint32_t start    = millis();
  uint32_t nextPass = 0;

  if (boot) {
    startSetupAP();                          // Setup network visible from the first second
    setNetView(forcePortal ? NV_SETUP : NV_CONNECTING);
    setNetLine("SEARCHING");
  } else {
    setNetLine("RECONNECTING");
    WiFi.reconnect();                        // The driver's own quick retry first
  }

  while (WiFi.status() != WL_CONNECTED) {
    uint32_t el = millis() - start;
    inSetupPortal = apActive;

    // A network entered on the setup page is tried straight away
    if (joinPending) {
      char js[33], jp[65];
      xSemaphoreTake(wifiMutex, portMAX_DELAY);
      strlcpy(js, joinSsid, sizeof(js));
      strlcpy(jp, joinPass, sizeof(jp));
      joinPending = false;
      xSemaphoreGive(wifiMutex);
      joinResult = JOIN_TRYING;
      setNetLine(js);
      if (tryConnect(js, jp, 15000, true)) { joinResult = JOIN_OK; break; }
      joinResult = JOIN_FAILED;
      continue;
    }

    if (boot) {
      if (forcePortal || el >= BOOT_CONNECT_WINDOW_MS) setNetView(NV_SETUP);
    } else {
      if (el > 2000) setNetView(NV_LOST);    // Ignore short blips
      if (el > LOST_SETUP_AFTER_MS && !apActive) startSetupAP();
      if (apActive) setNetLine("JOIN SpotifyDisc-Setup");
    }

    // Saved networks: back to back in the boot window, then every 30 s.
    // Not while someone is on the setup page: a scan would briefly cut
    // their connection. After a drop, the driver gets 20 s on its own.
    bool inWindow     = boot && !forcePortal && el < BOOT_CONNECT_WINDOW_MS;
    bool nobodyOnPage = !apActive || WiFi.softAPgetStationNum() == 0;
    bool driverTurn   = !boot && el < 20000;
    bool portalOnly   = forcePortal && el < 300000;       // Setup was asked for: 5 min to use it
    if (!driverTurn && !portalOnly && (inWindow || nobodyOnPage) && (int32_t)(millis() - nextPass) >= 0) {
      if (connectKnownOnce()) break;
      nextPass = millis() + (inWindow ? 500 : 30000);
    }

    if (scanRequested) {                     // Setup page wants a fresh list
      scanRequested = false;
      runScan();
      WiFi.scanDelete();
    }
    vTaskDelay(pdMS_TO_TICKS(200));
  }
}

void onWifiConnected(bool showGreen) {
  String ssid = WiFi.SSID();
  rememberNetwork(ssid.c_str(), WiFi.psk().c_str());
  int rssi = WiFi.RSSI();
  netBars = rssi > -60 ? 3 : rssi > -72 ? 2 : 1;
  inSetupPortal = false;
  Serial.printf("[wifi] connected to \"%s\" (%d dBm), IP %s\n",
                ssid.c_str(), rssi, WiFi.localIP().toString().c_str());

  if (showGreen) {
    setNetLine(ssid.c_str());
    setNetView(NV_CONNECTED);
  } else {
    setNetView(NV_NONE);
  }

  // Close the setup network. If a phone was just using it, give its page
  // a few seconds to show the result first.
  if (apActive) apStopAt = millis() + (WiFi.softAPgetStationNum() > 0 ? 15000 : 1);

  if (!everConnected) {
    everConnected = true;
    configTzTime(TIME_ZONE, "pool.ntp.org", "time.google.com");   // Clock for home mode
    if (MDNS.begin(HOSTNAME)) {
      MDNS.addService("http", "tcp", 80);
      Serial.printf("[web] http://%s.local\n", HOSTNAME);
    }
    Serial.printf("[mem] after connecting: free heap %u, largest block %u\n",
                  ESP.getFreeHeap(), heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
  }
}

// ==================== NETWORK TASK (CORE 0) =====================

void networkTask(void *param) {
  bool forcePortal = param != nullptr;

  connectLoop(true, forcePortal);
  onWifiConnected(true);

  for (;;) {
    if (WiFi.status() != WL_CONNECTED) {
      netOk = false;
      Serial.println("[wifi] connection lost");
      connectLoop(false, false);
      onWifiConnected(true);
      continue;
    }

    if (apStopAt && (int32_t)(millis() - apStopAt) >= 0) {
      apStopAt = 0;
      stopSetupAP();
    }

    pollSpotify();

    // Spotify keeps being polled in home mode too: holding the button
    // only returns to music when something is actually playing.
    if (cfgHome && (weatherFetchedAt == 0 || millis() - weatherFetchedAt > WEATHER_REFRESH_MS)) {
      if (fetchWeather()) weatherFetchedAt = millis();
      else weatherFetchedAt = millis() - WEATHER_REFRESH_MS + 60000;   // Retry in a minute
    }

    ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(POLL_INTERVAL_MS));   // Dashboard can wake it early
  }
}

// ======================== DASHBOARD PAGE ========================

const char INDEX_HTML[] PROGMEM = R"rawliteral(<!DOCTYPE html>
<html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,viewport-fit=cover">
<meta name="apple-mobile-web-app-capable" content="yes">
<meta name="apple-mobile-web-app-title" content="Disc">
<title>Spotify Disc</title>
<style>
:root{--bg:#0f0f12;--card:#1a1a1f;--line:#2a2a31;--text:#ececf1;--muted:#8b8b96;--ok:#3ddc84;--warn:#f5a524;--bad:#f25f5c}
@media (prefers-color-scheme:light){:root{--bg:#f4f4f6;--card:#fff;--line:#e3e3e8;--text:#15151a;--muted:#6b6b76;--ok:#12a150}}
*{box-sizing:border-box}
body{margin:0;background:var(--bg);color:var(--text);font:15px/1.45 -apple-system,system-ui,sans-serif;
padding:calc(16px + env(safe-area-inset-top)) 16px calc(24px + env(safe-area-inset-bottom))}
main{max-width:480px;margin:0 auto}
h1{font-size:22px;margin:4px 0 16px;display:flex;align-items:center;gap:10px}
.dot{width:10px;height:10px;border-radius:50%;background:var(--muted);transition:background .3s}
.card{background:var(--card);border:1px solid var(--line);border-radius:14px;padding:14px 16px;margin-bottom:12px}
h2{font-size:12px;letter-spacing:.08em;text-transform:uppercase;color:var(--muted);margin:0 0 8px;font-weight:600}
.row{display:flex;justify-content:space-between;gap:12px;padding:8px 0;border-top:1px solid var(--line)}
h2+.row{border-top:0}
.row>span:first-child{color:var(--muted);flex-shrink:0}
.row>span:last-child{text-align:right;overflow:hidden;text-overflow:ellipsis;white-space:nowrap}
.cover-wrap{position:relative;width:100%;max-width:260px;aspect-ratio:1;margin:4px auto 14px;border-radius:10px;overflow:hidden;background:var(--line)}
.cover{width:100%;height:100%;display:block;image-rendering:crisp-edges;image-rendering:pixelated;opacity:0;transition:opacity .3s}
.cover.on{opacity:1}
.cover-empty{position:absolute;inset:0;display:flex;align-items:center;justify-content:center;color:var(--muted);font-size:13px}
.cover-note{text-align:center;color:var(--muted);font-size:12px;margin:-8px 0 12px}
.lyric{font-size:15px;margin:12px 0 2px}
.lyric-next{color:var(--muted);font-size:13px;margin:0}
.lyric-state{color:var(--muted);font-size:12px;margin:8px 0 0}
.seg2{display:grid;grid-template-columns:repeat(3,1fr);gap:8px;margin-bottom:4px}
.remote{display:flex;align-items:center;gap:16px;margin-bottom:12px}
.card input:not([type]),.card input[type=password]{width:100%;box-sizing:border-box;margin:4px 0;padding:10px;border-radius:10px;border:1px solid var(--line);background:var(--card);color:var(--text);font:inherit}
button.mini{flex:none;padding:4px 12px;font-size:13px;border-radius:8px}
.tag{color:var(--muted);font-size:12px;margin-left:6px}
.remote-btn{flex:none;width:84px;height:84px;border-radius:50%;border:2px solid var(--line);background:var(--card);color:var(--text);font-size:30px;padding:0;touch-action:manipulation;-webkit-user-select:none;user-select:none;-webkit-touch-callout:none;transition:transform .12s,border-color .12s}
.remote-btn:active{transform:scale(.95)}
.remote-btn.holding{border-color:var(--ok);transition:transform .12s,border-color .7s}
.remote-mode{font-size:20px;font-weight:600}
.remote-hint{color:var(--muted);font-size:13px;margin-top:2px}
.remote-msg{color:var(--warn);font-size:13px;margin-top:6px;min-height:1em}
.track{font-size:18px;font-weight:600;margin:0}
.artist{color:var(--muted);margin:2px 0 10px}
.pill{font-size:12px;padding:3px 9px;border-radius:99px;background:var(--line)}
.ok{color:var(--ok)}.warn{color:var(--warn)}.bad{color:var(--bad)}
label{display:flex;justify-content:space-between;margin:12px 0 4px}
label span{color:var(--muted)}
input[type=range]{width:100%;accent-color:var(--ok)}
.sw{display:flex;justify-content:space-between;align-items:center;padding:10px 0;border-top:1px solid var(--line)}
.sw input{width:22px;height:22px;accent-color:var(--ok)}
.btns{display:flex;gap:8px;margin-top:12px}
button{flex:1;padding:12px;border-radius:10px;border:1px solid var(--line);background:transparent;color:var(--text);font:inherit}
button.danger{color:var(--bad);border-color:var(--bad)}
.seg{display:grid;grid-template-columns:repeat(4,1fr);gap:8px}
.seg button{padding:10px 4px}
button.active{border-color:var(--ok);color:var(--ok)}
.hint{color:var(--muted);font-size:13px;margin:0 0 10px}
select{width:100%;padding:10px;border-radius:10px;border:1px solid var(--line);background:var(--card);color:var(--text);font:inherit}
</style></head><body><main>
<h1><span class="dot" id="dot"></span>Spotify Disc</h1>

<div class="card remote">
<button class="remote-btn" id="remoteBtn" aria-label="Remote button"><span id="remoteIcon">&#9679;</span></button>
<div><div class="remote-mode" id="remoteMode">&hellip;</div>
<div class="remote-hint">Tap: next view &middot; Hold: home / music</div>
<div class="remote-msg" id="remoteMsg"></div></div>
</div>

<div class="card"><h2>Now playing</h2>
<div class="cover-wrap"><canvas class="cover" id="cover" width="64" height="64"></canvas>
<div class="cover-empty" id="coverEmpty">No cover loaded</div></div>
<p class="cover-note">64 &times; 64, as loaded on the panel</p>
<p class="track" id="track">&mdash;</p><p class="artist" id="artist"></p>
<p class="lyric" id="lyric"></p><p class="lyric-next" id="lyricNext"></p><p class="lyric-state" id="lyricState"></p>
<span class="pill" id="state">loading</span></div>

<div class="card"><h2>Saved networks</h2>
<p class="hint">Tried at startup in this order, most recently used first. The default network from the code always stays.</p>
<div id="known"></div>
<label>Add a network</label>
<input id="addSsid" placeholder="Network name" autocapitalize="off" autocorrect="off" spellcheck="false">
<input id="addPass" type="password" placeholder="Password">
<div class="btns"><button id="addNet">Add</button></div></div>

<div class="card"><h2>Connection</h2>
<div class="row"><span>WiFi</span><span id="wifi">&hellip;</span></div>
<div class="row"><span>Signal</span><span id="rssi">&hellip;</span></div>
<div class="row"><span>IP address</span><span id="ip">&hellip;</span></div>
<div class="row"><span>Spotify</span><span id="sp">&hellip;</span></div>
<div class="row"><span>Last update</span><span id="ago">&hellip;</span></div></div>

<div class="card"><h2>Display</h2>
<label>Mode</label>
<div class="seg2"><button data-mode="1">Lyrics</button><button data-mode="0">Disc</button><button data-mode="2">Cover</button></div>
<label>Brightness <span id="bv"></span></label><input type="range" id="brightness" min="5" max="255">
<label>Spin speed <span id="sv"></span></label><input type="range" id="spin" min="0" max="180" step="5">
<label>Lyrics timing <span id="lv"></span></label><input type="range" id="lyroff" min="-3000" max="3000" step="100">
<div class="sw" style="margin-top:12px"><span>Pause spin</span><input type="checkbox" id="paused"></div>
<div class="sw"><span>Spindle hole</span><input type="checkbox" id="spindle"></div>
</div>

<div class="card"><h2>Image</h2>
<p class="hint">Applied to each cover as it loads. Changes reload the current cover.</p>
<label>Black level <span id="kv"></span></label><input type="range" id="blk" min="0" max="60">
<label>Contrast <span id="gv"></span></label><input type="range" id="gam" min="100" max="180" step="5">
<label>Saturation <span id="tv"></span></label><input type="range" id="sat" min="80" max="180" step="5">
<label>Shadow cleanup <span id="hv"></span></label><input type="range" id="shd" min="0" max="100" step="5">
<div class="btns"><button id="imgReset">Reset to defaults</button></div></div>

<div class="card"><h2>White balance</h2>
<p class="hint">Corrects the LED colours. Turn on Panel test &rarr; Greys, then lower whichever channel is too strong until every band looks neutral grey. Applies instantly.</p>
<label>Red <span id="wrv"></span></label><input type="range" id="wbr" min="40" max="100">
<label>Green <span id="wgv"></span></label><input type="range" id="wbg" min="40" max="100">
<label>Blue <span id="wbv"></span></label><input type="range" id="wbb" min="40" max="100">
<div class="btns"><button id="wbReset">Reset to defaults</button></div></div>

<div class="card"><h2>Panel test</h2>
<p class="hint">Top half uses the R1/G1/B1 wires, bottom half R2/G2/B2.</p>
<div class="seg">
<button data-test="0">Off</button><button data-test="1">White</button><button data-test="6">Stripes</button><button data-test="5">Bars</button>
<button data-test="2">Red</button><button data-test="3">Green</button><button data-test="4">Blue</button><button data-test="7">Greys</button>
</div></div>

<div class="card"><h2>Panel signal</h2>
<p class="hint">For false colours, ghosting or a shifted image. Apply restarts the display. Now running: <span id="sigNow">&hellip;</span></p>
<label>Clock phase</label>
<select id="clkph"><option value="1">Normal</option><option value="0">Alternate</option></select>
<label>Latch blanking</label>
<select id="latch"><option value="0">Library default</option><option value="1">1</option>
<option value="2">2</option><option value="3">3</option><option value="4">4 (most)</option></select>
<div class="btns"><button id="applySig">Apply &amp; restart</button></div></div>

<div class="card"><h2>Device</h2>
<div class="row"><span>Uptime</span><span id="up">&hellip;</span></div>
<div class="row"><span>Free memory</span><span id="heap">&hellip;</span></div>
<div class="btns"><button id="restart">Restart</button><button id="wsetup">WiFi setup</button></div></div>
</main>
<script>
const $=id=>document.getElementById(id);
let loaded=false;const timers={};let coverVer=-1,coverBusy=false;
async function loadCover(ver){
 if(coverBusy)return;coverBusy=true;
 try{
  const r=await fetch('/api/cover?v='+ver);
  if(r.status!==200)return;
  const buf=await r.arrayBuffer();if(buf.byteLength!==8192)return;
  const dv=new DataView(buf),ctx=$('cover').getContext('2d'),img=ctx.createImageData(64,64),px=img.data;
  for(let i=0;i<4096;i++){
   const v=dv.getUint16(i*2,true);
   px[i*4]  =((v>>11)&31)*255/31;
   px[i*4+1]=((v>>5)&63)*255/63;
   px[i*4+2]=(v&31)*255/31;
   px[i*4+3]=255;
  }
  ctx.putImageData(img,0,0);coverVer=ver;
  $('cover').classList.add('on');$('coverEmpty').style.display='none';
 }catch(e){}finally{coverBusy=false;}
}
function clearCover(){
 coverVer=-1;$('cover').classList.remove('on');$('coverEmpty').style.display='';
}
function send(p){fetch('/api/settings',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:new URLSearchParams(p)});}
function debounced(k,v){clearTimeout(timers[k]);timers[k]=setTimeout(()=>send({[k]:v}),120);}
function set(el,t,c){el.textContent=t;el.className=c||'';}
function fmtUp(s){const d=Math.floor(s/86400),h=Math.floor(s%86400/3600),m=Math.floor(s%3600/60);return(d?d+'d ':'')+(h?h+'h ':'')+m+'m';}
function sig(r){return r>=-55?['Excellent','ok']:r>=-67?['Good','ok']:r>=-75?['Fair','warn']:['Weak','bad'];}
function labels(){$('bv').textContent=Math.round($('brightness').value/2.55)+'%';$('sv').textContent=$('spin').value+'°/s';const o=+$('lyroff').value;$('lv').textContent=o===0?'in sync':Math.abs(o/1000).toFixed(1)+' s '+(o>0?'earlier':'later');$('kv').textContent=$('blk').value==='0'?'off':$('blk').value;$('gv').textContent=$('gam').value==='100'?'off':'+'+($('gam').value-100)+'%';$('tv').textContent=$('sat').value+'%';$('hv').textContent=$('shd').value==='0'?'off':$('shd').value+'%';$('wrv').textContent=$('wbr').value+'%';$('wgv').textContent=$('wbg').value+'%';$('wbv').textContent=$('wbb').value+'%';}
function markMode(v){document.querySelectorAll('[data-mode]').forEach(b=>b.classList.toggle('active',b.dataset.mode==String(v)));}
async function poll(){
 try{
  const d=await(await fetch('/api/status')).json(),w=d.wifi,s=d.spotify;
  set($('wifi'),w.connected?w.ssid:'Disconnected',w.connected?'ok':'bad');
  const[q,c]=sig(w.rssi);set($('rssi'),q+' ('+w.rssi+' dBm)',c);
  $('ip').textContent=w.ip;
  const spOk=s.reachable&&s.tokenOk;
  set($('sp'),!s.reachable?'Unreachable':s.tokenOk?'Connected':'Auth error',spOk?'ok':'bad');
  $('ago').textContent=s.lastUpdateAgo<0?'never':s.lastUpdateAgo+'s ago';
  $('track').textContent=s.track||'Nothing playing';$('artist').textContent=s.artist||'';
  $('state').textContent=s.state;
  showRemote(d.settings.mode,d.settings.home);
  const L=d.lyrics||{state:'none'};
  const stTxt={loading:'Loading lyrics…',ready:'Synced lyrics · '+(L.lines||0)+' lines',notfound:'No synced lyrics for this song, showing the disc',instrumental:'Instrumental, showing the disc',error:'Lyrics unavailable right now'}[L.state]||'';
  $('lyric').textContent=L.state==='ready'?(L.current||'♪'):'';
  $('lyricNext').textContent=L.state==='ready'?(L.next||''):'';
  $('lyricState').textContent=stTxt;
  if(s.hasCover){if(s.coverVersion!==coverVer)loadCover(s.coverVersion);}else if(coverVer!==-1)clearCover();
  $('dot').style.background=(w.connected&&spOk)?'var(--ok)':'var(--bad)';
  $('up').textContent=fmtUp(d.device.uptime);
  $('heap').textContent=Math.round(d.device.freeHeap/1024)+' KB (low '+Math.round(d.device.minFreeHeap/1024)+' KB)';
  if(!loaded){const c=d.settings;$('brightness').value=c.brightness;$('spin').value=c.spinSpeed;
   $('paused').checked=c.paused;$('spindle').checked=c.spindle;markTest(c.test);markMode(c.mode);$('lyroff').value=c.lyricOffset;$('blk').value=c.blk;$('gam').value=c.gam;$('sat').value=c.sat;$('shd').value=c.shd;$('wbr').value=c.wbr;$('wbg').value=c.wbg;$('wbb').value=c.wbb;if(c.wbDef)wbDef=c.wbDef;$('clkph').value=c.clkPhase?1:0;$('latch').value=c.latch;labels();loaded=true;}
  const hz=d.device.clkHz;$('sigNow').textContent=(hz>=1e6?(hz/1e6)+' MHz':'clock '+hz)+', '+(d.settings.clkPhase?'normal':'alternate')+' phase, blanking '+d.device.latch;
 }catch(e){$('dot').style.background='var(--bad)';set($('sp'),'Device unreachable','bad');}
}
$('brightness').oninput=e=>{labels();debounced('brightness',e.target.value);};
$('spin').oninput=e=>{labels();debounced('spin',e.target.value);};
$('lyroff').oninput=e=>{labels();debounced('lyroff',e.target.value);};
['blk','gam','sat','shd'].forEach(id=>{$(id).oninput=labels;$(id).onchange=e=>send({[id]:e.target.value});});
$('imgReset').onclick=()=>{$('blk').value=20;$('gam').value=110;$('sat').value=100;$('shd').value=60;labels();send({blk:20,gam:110,sat:100,shd:60});};
['wbr','wbg','wbb'].forEach(id=>{$(id).oninput=e=>{labels();debounced(id,e.target.value);};});
let wbDef=[100,88,78];
$('wbReset').onclick=()=>{const[r,g,b]=wbDef;$('wbr').value=r;$('wbg').value=g;$('wbb').value=b;labels();send({wbr:r,wbg:g,wbb:b});};
const VIEW_NAMES=['Disc','Lyrics','Cover'],VIEW_ICONS=['◎','♫','▣'];
function showRemote(mode,home){$('remoteMode').textContent=home?'Home':VIEW_NAMES[mode]||'';$('remoteIcon').textContent=home?'⌂':VIEW_ICONS[mode]||'';markMode(home?-1:mode);}
let msgT=null;function remoteMsg(t){$('remoteMsg').textContent=t||'';clearTimeout(msgT);if(t)msgT=setTimeout(()=>{$('remoteMsg').textContent='';},4000);}
function press(kind){fetch('/api/button',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:new URLSearchParams({press:kind})})
 .then(r=>r.json()).then(j=>{showRemote(j.mode,j.home);remoteMsg(j.message);}).catch(()=>remoteMsg('Display not reachable'));}
(()=>{const b=$('remoteBtn');let t=null,done=true;
 b.addEventListener('pointerdown',e=>{e.preventDefault();done=false;b.classList.add('holding');
  t=setTimeout(()=>{t=null;done=true;b.classList.remove('holding');if(navigator.vibrate)navigator.vibrate(25);press('long');},700);});
 const end=e=>{if(t){clearTimeout(t);t=null;}b.classList.remove('holding');const tap=!done&&e.type==='pointerup';done=true;if(tap)press('short');};
 ['pointerup','pointercancel','pointerleave'].forEach(ev=>b.addEventListener(ev,end));
 b.addEventListener('contextmenu',e=>e.preventDefault());})();
document.querySelectorAll('[data-mode]').forEach(b=>b.onclick=()=>{send({mode:b.dataset.mode});markMode(b.dataset.mode);});
function markTest(v){document.querySelectorAll('[data-test]').forEach(b=>b.classList.toggle('active',b.dataset.test==String(v)));}
document.querySelectorAll('[data-test]').forEach(b=>b.onclick=()=>{send({test:b.dataset.test});markTest(b.dataset.test);});
['paused','spindle'].forEach(id=>$(id).onchange=e=>send({[id]:e.target.checked?1:0}));
$('applySig').onclick=()=>{
 fetch('/api/signal',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},
  body:new URLSearchParams({phase:$('clkph').value,latch:$('latch').value})});
 $('applySig').textContent='Restarting…';loaded=false;
 setTimeout(()=>{$('applySig').textContent='Apply & restart';},9000);
};
$('restart').onclick=()=>{if(confirm('Restart the display?'))fetch('/api/restart',{method:'POST'});};
$('wsetup').onclick=()=>{if(confirm('Reboot into WiFi setup? Join "SpotifyDisc-Setup" on your phone afterwards.'))fetch('/api/wifi-setup',{method:'POST'});};
function renderKnown(j){const box=$('known');box.innerHTML='';
 j.networks.forEach(n=>{const r=document.createElement('div');r.className='row';
  const a=document.createElement('span');a.textContent=n.ssid;
  if(n.current){const t=document.createElement('span');t.className='tag ok';t.textContent='connected';a.appendChild(t);}
  if(n.default){const t=document.createElement('span');t.className='tag';t.textContent='default';a.appendChild(t);}
  const b=document.createElement('span');
  if(!n.default){const x=document.createElement('button');x.className='mini';x.textContent='Forget';
   x.onclick=()=>{if(confirm('Forget '+n.ssid+'?'))fetch('/api/wifi/forget',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:new URLSearchParams({ssid:n.ssid})}).then(r=>r.json()).then(renderKnown);};
   b.appendChild(x);}
  r.append(a,b);box.appendChild(r);});}
function loadKnown(){fetch('/api/wifi/known').then(r=>r.json()).then(renderKnown).catch(()=>{});}
$('addNet').onclick=()=>{const s=$('addSsid').value.trim();if(!s)return;
 fetch('/api/wifi/add',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:new URLSearchParams({ssid:s,pass:$('addPass').value})})
  .then(r=>r.json()).then(j=>{renderKnown(j);$('addSsid').value='';$('addPass').value='';});};
loadKnown();
poll();setInterval(poll,2000);
</script></body></html>)rawliteral";

// ======================== WEB HANDLERS ==========================

// ========================= SETUP PAGE =========================
// Served on the "SpotifyDisc-Setup" network (192.168.4.1). Phones open it
// automatically, because every web address there leads to it.

const char SETUP_HTML[] PROGMEM = R"rawliteral(<!DOCTYPE html>
<html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Spotify Disc WiFi</title>
<style>
body{margin:0;padding:20px;background:#0f0f12;color:#ececf1;font:16px/1.45 -apple-system,system-ui,sans-serif}
main{max-width:420px;margin:0 auto}h1{font-size:22px;margin:4px 0 6px}
.muted{color:#8b8b96;font-size:14px}
.net{display:flex;justify-content:space-between;gap:10px;padding:12px 14px;margin:6px 0;border:1px solid #2a2a31;border-radius:12px;background:#1a1a1f}
.net.sel{border-color:#3ddc84}
input{width:100%;box-sizing:border-box;margin:6px 0;padding:12px;border-radius:10px;border:1px solid #2a2a31;background:#1a1a1f;color:#ececf1;font:inherit}
button{width:100%;margin-top:8px;padding:13px;border:0;border-radius:10px;background:#3ddc84;color:#08120c;font:inherit;font-weight:600}
#msg{margin-top:14px}.ok{color:#3ddc84}.bad{color:#f25f5c}
</style></head><body><main>
<h1>Connect the display</h1>
<p class="muted">Pick a network and enter its password. The display remembers it and finds it again on its own.</p>
<div id="list" class="muted">Scanning&hellip;</div>
<input id="ssid" placeholder="Network name" autocapitalize="off" autocorrect="off" spellcheck="false">
<input id="pass" type="password" placeholder="Password (leave empty for open networks)">
<button id="go">Connect</button>
<div id="msg" class="muted"></div>
</main>
<script>
const $=id=>document.getElementById(id);let chosen='';
const bars=r=>r>-60?'▂▄▆█':r>-70?'▂▄▆':r>-80?'▂▄':'▂';
async function scan(){try{
 const j=await(await fetch('/api/wifi/scan')).json(),L=$('list');
 if(!j.networks.length){L.textContent='Scanning…';return;}
 L.innerHTML='';
 j.networks.forEach(n=>{const d=document.createElement('div');d.className='net'+(n.ssid===chosen?' sel':'');
  const a=document.createElement('span');a.textContent=n.ssid+(n.known?'  ✓':'');
  const b=document.createElement('span');b.className='muted';b.textContent=(n.open?'open  ':'')+bars(n.rssi);
  d.append(a,b);d.onclick=()=>{chosen=n.ssid;$('ssid').value=n.ssid;$('pass').focus();scan();};L.appendChild(d);});
}catch(e){}}
scan();setInterval(scan,6000);
$('go').onclick=()=>{const s=$('ssid').value.trim();if(!s)return;
 fetch('/api/wifi/join',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:new URLSearchParams({ssid:s,pass:$('pass').value})});
 $('msg').className='muted';$('msg').textContent='Connecting to '+s+'…';watch();};
function watch(){const iv=setInterval(async()=>{try{
 const j=await(await fetch('/api/wifi/state')).json();
 if(j.join==='ok'){clearInterval(iv);$('msg').className='ok';
  $('msg').textContent='Connected to '+j.ssid+'. Switch your phone back to your usual WiFi and open http://spotify-disc.local (or http://'+j.ip+').';}
 else if(j.join==='failed'){clearInterval(iv);$('msg').className='bad';
  $('msg').textContent='Could not connect to '+j.joinSsid+'. Check the password and try again.';}
}catch(e){$('msg').className='muted';
 $('msg').textContent='Still connecting… If this page stops updating, the display has probably connected and closed this setup network.';}},1500);}
</script></body></html>)rawliteral";

bool onSetupNetwork() {
  return apActive && server.client().localIP() == WiFi.softAPIP();
}

void handleSetupPage() {
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "text/html", SETUP_HTML);
}

void handleWifiScan() {
  if (!scanCount || millis() - scanAt > 15000) scanRequested = true;
  DynamicJsonDocument doc(2560);
  JsonArray arr = doc.createNestedArray("networks");
  xSemaphoreTake(wifiMutex, portMAX_DELAY);
  for (int i = 0; i < scanCount; i++) {
    JsonObject n = arr.createNestedObject();
    n["ssid"]  = scanList[i].ssid;
    n["rssi"]  = scanList[i].rssi;
    n["open"]  = scanList[i].open;
    n["known"] = findKnownLocked(scanList[i].ssid) >= 0;
  }
  xSemaphoreGive(wifiMutex);
  String out;
  serializeJson(doc, out);
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", out);
}

void handleWifiJoin() {
  String ssid = server.arg("ssid");
  ssid.trim();
  if (!ssid.length()) { server.send(400, "application/json", "{\"ok\":false}"); return; }
  xSemaphoreTake(wifiMutex, portMAX_DELAY);
  strlcpy(joinSsid, ssid.c_str(), sizeof(joinSsid));
  strlcpy(joinPass, server.arg("pass").c_str(), sizeof(joinPass));
  joinResult  = JOIN_TRYING;
  joinPending = true;
  xSemaphoreGive(wifiMutex);
  server.send(200, "application/json", "{\"ok\":true}");
}

void handleWifiState() {
  static const char *JOIN_NAMES[] = { "idle", "trying", "ok", "failed" };
  StaticJsonDocument<384> doc;
  bool up = WiFi.status() == WL_CONNECTED;
  doc["connected"] = up;
  doc["ssid"]      = up ? WiFi.SSID() : String("");
  doc["ip"]        = up ? WiFi.localIP().toString() : String("");
  doc["join"]      = JOIN_NAMES[joinResult < 4 ? joinResult : 0];
  doc["joinSsid"]  = String(joinSsid);
  doc["ap"]        = (bool)apActive;
  String out;
  serializeJson(doc, out);
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", out);
}

void sendKnownList() {
  DynamicJsonDocument doc(1536);
  JsonArray arr = doc.createNestedArray("networks");
  String current = WiFi.status() == WL_CONNECTED ? WiFi.SSID() : String("");
  xSemaphoreTake(wifiMutex, portMAX_DELAY);
  for (int i = 0; i < knownCount; i++) {
    JsonObject n = arr.createNestedObject();
    n["ssid"]    = knownNets[i].ssid;
    n["default"] = !strcmp(knownNets[i].ssid, DEFAULT_WIFI_SSID);
    n["current"] = current == knownNets[i].ssid;
  }
  xSemaphoreGive(wifiMutex);
  String out;
  serializeJson(doc, out);
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", out);
}

void handleWifiKnown()  { sendKnownList(); }
void handleWifiForget() { forgetNetwork(server.arg("ssid").c_str()); sendKnownList(); }
void handleWifiAdd() {
  String ssid = server.arg("ssid");
  ssid.trim();
  addNetwork(ssid.c_str(), server.arg("pass").c_str());
  sendKnownList();
}

void handleRoot() {
  if (onSetupNetwork()) { handleSetupPage(); return; }
  server.send(200, "text/html", INDEX_HTML);
}

// Sends the exact 64x64 RGB565 buffer the renderer draws from (8 KB,
// little-endian). Streamed in 1 KB chunks: no big allocation, and the
// renderer is only paused for a memcpy at a time.
void handleCover() {
  if (!haveArt) {
    server.send(204);
    return;
  }

  const size_t total = sizeof(displayBuf);
  const uint8_t *src = (const uint8_t *)displayBuf;
  uint8_t chunk[1024];

  server.sendHeader("Cache-Control", "no-store");
  server.setContentLength(total);
  server.send(200, "application/octet-stream", "");

  for (size_t off = 0; off < total; off += sizeof(chunk)) {
    size_t n = min(sizeof(chunk), total - off);
    xSemaphoreTake(bufMutex, portMAX_DELAY);
    memcpy(chunk, src + off, n);
    xSemaphoreGive(bufMutex);
    server.sendContent((const char *)chunk, n);
  }
}

void handleStatus() {
  SpotifyStatus snap;
  xSemaphoreTake(statusMutex, portMAX_DELAY);
  snap = spStatus;
  xSemaphoreGive(statusMutex);

  StaticJsonDocument<2048> doc;

  JsonObject w = doc.createNestedObject("wifi");
  w["connected"] = (WiFi.status() == WL_CONNECTED);
  w["ssid"]      = WiFi.SSID();
  w["rssi"]      = WiFi.RSSI();
  w["ip"]        = WiFi.localIP().toString();

  JsonObject s = doc.createNestedObject("spotify");
  s["reachable"]     = (bool)netOk;
  s["tokenOk"]       = snap.tokenOk;
  s["state"]         = String(snap.state);
  s["track"]         = String(snap.track);
  s["artist"]        = String(snap.artist);
  s["lastHttp"]      = snap.lastHttp;
  s["lastUpdateAgo"] = snap.lastOkMs ? (long)((millis() - snap.lastOkMs) / 1000) : -1;
  s["hasCover"]      = (bool)haveArt;
  s["coverVersion"]  = (uint32_t)coverVersion;

  JsonObject d = doc.createNestedObject("device");
  d["uptime"]      = (uint32_t)(esp_timer_get_time() / 1000000ULL);
  d["freeHeap"]    = ESP.getFreeHeap();
  d["minFreeHeap"] = ESP.getMinFreeHeap();
  d["clkHz"]       = activeClkHz;
  d["latch"]       = activeLatch;

  JsonObject c = doc.createNestedObject("settings");
  c["brightness"] = (uint8_t)cfgBrightness;
  c["spinSpeed"]  = (float)cfgSpinSpeed;
  c["spindle"]    = (bool)cfgSpindle;
  c["paused"]     = (bool)cfgPaused;
  c["test"]       = (uint8_t)testPattern;
  c["clkPhase"]   = cfgClkPhase;
  c["latch"]      = cfgLatchBlank;
  c["mode"]       = (uint8_t)cfgMode;
  c["home"]       = (bool)cfgHome;
  c["lyricOffset"] = (int16_t)cfgLyricOffset;
  c["blk"]        = (uint8_t)cfgBlackLevel;
  c["gam"]        = (uint8_t)cfgContrast;
  c["sat"]        = (uint8_t)cfgSaturation;
  c["shd"]        = (uint8_t)cfgShadow;
  c["wbr"]        = (uint8_t)cfgWbR;
  c["wbg"]        = (uint8_t)cfgWbG;
  c["wbb"]        = (uint8_t)cfgWbB;
  JsonArray wbDef = c.createNestedArray("wbDef");   // For the dashboard's reset button
  wbDef.add(WB_R_DEFAULT); wbDef.add(WB_G_DEFAULT); wbDef.add(WB_B_DEFAULT);

  static const char *LYR_NAMES[] = { "none", "loading", "ready", "notfound", "instrumental", "error" };
  JsonObject ly = doc.createNestedObject("lyrics");
  uint8_t lst = lyricsState;
  ly["state"] = LYR_NAMES[lst < 6 ? lst : 0];
  if (lst == LYR_READY) {
    char cur[160] = "", nxt[160] = "";
    xSemaphoreTake(lyricsMutex, portMAX_DELAY);
    if (lyricsState == LYR_READY) {
      int idx = findLyricIndex(playbackPosMs());
      if (idx >= 0)             strlcpy(cur, lyricText + lyricOff[idx],     sizeof(cur));
      if (idx + 1 < lyricCount) strlcpy(nxt, lyricText + lyricOff[idx + 1], sizeof(nxt));
      ly["lines"] = lyricCount;
    }
    xSemaphoreGive(lyricsMutex);
    ly["current"] = String(cur);
    ly["next"]    = String(nxt);
  }

  String out;
  serializeJson(doc, out);
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", out);
}

void handleSettings() {
  if (server.hasArg("brightness")) {
    cfgBrightness   = constrain(server.arg("brightness").toInt(), 5, 255);
    brightnessDirty = true;
  }
  if (server.hasArg("spin"))    cfgSpinSpeed = constrain(server.arg("spin").toFloat(), 0.0f, 180.0f);
  if (server.hasArg("spindle")) { cfgSpindle = server.arg("spindle") == "1"; maskDirty = true; }
  if (server.hasArg("shd")) {
    cfgShadow = constrain(server.arg("shd").toInt(), 0, 100);
    redrawAll = true;                 // Applied at output: no need to reload the cover
  }
  if (server.hasArg("wbr") || server.hasArg("wbg") || server.hasArg("wbb")) {
    if (server.hasArg("wbr")) cfgWbR = constrain(server.arg("wbr").toInt(), 40, 100);
    if (server.hasArg("wbg")) cfgWbG = constrain(server.arg("wbg").toInt(), 40, 100);
    if (server.hasArg("wbb")) cfgWbB = constrain(server.arg("wbb").toInt(), 40, 100);
    wbDirty = true;                   // Applied at output, like the shadow cleanup
  }
  if (server.hasArg("blk") || server.hasArg("gam") || server.hasArg("sat")) {
    if (server.hasArg("blk")) cfgBlackLevel = constrain(server.arg("blk").toInt(), 0, 60);
    if (server.hasArg("gam")) cfgContrast   = constrain(server.arg("gam").toInt(), 100, 180);
    if (server.hasArg("sat")) cfgSaturation = constrain(server.arg("sat").toInt(), 80, 180);
    artReload = true;
    if (netTaskHandle) xTaskNotifyGive(netTaskHandle);
  }
  if (server.hasArg("mode")) {
    cfgMode = constrain(server.arg("mode").toInt(), 0, 2);
    if (cfgHome && isPlaying) cfgHome = false;     // Picking a view also leaves home mode
  }
  if (server.hasArg("lyroff")) cfgLyricOffset = constrain(server.arg("lyroff").toInt(), -3000, 3000);
  if (server.hasArg("test")) {
    testPattern = constrain(server.arg("test").toInt(), 0, 7);
    testDirty   = true;
  }
  if (server.hasArg("paused"))  cfgPaused    = server.arg("paused") == "1";

  settingsDirtyAt = millis() | 1;   // Saved to flash 2s after the last change
  server.send(200, "application/json", "{\"ok\":true}");
}

void handleRestart() {
  server.send(200, "application/json", "{\"ok\":true}");
  delay(300);
  ESP.restart();
}

void handleButton() {
  const char *msg = server.arg("press") == "long" ? remoteHold() : remoteTap();
  StaticJsonDocument<256> d;
  d["mode"]    = (uint8_t)cfgMode;
  d["home"]    = (bool)cfgHome;
  d["message"] = msg;
  String out;
  serializeJson(d, out);
  server.send(200, "application/json", out);
}

void handleSignal() {
  prefs.begin("disc", false);
  prefs.remove("clkmhz");           // Leftover from older firmware
  prefs.putBool("clkph", server.arg("phase") != "0");
  prefs.putUChar("latch", constrain(server.arg("latch").toInt(), 0, 4));
  prefs.end();

  server.send(200, "application/json", "{\"ok\":true}");
  delay(300);
  ESP.restart();
}

void handleWifiSetup() {
  server.send(200, "application/json", "{\"ok\":true}");
  rebootIntoSetup();
}

// ===================== WEB TASK (CORE 0) ========================

DNSServer dnsServer;

void webTask(void *param) {
  bool dnsOn = false;
  for (;;) {
    // On the setup network every name resolves to the display, so phones
    // open the setup page by themselves
    if (apActive && !dnsOn)  { dnsServer.start(53, "*", WiFi.softAPIP()); dnsOn = true; }
    if (!apActive && dnsOn)  { dnsServer.stop(); dnsOn = false; }
    if (dnsOn) dnsServer.processNextRequest();

    server.handleClient();

    // Batch flash writes: dragging a slider won't hammer the flash
    if (settingsDirtyAt && millis() - settingsDirtyAt > 2000) {
      settingsDirtyAt = 0;
      saveSettings();
    }

    vTaskDelay(pdMS_TO_TICKS(2));
  }
}

// ============================ SETUP =============================


void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.printf("\n[boot] reset reason: %d (1=power on, 3=software, 4=crash, 5-7=watchdog, 9=brownout)\n",
                (int)esp_reset_reason());

  pinMode(STATUS_LED_PIN, OUTPUT);
  pinMode(BOOT_BUTTON_PIN, INPUT_PULLUP);
  if (REMOTE_BUTTON_PIN >= 0) pinMode(REMOTE_BUTTON_PIN, INPUT_PULLUP);
  digitalWrite(STATUS_LED_PIN, LOW);

  bufMutex    = xSemaphoreCreateMutex();
  statusMutex = xSemaphoreCreateMutex();
  lyricsMutex = xSemaphoreCreateMutex();
  wifiMutex   = xSemaphoreCreateMutex();

  applyPanelProfile();    // New panel: reset the old panel's tuning once
  loadSettings();
  buildWbLut();
  setupMatrix();          // Allocate DMA memory first, while the heap is clean

  prefs.begin("disc", false);
  bool forcePortal = prefs.getBool("portal", false);   // "WiFi setup" button / BOOT hold
  if (forcePortal) prefs.putBool("portal", false);     // One-shot
  prefs.end();

  netViewSince = millis();
  netView = forcePortal ? NV_SETUP : NV_CONNECTING;    // Render loop animates from here

  xTaskCreatePinnedToCore(ledTask, "ledTask", 2048, nullptr, 1, nullptr, 0);

  // --- WiFi: saved networks ---
  WiFi.setHostname(HOSTNAME);
  WiFi.mode(WIFI_STA);
  loadKnownNets();
  ensureDefaultNet();
  importLegacyNetwork();
  esp_wifi_set_storage(WIFI_STORAGE_RAM);   // We keep our own list; no flash write per attempt
  WiFi.setAutoReconnect(true);
  // React to a drop immediately instead of at the next Spotify poll
  WiFi.onEvent([](arduino_event_id_t, arduino_event_info_t) {
    if (netTaskHandle) xTaskNotifyGive(netTaskHandle);
  }, ARDUINO_EVENT_WIFI_STA_DISCONNECTED);

  // --- Web server: dashboard, plus the setup page on the setup network ---
  server.on("/", HTTP_GET, handleRoot);
  server.on("/setup", HTTP_GET, handleSetupPage);
  server.on("/api/status", HTTP_GET, handleStatus);
  server.on("/api/cover", HTTP_GET, handleCover);
  server.on("/api/settings", HTTP_POST, handleSettings);
  server.on("/api/restart", HTTP_POST, handleRestart);
  server.on("/api/wifi-setup", HTTP_POST, handleWifiSetup);
  server.on("/api/signal", HTTP_POST, handleSignal);
  server.on("/api/button", HTTP_POST, handleButton);
  server.on("/api/wifi/scan", HTTP_GET, handleWifiScan);
  server.on("/api/wifi/join", HTTP_POST, handleWifiJoin);
  server.on("/api/wifi/state", HTTP_GET, handleWifiState);
  server.on("/api/wifi/known", HTTP_GET, handleWifiKnown);
  server.on("/api/wifi/forget", HTTP_POST, handleWifiForget);
  server.on("/api/wifi/add", HTTP_POST, handleWifiAdd);
  server.onNotFound([]() {
    if (onSetupNetwork()) {                 // Phones checking for a login page land here
      server.sendHeader("Location", "http://192.168.4.1/setup", true);
      server.send(302, "text/plain", "");
    } else {
      server.send(404, "text/plain", "Not found");
    }
  });
  server.begin();

  TJpgDec.setJpgScale(1);
  TJpgDec.setSwapBytes(false);
  TJpgDec.setCallback(jpegToBuffer);

  // The network task connects (showing the animations), then runs Spotify
  xTaskCreatePinnedToCore(webTask,     "webTask",     6144,           nullptr, 1, nullptr, 0);
  xTaskCreatePinnedToCore(networkTask, "networkTask", NET_TASK_STACK,
                          forcePortal ? (void *)1 : nullptr, 1, &netTaskHandle, 0);

  lastFrame = millis();
}

// ====================== RENDER LOOP (CORE 1) ====================

// What's currently on the panel, so each view knows when it must redraw
enum : uint8_t { V_NONE, V_DISC, V_LYRICS, V_COVER, V_HOME, V_NET, V_IDLE };
uint8_t  shownView     = V_NONE;
uint32_t lastHomeFrame = 0;
uint32_t lastNetFrame  = 0;

void loop() {
  uint32_t now = millis();
  const uint32_t framePeriod = 1000 / TARGET_FPS;
  uint32_t elapsed = now - lastFrame;
  if (elapsed < framePeriod) {
    delay(1);
    return;
  }
  lastFrame = now;

  if (brightnessDirty) {
    brightnessDirty = false;
    dma_display->setBrightness8(hwBrightness());
    wbDirty = true;                 // Software part of the dimming lives in the output table
  }

  if (wbDirty) {                    // White balance changed: same image, new output
    wbDirty = false;
    buildWbLut();
    if (testPattern) testDirty = true;
    shownView = V_NONE;
  }

  if (testDirty) {
    testDirty = false;
    if (testPattern) {
      drawTestPattern(testPattern);
    } else {
      dma_display->clearScreen();
      dma_display->flipDMABuffer();
    }
    shownView = V_NONE;             // Whatever comes next redraws fully
  }
  if (testPattern) return;

  if (redrawAll) {                  // e.g. shadow cleanup changed: same image, new output
    redrawAll = false;
    shownView = V_NONE;
  }

  if (maskDirty) {
    maskDirty = false;
    buildDiscMask(cfgSpindle);
    if (shownView == V_DISC) shownView = V_NONE;
  }

  // ---- Connection screens: over everything else, 25 frames per second ----
  if (netView != NV_NONE) {
    if (netView == NV_CONNECTED && (int32_t)(millis() - netViewSince) > 3000) {
      netView = NV_NONE;                       // Green screen done: back to normal
      dma_display->clearScreen();
      shownView = V_NONE;
    } else {
      if (shownView != V_NET || now - lastNetFrame >= 40) {
        lastNetFrame = now;
        renderNetView();
        shownView = V_NET;
      }
      return;
    }
  }

  // ---- Home: clock + weather, 10 frames per second for the animations ----
  if (cfgHome) {
    if (shownView != V_HOME || now - lastHomeFrame >= 100) {
      lastHomeFrame = now;
      renderHome();
      shownView = V_HOME;
    }
    return;
  }

  if (!haveArt) {
    if (bootIdle) {                            // Until the first cover: READY + IP
      if (shownView != V_IDLE || now - lastNetFrame >= 100) {
        lastNetFrame = now;
        renderIdle();
        shownView = V_IDLE;
      }
      return;
    }
    if (shownView != V_NONE) {
      dma_display->clearScreen();
      dma_display->flipDMABuffer();
      shownView = V_NONE;
    }
    return;
  }

  bool freshArt = newArt;
  if (newArt) {
    newArt = false;
    spinAngle = 0.0f;
  }

  // ---- Lyrics, when this song has synced lyrics (otherwise: the disc) ----
  if (cfgMode == MODE_LYRICS && lyricsState == LYR_READY) {
    bool force = shownView != V_LYRICS || shownCoverVer != coverVersion;
    shownCoverVer = coverVersion;
    shownView = V_LYRICS;
    updateLyricsView(force);
    return;
  }

  // ---- Plain cover ----
  if (cfgMode == MODE_COVER) {
    if (shownView != V_COVER || shownCoverVer != coverVersion) {
      drawCoverStatic();
      shownCoverVer = coverVersion;
      shownView = V_COVER;
    }
    return;
  }

  // ---- Spinning disc ----
  bool needsDraw = freshArt || shownView != V_DISC;
  if (isPlaying && !cfgPaused && cfgSpinSpeed > 0.0f) {
    spinAngle += cfgSpinSpeed * (elapsed / 1000.0f);
    if (spinAngle >= 360.0f) spinAngle -= 360.0f;
    needsDraw = true;
  }

  // Paused: the last frame simply stays up. No redraws, nothing that could
  // disturb a still image.
  if (!needsDraw) return;

  xSemaphoreTake(bufMutex, portMAX_DELAY);
  renderDisc(displayBuf, spinAngle);
  xSemaphoreGive(bufMutex);
  shownView = V_DISC;
}