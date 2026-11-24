# Spotify LED Matrix Display

A wireless "now playing" display for Spotify, built on an ESP32 and a 64×64 RGB LED matrix. The album cover of whatever is playing spins on the panel like a vinyl record, and it can switch to time-synced lyrics, a still cover, or a clock with live weather. Everything is controlled from a phone dashboard that the ESP32 serves itself.

<p align="center">
  <img width="400" alt="Album cover spinning as a disc on the 64x64 LED matrix" src="https://github.com/user-attachments/assets/cf2f2a33-821a-498d-8e56-f1583594218c" />
  <img width="400" alt="Spotify LED matrix display" src="https://github.com/user-attachments/assets/977c8d89-ab64-4d7b-a729-e25139ca81a8" />
</p>

https://github.com/user-attachments/assets/6144b79d-7bca-4691-82dc-c6e0ea3cb930


## Features

- **Three music views**
  - **Disc**: the album cover is cut into a circle and rotated in real time at 30 FPS, with bilinear sampling so edges stay smooth while it turns.
  - **Lyrics**: time-synced lyrics (from [LRCLIB](https://lrclib.net)) that follow the playback position, with a fade between lines and an adjustable sync offset.
  - **Cover**: the plain album cover.
- **Home mode**: a clock with the current weather (from [Open-Meteo](https://open-meteo.com)), animated weather icons and a sky gradient that follows the time of day.
- **Phone dashboard** at `http://spotify-disc.local`: switch views, set brightness and spin speed, tune the image and the panel, and see the connection status. It can be added to the iPhone home screen and opens like an app.
- **Wi-Fi handling written from scratch**: remembers up to 8 networks, and opens a setup hotspot (`SpotifyDisc-Setup`) with a captive portal when no known network is in range.
- **Image processing for LED panels**: black level, contrast curve, saturation, shadow desaturation and a per-channel white-balance table, so album art looks right on LEDs instead of washed out or tinted.
- **Optional physical button** for switching views without the phone.

## How it works

```
                 ┌──────────────────────── ESP32 (dual core) ─────────────────────────┐
 Spotify API ──► │ Core 0  networkTask   poll "currently playing", refresh OAuth token │
 LRCLIB      ──► │                       download + decode JPEG cover, fetch lyrics    │
 Open-Meteo  ──► │         webTask       phone dashboard + JSON API (WebServer, mDNS)  │
                 │         ledTask       onboard status LED                            │
                 │                                                                     │
                 │ Core 1  loop()        rendering only: rotate, mask, colour-correct  │ ──► 64×64 HUB75 panel
                 └─────────────────────────────────────────────────────────────────────┘
```

- All network work runs on core 0, and core 1 does nothing but draw frames, so a slow download never makes the disc stutter. The two cores share the image through a mutex-protected buffer.
- The panel is driven over I2S with DMA using the `ESP32-HUB75-MatrixPanel-I2S-DMA` library.
- Spotify access uses the OAuth refresh-token flow. The ESP32 swaps the stored refresh token for a short-lived access token and renews it before it expires.
- Settings changed on the dashboard are saved to flash (`Preferences`) and survive restarts.

## Hardware

| Part | Notes |
|---|---|
| ESP32 DevKit V1 (30-pin, WROOM) | Any ESP32 with enough free GPIOs works |
| 64×64 HUB75E RGB LED panel | Built for a P2.5 panel (Chipone ICN2037BP drivers); also ran on a P3 panel (FM6124) |
| 5 V / 4 A power supply | Powers the panel and the ESP32 (5V → VIN) |
| Push button (optional) | Between a free GPIO (e.g. D21) and GND |

### Wiring

| HUB75 | ESP32 | HUB75 | ESP32 |
|---|---|---|---|
| R1 | D25 | A | D23 |
| G1 | D26 | B | D22 |
| B1 | D27 | C | D5 |
| R2 | D14 | D | TX2 (GPIO 17) |
| G2 | D33 | E | D32 |
| B2 | D13 | LAT | D4 |
| CLK | RX2 (GPIO 16) | OE | D15 |
| GND | GND | | |

## Setup

### 1. Libraries (Arduino IDE → Library Manager)

- ESP32-HUB75-MatrixPanel-I2S-DMA (mrfaptastic)
- Adafruit GFX Library
- TJpg_Decoder (Bodmer)
- ArduinoJson, **version 6**

Board: *ESP32 Dev Module* from the Espressif ESP32 board package.

### 2. Spotify credentials

1. Create an app in the [Spotify Developer Dashboard](https://developer.spotify.com/dashboard) and add `http://127.0.0.1:8888/callback` as a redirect URI.
2. Run the helper script, which opens the Spotify login and prints a refresh token:
   ```bash
   python3 tools/get_refresh_token.py <client_id> <client_secret>
   ```

### 3. Secrets file

```bash
cp firmware/spotify_disc/secrets.example.h firmware/spotify_disc/secrets.h
```

Fill in your Wi-Fi name and password and the three Spotify values. `secrets.h` is in `.gitignore`, so your credentials stay on your machine.

### 4. Flash

Open `firmware/spotify_disc/spotify_disc.ino` in the Arduino IDE and upload. Then open `http://spotify-disc.local` on a phone or computer on the same network.

To add another Wi-Fi network later, hold the **BOOT** button for 3 seconds, join `SpotifyDisc-Setup` with your phone, and pick the network.

## Engineering notes

Some problems that came up during the build, and how they were solved:

- **Missing green bits on the lower half.** G2 was first wired to GPIO 12. GPIO 12 is a boot strapping pin on the ESP32 and corrupted the signal, so G2 was moved to GPIO 33.
- **Ghosting on the P2.5 panel.** The ICN2037 drivers switch off slowly, so each row bled into the next and dark areas never went fully black. A wider blanking window around each latch, adjustable from the dashboard, fixed it.
- **Colour shift at low brightness.** Below about 30 % hardware brightness, the shortest PWM pulses became too short for the ICN2037 and colours drifted green or pink. The panel now never goes below a fixed hardware level, and lower brightness is done in software by scaling pixel values.
- **Cold, cyan-tinted whites.** The small LEDs on P2.5 panels are much stronger in blue and green than in red. A per-channel white-balance table, tuned live with a grey test pattern on the dashboard, corrects it.
- **Smooth rotation on 64×64 pixels.** Rotating with nearest-neighbour sampling made the edges shimmer. The renderer uses fixed-point bilinear sampling and a soft-edged circular mask, which costs under 1 ms per frame.

## Project structure

```
firmware/spotify_disc/
  spotify_disc.ino        firmware (rendering, networking, dashboard)
  secrets.example.h       template for your credentials
tools/
  get_refresh_token.py    gets a Spotify refresh token
```

## License

MIT. See [LICENSE](LICENSE).
