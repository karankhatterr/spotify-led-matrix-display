#!/usr/bin/env python3
"""Get a Spotify refresh token for the LED matrix display.

Usage:
    python3 get_refresh_token.py <client_id> <client_secret>

Add http://127.0.0.1:8888/callback as a redirect URI in your Spotify app first.
Uses only the Python standard library.
"""
import base64
import http.server
import json
import sys
import urllib.parse
import urllib.request
import webbrowser

REDIRECT_URI = "http://127.0.0.1:8888/callback"
SCOPES = "user-read-currently-playing user-read-playback-state"


def main():
    if len(sys.argv) != 3:
        sys.exit(__doc__)
    client_id, client_secret = sys.argv[1], sys.argv[2]
    code_holder = {}

    class Handler(http.server.BaseHTTPRequestHandler):
        def do_GET(self):
            query = urllib.parse.parse_qs(urllib.parse.urlparse(self.path).query)
            code_holder["code"] = query.get("code", [None])[0]
            code_holder["error"] = query.get("error", [None])[0]
            self.send_response(200)
            self.send_header("Content-Type", "text/plain; charset=utf-8")
            self.end_headers()
            self.wfile.write(b"Done. You can close this tab and go back to the terminal.")

        def log_message(self, *args):
            pass

    auth_url = "https://accounts.spotify.com/authorize?" + urllib.parse.urlencode({
        "client_id": client_id,
        "response_type": "code",
        "redirect_uri": REDIRECT_URI,
        "scope": SCOPES,
    })
    print("Opening the Spotify login in your browser. If it does not open, visit:\n" + auth_url)
    webbrowser.open(auth_url)

    with http.server.HTTPServer(("127.0.0.1", 8888), Handler) as server:
        server.handle_request()

    if not code_holder.get("code"):
        sys.exit("Authorization failed: %s" % code_holder.get("error"))

    basic = base64.b64encode(f"{client_id}:{client_secret}".encode()).decode()
    request = urllib.request.Request(
        "https://accounts.spotify.com/api/token",
        data=urllib.parse.urlencode({
            "grant_type": "authorization_code",
            "code": code_holder["code"],
            "redirect_uri": REDIRECT_URI,
        }).encode(),
        headers={"Authorization": "Basic " + basic},
    )
    with urllib.request.urlopen(request) as response:
        tokens = json.load(response)

    print("\nRefresh token (put this in secrets.h):\n")
    print(tokens["refresh_token"])


if __name__ == "__main__":
    main()
