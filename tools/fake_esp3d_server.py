#!/usr/bin/env python3
"""
Tiny ESP3D HTTP update endpoint emulator for testing tools/himill_flash.py.

It implements just enough ESP3D behavior for a local smoke test:
  GET  /login
  GET  /command?cmd=[ESP800]json
  GET  /updatefw
  POST /updatefw

Example:
  python3 tools/fake_esp3d_server.py --port 18080 --requests 3
  python3 tools/himill_flash.py esp3d --host 127.0.0.1:18080 --no-wait firmware.bin
"""

import argparse
from http.server import BaseHTTPRequestHandler, HTTPServer
import json
import re
import sys
import urllib.parse


ESP32_IMAGE_MAGIC = b"\xe9"


class FakeESP3DHandler(BaseHTTPRequestHandler):
    server_version = "FakeESP3D/1.0"

    def _send_json(self, status, payload):
        body = json.dumps(payload).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Connection", "close")
        self.end_headers()
        self.wfile.write(body)

    def _send_text(self, status, text):
        body = text.encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "text/plain")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Connection", "close")
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        path = urllib.parse.urlparse(self.path).path

        if path == "/login":
            self._send_json(200, {"status": "ok", "authentication_lvl": "admin"})
            return

        if path == "/command":
            self._send_json(
                200,
                {
                    "cmd": "800",
                    "status": "ok",
                    "data": {
                        "FWVersion": "fake-test",
                        "Hostname": "fake-esp3d",
                        "Authentication": "Disabled",
                        "WebUpdate": "Enabled",
                    },
                },
            )
            return

        if path == "/updatefw":
            self._send_json(200, {"status": "no file"})
            return

        if path == "/last-upload":
            self._send_json(200, self.server.last_upload or {"status": "none"})
            return

        self.send_error(404)

    def do_POST(self):
        if urllib.parse.urlparse(self.path).path != "/updatefw":
            self.send_error(404)
            return

        try:
            length = int(self.headers.get("Content-Length", "0"))
        except ValueError:
            self._send_text(400, "invalid Content-Length")
            return

        body = self.rfile.read(length)
        errors = validate_update_upload(self.headers.get("Content-Type", ""), body)
        if errors:
            self._send_json(500, {"status": "error", "errors": errors})
            return

        filename = extract_first(body, rb'filename="([^"]+)"') or b"?"
        size_field = extract_first(body, rb'name="([^"]+S)"') or b"?"
        self.server.last_upload = {
            "status": "ok",
            "bytes": length,
            "filename": filename.decode("utf-8", errors="replace"),
            "size_field": size_field.decode("utf-8", errors="replace"),
        }
        self._send_json(200, {"status": "ok"})

    def log_message(self, fmt, *args):
        if not self.server.quiet:
            sys.stderr.write(fmt % args + "\n")


def extract_first(body, pattern):
    match = re.search(pattern, body)
    return match.group(1) if match else None


def validate_update_upload(content_type, body):
    errors = []

    if "multipart/form-data" not in content_type:
        errors.append("Content-Type is not multipart/form-data")
    if b'name="myfile[]"' not in body:
        errors.append('missing firmware field name "myfile[]"')
    if not re.search(rb'name="/[^"]+\.binS"', body):
        errors.append('missing ESP3D size field name "/<firmware>.binS"')
    if not re.search(rb'filename="/[^"]+\.bin"', body):
        errors.append('missing ESP3D upload filename "/<firmware>.bin"')
    if ESP32_IMAGE_MAGIC not in body:
        errors.append("firmware payload does not contain ESP32 image magic 0xE9")

    return errors


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=18080)
    parser.add_argument(
        "--requests",
        type=int,
        default=0,
        help="Stop after this many HTTP requests; 0 means serve forever",
    )
    parser.add_argument("--quiet", action="store_true")
    args = parser.parse_args()

    server = HTTPServer((args.host, args.port), FakeESP3DHandler)
    server.last_upload = None
    server.quiet = args.quiet

    print(f"Fake ESP3D server listening on http://{args.host}:{args.port}")
    if args.requests:
        for _ in range(args.requests):
            server.handle_request()
    else:
        server.serve_forever()


if __name__ == "__main__":
    main()
