#!/usr/bin/env python3
"""Local-network web interface for robotd."""

from __future__ import annotations

import argparse
import json
import mimetypes
import os
import socket
import subprocess
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any


WEB_ROOT = Path(__file__).parent / "web"
MAX_REQUEST_BYTES = 4096
SHUTDOWN_COMMAND = ("sudo", "/sbin/shutdown", "-h", "now")
ALLOWED_ACTIONS = {
    "service_status",
    "status",
    "velocity",
    "move",
    "turn",
    "stop",
    "estop",
    "clear_estop",
    "configure",
}


def default_socket_path() -> Path:
    runtime = os.environ.get("XDG_RUNTIME_DIR", f"/run/user/{os.getuid()}")
    return Path(runtime) / "r2w-robotd.sock"


def robotd_request(path: Path, payload: dict[str, Any]) -> dict[str, Any]:
    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as connection:
        connection.settimeout(2.0)
        connection.connect(str(path))
        connection.sendall((json.dumps(payload) + "\n").encode("utf-8"))
        response = connection.makefile("rb").readline()
    if not response:
        raise RuntimeError("robotd closed the connection without a response")
    return json.loads(response.decode("utf-8"))


class WebHandler(BaseHTTPRequestHandler):
    server_version = "R2WWeb/1"

    def do_GET(self) -> None:
        if self.path == "/api/status":
            self._proxy({"action": "service_status"})
            return
        if self.path == "/favicon.ico":
            self.send_response(204)
            self.end_headers()
            return

        static_paths = {
            "/": "index.html",
            "/index.html": "index.html",
            "/app.js": "app.js",
            "/styles.css": "styles.css",
        }
        filename = static_paths.get(self.path)
        if filename is None:
            self.send_error(404)
            return
        self._serve_file(WEB_ROOT / filename)

    def do_POST(self) -> None:
        if self.path == "/api/shutdown":
            self._shutdown()
            return
        if self.path != "/api/command":
            self.send_error(404)
            return

        try:
            length = int(self.headers.get("Content-Length", "0"))
            if length <= 0 or length > MAX_REQUEST_BYTES:
                raise ValueError("invalid request size")
            content_type = self.headers.get("Content-Type", "")
            if not content_type.startswith("application/json"):
                raise ValueError("Content-Type must be application/json")
            payload = json.loads(self.rfile.read(length).decode("utf-8"))
            if not isinstance(payload, dict):
                raise ValueError("request must be a JSON object")
            if payload.get("action") not in ALLOWED_ACTIONS:
                raise ValueError("unsupported action")
        except (UnicodeDecodeError, ValueError, json.JSONDecodeError) as error:
            self._send_json({"ok": False, "error": str(error)}, 400)
            return

        self._proxy(payload)

    def _shutdown(self) -> None:
        try:
            length = int(self.headers.get("Content-Length", "0"))
            if length <= 0 or length > MAX_REQUEST_BYTES:
                raise ValueError("invalid request size")
            payload = json.loads(self.rfile.read(length).decode("utf-8"))
            if payload != {"confirm": "shutdown"}:
                raise ValueError("shutdown confirmation required")
        except (UnicodeDecodeError, ValueError, json.JSONDecodeError) as error:
            self._send_json({"ok": False, "error": str(error)}, 400)
            return
        try:
            subprocess.run(SHUTDOWN_COMMAND, check=True, timeout=5)
        except (OSError, subprocess.SubprocessError) as error:
            self._send_json(
                {"ok": False, "error": f"shutdown failed: {error}"}, 500
            )
            return
        self._send_json({"ok": True}, 200)

    def log_message(self, format: str, *args: object) -> None:
        if self.path == "/api/status":
            return
        print(f"{self.client_address[0]} - {format % args}", flush=True)

    def _proxy(self, payload: dict[str, Any]) -> None:
        try:
            response = robotd_request(self.server.robotd_socket, payload)  # type: ignore
            self._send_json(response, 200 if response.get("ok") else 409)
        except (OSError, RuntimeError, json.JSONDecodeError) as error:
            self._send_json(
                {"ok": False, "error": f"robot service unavailable: {error}"},
                503,
            )

    def _serve_file(self, path: Path) -> None:
        try:
            content = path.read_bytes()
        except OSError:
            self.send_error(404)
            return
        content_type = mimetypes.guess_type(path.name)[0] or "application/octet-stream"
        self.send_response(200)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(content)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.send_header("Content-Security-Policy", "default-src 'self'")
        self.end_headers()
        self.wfile.write(content)

    def _send_json(self, payload: dict[str, Any], status: int) -> None:
        content = json.dumps(payload).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(content)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.end_headers()
        self.wfile.write(content)


class WebServer(ThreadingHTTPServer):
    daemon_threads = True

    def __init__(
        self, address: tuple[str, int], robotd_socket: Path
    ) -> None:
        self.robotd_socket = robotd_socket
        super().__init__(address, WebHandler)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="R2W local robot web UI")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8080)
    parser.add_argument("--socket", type=Path, default=default_socket_path())
    return parser


def main() -> int:
    args = build_parser().parse_args()
    try:
        with WebServer((args.host, args.port), args.socket) as server:
            print(
                f"robot web interface listening on {args.host}:{args.port}",
                flush=True,
            )
            server.serve_forever()
    except KeyboardInterrupt:
        pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
