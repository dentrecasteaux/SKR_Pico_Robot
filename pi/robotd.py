#!/usr/bin/env python3
"""Persistent Raspberry Pi executive link for the R2W robot."""

from __future__ import annotations

import argparse
import json
import os
import signal
import socketserver
import threading
import time
from pathlib import Path
from typing import Any, Optional

import serial

from robotctl import Reply, RobotClient, RobotLinkError, resolve_port


DEFAULT_PICO_LEASE_MS = 1000
DEFAULT_CONTROL_LEASE_MS = 1000
STATUS_PERIOD_SECONDS = 1.0
RECONNECT_PERIOD_SECONDS = 1.0


def default_socket_path() -> Path:
    runtime = os.environ.get("XDG_RUNTIME_DIR", f"/run/user/{os.getuid()}")
    return Path(runtime) / "r2w-robotd.sock"


def reply_fields(reply: Reply) -> dict[str, str]:
    result: dict[str, str] = {}
    for field in reply.fields:
        if "=" in field:
            key, value = field.split("=", 1)
            result[key] = value
    return result


class RobotController:
    def __init__(self, port: Optional[str]) -> None:
        self._requested_port = port
        self._client: Optional[RobotClient] = None
        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._velocity: Optional[tuple[float, float, int]] = None
        self._control_deadline = 0.0
        self._next_velocity_renewal = 0.0
        self._next_status_poll = 0.0
        self._next_reconnect = 0.0
        self._latest_status: dict[str, str] = {}
        self._latest_status_line = ""
        self._last_event = ""
        self._last_error = ""
        self._worker = threading.Thread(
            target=self._run_worker, name="robotd-link", daemon=True
        )

    def start(self) -> None:
        self._worker.start()

    def close(self) -> None:
        self._stop_event.set()
        self._worker.join(timeout=2.0)
        with self._lock:
            if self._client is not None and self._velocity is not None:
                try:
                    self._client.command("STOP")
                except (RobotLinkError, serial.SerialException):
                    pass
            self._close_client()

    def handle(self, request: dict[str, Any]) -> dict[str, Any]:
        action = request.get("action")
        if not isinstance(action, str):
            raise ValueError("request requires a string action")

        with self._lock:
            if action == "service_status":
                return {"ok": True, "service": self.snapshot()}

            client = self._require_client()

            if action == "ping":
                reply = client.command("PING")
            elif action == "status":
                reply = client.command("STATUS")
                self._record_status(reply)
            elif action == "velocity":
                linear = self._number(request, "linear")
                angular = self._number(request, "angular")
                pico_lease = self._integer(
                    request, "lease_ms", DEFAULT_PICO_LEASE_MS
                )
                control_lease = self._integer(
                    request, "hold_ms", DEFAULT_CONTROL_LEASE_MS
                )
                if not 200 <= control_lease <= 5000:
                    raise ValueError("hold_ms must be between 200 and 5000")
                reply = client.command(
                    "SET_VELOCITY",
                    f"V={linear:g}",
                    f"W={angular:g}",
                    f"LEASE={pico_lease}",
                )
                now = time.monotonic()
                self._velocity = (linear, angular, pico_lease)
                self._control_deadline = now + control_lease / 1000.0
                self._next_velocity_renewal = now + pico_lease / 3000.0
            elif action == "move":
                self._clear_velocity()
                reply = client.command(
                    "MOVE", f"DIST={self._number(request, 'distance'):g}"
                )
            elif action == "turn":
                self._clear_velocity()
                reply = client.command(
                    "TURN", f"ANGLE={self._number(request, 'angle'):g}"
                )
            elif action == "stop":
                self._clear_velocity()
                reply = client.command("STOP")
            elif action == "estop":
                self._clear_velocity()
                reply = client.command("ESTOP")
            elif action == "clear_estop":
                reply = client.command("CLEAR_ESTOP")
            else:
                raise ValueError(f"unknown action: {action}")

            return {
                "ok": True,
                "reply": reply.line,
                "service": self.snapshot(),
            }

    def snapshot(self) -> dict[str, Any]:
        return {
            "connected": self._client is not None,
            "velocity_controlled": self._velocity is not None,
            "status": dict(self._latest_status),
            "status_line": self._latest_status_line,
            "last_event": self._last_event,
            "last_error": self._last_error,
        }

    def _run_worker(self) -> None:
        while not self._stop_event.wait(0.05):
            with self._lock:
                try:
                    self._worker_step()
                except (RobotLinkError, serial.SerialException, OSError) as error:
                    self._last_error = str(error)
                    self._close_client()

    def _worker_step(self) -> None:
        now = time.monotonic()
        if self._client is None:
            if now < self._next_reconnect:
                return
            self._next_reconnect = now + RECONNECT_PERIOD_SECONDS
            self._connect()
            return

        if self._velocity is not None:
            if now >= self._control_deadline:
                self._clear_velocity()
                self._client.command("STOP")
                self._next_status_poll = now
                return
            if now >= self._next_velocity_renewal:
                linear, angular, pico_lease = self._velocity
                self._client.command(
                    "SET_VELOCITY",
                    f"V={linear:g}",
                    f"W={angular:g}",
                    f"LEASE={pico_lease}",
                )
                self._next_velocity_renewal = (
                    now + pico_lease / 3000.0
                )
                return

        if now >= self._next_status_poll:
            reply = self._client.command("STATUS")
            self._record_status(reply)
            self._next_status_poll = now + STATUS_PERIOD_SECONDS

    def _connect(self) -> None:
        port = resolve_port(self._requested_port)
        self._client = RobotClient(
            port, timeout=0.75, event_handler=self._record_event
        )
        self._last_error = ""
        self._next_status_poll = 0.0

    def _require_client(self) -> RobotClient:
        if self._client is None:
            self._connect()
        if self._client is None:
            raise RobotLinkError("Pico is not connected")
        return self._client

    def _close_client(self) -> None:
        if self._client is not None:
            self._client.close()
            self._client = None
        self._clear_velocity()

    def _clear_velocity(self) -> None:
        self._velocity = None
        self._control_deadline = 0.0
        self._next_velocity_renewal = 0.0

    def _record_status(self, reply: Reply) -> None:
        self._latest_status = reply_fields(reply)
        self._latest_status_line = reply.line

    def _record_event(self, reply: Reply) -> None:
        self._last_event = reply.line

    @staticmethod
    def _number(request: dict[str, Any], name: str) -> float:
        value = request.get(name)
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ValueError(f"{name} must be a number")
        return float(value)

    @staticmethod
    def _integer(
        request: dict[str, Any], name: str, default: int
    ) -> int:
        value = request.get(name, default)
        if isinstance(value, bool) or not isinstance(value, int):
            raise ValueError(f"{name} must be an integer")
        return value


class RequestHandler(socketserver.StreamRequestHandler):
    def handle(self) -> None:
        try:
            raw = self.rfile.readline(16385)
            if not raw or len(raw) > 16384:
                raise ValueError("request is empty or too large")
            request = json.loads(raw.decode("utf-8"))
            if not isinstance(request, dict):
                raise ValueError("request must be a JSON object")
            response = self.server.controller.handle(request)  # type: ignore
        except Exception as error:
            response = {"ok": False, "error": str(error)}
        self.wfile.write((json.dumps(response) + "\n").encode("utf-8"))


class UnixServer(socketserver.ThreadingUnixStreamServer):
    daemon_threads = True
    allow_reuse_address = True

    def __init__(
        self, path: str, controller: RobotController
    ) -> None:
        self.controller = controller
        super().__init__(path, RequestHandler)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="R2W robot control service")
    parser.add_argument("--port", help="override the Pico serial device")
    parser.add_argument(
        "--socket",
        type=Path,
        default=default_socket_path(),
        help="local Unix socket path",
    )
    return parser


def main() -> int:
    args = build_parser().parse_args()
    args.socket.parent.mkdir(parents=True, exist_ok=True)
    if args.socket.exists():
        args.socket.unlink()

    controller = RobotController(args.port)
    controller.start()

    def request_shutdown(*_: object) -> None:
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, request_shutdown)
    try:
        with UnixServer(str(args.socket), controller) as server:
            os.chmod(args.socket, 0o660)
            print(f"robotd listening on {args.socket}", flush=True)
            server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        controller.close()
        if args.socket.exists():
            args.socket.unlink()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
