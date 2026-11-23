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
from collections import deque
from pathlib import Path
from typing import Any, Optional

import serial

from robotctl import Reply, RobotClient, RobotLinkError, resolve_port
from runlog import BlackBoxCollector, RunRecorder, list_runs


DEFAULT_PICO_LEASE_MS = 1000
DEFAULT_CONTROL_LEASE_MS = 1000
STATUS_PERIOD_SECONDS = 1.0
RECONNECT_PERIOD_SECONDS = 1.0
WORKER_TICK_SECONDS = 0.025
IMU_PERIOD_SECONDS = 0.1
IMU_HISTORY_POINTS = 600  # 60 s at 10 Hz
IMU_HISTORY_FIELDS = ("ROLL", "PITCH", "YAW", "P", "Q", "R", "AX", "AY", "AZ")
# Balancing chassis wheel (CALIBRATION.md); used only to add mm/s columns.
WHEEL_DIAMETER_MM = 62.33
FULL_STEPS_PER_REV = 200
BB_DUMP_TIMEOUT_SECONDS = 30.0


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
        # IMU: None = not yet known, False = firmware has no IMU commands.
        self._imu_supported: Optional[bool] = None
        self._latest_imu: dict[str, str] = {}
        self._imu_history: deque[dict[str, Any]] = deque(
            maxlen=IMU_HISTORY_POINTS
        )
        self._imu_point = 0
        self._imu_epoch = time.monotonic()
        self._next_imu_poll = 0.0
        # Phase 6 telemetry
        self._recorder: Optional[RunRecorder] = None
        self._blackbox: Optional[BlackBoxCollector] = None
        self._last_run: dict[str, Any] = {}
        self._last_blackbox: dict[str, Any] = {}
        self._log_status: dict[str, str] = {}
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
            if action == "imu_history":
                return {"ok": True, **self._history_since(request)}

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
            elif action == "configure":
                current_ma = self._integer(request, "current_ma", 1000)
                microsteps = self._integer(request, "microsteps", 16)
                acceleration = self._number(request, "acceleration_mm_s2")
                mode = request.get("tmc_mode")
                if mode not in {"STEALTHCHOP", "SPREADCYCLE"}:
                    raise ValueError(
                        "tmc_mode must be STEALTHCHOP or SPREADCYCLE"
                    )
                reply = client.command(
                    "CONFIGURE",
                    f"CURRENT_MA={current_ma}",
                    f"MICROSTEPS={microsteps}",
                    f"ACCEL_MM_S2={acceleration:g}",
                    f"TMC_MODE={mode}",
                )
                self._record_status(client.command("STATUS"))
            elif action == "imu":
                reply = client.command("IMU")
                self._record_imu(reply)
            elif action == "imu_cal":
                reply = client.command("IMU_CAL")
            elif action == "imu_zero_yaw":
                reply = client.command("IMU_ZERO_YAW")
            elif action == "imu_set":
                fields = []
                if "tau" in request:
                    fields.append(f"TAU={self._number(request, 'tau'):g}")
                if "auto_bias" in request:
                    auto_bias = request.get("auto_bias")
                    if not isinstance(auto_bias, bool):
                        raise ValueError("auto_bias must be true or false")
                    fields.append(f"AUTO_BIAS={1 if auto_bias else 0}")
                if not fields:
                    raise ValueError("imu_set needs tau and/or auto_bias")
                reply = client.command("IMU_SET", *fields)
            elif action in ("bal_set", "bal_arm", "bal_stop", "bal_status",
                            "bal_kick"):
                fields: list[str] = []
                if action == "bal_set":
                    values = request.get("set")
                    if not isinstance(values, dict) or not values:
                        raise ValueError("bal_set needs a non-empty 'set' object")
                    for key, value in values.items():
                        if (not isinstance(key, str)
                                or not key.replace("_", "").isalnum()
                                or isinstance(value, bool)
                                or not isinstance(value, (int, float))):
                            raise ValueError(f"bad balance parameter {key!r}")
                        fields.append(f"{key.upper()}={value:g}")
                if action == "bal_kick":
                    fields.append(f"U={self._number(request, 'u'):g}")
                    fields.append(f"MS={self._integer(request, 'ms', 40)}")
                if action == "bal_arm":
                    self._clear_velocity()
                    if request.get("dry"):
                        fields.append("DRY=1")
                reply = client.command(action.upper(), *fields)
                return {
                    "ok": True,
                    "reply": reply.line,
                    "balance": reply_fields(reply),
                    "service": self.snapshot(),
                }
            elif action == "hold":
                self._clear_velocity()
                lease = self._integer(request, "lease_ms", 30000)
                reply = client.command("HOLD", f"LEASE={lease}")
            elif action == "log_start":
                if self._recorder is not None:
                    raise ValueError("a log run is already recording")
                name = request.get("name", "run")
                notes = request.get("notes", "")
                if not isinstance(name, str) or not isinstance(notes, str):
                    raise ValueError("name and notes must be strings")
                divisor = self._integer(request, "div", 2)
                duration = request.get("duration_s")
                if duration is not None and (
                    isinstance(duration, bool)
                    or not isinstance(duration, (int, float))
                    or duration <= 0
                ):
                    raise ValueError("duration_s must be a positive number")
                self._record_status(client.command("STATUS"))
                self._recorder = RunRecorder(
                    name, divisor, notes, duration,
                    dict(self._latest_status), dict(self._latest_imu),
                    self._steps_per_mm(),
                )
                try:
                    reply = client.command("LOG_START", f"DIV={divisor}")
                except RobotLinkError:
                    self._finish_recorder({"error": "LOG_START rejected"})
                    raise
                self._log_status = reply_fields(reply)
                return {
                    "ok": True,
                    "reply": reply.line,
                    "run": self._recorder.path.name,
                    "service": self.snapshot(),
                }
            elif action == "log_stop":
                reply = client.command("LOG_STOP")
                self._log_status = reply_fields(reply)
                client.poll_events()
                result = self._finish_recorder(self._log_status)
                return {
                    "ok": True,
                    "reply": reply.line,
                    "run": result,
                    "service": self.snapshot(),
                }
            elif action == "log_status":
                reply = client.command("LOG_STATUS")
                self._log_status = reply_fields(reply)
            elif action == "bb_freeze":
                reply = client.command("BB_FREEZE")
                self._log_status = reply_fields(reply)
            elif action == "bb_arm":
                reply = client.command("BB_ARM")
                self._log_status = reply_fields(reply)
            elif action == "bb_dump":
                if self._blackbox is not None:
                    raise ValueError("a black-box dump is already in progress")
                status = reply_fields(client.command("LOG_STATUS"))
                if status.get("BB") != "FROZEN":
                    raise ValueError(
                        "black box is not frozen (send bb_freeze first)"
                    )
                self._blackbox = BlackBoxCollector(
                    dict(self._latest_status), status, self._steps_per_mm()
                )
                try:
                    reply = client.command("BB_DUMP")
                except RobotLinkError:
                    self._blackbox.close({"error": "BB_DUMP rejected"})
                    self._blackbox = None
                    raise
                self._log_status = reply_fields(reply)
                return {
                    "ok": True,
                    "reply": reply.line,
                    "run": self._blackbox.path.name,
                    "service": self.snapshot(),
                }
            elif action == "runs":
                return {"ok": True, "runs": list_runs(),
                        "service": self.snapshot()}
            elif action == "imu_window_start":
                reply = client.command("IMU_WINDOW_START")
            elif action == "imu_window":
                reply = client.command("IMU_WINDOW")
                return {
                    "ok": True,
                    "reply": reply.line,
                    "window": reply_fields(reply),
                    "service": self.snapshot(),
                }
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
            "imu_supported": self._imu_supported,
            "imu": dict(self._latest_imu),
            "imu_point": self._imu_point,
            "log": self._telemetry_snapshot(),
        }

    def _telemetry_snapshot(self) -> dict[str, Any]:
        result: dict[str, Any] = {
            "firmware": dict(self._log_status),
            "last_run": self._last_run,
            "last_blackbox": self._last_blackbox,
        }
        if self._recorder is not None:
            result["recording"] = {
                "run": self._recorder.path.name,
                **self._recorder.summary(),
            }
        if self._blackbox is not None:
            result["dumping"] = {
                "run": self._blackbox.path.name,
                **self._blackbox.summary(),
            }
        return result

    def _steps_per_mm(self) -> Optional[float]:
        try:
            microsteps = int(self._latest_status.get("MICROSTEPS", ""))
        except ValueError:
            return None
        import math
        return FULL_STEPS_PER_REV * microsteps / (math.pi * WHEEL_DIAMETER_MM)

    def _finish_recorder(self, firmware: dict[str, str]) -> dict[str, Any]:
        if self._recorder is None:
            return {}
        meta = self._recorder.close({"firmware_at_stop": firmware})
        self._last_run = {"run": self._recorder.path.name,
                          **meta.get("summary", {})}
        self._recorder = None
        return self._last_run

    def _finish_blackbox(self, end: dict[str, str],
                         complete: bool = True) -> None:
        if self._blackbox is None:
            return
        if complete:
            meta = self._blackbox.finish(end)
        else:
            meta = self._blackbox.close({"error": "dump incomplete", **end})
        self._last_blackbox = {"run": self._blackbox.path.name,
                               "complete": complete,
                               **meta.get("summary", {})}
        self._blackbox = None

    def _run_worker(self) -> None:
        while not self._stop_event.wait(WORKER_TICK_SECONDS):
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

        # Drain unsolicited lines (LOG/BB telemetry, DONE events) every tick.
        self._client.poll_events()
        if self._recorder is not None and self._recorder.expired():
            reply = self._client.command("LOG_STOP")
            self._log_status = reply_fields(reply)
            self._client.poll_events()
            self._finish_recorder(self._log_status)
            return
        if (
            self._blackbox is not None
            and now - self._blackbox.started > BB_DUMP_TIMEOUT_SECONDS
        ):
            self._finish_blackbox({}, complete=False)

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
            return

        if self._imu_supported is not False and now >= self._next_imu_poll:
            self._next_imu_poll = now + IMU_PERIOD_SECONDS
            try:
                self._record_imu(self._client.command("IMU"))
                self._imu_supported = True
            except RobotLinkError as error:
                text = str(error)
                # Firmware without the IMU rejects the command: stop polling
                # until the next reconnect. Other errors propagate.
                if "UNKNOWN_COMMAND" in text or "UNSUPPORTED_COMMAND" in text:
                    self._imu_supported = False
                    self._latest_imu = {}
                else:
                    raise

    def _connect(self) -> None:
        port = resolve_port(self._requested_port)
        self._client = RobotClient(
            port, timeout=0.75, event_handler=self._record_event
        )
        self._last_error = ""
        self._next_status_poll = 0.0
        self._imu_supported = None
        self._next_imu_poll = 0.0

    def _require_client(self) -> RobotClient:
        if self._client is None:
            self._connect()
        if self._client is None:
            raise RobotLinkError("Pico is not connected")
        return self._client

    def _close_client(self) -> None:
        if self._recorder is not None:
            self._finish_recorder({"error": "link lost"})
        if self._blackbox is not None:
            self._finish_blackbox({"error": "link lost"}, complete=False)
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
        if reply.kind == "LOG":
            if self._recorder is not None:
                self._recorder.add_line(reply.fields)
            return
        if reply.kind == "BB":
            if self._blackbox is not None:
                self._blackbox.add_line(reply.fields)
            return
        if reply.kind == "BBEND":
            self._finish_blackbox(reply_fields(reply))
            return
        self._last_event = reply.line

    def _record_imu(self, reply: Reply) -> None:
        fields = reply_fields(reply)
        self._latest_imu = fields
        if fields.get("STATE") not in {"OK", "CALIBRATING"}:
            return
        point: dict[str, Any] = {
            "n": self._imu_point + 1,
            "t": round(time.monotonic() - self._imu_epoch, 3),
        }
        for key in IMU_HISTORY_FIELDS:
            try:
                point[key.lower()] = float(fields[key])
            except (KeyError, ValueError):
                return
        self._imu_point += 1
        self._imu_history.append(point)

    def _history_since(self, request: dict[str, Any]) -> dict[str, Any]:
        since = request.get("since", 0)
        if isinstance(since, bool) or not isinstance(since, int):
            raise ValueError("since must be an integer")
        points = [p for p in self._imu_history if p["n"] > since]
        return {"points": points, "latest": self._imu_point}

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
