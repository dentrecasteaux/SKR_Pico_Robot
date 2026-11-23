#!/usr/bin/env python3
"""Local-network web interface for robotd."""

from __future__ import annotations

import argparse
import json
import mimetypes
import os
import socket
import subprocess
import threading
import time
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
    "imu",
    "imu_history",
    "imu_cal",
    "imu_zero_yaw",
    "imu_set",
    "imu_window_start",
    "imu_window",
    "log_start",
    "log_stop",
    "log_status",
    "bb_freeze",
    "bb_arm",
    "bb_dump",
    "runs",
    "hold",
    "bal_set",
    "bal_arm",
    "bal_stop",
    "bal_status",
    "bal_kick",
    "tuning",
    "tuning_save",
    "tuning_clear",
}
RUNS_DIR = Path.home() / "runs"
RUN_FILES = {"data.csv": "text/csv", "meta.json": "application/json",
             "balance_summary.json": "application/json"}


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


def _call(sock: Path, action: str, **kw: Any) -> dict[str, Any]:
    reply = robotd_request(sock, {"action": action, **kw})
    if not reply.get("ok"):
        raise RuntimeError(f"{action}: {reply.get('error', 'failed')}")
    return reply


class BalanceSession:
    """Start/stop of a balance run from the web page (like balance_run.py)."""

    def __init__(self) -> None:
        self.lock = threading.Lock()
        self.run: str | None = None
        self.notes = ""
        self.gains: dict[str, str] = {}

    def start(self, sock: Path, notes: str, record: bool) -> dict[str, Any]:
        # Imported here: balance_tools needs robotd/robotdctl on the path.
        from balance_tools import DEFAULTS, SAFETY_KEYS
        with self.lock:
            st = _call(sock, "bal_status")["balance"]
            if st.get("BAL") in ("ARMING", "RUNNING"):
                raise RuntimeError("already balancing")
            _call(sock, "bal_stop")
            time.sleep(0.3)
            _call(sock, "stop")
            base = _call(sock, "bal_status")["balance"]
            wrong = {k: DEFAULTS[k] for k in SAFETY_KEYS
                     if abs(float(base[k]) - DEFAULTS[k]) > 1e-3}
            if wrong:
                _call(sock, "bal_set", set=wrong)
                base = _call(sock, "bal_status")["balance"]
            self.gains = {k: base.get(k, "") for k in
                          ("KP", "KD", "KV", "KX", "TRIM", "DRIVE_LEAN",
                           "DRIVE_ACCEL", "AMAX", "VMAX")}
            self.run = None
            self.notes = notes
            if record:
                note = (f"web {notes} gains {base['KP']}/{base['KD']}/"
                        f"{base['KV']}/{base['KX']} trim {base['TRIM']}")
                try:
                    reply = _call(sock, "log_start", name="balance", div=2,
                                  duration_s=900, notes=note)
                except RuntimeError as error:
                    if "already recording" not in str(error):
                        raise
                    _call(sock, "log_stop")
                    reply = _call(sock, "log_start", name="balance", div=2,
                                  duration_s=900, notes=note)
                self.run = reply["run"]
            self.pwm_start = self._pwm(sock)
            _call(sock, "bal_arm")
            return {"ok": True, "run": self.run, "reset": wrong,
                    "gains": self.gains}

    @staticmethod
    def _pwm(sock: Path) -> dict[str, str]:
        """Drivers' StealthChop PWM state (learned values included)."""
        try:
            pwm = _call(sock, "tmc_pwm").get("pwm", {})
        except (RuntimeError, OSError):
            return {}
        keep = ("OFS", "GRAD", "AUTOSCALE", "AUTOGRAD", "OFS_AUTO", "GRAD_AUTO",
                "SCALE_SUM")
        return {k: v for k, v in pwm.items() if k[2:] in keep}

    def stop(self, sock: Path) -> dict[str, Any]:
        from balance_tools import kick_l_eff, load
        with self.lock:
            _call(sock, "bal_stop")
            time.sleep(0.7)
            final = _call(sock, "bal_status")["balance"]
            run, self.run = self.run, None
            if run is None:
                return {"ok": True, "run": None, "final": final}
            _call(sock, "log_stop")
            bb = None
            try:
                fw = _call(sock, "log_status")["service"]["log"]["firmware"]
                if fw.get("BB") == "FROZEN":
                    bb = _call(sock, "bb_dump")["run"]
            except (RuntimeError, KeyError):
                pass
            summary: dict[str, Any] = {
                "tmc_pwm_start": getattr(self, "pwm_start", {}),
                "tmc_pwm_end": self._pwm(sock),
                "run": run, "blackbox": bb, "source": "web",
                "stood_s": None,
                "end_reason": final.get("REASON"),
                "max_err_deg": final.get("MAX_ERR"),
                "max_u": final.get("MAX_U"),
                "gains": self.gains, "notes": self.notes,
            }
            try:
                rows = load(run)
                # Engaged time: from the first controller output (u is zero
                # while arming) to the last balance-active sample.
                act = [r for r in rows if int(r["flags"]) & 0x20
                       and not int(r["flags"]) & 0x40]
                engaged = [r for r in act if r["u_mm_s2"] != 0]
                if engaged:
                    summary["stood_s"] = round(act[-1]["t_s"] - engaged[0]["t_s"], 2)
                if any(int(r["flags"]) & 0x80 and int(r["flags"]) & 0x20
                       for r in rows):
                    summary["kick"] = kick_l_eff(rows)
            except (OSError, ValueError, KeyError) as error:
                summary["analysis_error"] = str(error)
            (RUNS_DIR / run / "balance_summary.json").write_text(
                json.dumps(summary, indent=2))
            return {"ok": True, "summary": summary}


def runs_with_summaries(sock: Path) -> dict[str, Any]:
    reply = robotd_request(sock, {"action": "runs"})
    for item in reply.get("runs", []):
        path = RUNS_DIR / item["run"] / "balance_summary.json"
        if path.exists():
            try:
                item["balance"] = json.loads(path.read_text())
            except (OSError, ValueError):
                pass
        meta = RUNS_DIR / item["run"] / "meta.json"
        try:
            item["notes"] = json.loads(meta.read_text()).get("notes", "")
        except (OSError, ValueError):
            pass
    return reply


class WebHandler(BaseHTTPRequestHandler):
    server_version = "R2WWeb/1"

    def do_GET(self) -> None:
        if self.path == "/api/status":
            self._proxy({"action": "service_status"})
            return
        if self.path == "/api/runs":
            try:
                reply = runs_with_summaries(self.server.robotd_socket)  # type: ignore
                self._send_json(reply, 200 if reply.get("ok") else 409)
            except (OSError, RuntimeError, json.JSONDecodeError) as error:
                self._send_json({"ok": False, "error": str(error)}, 503)
            return
        if self.path == "/api/balance":
            self._proxy({"action": "bal_status"})
            return
        if self.path.startswith("/runs/"):
            self._serve_run_file(self.path[len("/runs/"):])
            return
        if self.path == "/favicon.ico":
            self.send_response(204)
            self.end_headers()
            return

        static_paths = {
            "/": "balance.html",
            "/balance.html": "balance.html",
            "/balance.js": "balance.js",
            "/balance.css": "balance.css",
            "/bench": "index.html",
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
        if self.path in ("/api/balance/start", "/api/balance/stop"):
            self._balance_session(self.path.rsplit("/", 1)[1])
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

    def _balance_session(self, which: str) -> None:
        try:
            length = int(self.headers.get("Content-Length", "0") or 0)
            if length > MAX_REQUEST_BYTES:
                raise ValueError("invalid request size")
            payload = json.loads(self.rfile.read(length).decode("utf-8")) \
                if length else {}
            if not isinstance(payload, dict):
                raise ValueError("request must be a JSON object")
            notes = payload.get("notes", "")
            if not isinstance(notes, str) or len(notes) > 200:
                raise ValueError("notes must be a short string")
        except (UnicodeDecodeError, ValueError, json.JSONDecodeError) as error:
            self._send_json({"ok": False, "error": str(error)}, 400)
            return
        session: BalanceSession = self.server.balance  # type: ignore
        sock = self.server.robotd_socket  # type: ignore
        try:
            if which == "start":
                result = session.start(sock, notes, bool(payload.get("record", True)))
            else:
                result = session.stop(sock)
            self._send_json(result, 200)
        except (OSError, RuntimeError, KeyError, ValueError,
                json.JSONDecodeError) as error:
            self._send_json({"ok": False, "error": str(error)}, 409)

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
        if self.path in ("/api/status", "/api/balance"):
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

    def _serve_run_file(self, rest: str) -> None:
        # /runs/<run folder>/<data.csv|meta.json>, nothing else.
        parts = rest.split("/")
        if (
            len(parts) != 2
            or parts[1] not in RUN_FILES
            or not parts[0]
            or parts[0].startswith(".")
            or "\\" in parts[0]
        ):
            self.send_error(404)
            return
        path = RUNS_DIR / parts[0] / parts[1]
        try:
            if path.resolve().parent.parent != RUNS_DIR.resolve():
                raise OSError
            content = path.read_bytes()
        except OSError:
            self.send_error(404)
            return
        self.send_response(200)
        self.send_header("Content-Type", RUN_FILES[parts[1]])
        self.send_header("Content-Length", str(len(content)))
        self.send_header(
            "Content-Disposition",
            f'attachment; filename="{parts[0]}_{parts[1]}"',
        )
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Content-Type-Options", "nosniff")
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
        self.balance = BalanceSession()
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
