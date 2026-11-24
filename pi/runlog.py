"""Run recording for robotd (commissioning plan Phase 6).

The firmware streams telemetry as positional lines:

    R2W/1 LOG <seq> <t_us> <pitch> <roll> <yaw> <p> <q> <r> <ax> <ay> <az>
              <u> <v_left> <v_right> <mode> <flags>
    R2W/1 BB  ... (same fields, black-box dump)
    R2W/1 BBEND COUNT=<n> REASON=<why> FROZEN_T_US=<t>

Angles are centidegrees, rates centidegrees/s, accelerations milli-g, u mm/s²,
wheel speeds steps/s. This module converts them to engineering units and
writes one folder per run under ~/runs/:

    <YYYY-MM-DD_HHMMSS>_<name>/data.csv
    <YYYY-MM-DD_HHMMSS>_<name>/meta.json
"""

from __future__ import annotations

import csv
import json
import re
import time
from datetime import datetime
from pathlib import Path
from typing import Any, Optional, Sequence

RUNS_DIR = Path.home() / "runs"
FIELD_COUNT = 16
CSV_HEADER = (
    "seq", "t_us", "t_s", "pitch_deg", "roll_deg", "yaw_deg",
    "p_dps", "q_dps", "r_dps", "ax_g", "ay_g", "az_g", "u_mm_s2",
    "v_left_sps", "v_right_sps", "v_left_mm_s", "v_right_mm_s",
    "mode", "flags",
)
MODE_NAMES = {0: "IDLE", 1: "VELOCITY", 2: "MOVE", 3: "TURN", 4: "ESTOP",
              5: "HOLD", 6: "BALANCE"}
FLAG_NAMES = {
    0x01: "LATE", 0x02: "STALL", 0x04: "ACC_REJECT", 0x08: "CALIBRATING",
    0x10: "NO_ATTITUDE", 0x20: "BALANCE_ACTIVE", 0x40: "BALANCE_DRY",
    0x80: "MOTORS_BUSY",
}
_SAFE_NAME = re.compile(r"[^A-Za-z0-9_.-]+")


def safe_name(name: str) -> str:
    cleaned = _SAFE_NAME.sub("-", name.strip())[:48].strip("-")
    return cleaned or "run"


def new_run_dir(name: str) -> Path:
    stamp = datetime.now().strftime("%Y-%m-%d_%H%M%S")
    path = RUNS_DIR / f"{stamp}_{safe_name(name)}"
    suffix = 1
    while path.exists():
        suffix += 1
        path = RUNS_DIR / f"{stamp}_{safe_name(name)}_{suffix}"
    path.mkdir(parents=True)
    return path


class _CsvRun:
    """Common writer: converts positional telemetry fields to a CSV row."""

    def __init__(self, path: Path, meta: dict[str, Any],
                 steps_per_mm: Optional[float]) -> None:
        self.path = path
        self.meta = meta
        self.steps_per_mm = steps_per_mm
        self.rows = 0
        self.bad_lines = 0
        self.seq_gaps = 0           # missing records (beyond the divisor)
        self.max_dt_us = 0
        self.first_seq: Optional[int] = None
        self.last_seq: Optional[int] = None
        self.first_t: Optional[int] = None
        self.last_t: Optional[int] = None
        self._file = open(path / "data.csv", "w", newline="")
        self._csv = csv.writer(self._file)
        self._csv.writerow(CSV_HEADER)
        self._last_flush = time.monotonic()
        self._write_meta()

    def add(self, fields: Sequence[str], step: int) -> None:
        if len(fields) != FIELD_COUNT:
            self.bad_lines += 1
            return
        try:
            v = [int(x) for x in fields]
        except ValueError:
            self.bad_lines += 1
            return
        seq, t_us = v[0], v[1]
        if self.first_seq is None:
            self.first_seq, self.first_t = seq, t_us
        else:
            assert self.last_seq is not None and self.last_t is not None
            expected = self.last_seq + step
            if seq > expected:
                self.seq_gaps += (seq - expected) // step
            dt = (t_us - self.last_t) & 0xFFFFFFFF
            self.max_dt_us = max(self.max_dt_us, dt)
        self.last_seq, self.last_t = seq, t_us
        assert self.first_t is not None
        t_s = ((t_us - self.first_t) & 0xFFFFFFFF) / 1e6
        if self.steps_per_mm:
            vl_mm = round(v[12] / self.steps_per_mm, 2)
            vr_mm = round(v[13] / self.steps_per_mm, 2)
        else:
            vl_mm = vr_mm = ""
        self._csv.writerow((
            seq, t_us, f"{t_s:.6f}",
            v[2] / 100, v[3] / 100, v[4] / 100,
            v[5] / 100, v[6] / 100, v[7] / 100,
            v[8] / 1000, v[9] / 1000, v[10] / 1000,
            v[11], v[12], v[13], vl_mm, vr_mm, v[14], v[15],
        ))
        self.rows += 1
        now = time.monotonic()
        if now - self._last_flush > 1.0:
            self._file.flush()
            self._last_flush = now

    def summary(self) -> dict[str, Any]:
        duration = None
        if self.first_t is not None and self.last_t is not None:
            duration = round(((self.last_t - self.first_t) & 0xFFFFFFFF) / 1e6, 3)
        return {
            "rows": self.rows,
            "duration_s": duration,
            "first_seq": self.first_seq,
            "last_seq": self.last_seq,
            "seq_gaps": self.seq_gaps,
            "max_dt_us": self.max_dt_us,
            "bad_lines": self.bad_lines,
        }

    def close(self, extra: Optional[dict[str, Any]] = None) -> dict[str, Any]:
        self._file.close()
        self.meta["finished"] = datetime.now().isoformat(timespec="seconds")
        self.meta["summary"] = self.summary()
        if extra:
            self.meta.update(extra)
        self._write_meta()
        return self.meta

    def _write_meta(self) -> None:
        (self.path / "meta.json").write_text(json.dumps(self.meta, indent=2))


class RunRecorder(_CsvRun):
    """A LOG_START ... LOG_STOP stream."""

    def __init__(self, name: str, divisor: int, notes: str,
                 duration_s: Optional[float], status: dict[str, str],
                 imu: dict[str, str], steps_per_mm: Optional[float],
                 settings: Optional[dict[str, Any]] = None) -> None:
        self.divisor = divisor
        self.deadline = (time.monotonic() + duration_s) if duration_s else None
        meta = {
            "kind": "log",
            "name": name,
            "notes": notes,
            "divisor": divisor,
            "rate_hz": 500 / divisor,
            "started": datetime.now().isoformat(timespec="seconds"),
            "duration_requested_s": duration_s,
            "status_at_start": status,
            "imu_at_start": imu,
            "settings_at_start": settings or {},
            "units": "angles deg, rates deg/s, accel g, u mm/s2, wheel speeds "
                     "steps/s and mm/s (forward positive)",
            "modes": MODE_NAMES,
            "flags": {f"0x{k:02x}": v for k, v in FLAG_NAMES.items()},
        }
        super().__init__(new_run_dir(name), meta, steps_per_mm)

    def add_line(self, fields: Sequence[str]) -> None:
        self.add(fields, self.divisor)

    def expired(self) -> bool:
        return self.deadline is not None and time.monotonic() >= self.deadline


class BlackBoxCollector(_CsvRun):
    """A BB_DUMP: BB lines until BBEND."""

    def __init__(self, status: dict[str, str], log_status: dict[str, str],
                 steps_per_mm: Optional[float]) -> None:
        reason = log_status.get("BB_REASON", "NONE")
        meta = {
            "kind": "blackbox",
            "name": f"bb_{reason.lower()}",
            "reason": reason,
            "expected_count": log_status.get("BB_COUNT"),
            "dumped_at": datetime.now().isoformat(timespec="seconds"),
            "status_at_dump": status,
            "complete": False,
        }
        super().__init__(new_run_dir(meta["name"]), meta, steps_per_mm)
        self.started = time.monotonic()
        self.complete = False

    def add_line(self, fields: Sequence[str]) -> None:
        self.add(fields, 1)

    def finish(self, end_fields: dict[str, str]) -> dict[str, Any]:
        self.complete = True
        return self.close({"complete": True, "bbend": end_fields})


def list_runs(limit: int = 50) -> list[dict[str, Any]]:
    if not RUNS_DIR.exists():
        return []
    runs = []
    for path in sorted(RUNS_DIR.iterdir(), reverse=True)[:limit]:
        if not path.is_dir():
            continue
        item: dict[str, Any] = {"run": path.name}
        meta_path = path / "meta.json"
        if meta_path.exists():
            try:
                meta = json.loads(meta_path.read_text())
                item["kind"] = meta.get("kind")
                item["summary"] = meta.get("summary")
            except (OSError, ValueError):
                pass
        data = path / "data.csv"
        item["bytes"] = data.stat().st_size if data.exists() else 0
        item["files"] = sorted(p.name for p in path.iterdir() if p.is_file())
        runs.append(item)
    return runs
