"""Saved balance tuning (web page "Save"), re-applied by robotd at SKR boot.

Standard library only; shared by robotd, robot_web and the balance scripts.
"""
from __future__ import annotations

import json
import math
import os
from pathlib import Path
from typing import Any

TUNING_PATH = Path.home() / ".config" / "r2w" / "balance_tuning.json"

# Firmware defaults (Balance::Params, 23 Nov 2026) for the keys the web page
# can tune and save. Safety limits are deliberately not here.
TUNING_DEFAULTS: dict[str, float] = {
    "KP": 51.4, "KD": 4.37, "KV": 10.0, "KX": 11.8, "TRIM": 1.9,
    "DRIVE_LEAN": 2.0, "DRIVE_ACCEL": 0.8, "DRIVE_FF": 1.0, "TRIM_RATE": 0.25,
}
TUNING_KEYS = tuple(TUNING_DEFAULTS)

# Allowed ranges (the firmware checks its own limits as well).
TUNING_RANGES: dict[str, tuple[float, float]] = {
    "KP": (15.0, 80.0), "KD": (1.0, 10.0), "KV": (0.0, 15.0),
    "KX": (0.0, 15.0), "TRIM": (-10.0, 10.0), "DRIVE_LEAN": (0.0, 5.0),
    "DRIVE_ACCEL": (0.1, 1.5), "DRIVE_FF": (0.0, 2.0), "TRIM_RATE": (0.0, 2.0),
}


def validate(values: Any) -> dict[str, float]:
    if not isinstance(values, dict) or not values:
        raise ValueError("tuning must be a non-empty object")
    clean: dict[str, float] = {}
    for key, value in values.items():
        if key not in TUNING_RANGES:
            raise ValueError(f"not a tuning key: {key}")
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ValueError(f"{key} must be a number")
        value = float(value)
        lo, hi = TUNING_RANGES[key]
        if not math.isfinite(value) or not lo <= value <= hi:
            raise ValueError(f"{key} must be between {lo:g} and {hi:g}")
        clean[key] = value
    return clean


def load_tuning() -> dict[str, float]:
    try:
        return validate(json.loads(TUNING_PATH.read_text()))
    except (OSError, ValueError):
        return {}


def save_tuning(values: dict[str, float]) -> dict[str, float]:
    clean = validate(values)
    TUNING_PATH.parent.mkdir(parents=True, exist_ok=True)
    tmp = TUNING_PATH.with_suffix(".tmp")
    tmp.write_text(json.dumps(clean, indent=2, sort_keys=True))
    os.replace(tmp, TUNING_PATH)
    return clean


def clear_tuning() -> None:
    try:
        TUNING_PATH.unlink()
    except FileNotFoundError:
        pass
