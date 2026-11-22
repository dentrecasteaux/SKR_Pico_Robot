"""Shared helpers for the balance test scripts (pure Python, no numpy)."""
from __future__ import annotations

import csv
import math
import statistics
from pathlib import Path

from robotd import default_socket_path
from robotdctl import request

RUNS_DIR = Path.home() / "runs"
G = 9.80665
D2R = math.pi / 180
FLAG_ACTIVE = 0x20
FLAG_DRY = 0x40
SOCK = default_socket_path()

# Firmware defaults (Balance::Params). Floor runs restore the safety values.
DEFAULTS = {"KP": 39.0, "KD": 2.9, "KV": 0.0, "KX": 0.0, "TRIM": -3.9,
            "AMAX": 6.0, "VMAX": 0.55, "SOFT": 0.3, "TILT_FWD": 5.0,
            "TILT_BACK": -20.6, "ARM_WINDOW": 2.0, "ARM_RATE": 20.0,
            "ARM_TIMEOUT": 20.0, "RUNAWAY": 0.3}
SAFETY_KEYS = ("AMAX", "VMAX", "SOFT", "TILT_FWD", "TILT_BACK", "ARM_WINDOW",
               "ARM_RATE", "ARM_TIMEOUT", "RUNAWAY")


def call(action: str, **kw) -> dict:
    reply = request(SOCK, {"action": action, **kw})
    if not reply.get("ok"):
        raise RuntimeError(f"{action}: {reply.get('error', reply)}")
    return reply


def balance_status() -> dict[str, str]:
    return call("bal_status")["balance"]


def load(run: str) -> list[dict[str, float]]:
    with open(RUNS_DIR / run / "data.csv", newline="") as fh:
        return [{k: float(v) if v not in ("", None) else 0.0 for k, v in r.items()}
                for r in csv.DictReader(fh)]


def active_rows(rows: list[dict[str, float]]) -> list[dict[str, float]]:
    return [r for r in rows if int(r["flags"]) & FLAG_ACTIVE]


def linreg(xs: list[float], ys: list[float]) -> tuple[float, float, float]:
    """Return slope, intercept and correlation coefficient."""
    n = len(xs)
    if n < 3:
        return float("nan"), float("nan"), float("nan")
    mx, my = statistics.fmean(xs), statistics.fmean(ys)
    sxx = sum((x - mx) ** 2 for x in xs)
    syy = sum((y - my) ** 2 for y in ys)
    sxy = sum((x - mx) * (y - my) for x, y in zip(xs, ys))
    if sxx == 0 or syy == 0:
        return float("nan"), float("nan"), float("nan")
    slope = sxy / sxx
    return slope, my - slope * mx, sxy / math.sqrt(sxx * syy)


def l_eff_estimate(rows: list[dict[str, float]], trim_deg: float) -> dict:
    """Estimate l_eff from theta'' = (g·e − u) / l_eff during balancing.

    theta'' from the logged pitch rate (central difference over ±5 samples);
    e = pitch − trim (rad); u from the log (mm/s²).
    """
    act = [r for r in active_rows(rows) if not int(r["flags"]) & FLAG_DRY]
    k = 5
    xs, ys = [], []
    for i in range(k, len(act) - k):
        dt = act[i + k]["t_s"] - act[i - k]["t_s"]
        if dt <= 0 or dt > 0.1:
            continue
        acc = (act[i + k]["q_dps"] - act[i - k]["q_dps"]) * D2R / dt
        e = (act[i]["pitch_deg"] - trim_deg) * D2R
        xs.append(G * e - act[i]["u_mm_s2"] / 1000.0)
        ys.append(acc)
    slope, _, r = linreg(xs, ys)
    if not (slope > 0):
        return {"samples": len(xs), "l_eff_mm": None, "r": r}
    return {"samples": len(xs), "l_eff_mm": round(1000 / slope, 1),
            "r": round(r, 3)}
