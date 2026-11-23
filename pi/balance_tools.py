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
FLAG_KICK = 0x80   # with FLAG_ACTIVE: an identification kick is applied
SOCK = default_socket_path()

# Firmware defaults (Balance::Params). Floor runs restore the safety values.
DEFAULTS = {"KP": 36.7, "KD": 4.19, "KV": 4.81, "KX": 2.94, "TRIM": -3.9,
            "AMAX": 6.0, "VMAX": 0.55, "SOFT": 0.3, "TILT_FWD": 5.0,
            "TILT_BACK": -20.6, "ARM_WINDOW": 2.0, "ARM_RATE": 20.0,
            "ARM_TIMEOUT": 20.0, "RUNAWAY": 0.3, "ACCEL_COMP": 1}
# IMU pitch resting on the back training wheel when the tilt limits were set
# (Phase 5, 22 Nov). The tilt limits are absolute IMU angles.
BACK_STOP_DEG = -22.6
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


def kick_l_eff(rows: list[dict[str, float]], pad_s: float = 0.012) -> dict:
    """l_eff from identification kicks (BAL_KICK, plan 9.4).

    Over a short window around each kick the pendulum obeys
    theta'' = (g·(theta − theta0) − u) / l, so
    l = ∫(g·(theta − theta0) − u) dt / Δq.
    theta0 is the mean pitch over the 0.45 s before the kick; u is the
    logged total command (controller plus kick); Δq is the pitch-rate change
    across the window (padded for IMU filter and step delays).
    """
    act = [r for r in rows if int(r["flags"]) & FLAG_ACTIVE
           and not int(r["flags"]) & FLAG_DRY]
    t = [r["t_s"] for r in act]
    kicks = []
    i = 1
    while i < len(act):
        if int(act[i]["flags"]) & FLAG_KICK and not int(act[i - 1]["flags"]) & FLAG_KICK:
            j = i
            while j + 1 < len(act) and int(act[j + 1]["flags"]) & FLAG_KICK:
                j += 1
            kicks.append((i, j))
            i = j + 1
        else:
            i += 1
    results = []
    for i, j in kicks:
        t0, t1 = t[i] - pad_s, t[j] + pad_s
        pre = [r["pitch_deg"] for r in act if t[i] - 0.5 <= r["t_s"] < t[i] - 0.05]
        win = [k for k in range(len(act)) if t0 <= t[k] <= t1]
        if len(pre) < 20 or len(win) < 5 or win[0] < 3 or win[-1] + 3 >= len(act):
            continue
        theta0 = statistics.fmean(pre)
        integral = 0.0
        for a, b in zip(win, win[1:]):
            dt = t[b] - t[a]
            r = act[a]
            integral += (G * (r["pitch_deg"] - theta0) * D2R - r["u_mm_s2"] / 1000) * dt
        q_before = statistics.fmean(act[k]["q_dps"] for k in range(win[0] - 2, win[0] + 1))
        q_after = statistics.fmean(act[k]["q_dps"] for k in range(win[-1], win[-1] + 3))
        dq = (q_after - q_before) * D2R
        u_kick = statistics.fmean(act[k]["u_mm_s2"] for k in range(i, j + 1)) / 1000
        if abs(dq) < 1e-3:
            continue
        results.append({"t_s": round(t[i], 2), "u_mean": round(u_kick, 2),
                        "dq_dps": round(dq / D2R, 1),
                        "l_eff_mm": round(integral / dq * 1000, 1)})
    ls = [r["l_eff_mm"] for r in results]
    return {"kicks": results,
            "l_eff_mm": round(statistics.median(ls), 1) if ls else None,
            "spread_mm": (round(min(ls), 1), round(max(ls), 1)) if ls else None}
