#!/usr/bin/env python3
"""Phase 11 comparison metrics for compare_run.py runs.

    python3 analysis/compare.py RUN [RUN ...]            # markdown table
    python3 analysis/compare.py RUN ... --json out.json

RUN is a folder in runs/ (default ../runs relative to the repo, or $R2W_RUNS).
Needs numpy, scipy and pandas.

Metrics (all from the log; v and x are the commanded wheel speed and position):
  stand   RMS pitch about its mean, RMS pitch rate, RMS u, position drift and range
  kicks   ensemble ring frequency and damping (damped sine fitted from 0.12 s)
  pushes  peak pitch change, peak distance travelled, recovery time to within
          0.3 deg for 0.5 s, peak |u|
  steps   per speed step: time to 90 % of the commanded speed, overshoot,
          initial wrong-way speed, peak pitch change, settling time after the
          stop (|v| < 20 mm/s), position overshoot at the stop
  moving  pitch-rate RMS in the 3.8-5.2 Hz and 16-25 Hz bands while |v| > 100 mm/s
"""
from __future__ import annotations

import argparse
import json
import os
import sys

import numpy as np
import pandas as pd
from scipy.optimize import curve_fit
from scipy.signal import butter, filtfilt

HERE = os.path.dirname(os.path.abspath(__file__))
RUNS = os.environ.get("R2W_RUNS", os.path.join(HERE, "..", "..", "runs"))


def load(run: str):
    d = pd.read_csv(os.path.join(RUNS, run, "data.csv")).dropna()
    meta = json.load(open(os.path.join(RUNS, run, "compare.json")))
    d["flags"] = d["flags"].astype(int)
    d["mode"] = d["mode"].astype(int)
    d["v"] = (d.v_left_mm_s + d.v_right_mm_s) / 2
    d["x"] = np.cumsum(d.v.to_numpy()) * np.gradient(d.t_s.to_numpy())
    return d, meta


def window(d, t0, t1):
    return d[(d.t_s >= t0) & (d.t_s < t1) & ((d["flags"] & 0x20) > 0)]


def events(meta, name):
    return [e for e in meta["events"] if e["event"] == name]


def ev_time(meta, name, default=None):
    e = events(meta, name)
    return e[0]["t"] if e else default


def kick_onsets(d, t_from, t_to):
    k = (((d["flags"] & 0x80) > 0) & ((d["flags"] & 0x20) > 0)).to_numpy().astype(int)
    idx = np.flatnonzero(np.diff(k) == 1) + 1
    t = d.t_s.to_numpy()
    return [i for i in idx if t_from <= t[i] < t_to]


def ring_fit(d, onsets, post=1.6):
    t = d.t_s.to_numpy()
    q = d.q_dps.to_numpy()
    u = d.u_mm_s2.to_numpy()
    n0, n1 = 25, int(post / 0.004)
    segs = []
    for i in onsets:
        if i - n0 < 0 or i + n1 >= len(q):
            continue
        s = np.sign(u[i] - u[i - 1]) or 1.0
        segs.append(s * (q[i - n0:i + n1] - q[i - n0:i].mean()))
    if not segs:
        return None
    m = np.mean(segs, 0)
    tt = np.arange(-n0, n1) * 0.004
    sel = (tt > 0.12) & (tt < 1.4)

    def f(t_, a, z, fr, ph, c):
        return a * np.exp(-z * 2 * np.pi * fr * (t_ - 0.12)) * np.cos(2 * np.pi * fr * (t_ - 0.12) + ph) + c

    best = None
    for f0 in (2.5, 3.0, 4.0, 5.0):
        try:
            p, _ = curve_fit(f, tt[sel], m[sel], p0=[20, 0.2, f0, 0, 0],
                             bounds=([0, 0, 1, -7, -20], [200, 1.5, 8, 7, 20]), maxfev=20000)
            r = np.sqrt(np.mean((f(tt[sel], *p) - m[sel]) ** 2))
            if best is None or r < best[1]:
                best = (p, r)
        except RuntimeError:
            pass
    if best is None:
        return None
    return {"n": len(segs), "hz": round(float(best[0][2]), 2), "zeta": round(float(best[0][1]), 3),
            "peak_q_dps": round(float(np.max(np.abs(m))), 1)}


def push_metrics(d, onsets):
    t = d.t_s.to_numpy()
    th = d.pitch_deg.to_numpy()
    x = d.x.to_numpy()
    u = d.u_mm_s2.to_numpy() / 1000
    out = []
    for i in onsets:
        pre = slice(max(0, i - 125), i)
        th0, x0 = th[pre].mean(), x[i]
        post = slice(i, min(len(t), i + int(3.0 / 0.004)))
        dth = th[post] - th0
        k_peak = int(np.argmax(np.abs(dth)))
        rec = None
        inside = np.abs(dth) < 0.3
        hold = int(0.5 / 0.004)
        for j in range(k_peak, len(inside) - hold):
            if inside[j:j + hold].all():
                rec = round(float(t[i + j] - t[i]), 2)
                break
        out.append({"t": round(float(t[i]), 2), "dir": int(np.sign(u[i]) or 1),
                    "peak_dpitch_deg": round(float(dth[k_peak]), 2),
                    "peak_dist_mm": round(float(np.max(np.abs(x[post] - x0))), 0),
                    "recovery_s": rec, "peak_u": round(float(np.max(np.abs(u[post]))), 2)})
    return out


def step_metrics(d, meta):
    steps = events(meta, "step")
    t = d.t_s.to_numpy()
    v = d.v.to_numpy()
    th = d.pitch_deg.to_numpy()
    x = d.x.to_numpy()
    out = []
    for k, e in enumerate(steps):
        if not e["v_mm_s"]:
            continue
        V = e["v_mm_s"]
        t0 = e["t"]
        t_stop = steps[k + 1]["t"] if k + 1 < len(steps) else t0 + 2.5
        t_end = steps[k + 2]["t"] if k + 2 < len(steps) else t_stop + 4.0
        on = (t >= t0) & (t < t_stop)
        if not on.any():
            continue
        i0 = np.flatnonzero(on)[0]
        sv = np.sign(V)
        vo = v[on] * sv
        reach = np.flatnonzero(vo >= 0.9 * abs(V))
        early = (t >= t0) & (t < t0 + 0.5)
        after = (t >= t_stop) & (t < t_end)
        va = v[after]
        settle = None
        if after.any():
            ta = t[after]
            calm = np.abs(va) < 20
            for j in range(len(calm)):
                if calm[j:].all():
                    settle = round(float(ta[j] - t_stop), 2)
                    break
        i_stop = np.flatnonzero(t >= t_stop)[0] if (t >= t_stop).any() else len(t) - 1
        out.append({
            "v_mm_s": V,
            "t90_s": round(float(t[on][reach[0]] - t0), 2) if len(reach) else None,
            "overshoot_pct": round(float((vo.max() / abs(V) - 1) * 100), 1),
            "wrong_way_mm_s": round(float(max(0.0, -(v[early] * sv).min())), 1),
            "peak_dpitch_deg": round(float(np.max(np.abs(th[on] - th[max(0, i0 - 125):i0].mean()))), 2),
            "stop_settle_s": settle,
            "stop_overshoot_mm": round(float(np.max((x[after] - x[i_stop]) * sv)) if after.any() else 0.0, 0),
        })
    return out


def band_rms(d, lo, hi):
    q = d.q_dps.to_numpy()
    b, a = butter(2, [lo / 125, hi / 125], "band")
    return filtfilt(b, a, q)


def metrics(run: str) -> dict:
    d, meta = load(run)
    b = meta["settings"]["balance"]
    imu = meta["settings"].get("imu", {}).get("service", {}).get("imu", {})
    res = {"run": run, "label": meta["label"], "result": meta["result"],
           "settings": {k: b.get(k) for k in ("KP", "KD", "KV", "KX", "ACCEL_COMP", "NOTCH_HZ", "NOTCH_Q")}
           | {"TAU": imu.get("TAU")}}
    act = d[(d["flags"] & 0x20) > 0]
    res["balanced_s"] = round(float(len(act) * 0.004), 1)
    t_s0, t_s1 = ev_time(meta, "stand_start"), ev_time(meta, "stand_end")
    if t_s0 is not None and t_s1 is not None:
        w = window(d, t_s0, t_s1)
        res["stand"] = {
            "pitch_rms_deg": round(float(w.pitch_deg.std()), 3),
            "rate_rms_dps": round(float(w.q_dps.std()), 2),
            "u_rms": round(float((w.u_mm_s2 / 1000).std()), 3),
            "drift_mm": round(float(w.x.iloc[-1] - w.x.iloc[0]), 0),
            "range_mm": round(float(w.x.max() - w.x.min()), 0),
        }
    k0, k1 = ev_time(meta, "kick_start"), ev_time(meta, "kick_end")
    if k0 is not None:
        res["kicks"] = ring_fit(d, kick_onsets(d, k0, k1 if k1 is not None else 1e9))
    p0, p1 = ev_time(meta, "push_start"), ev_time(meta, "push_end")
    if p0 is not None:
        pm = push_metrics(d, kick_onsets(d, p0, p1 if p1 is not None else 1e9))
        if pm:
            recs = [p["recovery_s"] for p in pm if p["recovery_s"] is not None]
            res["pushes"] = {
                "n": len(pm),
                "peak_dpitch_deg": round(float(np.mean([abs(p["peak_dpitch_deg"]) for p in pm])), 2),
                "peak_dist_mm": round(float(np.mean([p["peak_dist_mm"] for p in pm])), 0),
                "recovery_s": round(float(np.mean(recs)), 2) if recs else None,
                "peak_u": round(float(np.max([p["peak_u"] for p in pm])), 2),
                "each": pm,
            }
    sm = step_metrics(d, meta)
    if sm:
        def avg(key):
            vals = [abs(s[key]) for s in sm if s[key] is not None]
            return round(float(np.mean(vals)), 2) if vals else None
        res["steps"] = {k: avg(k) for k in ("t90_s", "overshoot_pct", "wrong_way_mm_s",
                                           "peak_dpitch_deg", "stop_settle_s", "stop_overshoot_mm")}
        res["steps"]["each"] = sm
    mv = (d.v.abs() > 100).to_numpy() & ((d["flags"] & 0x20) > 0).to_numpy()
    if mv.sum() > 100:
        res["moving"] = {"band_4_5hz_dps": round(float(band_rms(d, 3.8, 5.2)[mv].std()), 2),
                         "band_20hz_dps": round(float(band_rms(d, 16, 25)[mv].std()), 2),
                         "u_sd": round(float((d.u_mm_s2[mv] / 1000).std()), 3),
                         "acc_rejects": round(float(((d["flags"][mv] & 4) > 0).mean()), 3)}
    return res


ROWS = [
    ("Result", lambda r: r["result"]),
    ("KP / KD / KV / KX", lambda r: " / ".join(str(r["settings"][k]) for k in ("KP", "KD", "KV", "KX"))),
    ("Comp / notch / τ", lambda r: f"{r['settings']['ACCEL_COMP']} / {r['settings']['NOTCH_HZ']} Hz Q{r['settings']['NOTCH_Q']} / {r['settings']['TAU']} s"),
    ("Balanced (s)", lambda r: r["balanced_s"]),
    ("Stand: pitch RMS (°)", lambda r: r.get("stand", {}).get("pitch_rms_deg")),
    ("Stand: rate RMS (°/s)", lambda r: r.get("stand", {}).get("rate_rms_dps")),
    ("Stand: u RMS (m/s²)", lambda r: r.get("stand", {}).get("u_rms")),
    ("Stand: drift / range (mm)", lambda r: f"{r['stand']['drift_mm']:.0f} / {r['stand']['range_mm']:.0f}" if "stand" in r else None),
    ("Kicks: ring (Hz, ζ)", lambda r: f"{r['kicks']['hz']} Hz, ζ {r['kicks']['zeta']}" if r.get("kicks") else None),
    ("Pushes: peak pitch (°)", lambda r: r.get("pushes", {}).get("peak_dpitch_deg")),
    ("Pushes: distance (mm)", lambda r: r.get("pushes", {}).get("peak_dist_mm")),
    ("Pushes: recovery (s)", lambda r: r.get("pushes", {}).get("recovery_s")),
    ("Pushes: peak |u| (m/s²)", lambda r: r.get("pushes", {}).get("peak_u")),
    ("Steps: t90 (s)", lambda r: r.get("steps", {}).get("t90_s")),
    ("Steps: overshoot (%)", lambda r: r.get("steps", {}).get("overshoot_pct")),
    ("Steps: wrong-way (mm/s)", lambda r: r.get("steps", {}).get("wrong_way_mm_s")),
    ("Steps: peak pitch (°)", lambda r: r.get("steps", {}).get("peak_dpitch_deg")),
    ("Steps: stop settle (s)", lambda r: r.get("steps", {}).get("stop_settle_s")),
    ("Steps: stop overshoot (mm)", lambda r: r.get("steps", {}).get("stop_overshoot_mm")),
    ("Moving: 4–5 Hz / 20 Hz (°/s)", lambda r: f"{r['moving']['band_4_5hz_dps']} / {r['moving']['band_20hz_dps']}" if r.get("moving") else None),
    ("Moving: u sd (m/s²)", lambda r: r.get("moving", {}).get("u_sd")),
]


def table(results: list[dict]) -> str:
    head = "| Metric | " + " | ".join(r["label"] for r in results) + " |"
    sep = "|---|" + "---|" * len(results)
    lines = [head, sep]
    for name, fn in ROWS:
        vals = []
        for r in results:
            try:
                v = fn(r)
            except (KeyError, TypeError):
                v = None
            vals.append("—" if v is None else str(v))
        lines.append(f"| {name} | " + " | ".join(vals) + " |")
    return "\n".join(lines)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("runs", nargs="+")
    ap.add_argument("--json")
    args = ap.parse_args()
    results = [metrics(r) for r in args.runs]
    print(table(results))
    if args.json:
        json.dump(results, open(args.json, "w"), indent=1)
    return 0


if __name__ == "__main__":
    sys.exit(main())
