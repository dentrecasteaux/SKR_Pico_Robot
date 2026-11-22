#!/usr/bin/env python3
"""Free-fall test (commissioning plan Phase 7.3 / 7.4).

Measures how fast the robot falls (omega0, so l_eff = g / omega0^2) and the
angle it balances at (theta_eq, the unstable equilibrium = theta_trim).

The wheels are locked with HOLD. For each fall you hold the robot upright
near its balance point, keep it still, then let go gently (open your hands,
don't push). It falls onto a training wheel. Each fall is logged at 500 Hz
and fitted to the inverted-pendulum solution

    theta(t) = theta_eq + A·e^(w·t) + B·e^(−w·t)

using the gyro-integrated pitch (the filtered pitch is disturbed by the
fall's own accelerations). A second estimate regresses angular
acceleration against angle: theta'' = w²·(theta − theta_eq).

Run it interactively (note ssh -t):

    ssh -t robot@r2w-pi.local 'cd r2w && python3 fall_test.py'
    python3 fall_test.py --falls 4
    python3 fall_test.py --analyse 2026-11-23_071500_fall1 ...   # re-analyse

Pure Python; no numpy needed.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import statistics
import time
from pathlib import Path

RUNS_DIR = Path.home() / "runs"
G = 9.81
START_DEV_DEG = 1.0     # fit window starts this far from the held angle...
END_DEV_DEG = 8.0       # ...and ends here (before the training wheel)


# ------------------------------------------------------------------ analysis
def load(run: str) -> dict[str, list[float]]:
    with open(RUNS_DIR / run / "data.csv", newline="") as fh:
        rows = list(csv.DictReader(fh))
    return {
        "t": [float(r["t_s"]) for r in rows],
        "pitch": [float(r["pitch_deg"]) for r in rows],
        "q": [float(r["q_dps"]) for r in rows],
    }


def solve3(m: list[list[float]], v: list[float]) -> list[float]:
    """Solve a 3x3 linear system (Gaussian elimination, partial pivoting)."""
    a = [row[:] + [v[i]] for i, row in enumerate(m)]
    for c in range(3):
        p = max(range(c, 3), key=lambda r: abs(a[r][c]))
        a[c], a[p] = a[p], a[c]
        if abs(a[c][c]) < 1e-12:
            raise ValueError("singular")
        for r in range(c + 1, 3):
            f = a[r][c] / a[c][c]
            for k in range(c, 4):
                a[r][k] -= f * a[c][k]
    x = [0.0] * 3
    for r in (2, 1, 0):
        x[r] = (a[r][3] - sum(a[r][k] * x[k] for k in range(r + 1, 3))) / a[r][r]
    return x


def fit_exp(t: list[float], th: list[float], w: float) -> tuple[list[float], float]:
    """Least squares of th = c0 + c1·e^(w t) + c2·e^(−w t) for fixed w."""
    t0 = t[0]
    basis = [(1.0, math.exp(w * (x - t0)), math.exp(-w * (x - t0))) for x in t]
    m = [[sum(b[i] * b[j] for b in basis) for j in range(3)] for i in range(3)]
    v = [sum(b[i] * y for b, y in zip(basis, th)) for i in range(3)]
    c = solve3(m, v)
    res = [y - (c[0] + c[1] * b[1] + c[2] * b[2]) for b, y in zip(basis, th)]
    return c, math.sqrt(sum(r * r for r in res) / len(res))


def analyse(run: str) -> dict:
    d = load(run)
    t, pitch, q = d["t"], d["pitch"], d["q"]
    n = len(t)
    if n < 200:
        return {"run": run, "error": "too few samples"}

    # Held angle: median pitch over the first 0.5 s (robot held still).
    n_hold = max(10, sum(1 for x in t if x < 0.5))
    held = statistics.median(pitch[:n_hold])

    # End of the fit window: first sample 8° away from the held angle.
    i2 = next((i for i in range(n) if abs(pitch[i] - held) >= END_DEV_DEG), None)
    if i2 is None:
        return {"run": run, "error": f"never fell {END_DEV_DEG}° from {held:.1f}°"}
    direction = 1 if pitch[i2] > held else -1

    # Gyro-integrated angle, anchored to the filter during the held period
    # (pitch-rate sign checked against the filter over the whole fall).
    def integrate(sign: int) -> list[float]:
        out = [0.0] * n
        for i in range(1, n):
            out[i] = out[i - 1] + sign * 0.5 * (q[i] + q[i - 1]) * (t[i] - t[i - 1])
        return out

    best_sign, best_err = 1, float("inf")
    for sign in (1, -1):
        g = integrate(sign)
        offset = statistics.median(p - x for p, x in zip(pitch[:n_hold], g[:n_hold]))
        err = abs((g[i2] + offset) - pitch[i2])
        if err < best_err:
            best_sign, best_err = sign, err
    gyro = integrate(best_sign)
    offset = statistics.median(p - x for p, x in zip(pitch[:n_hold], gyro[:n_hold]))
    theta = [x + offset for x in gyro]

    # Start of the fit window: last sample before i2 within 1° of the held
    # angle (the fall has clearly begun; hands are off).
    i1 = next((i for i in range(i2, 0, -1) if abs(theta[i] - held) <= START_DEV_DEG), 0)
    if i2 - i1 < 30:
        return {"run": run, "error": "fit window too short"}
    tw, thw = t[i1:i2 + 1], theta[i1:i2 + 1]

    # Scan w, then refine (golden section) for the least residual.
    def rms(w: float) -> float:
        try:
            return fit_exp(tw, thw, w)[1]
        except ValueError:
            return float("inf")

    grid = [3 + 0.25 * k for k in range(int((40 - 3) / 0.25))]
    w0 = min(grid, key=rms)
    lo, hi = max(1.0, w0 - 0.5), w0 + 0.5
    gr = (math.sqrt(5) - 1) / 2
    for _ in range(40):
        a, b = hi - gr * (hi - lo), lo + gr * (hi - lo)
        if rms(a) < rms(b):
            hi = b
        else:
            lo = a
    w = 0.5 * (lo + hi)
    c, res = fit_exp(tw, thw, w)

    # Cross-check: theta'' = w²(theta − theta_eq), from the smoothed gyro rate.
    k = 5
    acc_pts = []
    for i in range(i1 + k, i2 - k):
        dq = best_sign * (q[i + k] - q[i - k]) / (t[i + k] - t[i - k])
        acc_pts.append((theta[i], dq))
    reg = None
    if len(acc_pts) > 20:
        mx = statistics.fmean(p[0] for p in acc_pts)
        my = statistics.fmean(p[1] for p in acc_pts)
        sxx = sum((p[0] - mx) ** 2 for p in acc_pts)
        sxy = sum((p[0] - mx) * (p[1] - my) for p in acc_pts)
        if sxx > 0 and sxy > 0:
            slope = sxy / sxx
            reg = {"omega": round(math.sqrt(slope), 2),
                   "theta_eq_deg": round(mx - my / slope, 2)}

    return {
        "run": run,
        "direction": "forward" if direction > 0 else "back",
        "held_deg": round(held, 2),
        "omega": round(w, 2),
        "l_eff_mm": round(1000 * G / (w * w), 1),
        "theta_eq_deg": round(c[0], 2),
        "fit_rms_deg": round(res, 3),
        "window_s": round(tw[-1] - tw[0], 3),
        "window_samples": len(tw),
        "pitch_rate_sign": best_sign,
        "regression": reg,
    }


def report(results: list[dict]) -> dict:
    good = [r for r in results if "error" not in r]
    print()
    print(f"{'run':<34} {'dir':<8} {'held°':>6} {'ω rad/s':>8} {'l_eff mm':>9}"
          f" {'θ_eq°':>7} {'rms°':>6} {'ω (reg)':>8} {'θ_eq (reg)':>10}")
    for r in results:
        if "error" in r:
            print(f"{r['run']:<34} ERROR: {r['error']}")
            continue
        reg = r.get("regression") or {}
        print(f"{r['run']:<34} {r['direction']:<8} {r['held_deg']:6.2f} {r['omega']:8.2f}"
              f" {r['l_eff_mm']:9.1f} {r['theta_eq_deg']:7.2f} {r['fit_rms_deg']:6.3f}"
              f" {reg.get('omega', float('nan')):8.2f} {reg.get('theta_eq_deg', float('nan')):10.2f}")
    summary: dict = {"falls": len(results), "good": len(good)}
    if good:
        ws = [r["omega"] for r in good]
        eqs = [r["theta_eq_deg"] for r in good]
        w_mean = statistics.fmean(ws)
        summary.update(
            omega_mean=round(w_mean, 2),
            omega_sd=round(statistics.stdev(ws), 2) if len(ws) > 1 else None,
            l_eff_mm=round(1000 * G / w_mean ** 2, 1),
            theta_trim_deg=round(statistics.fmean(eqs), 2),
            theta_trim_sd=round(statistics.stdev(eqs), 2) if len(eqs) > 1 else None,
        )
        print(f"\nω₀ = {summary['omega_mean']} rad/s (sd {summary['omega_sd']})  →  "
              f"l_eff = {summary['l_eff_mm']} mm   (swing test: 95 mm, ω₀ 10.1)")
        print(f"θ_trim = {summary['theta_trim_deg']}° IMU pitch (sd {summary['theta_trim_sd']})")
        dirs = {r["direction"] for r in good}
        if len(dirs) < 2:
            print("Note: falls in only one direction; θ_trim is better with both.")
    return summary


# --------------------------------------------------------------- test driver
def run_test(falls: int) -> None:
    from robotd import default_socket_path
    from robotdctl import request

    sock = default_socket_path()

    def call(action: str, **kw) -> dict:
        reply = request(sock, {"action": action, **kw})
        if not reply.get("ok"):
            raise RuntimeError(f"{action}: {reply.get('error', reply)}")
        return reply

    print(__doc__.split("Run it")[0])
    print("Safety: keep the kill switch in reach. The wheels are locked for up to "
          "2 minutes at a time; STOP or Ctrl-C releases them.\n")
    runs = []
    try:
        for i in range(1, falls + 1):
            side = "FORWARD" if i % 2 else "BACK"
            call("hold", lease_ms=120000)
            input(f"Fall {i}/{falls} (aim to fall {side}): hold the robot upright "
                  f"near balance, keep still, press Enter...")
            run = call("log_start", name=f"fall{i}_{side.lower()}", div=1,
                       notes="fall_test.py free fall, wheels held",
                       duration_s=12)["run"]
            print("   ...keep still for 1 s, then let go gently.")
            input("   Press Enter once it rests on the training wheel. ")
            call("log_stop")
            runs.append(run)
            res = analyse(run)
            if "error" in res:
                print(f"   {res['error']} - repeat this one if you like.")
            else:
                print(f"   ω = {res['omega']} rad/s, θ_eq = {res['theta_eq_deg']}°, "
                      f"fit rms {res['fit_rms_deg']}°")
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        try:
            call("stop")
        except Exception:  # noqa: BLE001
            pass
    results = [analyse(r) for r in runs]
    summary = report(results)
    out = RUNS_DIR / f"{time.strftime('%Y-%m-%d_%H%M%S')}_fall_summary.json"
    out.write_text(json.dumps({"summary": summary, "falls": results}, indent=2))
    print(f"\nSaved {out}")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--falls", type=int, default=6)
    ap.add_argument("--analyse", nargs="+", metavar="RUN",
                    help="re-analyse existing runs instead of testing")
    args = ap.parse_args()
    if args.analyse:
        report([analyse(r) for r in args.analyse])
    else:
        run_test(args.falls)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
