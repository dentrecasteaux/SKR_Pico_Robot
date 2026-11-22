#!/usr/bin/env python3
"""Phase 9.1: balance controller sign check, motors OFF.

The most important single test before balancing. The controller runs with
the drivers disabled while you tilt the robot by hand. It must command
FORWARD wheel acceleration (u > 0) when the robot leans FORWARD of its
balance point, and backward when it leans back. Otherwise the robot would
drive away from its own fall.

    ssh -t robot@r2w-pi.local 'cd r2w && python3 dryrun_check.py'

Hold the robot (on the floor or in your hands; the wheels stay free) near
upright. When prompted, tilt it slowly forward a few degrees and hold,
then back past upright and hold, then return. The script checks that
u = Kp·(theta − trim) + Kd·theta_dot holds, with the right sign.
"""
from __future__ import annotations

import time

from balance_tools import D2R, FLAG_DRY, active_rows, balance_status, call, linreg, load


def main() -> int:
    print(__doc__)
    call("stop")
    st = balance_status()
    kp, kd, trim, amax = (float(st["KP"]), float(st["KD"]), float(st["TRIM"]),
                          float(st["AMAX"]))
    print(f"Gains: KP={kp} KD={kd} TRIM={trim}° AMAX={amax} m/s²")
    input("Hold the robot near upright, then press Enter to start the dry run...")
    run = call("log_start", name="dryrun", div=1, duration_s=60,
               notes="dryrun_check.py, motors off")["run"]
    call("bal_arm", dry=True)
    time.sleep(0.4)
    print("Now: tilt slowly FORWARD about 4°, hold 2 s; then BACK past upright "
          "about 4°, hold 2 s; then return to upright.")
    print("Watch the live u (forward wheel acceleration):")
    try:
        end = time.monotonic() + 30
        while time.monotonic() < end:
            s = balance_status()
            print(f"\r   error {float(s['ERR']):+6.2f}°   rate {float(s['RATE']):+7.1f}°/s"
                  f"   u {float(s['U']):+6.2f} m/s²   ({s['BAL']})   Ctrl-C when done ",
                  end="", flush=True)
            if s["BAL"] != "DRY":
                break
            time.sleep(0.15)
    except KeyboardInterrupt:
        pass
    print()
    call("bal_stop")
    time.sleep(0.2)
    call("log_stop")

    rows = [r for r in active_rows(load(run)) if int(r["flags"]) & FLAG_DRY]
    rows = [r for r in rows if abs(r["u_mm_s2"]) < 0.98 * amax * 1000]   # unclamped
    if len(rows) < 200:
        print(f"FAIL  only {len(rows)} unclamped dry-run samples in {run}")
        return 1
    e = [(r["pitch_deg"] - trim) * D2R for r in rows]
    u = [r["u_mm_s2"] / 1000 for r in rows]
    rng = max(r["pitch_deg"] for r in rows) - min(r["pitch_deg"] for r in rows)
    slope, _, corr = linreg(e, u)
    fwd = [ui for ei, ui in zip(e, u) if ei > 2 * D2R]
    back = [ui for ei, ui in zip(e, u) if ei < -2 * D2R]
    ok_sign = slope > 0 and (not fwd or min(fwd) > -0.5) and (not back or max(back) < 0.5)
    print(f"run {run}: {len(rows)} samples, pitch range {rng:.1f}°")
    print(f"  u vs error: slope {slope:.1f} m/s²/rad (KP {kp}), correlation {corr:.3f}")
    print(f"  leaning forward >2°: {len(fwd)} samples, mean u {sum(fwd) / len(fwd) if fwd else float('nan'):+.2f}")
    print(f"  leaning back   >2°: {len(back)} samples, mean u {sum(back) / len(back) if back else float('nan'):+.2f}")
    checks = [
        ("tilted both ways", len(fwd) > 50 and len(back) > 50),
        ("sign: lean forward → u forward", ok_sign),
        ("gain matches KP (±35%, rate term included)", 0.65 * kp < slope < 1.35 * kp),
    ]
    for name, ok in checks:
        print(f"  {'PASS' if ok else 'FAIL'}  {name}")
    ok = all(c[1] for c in checks)
    print(f"\nPhase 9.1 {'PASS' if ok else 'FAIL'}")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
