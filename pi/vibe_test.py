#!/usr/bin/env python3
"""IMU vibration test (commissioning plan Phase 5.4).

Robot ON THE STAND, wheels clear. For each chopper mode and each wheel speed,
runs both wheels forward, waits for them to reach speed, then records a 4 s
IMU statistics window (the firmware's IMU_WINDOW, every 500 Hz sample) and
prints the noise. Results are also written to ~/runs/<timestamp>_vibe/.

    cd ~/r2w && python3 vibe_test.py               # both modes, default speeds
    python3 vibe_test.py --modes SPREADCYCLE --speeds 0 1 2

Restores 1000 mA / 16 microsteps / 150 mm/s² / StealthChop at the end.
Ctrl-C stops the motors (and they stop by themselves within 5 s anyway).
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import time
from datetime import datetime
from pathlib import Path

from robotd import default_socket_path
from robotdctl import request

WHEEL_D_MM = 62.33
FIELDS = ["GX_SD", "GY_SD", "GZ_SD", "AX_SD", "AY_SD", "AZ_SD",
          "AMAG_MIN", "AMAG_MAX", "PITCH_MIN", "PITCH_MAX",
          "ACC_REJECTS", "SAMPLES", "MOVING_SAMPLES", "LATE", "ERRORS",
          "STALLS", "LOOP_MAX_US", "GAP_MAX_US"]

SOCK = default_socket_path()


def call(action: str, **kw) -> dict:
    reply = request(SOCK, {"action": action, **kw})
    if not reply.get("ok"):
        raise RuntimeError(f"{action}: {reply}")
    return reply


def parse_kv(line: str) -> dict[str, str]:
    return dict(tok.split("=", 1) for tok in line.split() if "=" in tok)


def configure(mode: str, accel: float, current: int = 1000) -> None:
    call("stop")
    time.sleep(0.6)  # let the motors disable so CONFIGURE is accepted
    call("configure", current_ma=current, microsteps=16,
         acceleration_mm_s2=accel, tmc_mode=mode)


def run_point(mm_s: float, settle_s: float, window_s: float) -> dict[str, str]:
    def drive() -> None:
        if mm_s:
            call("velocity", linear=mm_s, angular=0.0, lease_ms=1000,
                 hold_ms=5000)

    drive()
    t_end = time.monotonic() + settle_s
    while time.monotonic() < t_end:
        time.sleep(0.2)
    call("imu_window_start")
    t0 = time.monotonic()
    last = t0
    while time.monotonic() - t0 < window_s:
        if time.monotonic() - last > 2.0:
            drive()
            last = time.monotonic()
        time.sleep(0.1)
    stats = parse_kv(call("imu_window")["reply"])
    call("stop")
    time.sleep(1.0)  # decelerate / settle between points
    return stats


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--modes", nargs="+", default=["STEALTHCHOP", "SPREADCYCLE"],
                    choices=["STEALTHCHOP", "SPREADCYCLE"])
    ap.add_argument("--speeds", nargs="+", type=float, default=[0, 0.5, 1, 2, 3],
                    help="wheel speeds in rev/s")
    ap.add_argument("--window", type=float, default=4.0, help="seconds per window")
    ap.add_argument("--accel", type=float, default=1000.0, help="mm/s² for spin-up")
    args = ap.parse_args()

    out = Path.home() / "runs" / (datetime.now().strftime("%Y-%m-%d_%H%M%S") + "_vibe")
    out.mkdir(parents=True, exist_ok=True)
    rows = []
    print(f"Writing {out}")
    hdr = (f"{'mode':<11} {'rev/s':>5} {'mm/s':>5} | {'gx sd':>6} {'gy sd':>6} {'gz sd':>6}"
           f" dps | {'ax sd':>5} {'ay sd':>5} {'az sd':>5} mg | {'|a| range g':>11}"
           f" | {'pitch pk-pk':>11} | rej loopmax")
    print(hdr)
    print("-" * len(hdr))
    try:
        for mode in args.modes:
            configure(mode, args.accel)
            for rps in args.speeds:
                mm_s = rps * math.pi * WHEEL_D_MM
                settle = 0.5 + mm_s / args.accel + 1.0
                s = run_point(mm_s, settle, args.window)
                row = {"mode": mode, "rev_s": rps, "mm_s": round(mm_s, 1),
                       **{k: s.get(k, "") for k in FIELDS}}
                rows.append(row)
                f = lambda k: float(s.get(k, "nan"))
                print(f"{mode:<11} {rps:5.1f} {mm_s:5.0f} | {f('GX_SD'):6.3f} {f('GY_SD'):6.3f}"
                      f" {f('GZ_SD'):6.3f}     | {1000*f('AX_SD'):5.1f} {1000*f('AY_SD'):5.1f}"
                      f" {1000*f('AZ_SD'):5.1f}    | {f('AMAG_MIN'):5.3f}-{f('AMAG_MAX'):5.3f}"
                      f" | {f('PITCH_MAX')-f('PITCH_MIN'):9.2f}°  | {s.get('ACC_REJECTS','?'):>3}"
                      f" {s.get('LOOP_MAX_US','?'):>5}")
                if s.get("ERRORS", "0") != "0" or s.get("STALLS", "0") != "0":
                    print(f"    !! IMU errors={s.get('ERRORS')} stalls={s.get('STALLS')}")
    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        try:
            configure("STEALTHCHOP", 150.0)
        except Exception as e:  # noqa: BLE001
            print(f"(could not restore defaults: {e})")
        with open(out / "data.csv", "w", newline="") as fh:
            if rows:
                w = csv.DictWriter(fh, fieldnames=list(rows[0]))
                w.writeheader()
                w.writerows(rows)
        (out / "meta.json").write_text(json.dumps(
            {"test": "vibe", "args": vars(args), "wheel_d_mm": WHEEL_D_MM,
             "current_ma": 1000, "microsteps": 16}, indent=2))
        print("Defaults restored (1000 mA, 1/16, 150 mm/s², StealthChop).")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
