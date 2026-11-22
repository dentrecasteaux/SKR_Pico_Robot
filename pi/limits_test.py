#!/usr/bin/env python3
"""Stepper speed and acceleration limits on the stand (Phase 7.1).

Robot ON THE STAND, wheels clear. Without encoders the firmware can't see a
stall, so you watch and listen. After each step the script asks:
  Enter = ran cleanly,  s = stalled / stuttered / buzzed,  q = quit.

Part A, top speed: ramp to 400, 600, 800 and 1000 mm/s (the firmware limit) at
2 m/s², hold 2 s, ramp down.
Part B, reversals: ±300 mm/s square wave at 2, 5, 10 and 20 m/s² (the
firmware ceiling), 3 reversals each.

Both parts run in StealthChop, then SpreadCycle. Every step is logged
(~/runs/<time>_lim_...), and the answers go into
~/runs/<time>_limits_summary.json.

    ssh -t robot@r2w-pi.local 'cd r2w && python3 limits_test.py'
    python3 limits_test.py --modes SPREADCYCLE --part B
"""
from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

from robotd import default_socket_path
from robotdctl import request

SOCK = default_socket_path()
RUNS = Path.home() / "runs"
SPEEDS = [400, 600, 800, 1000]          # mm/s
ACCELS = [2000, 5000, 10000, 20000]     # mm/s²
REV_SPEED = 300                         # mm/s


def call(action: str, **kw) -> dict:
    reply = request(SOCK, {"action": action, **kw})
    if not reply.get("ok"):
        raise RuntimeError(f"{action}: {reply.get('error', reply)}")
    return reply


def configure(mode: str, accel: float) -> None:
    call("stop")
    time.sleep(0.8)
    call("configure", current_ma=1000, microsteps=16,
         acceleration_mm_s2=accel, tmc_mode=mode)


def drive(v: float, seconds: float) -> None:
    """Hold velocity v for `seconds`, renewing the lease."""
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        call("velocity", linear=v, angular=0.0, lease_ms=500, hold_ms=600)
        time.sleep(0.1)


def ask(prompt: str) -> str:
    while True:
        a = input(f"   {prompt}  [Enter=clean, s=stalled, q=quit] ").strip().lower()
        if a in ("", "s", "q"):
            return {"": "clean", "s": "stalled", "q": "quit"}[a]


def step(name: str, action, prompt: str) -> str:
    run = call("log_start", name=name, div=2, notes=prompt)["run"]
    try:
        action()
    finally:
        call("stop")
        time.sleep(0.5)
        call("log_stop")
    result = ask(prompt)
    return run, result


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--modes", nargs="+", default=["STEALTHCHOP", "SPREADCYCLE"],
                    choices=["STEALTHCHOP", "SPREADCYCLE"])
    ap.add_argument("--part", choices=["A", "B", "AB"], default="AB")
    args = ap.parse_args()

    print(__doc__.split("Both parts")[0])
    input("Robot on the stand, wheels clear, kill switch in reach? Press Enter to start.")
    results = []
    try:
        for mode in args.modes:
            print(f"\n=== {mode} ===")
            if "A" in args.part:
                configure(mode, 2000)
                for v in SPEEDS:
                    print(f"A  {mode}: ramp to {v} mm/s ({v / 195.8:.1f} rev/s), hold 2 s")
                    ramp = v / 2000 + 0.3
                    run, res = step(f"lim_A_{mode[:6].lower()}_{v}",
                                    lambda v=v, ramp=ramp: drive(v, ramp + 2.0),
                                    f"{mode} {v} mm/s")
                    results.append({"part": "A", "mode": mode, "speed_mm_s": v,
                                    "result": res, "run": run})
                    if res == "quit":
                        raise KeyboardInterrupt
                    if res == "stalled":
                        print("   (skipping higher speeds in this mode)")
                        break
            if "B" in args.part:
                for a in ACCELS:
                    configure(mode, a)
                    flip = 2 * REV_SPEED / a + 0.25
                    print(f"B  {mode}: ±{REV_SPEED} mm/s reversals at {a / 1000:g} m/s²")

                    def reversals(flip=flip):
                        for _ in range(3):
                            drive(REV_SPEED, flip)
                            drive(-REV_SPEED, flip)

                    run, res = step(f"lim_B_{mode[:6].lower()}_{a}", reversals,
                                    f"{mode} reversals {a / 1000:g} m/s²")
                    results.append({"part": "B", "mode": mode, "accel_mm_s2": a,
                                    "result": res, "run": run})
                    if res == "quit":
                        raise KeyboardInterrupt
                    if res == "stalled":
                        print("   (skipping higher accelerations in this mode)")
                        break
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        try:
            configure("STEALTHCHOP", 150)
        except Exception as e:  # noqa: BLE001
            print(f"(could not restore defaults: {e})")

    print("\nSummary")
    for r in results:
        what = (f"{r['speed_mm_s']} mm/s" if r["part"] == "A"
                else f"{r['accel_mm_s2'] / 1000:g} m/s²")
        print(f"  {r['part']}  {r['mode']:<11} {what:>10}  {r['result']}")
    out = RUNS / f"{time.strftime('%Y-%m-%d_%H%M%S')}_limits_summary.json"
    out.write_text(json.dumps(results, indent=2))
    print(f"Saved {out}\nDefaults restored (1000 mA, 1/16, 150 mm/s², StealthChop).")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
