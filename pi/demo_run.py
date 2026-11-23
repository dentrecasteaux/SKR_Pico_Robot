#!/usr/bin/env python3
"""Film sequence: a short, repeatable drive-and-turn demo, logged like a test.

  1. engage     lift the robot upright; it engages and soft-starts
  2. settle     3 s, then a countdown ("rolling in 3, 2, 1") so you can start filming
  3. balance    1 s still
  4. out        forward 400 mm at 200 mm/s, then a short pause
  5. turn 1     360° one way (60 °/s), then a short pause
  6. back       reverse 400 mm at 200 mm/s, to the start point, then a pause
  7. turn 2     360° the other way, then a pause
  8. finish     2 s still, then it keeps balancing until you press Enter
                (so the clip doesn't end with it dropping onto a training wheel)

Distances and angles are timed from the firmware's own ramps. The speed and
turn ramps are symmetric, so a command held for T seconds gives exactly
v·T of reference travel (or w·T of reference turn), wherever the ramps are.
The robot follows the reference with its usual lag and stop overshoot. The
log, plus demo.json with the event times, the position (X, X_REF) and the
IMU yaw after each move, is saved like any other run.

    ssh -t robot@r2w-pi.local 'cd r2w && python3 demo_run.py'
    python3 demo_run.py --turn 180          # about-face: the return leg drives forwards
    python3 demo_run.py --distance 300 --speed 150 --turn-rate 45

It uses whatever controller settings are live (the everyday defaults).
Leave 0.6 m clear in front, behind and around. Ctrl-C ends balancing at
any time (the robot drops onto a training wheel) and still saves the log.
"""
from __future__ import annotations

import argparse
import json
import time

from balance_tools import DEFAULTS, RUNS_DIR, SAFETY_KEYS, balance_status, call

LEASE_MS, HOLD_MS, RENEW_S = 1000, 1500, 0.2


class Aborted(Exception):
    pass


class Demo:
    def __init__(self, t_log0: float):
        self.t_log0 = t_log0
        self.events: list[dict] = []
        self.engaged = False

    def t_log(self) -> float:
        return round(time.monotonic() - self.t_log0, 3)

    def mark(self, name: str, **extra) -> None:
        self.events.append({"t": self.t_log(), "event": name, **extra})
        print(f"\n[{self.t_log():6.1f} s] {name} {extra if extra else ''}")

    def status(self) -> dict[str, str]:
        s = balance_status()
        if self.engaged and s["BAL"] in ("TRIPPED", "OFF"):
            self.mark("ended", reason=s["REASON"])
            raise Aborted(s["REASON"])
        return s

    def wait(self, seconds: float) -> None:
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            s = self.status()
            print(f"\r   err {float(s['ERR']):+6.2f}°  v {float(s['V']):+5.2f} m/s  "
                  f"x {float(s['X']):+5.3f} m   ", end="", flush=True)
            time.sleep(0.1)

    def command(self, linear_mm_s: float, angular_dps: float, seconds: float) -> None:
        """Hold a drive command for exactly `seconds`, then stop (keep balancing)."""
        t0 = time.monotonic()
        end = t0 + seconds
        next_renew = t0
        while True:
            now = time.monotonic()
            if now >= end:
                break
            if now >= next_renew:
                call("velocity", linear=linear_mm_s, angular=angular_dps,
                     lease_ms=LEASE_MS, hold_ms=HOLD_MS)
                next_renew = now + RENEW_S
                self.status()
            time.sleep(max(0.0, min(next_renew, end) - time.monotonic()))
        call("stop")              # balancing: zero drive, keep balancing

    def snapshot(self, name: str) -> None:
        s = self.status()
        yaw = None
        try:
            yaw = float(call("imu")["service"]["imu"].get("YAW"))
        except (RuntimeError, TypeError, ValueError, KeyError):
            pass
        self.mark(name, x=float(s["X"]), x_ref=float(s["X_REF"]),
                  pitch_err=float(s["ERR"]), imu_yaw=yaw)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--distance", type=float, default=400.0, help="each leg, mm (<= 1000)")
    ap.add_argument("--speed", type=float, default=200.0, help="drive speed, mm/s (<= 300)")
    ap.add_argument("--turn", type=float, default=360.0,
                    help="each turn, degrees: 360 (return leg in reverse) or 180 (about-face)")
    ap.add_argument("--turn-rate", type=float, default=60.0, help="turn rate, °/s (<= 90)")
    ap.add_argument("--pause", type=float, default=1.5, help="pause after each move, s")
    ap.add_argument("--start-hold", type=float, default=1.0, help="still before moving, s")
    ap.add_argument("--end-hold", type=float, default=2.0, help="still at the end, s")
    ap.add_argument("--settle", type=float, default=3.0, help="after engaging, before the countdown, s")
    ap.add_argument("--no-wait", action="store_true",
                    help="stop balancing right after the end hold instead of waiting for Enter")
    ap.add_argument("--notes", default="")
    args = ap.parse_args()
    if not 0 < args.speed <= 300:
        raise SystemExit("--speed must be 0-300 mm/s")
    if not 0 < args.distance <= 1000:
        raise SystemExit("--distance must be 0-1000 mm")
    if not 0 < args.turn_rate <= 90:
        raise SystemExit("--turn-rate must be 0-90 °/s")
    turn_mod = args.turn % 360
    if turn_mod not in (0.0, 180.0) or args.turn <= 0:
        raise SystemExit("--turn must be a multiple of 180°")
    back_sign = -1 if turn_mod == 0 else +1     # after an about-face, drive forwards to return

    call("bal_stop")
    time.sleep(0.7)
    call("stop")
    time.sleep(0.3)
    base = balance_status()
    wrong = {k: DEFAULTS[k] for k in SAFETY_KEYS if abs(float(base[k]) - DEFAULTS[k]) > 1e-3}
    if wrong:
        print("Resetting safety settings to defaults: "
              + ", ".join(f"{k} {base[k]} -> {v:g}" for k, v in wrong.items()))
        call("bal_set", set=wrong)
        base = balance_status()

    drive_s = args.distance / args.speed
    turn_s = args.turn / args.turn_rate
    total = (args.settle + 3 + args.start_hold + 2 * (drive_s + turn_s) + 4 * args.pause
             + args.end_hold + 2)
    print(f"Gains KP {base['KP']} KD {base['KD']} KV {base['KV']} KX {base['KX']}. "
          f"Sequence about {total:.0f} s after engaging: {args.distance:.0f} mm at "
          f"{args.speed:.0f} mm/s, {args.turn:.0f}° at {args.turn_rate:.0f} °/s, and back.")
    print("Leave 0.6 m clear in front, behind and around.")
    input("Kill switch in reach? Press Enter to arm, then lift it upright...")

    reply = call("log_start", name="demo", div=2, duration_s=total + 120,
                 notes=f"demo_run.py {args.notes}".strip())
    run_name = reply["run"]
    d = Demo(time.monotonic())
    call("bal_arm")
    d.mark("armed")
    result = "complete"
    try:
        deadline = time.monotonic() + 25
        while balance_status()["BAL"] != "RUNNING":
            if time.monotonic() > deadline:
                raise Aborted("not engaged")
            time.sleep(0.05)
        d.engaged = True
        d.mark("engaged")
        d.wait(args.settle)
        try:
            call("imu_zero_yaw")
        except RuntimeError:
            pass
        for n in (3, 2, 1):
            print(f"\n   rolling in {n}...", flush=True)
            d.wait(1.0)
        d.snapshot("start")
        d.wait(args.start_hold)

        d.mark("out", mm=args.distance, mm_s=args.speed)
        d.command(args.speed, 0.0, drive_s)
        d.wait(args.pause)
        d.snapshot("out_done")

        d.mark("turn1", deg=args.turn, dps=args.turn_rate)
        d.command(0.0, args.turn_rate, turn_s)
        d.wait(args.pause)
        d.snapshot("turn1_done")

        d.mark("back", mm=args.distance, mm_s=back_sign * args.speed)
        d.command(back_sign * args.speed, 0.0, drive_s)
        d.wait(args.pause)
        d.snapshot("back_done")

        d.mark("turn2", deg=-args.turn, dps=-args.turn_rate)
        d.command(0.0, -args.turn_rate, turn_s)
        d.wait(args.end_hold)
        d.snapshot("finish")
        print("\n\nSequence done. Cut!")
        if not args.no_wait:
            print("Still balancing. Catch it, then press Enter to stop (Ctrl-C works too).")
            import select
            import sys
            while True:                       # keep the trip check running while waiting
                if select.select([sys.stdin], [], [], 0.2)[0]:
                    sys.stdin.readline()
                    break
                d.status()
    except Aborted as reason:
        result = f"ended early: {reason}"
    except KeyboardInterrupt:
        result = "stopped by Ctrl-C"
        d.mark("ctrl_c")
    finally:
        call("stop")
        call("bal_stop")
        time.sleep(0.7)
        call("log_stop")

    bb = None
    try:
        if call("log_status")["service"]["log"]["firmware"].get("BB") == "FROZEN":
            bb = call("bb_dump")["run"]
    except RuntimeError:
        pass
    final = balance_status()
    record = {"kind": "demo", "result": result, "notes": args.notes,
              "args": vars(args), "settings": base, "events": d.events, "blackbox": bb,
              "final": {k: final.get(k) for k in ("REASON", "MAX_ERR", "MAX_U", "TRIPS")}}
    (RUNS_DIR / run_name / "demo.json").write_text(json.dumps(record, indent=2))

    snaps = {e["event"]: e for e in d.events if "x" in e}
    if "start" in snaps:
        x0, y0 = snaps["start"]["x"], snaps["start"].get("imu_yaw")
        print("\nWhere it ended up (relative to the start):")
        for name in ("out_done", "turn1_done", "back_done", "finish"):
            if name in snaps:
                e = snaps[name]
                yaw = (f", IMU yaw {e['imu_yaw'] - y0:+.0f}°"
                       if e.get("imu_yaw") is not None and y0 is not None else "")
                print(f"  {name:11s} x {1000 * (e['x'] - x0):+5.0f} mm "
                      f"(ref {1000 * (e['x_ref'] - snaps['start']['x_ref']):+5.0f}){yaw}")
    print(f"\nRun {run_name}: {result}. Saved demo.json"
          + (f"; black box {bb}" if bb else "") + ".")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
