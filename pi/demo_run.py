#!/usr/bin/env python3
"""Film sequence: a short, repeatable drive-and-turn demo, logged like a test.

  1. engage     lift the robot upright; it engages and soft-starts
  2. settle     1 s (the soft start takes 0.3 s); --countdown adds "rolling in 3, 2, 1"
  3. out        forward 400 mm at 200 mm/s
  4. turn 1     360° one way (60 °/s)
  5. back       reverse to where it actually started
  6. turn 2     360° the other way
  7. finish     2 s still, then it keeps balancing until you press Enter
                (so the clip doesn't end with it dropping onto a training wheel)

The moves are profiled by the firmware (MOVE / TURN while balancing, from
24 Nov): it ramps up to MOVE_V / TURN_W and brakes onto the target itself,
the distance on x_ref and the angle on the integrated IMU yaw rate. So the
Pi's round-trip delays no longer decide where it stops. The script only
waits for MOVING=0 between moves.

A position integral (KI, default 3 here, limited to KI_MAX 2.5 m/s²; firmware default 0 = off) pulls the
robot back to its reference after a stop or a turn, instead of leaving it
parked in the tyre contact band. MOVE_V, TURN_W and KI are set for this run
only and put back at the end.

    ssh -t robot@r2w-pi.local 'cd r2w && python3 demo_run.py'
    python3 demo_run.py --turn 180          # about-face: the return leg drives forwards
    python3 demo_run.py --ki 0              # without the position integral
    python3 demo_run.py --distance 300 --speed 150 --turn-rate 45

Needs firmware with MOVE / TURN while balancing (BAL_STATUS shows MOVE_V).
Leave 0.6 m clear in front, behind and around. Ctrl-C ends balancing at any
time (the robot drops onto a training wheel) and still saves the log.
"""
from __future__ import annotations

import argparse
import json
import select
import sys
import time

from balance_tools import DEFAULTS, RUNS_DIR, SAFETY_KEYS, balance_status, call


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
            time.sleep(0.05)

    def until_done(self, timeout: float) -> None:
        """Wait for the firmware's profiled move to finish (MOVING=0)."""
        time.sleep(0.05)
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            s = self.status()
            if int(float(s.get("MOVING", 0))) == 0:
                return
            print(f"\r   x {float(s['X']):+5.3f} m  heading {float(s['HEADING']):+7.1f}°   ",
                  end="", flush=True)
            time.sleep(0.05)
        self.mark("move_timeout")
        call("stop")

    def move(self, mm: float, speed: float) -> None:
        self.mark("move", mm=round(mm, 1))
        call("move", distance=mm)
        self.until_done(abs(mm) / speed + 6.0)

    def turn(self, deg: float, rate: float) -> float:
        h0 = float(self.status()["HEADING"])
        self.mark("turn", deg=deg)
        call("turn", angle=deg)
        self.until_done(abs(deg) / rate + 6.0)
        return float(self.status()["HEADING"]) - h0

    def snapshot(self, name: str, **extra) -> dict[str, str]:
        s = self.status()
        self.mark(name, x=float(s["X"]), x_ref=float(s["X_REF"]),
                  heading=float(s["HEADING"]), pitch_err=float(s["ERR"]),
                  xint=float(s.get("XINT", 0)), **extra)
        return s


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--distance", type=float, default=400.0, help="each leg, mm (<= 1000)")
    ap.add_argument("--speed", type=float, default=200.0, help="drive speed, mm/s (<= 300)")
    ap.add_argument("--turn", type=float, default=360.0,
                    help="each turn, degrees: 360 (return leg in reverse) or 180 (about-face)")
    ap.add_argument("--turn-rate", type=float, default=60.0, help="turn rate, °/s (<= 90)")
    ap.add_argument("--ki", type=float, default=3.0,
                    help="position integral for this run (0 = off; firmware default 0)")
    ap.add_argument("--pause", type=float, default=0.5, help="pause after each move, s")
    ap.add_argument("--start-hold", type=float, default=0.0, help="still before moving, s")
    ap.add_argument("--end-hold", type=float, default=2.0, help="still at the end, s")
    ap.add_argument("--settle", type=float, default=1.0,
                    help="after engaging, before the first move, s")
    ap.add_argument("--countdown", action="store_true",
                    help="print a 3 s countdown before the first move")
    ap.add_argument("--no-wait", action="store_true",
                    help="stop balancing right after the end hold instead of waiting for Enter")
    ap.add_argument("--set", nargs="*", default=[], metavar="KEY=value",
                    help="extra BAL_SET values for this run only (e.g. KV=10 KX=11.8)")
    ap.add_argument("--notes", default="")
    args = ap.parse_args()
    if not 20 <= args.speed <= 300:
        raise SystemExit("--speed must be 20-300 mm/s")
    if not 0 < args.distance <= 1000:
        raise SystemExit("--distance must be 0-1000 mm")
    if not 5 <= args.turn_rate <= 90:
        raise SystemExit("--turn-rate must be 5-90 °/s")
    if not 0 <= args.ki <= 20:
        raise SystemExit("--ki must be 0-20")
    turn_mod = args.turn % 360
    if turn_mod not in (0.0, 180.0) or args.turn <= 0:
        raise SystemExit("--turn must be a multiple of 180°")
    back_sign = -1 if turn_mod == 0 else +1     # after an about-face, drive forwards to return

    call("bal_stop")
    time.sleep(0.7)
    call("stop")
    time.sleep(0.3)
    base = balance_status()
    if "MOVE_V" not in base:
        raise SystemExit("This firmware has no MOVE/TURN while balancing: build and flash "
                         "the current firmware first.")
    wrong = {k: DEFAULTS[k] for k in SAFETY_KEYS if abs(float(base[k]) - DEFAULTS[k]) > 1e-3}
    if wrong:
        print("Resetting safety settings to defaults: "
              + ", ".join(f"{k} {base[k]} -> {v:g}" for k, v in wrong.items()))
        call("bal_set", set=wrong)
        base = balance_status()
    temp = {"MOVE_V": args.speed / 1000.0, "TURN_W": args.turn_rate, "KI": args.ki}
    for item in args.set:
        key, _, value = item.partition("=")
        if not key or not value:
            raise SystemExit(f"--set expects KEY=value, got {item!r}")
        temp[key.upper()] = float(value)
    restore = {k: float(base[k]) for k in temp if k in base}
    call("bal_set", set=temp)
    try:
        return run(args, base, back_sign)
    finally:
        try:
            call("bal_stop")
            time.sleep(0.7)
            call("bal_set", set=restore)
        except Exception as error:  # noqa: BLE001
            print(f"WARNING: could not restore settings ({error}); check with "
                  "check_config.py or power-cycle the SKR")


def run(args, base: dict[str, str], back_sign: int) -> int:
    total = (args.settle + (3 if args.countdown else 0) + args.start_hold
             + 2 * (args.distance / args.speed + 1.5 + args.turn / args.turn_rate + 1.0)
             + 4 * args.pause + args.end_hold)
    print(f"Gains KP {base['KP']} KD {base['KD']} KV {base['KV']} KX {base['KX']}, KI {args.ki:g}. "
          f"Sequence about {total:.0f} s after engaging: {args.distance:.0f} mm at "
          f"{args.speed:.0f} mm/s, {args.turn:.0f}° at {args.turn_rate:.0f} °/s, and back.")
    print("Leave 0.6 m clear in front, behind and around.")
    input("Kill switch in reach? Press Enter to arm, then lift it upright...")

    reply = call("log_start", name="demo", div=2, duration_s=total + 120,
                 notes=f"demo_run.py ki={args.ki:g} {args.notes}".strip())
    run_name = reply["run"]
    d = Demo(time.monotonic())
    call("bal_arm")
    d.mark("armed")
    result = "complete"
    turned: dict[str, float] = {}
    try:
        deadline = time.monotonic() + 25
        while balance_status()["BAL"] != "RUNNING":
            if time.monotonic() > deadline:
                raise Aborted("not engaged")
            time.sleep(0.05)
        d.engaged = True
        d.mark("engaged")
        d.wait(args.settle)
        if args.countdown:
            for n in (3, 2, 1):
                print(f"\n   rolling in {n}...", flush=True)
                d.wait(1.0)
        start = d.snapshot("start")
        x_start = float(start["X"])
        d.wait(args.start_hold)

        d.move(args.distance, args.speed)
        d.wait(args.pause)
        d.snapshot("out_done")

        turned["turn1"] = d.turn(args.turn, args.turn_rate)
        d.wait(args.pause)
        s = d.snapshot("turn1_done", turned=turned["turn1"])

        # Back to where the robot actually started (by X), whatever it crept in
        # the turn. After an about-face, driving forwards (X increasing) returns it.
        d.move(back_sign * (float(s["X"]) - x_start) * 1000.0, args.speed)
        d.wait(args.pause)
        d.snapshot("back_done")

        turned["turn2"] = d.turn(-args.turn, args.turn_rate)
        d.wait(args.end_hold)
        d.snapshot("finish", turned=turned["turn2"])
        print("\n\nSequence done. Cut!")
        if not args.no_wait:
            print("Still balancing. Catch it, then press Enter to stop (Ctrl-C works too).")
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
        x0, r0 = snaps["start"]["x"], snaps["start"]["x_ref"]
        print("\nWhere it ended up (wheel travel relative to the start):")
        for name in ("out_done", "turn1_done", "back_done", "finish"):
            if name in snaps:
                e = snaps[name]
                t = f", turned {e['turned']:+.0f}°" if e.get("turned") is not None else ""
                print(f"  {name:11s} x {1000 * (e['x'] - x0):+5.0f} mm "
                      f"(ref {1000 * (e['x_ref'] - r0):+5.0f}){t}")
    print(f"\nRun {run_name}: {result}. Saved demo.json"
          + (f"; black box {bb}" if bb else "") + ".")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
