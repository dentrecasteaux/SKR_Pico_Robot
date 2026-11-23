#!/usr/bin/env python3
"""Film sequence: a short, repeatable drive-and-turn demo, logged like a test.

  1. engage     lift the robot upright; it engages and soft-starts
  2. settle     3 s, then a countdown ("rolling in 3, 2, 1") so you can start filming
  3. go         straight into the first move (--start-hold to add a pause)
  4. out        forward 400 mm at 200 mm/s, then a short pause
  5. turn 1     360° one way (60 °/s), then a short pause
  6. back       reverse 400 mm at 200 mm/s, to the start point, then a pause
  7. turn 2     360° the other way, then a pause
  8. finish     2 s still, then it keeps balancing until you press Enter
                (so the clip doesn't end with it dropping onto a training wheel)

Each move is closed on a measurement, not timed:
  - legs: the drive is stopped when the reference position X_REF is one
    stopping distance (simulated from the firmware's own S-curve ramp) short
    of the target. The return leg aims for where the robot actually started
    (X), so any creep during the turn is taken out on the way back.
  - turns: the turn is stopped from the IMU yaw (the real body rotation, so
    wheel slip and track-width error don't matter), one ramp-down angle
    (w²/2·180 °/s²) short of the target.
After each stop it checks that V_REF and W_REF really reach zero, and sends
STOP again if not. The log, plus demo.json with the event times, position
(X, X_REF) and turned angle after each move, is saved like any other run.

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
# Firmware ramp constants (src/Balance.cpp)
DRIVE_JERK, DRIVE_APPROACH, TURN_ACCEL_DPS2, FW_DT = 4.0, 4.0, 180.0, 0.002
LATENCY_S = 0.03       # status poll + STOP round trip


def stop_distance(v: float, drive_accel: float) -> float:
    """Reference travel (m) after STOP from cruise at v (m/s), as the firmware ramps it."""
    v_ref, a_ref, x = abs(v), 0.0, 0.0
    for _ in range(int(10 / FW_DT)):
        a_want = max(-drive_accel, min(drive_accel, DRIVE_APPROACH * (0.0 - v_ref)))
        dj = DRIVE_JERK * FW_DT
        a_ref += max(-dj, min(dj, a_want - a_ref))
        v_ref += a_ref * FW_DT
        x += v_ref * FW_DT
        if v_ref <= 1e-4 and abs(a_ref) < 1e-3:
            break
    return x


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

    def renew(self, linear_mm_s: float, angular_dps: float) -> None:
        call("velocity", linear=linear_mm_s, angular=angular_dps,
             lease_ms=LEASE_MS, hold_ms=HOLD_MS)

    def stop_and_confirm(self) -> None:
        """STOP, then make sure the drive set points really ramp to zero."""
        call("stop")              # balancing: zero drive, keep balancing
        end = time.monotonic() + 3.0
        resent = False
        while time.monotonic() < end:
            s = self.status()
            if abs(float(s["V_REF"])) < 1e-3 and abs(float(s["W_REF"])) < 0.5:
                return
            if not resent and time.monotonic() > end - 2.0:
                call("stop")
                resent = True
                self.mark("stop_resent", v_ref=float(s["V_REF"]), w_ref=float(s["W_REF"]))
            time.sleep(0.05)
        self.mark("stop_not_confirmed")

    def leg(self, target_ref_m: float, speed_mm_s: float, drive_accel: float) -> None:
        """Drive until X_REF reaches target_ref_m (stopping on the ramp)."""
        x0 = float(self.status()["X_REF"])
        sign = 1.0 if target_ref_m > x0 else -1.0
        v = sign * speed_mm_s / 1000.0
        brake = stop_distance(v, drive_accel) + abs(v) * LATENCY_S
        t_end = time.monotonic() + abs(target_ref_m - x0) / abs(v) + 3.0   # safety timeout
        next_renew = 0.0
        while time.monotonic() < t_end:
            now = time.monotonic()
            if now >= next_renew:
                self.renew(v * 1000.0, 0.0)
                next_renew = now + RENEW_S
            remaining = sign * (target_ref_m - float(self.status()["X_REF"]))
            if remaining <= brake:
                break
            time.sleep(0.01)
        else:
            self.mark("leg_timeout")
        self.stop_and_confirm()

    def yaw(self) -> float | None:
        try:
            return float(call("imu")["service"]["imu"].get("YAW"))
        except (RuntimeError, TypeError, ValueError, KeyError):
            return None

    def turn(self, degrees: float, rate_dps: float) -> float | None:
        """Turn `degrees` (sign = direction) by the IMU yaw; returns the angle turned."""
        w = rate_dps if degrees > 0 else -rate_dps
        brake = w * w / (2 * TURN_ACCEL_DPS2) + abs(w) * LATENCY_S
        last = self.yaw()
        turned = 0.0
        t_end = time.monotonic() + abs(degrees) / rate_dps * 1.5 + 2.0
        next_renew, n = 0.0, 0
        if last is None:          # no IMU yaw: fall back to timing (symmetric ramps)
            self.mark("turn_timed_no_imu")
            t_stop = time.monotonic() + abs(degrees) / rate_dps
            while time.monotonic() < t_stop:
                self.renew(0.0, w)
                self.status()
                time.sleep(min(RENEW_S, max(0.0, t_stop - time.monotonic())))
            self.stop_and_confirm()
            return None
        while time.monotonic() < t_end:
            now = time.monotonic()
            if now >= next_renew:
                self.renew(0.0, w)
                next_renew = now + RENEW_S
            y = self.yaw()
            if y is not None:
                d = (y - last + 180.0) % 360.0 - 180.0      # unwrap
                turned += d
                last = y
            n += 1
            if n % 5 == 0:
                self.status()     # trip check
            if abs(turned) >= abs(degrees) - brake:
                break
            time.sleep(0.005)
        else:
            self.mark("turn_timeout")
        self.stop_and_confirm()
        # add the ramp-down actually turned
        end = time.monotonic() + 0.3
        while time.monotonic() < end:
            y = self.yaw()
            if y is not None:
                turned += (y - last + 180.0) % 360.0 - 180.0
                last = y
            time.sleep(0.02)
        return turned

    def snapshot(self, name: str, **extra) -> dict[str, str]:
        s = self.status()
        self.mark(name, x=float(s["X"]), x_ref=float(s["X_REF"]),
                  pitch_err=float(s["ERR"]), imu_yaw=self.yaw(), **extra)
        return s


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--distance", type=float, default=400.0, help="each leg, mm (<= 1000)")
    ap.add_argument("--speed", type=float, default=200.0, help="drive speed, mm/s (<= 300)")
    ap.add_argument("--turn", type=float, default=360.0,
                    help="each turn, degrees: 360 (return leg in reverse) or 180 (about-face)")
    ap.add_argument("--turn-rate", type=float, default=60.0, help="turn rate, °/s (<= 90)")
    ap.add_argument("--pause", type=float, default=1.5, help="pause after each move, s")
    ap.add_argument("--start-hold", type=float, default=0.0, help="still before moving, s")
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
    drive_accel = float(base.get("DRIVE_ACCEL", 0.8))
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
        start = d.snapshot("start")
        x_start, ref_start = float(start["X"]), float(start["X_REF"])
        d.wait(args.start_hold)

        d.mark("out", mm=args.distance, mm_s=args.speed)
        d.leg(ref_start + args.distance / 1000.0, args.speed, drive_accel)
        d.wait(args.pause)
        d.snapshot("out_done")

        d.mark("turn1", deg=args.turn, dps=args.turn_rate)
        t1 = d.turn(args.turn, args.turn_rate)
        d.wait(args.pause)
        s = d.snapshot("turn1_done", turned=t1)

        # Back to where the robot actually started (by X), whatever it crept in
        # the turn. After an about-face, driving forwards (X increasing) returns it.
        x_now, ref_now = float(s["X"]), float(s["X_REF"])
        target = ref_now + back_sign * (x_now - x_start)
        d.mark("back", mm_s=back_sign * args.speed, ref_target=round(target, 3))
        d.leg(target, args.speed, drive_accel)
        d.wait(args.pause)
        d.snapshot("back_done")

        d.mark("turn2", deg=-args.turn, dps=-args.turn_rate)
        t2 = d.turn(-args.turn, args.turn_rate)
        d.wait(args.end_hold)
        d.snapshot("finish", turned=t2)
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
        x0, r0 = snaps["start"]["x"], snaps["start"]["x_ref"]
        print("\nWhere it ended up (wheel travel relative to the start):")
        for name in ("out_done", "turn1_done", "back_done", "finish"):
            if name in snaps:
                e = snaps[name]
                turned = f", turned {e['turned']:+.0f}°" if e.get("turned") is not None else ""
                print(f"  {name:11s} x {1000 * (e['x'] - x0):+5.0f} mm "
                      f"(ref {1000 * (e['x_ref'] - r0):+5.0f}){turned}")
    print(f"\nRun {run_name}: {result}. Saved demo.json"
          + (f"; black box {bb}" if bb else "") + ".")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
