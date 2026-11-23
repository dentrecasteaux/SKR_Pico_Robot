#!/usr/bin/env python3
"""Phase 11 comparison protocol: one logged run per controller set.

The same test sequence for every controller, so runs compare directly:

  1. settle     10 s after engaging (nothing applied)
  2. stand      60 s quiet standing                   -> RMS angle, rate, u, drift
  3. kicks      10 standard kicks, ±2 m/s², 40 ms     -> ring frequency, damping
  4. pushes     4 large kicks, ±3 m/s², 60 ms         -> peak angle, distance, recovery
  5. steps      speed steps +V 5 s, 0 10 s, −V 5 s, 0 10 s
                                                      -> tracking, overshoot, settling,
                                                         final position error
  6. end        balancing stops, the log and black box are saved

Every setting that matters is written to the run's compare.json at the
start (BAL_STATUS, IMU, STATUS, TMC PWM), together with the time of each
segment in the log's own clock. analysis/compare.py computes the metrics.

    ssh -t robot@r2w-pi.local 'cd r2w && python3 compare_run.py --label everyday'
    python3 compare_run.py --label lqr --set KP=73.5 KD=4.63 KV=10 KX=11.8
    python3 compare_run.py --label lqr-tau2 --set KP=73.5 KD=4.63 KV=10 KX=11.8 \
        ACCEL_COMP=1 NOTCH_HZ=0 --tau 2.0

--set and --tau changes last for this run only: they are put back at the end.
The speed step defaults to 150 mm/s (--speed; 100 is a good common value).
Each step drives for 5 s, so at 100 mm/s it goes about 0.5 m out and back:
leave 1 m clear in front and behind. Ctrl-C at any time ends balancing (the robot drops onto a training
wheel) and still saves everything recorded so far.
"""
from __future__ import annotations

import argparse
import json
import time

from balance_tools import DEFAULTS, RUNS_DIR, SAFETY_KEYS, balance_status, call

SETTLE_S = 10.0
STAND_S = 60.0
KICKS = 10
KICK_U, KICK_MS, KICK_GAP_S = 2.0, 40, 3.0
PUSHES = 4
PUSH_U, PUSH_MS, PUSH_GAP_S = 3.0, 60, 5.0
STEP_ON_S, STEP_OFF_S = 5.0, 10.0   # long enough to reach speed and settle
STEP_HOLD_MS, STEP_LEASE_MS, STEP_RENEW_S = 1500, 1000, 0.2


class Aborted(Exception):
    pass


def parse_set(items: list[str]) -> dict[str, float]:
    out = {}
    for item in items:
        key, _, value = item.partition("=")
        if not key or not value:
            raise SystemExit(f"--set expects KEY=value, got {item!r}")
        out[key.upper()] = float(value)
    return out


class Session:
    def __init__(self, t_log0: float):
        self.t_log0 = t_log0          # monotonic time of LOG_START
        self.events: list[dict] = []
        self.engaged_at: float | None = None

    def t_log(self) -> float:
        return round(time.monotonic() - self.t_log0, 3)

    def mark(self, name: str, **extra) -> None:
        self.events.append({"t": self.t_log(), "event": name, **extra})
        print(f"\n[{self.t_log():6.1f} s] {name} {extra if extra else ''}")

    def status(self) -> dict[str, str]:
        s = balance_status()
        if s["BAL"] in ("TRIPPED", "OFF") and self.engaged_at is not None:
            self.mark("ended", reason=s["REASON"])
            raise Aborted(s["REASON"])
        return s

    def wait(self, seconds: float) -> None:
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            s = self.status()
            print(f"\r   err {float(s['ERR']):+6.2f}°  v {float(s['V']):+5.2f} m/s  "
                  f"x {float(s['X']):+5.2f} m   ", end="", flush=True)
            time.sleep(0.1)

    def wait_calm(self, timeout: float = 6.0) -> None:
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            s = self.status()
            if abs(float(s["RATE"])) < 10 and abs(float(s["V"])) < 0.1:
                return
            time.sleep(0.05)

    def kicks(self, label: str, n: int, u: float, ms: int, gap: float) -> None:
        self.mark(f"{label}_start", n=n, u=u, ms=ms)
        for i in range(n):
            self.wait_calm()
            sign = 1 if i % 2 == 0 else -1
            call("bal_kick", u=sign * u, ms=ms)
            self.events.append({"t": self.t_log(), "event": label, "u": sign * u, "ms": ms})
            print(f"\n   {label} {i + 1}/{n} ({sign * u:+g} m/s², {ms} ms)")
            self.wait(gap)
        self.mark(f"{label}_end")

    def drive(self, v_mm_s: float, seconds: float) -> None:
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            call("velocity", linear=v_mm_s, angular=0.0,
                 lease_ms=STEP_LEASE_MS, hold_ms=STEP_HOLD_MS)
            self.status()
            time.sleep(STEP_RENEW_S)

    def steps(self, speed: float) -> None:
        self.mark("steps_start", speed_mm_s=speed)
        for v in (speed, 0.0, -speed, 0.0):
            self.events.append({"t": self.t_log(), "event": "step", "v_mm_s": v})
            print(f"\n   step to {v:+.0f} mm/s")
            if v:
                self.drive(v, STEP_ON_S)
            else:
                call("stop")          # balancing: zero drive, keep balancing
                self.wait(STEP_OFF_S)
        self.mark("steps_end")


def snapshot() -> dict:
    out = {"balance": balance_status()}
    for action in ("status", "imu", "tmc_pwm", "tuning"):
        try:
            reply = call(action)
            out[action] = {k: reply[k] for k in ("service", "pwm", "saved", "reply")
                           if k in reply}
        except RuntimeError as error:
            out[action] = {"error": str(error)}
    return out


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--label", required=True, help="short name for this controller set")
    ap.add_argument("--set", nargs="*", default=[], metavar="KEY=value",
                    help="BAL_SET values for this run only")
    ap.add_argument("--tau", type=float, help="IMU filter time constant for this run only")
    ap.add_argument("--speed", type=float, default=150.0, help="speed step, mm/s (<= 300)")
    ap.add_argument("--no-steps", action="store_true", help="skip the speed steps")
    ap.add_argument("--push-u", type=float, default=PUSH_U, help="push size, m/s^2 (<= 4)")
    ap.add_argument("--push-ms", type=int, default=PUSH_MS, help="push length, ms (<= 200)")
    ap.add_argument("--notes", default="")
    args = ap.parse_args()
    temp = parse_set(args.set)
    if not 0 < args.speed <= 300:
        raise SystemExit("--speed must be 0-300 mm/s")

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
    restore = {k: float(base[k]) for k in temp if k in base}
    tau_before = None
    try:
        tau_before = float(call("imu")["service"]["imu"].get("TAU"))
    except (RuntimeError, TypeError, ValueError):
        pass
    try:
        if temp:
            call("bal_set", set=temp)
        if args.tau is not None:
            call("imu_set", tau=args.tau)
            time.sleep(0.3)
        return run(args)
    finally:
        try:
            call("bal_stop")
            time.sleep(0.7)
            if restore:
                call("bal_set", set=restore)
            if args.tau is not None and tau_before is not None:
                call("imu_set", tau=tau_before)
        except Exception as error:  # noqa: BLE001
            print(f"WARNING: could not restore settings ({error}); check with "
                  "check_config.py or power-cycle the SKR")


def run(args) -> int:
    settings = snapshot()
    b = settings["balance"]
    print(f"Controller '{args.label}': KP {b['KP']} KD {b['KD']} KV {b['KV']} KX {b['KX']} "
          f"ACCEL_COMP {b.get('ACCEL_COMP')} NOTCH {b.get('NOTCH_HZ')}/{b.get('NOTCH_Q')} "
          f"TAU {settings.get('imu', {}).get('service', {}).get('imu', {}).get('TAU')}")
    total = (SETTLE_S + STAND_S + KICKS * 4 + PUSHES * 6
             + (0 if args.no_steps else 2 * (STEP_ON_S + STEP_OFF_S)) + 30)
    print(f"About {total / 60:.1f} min. Leave 1 m clear in front and behind.")
    input("Kill switch in reach? Press Enter to arm, then lift it upright...")

    reply = call("log_start", name=f"compare_{args.label}", div=2, duration_s=total + 60,
                 notes=f"compare_run.py {args.label} {args.notes}".strip())
    s = Session(time.monotonic())
    run_name = reply["run"]
    call("bal_arm")
    s.mark("armed")
    result = "complete"
    try:
        deadline = time.monotonic() + 25
        while balance_status()["BAL"] != "RUNNING":
            if time.monotonic() > deadline:
                raise Aborted("not engaged")
            time.sleep(0.05)
        s.engaged_at = time.monotonic()
        s.mark("engaged")
        s.wait(SETTLE_S)
        s.mark("stand_start")
        s.wait(STAND_S)
        s.mark("stand_end")
        s.kicks("kick", KICKS, KICK_U, KICK_MS, KICK_GAP_S)
        s.kicks("push", PUSHES, args.push_u, args.push_ms, PUSH_GAP_S)
        if not args.no_steps:
            s.steps(args.speed)
        s.wait(2.0)
    except Aborted as reason:
        result = f"ended early: {reason}"
    except KeyboardInterrupt:
        result = "stopped by Ctrl-C"
        s.mark("ctrl_c")
    finally:
        call("bal_stop")
        time.sleep(0.7)
        call("log_stop")
    final = balance_status()
    bb = None
    try:
        if call("log_status")["service"]["log"]["firmware"].get("BB") == "FROZEN":
            bb = call("bb_dump")["run"]
    except RuntimeError:
        pass
    record = {"label": args.label, "notes": args.notes, "result": result,
              "speed_mm_s": None if args.no_steps else args.speed,
              "temp_settings": args.set, "tau": args.tau,
              "settings": settings, "events": s.events, "blackbox": bb,
              "final": {k: final.get(k) for k in ("REASON", "MAX_ERR", "MAX_U", "TRIPS")}}
    (RUNS_DIR / run_name / "compare.json").write_text(json.dumps(record, indent=2))
    print(f"\nRun {run_name}: {result}. Saved compare.json"
          + (f"; black box {bb}" if bb else "") + ".")
    print("Sync the runs to the Mac and ask for the comparison "
          "(analysis/compare.py).")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
