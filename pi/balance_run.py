#!/usr/bin/env python3
"""Phase 9.2/9.3: a logged balance attempt.

Arms the controller (motors ON), records the attempt at 500 Hz and, when it
ends, saves the black box and prints a summary, including an estimate of
l_eff from the robot's own response.

    ssh -t robot@r2w-pi.local 'cd r2w && python3 balance_run.py'
    python3 balance_run.py --kp 30 --kd 2.5          # set gains first
    python3 balance_run.py --scale 0.3 --stand       # 9.2: wheels up, 30% gains

How to use it on the floor (9.3):
  1. The robot rests on a training wheel. Start the script; it arms.
  2. Lift the robot slowly towards upright with your hands either side.
     It engages by itself when within ARM_WINDOW of the balance point
     (the wheels then start working; let go gently).
  3. Keep your hands near, not touching. It ends on a trip (tilt or
     runaway), when you press Ctrl-C, or after --seconds.
Kill switch in reach. Expect PD alone to stand for a few seconds and then
drift away in one direction: that is a success for 9.3.
"""
from __future__ import annotations

import argparse
import json
import time

from balance_tools import (DEFAULTS, RUNS_DIR, SAFETY_KEYS, balance_status, call,
                           l_eff_estimate, load)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--kp", type=float)
    ap.add_argument("--kd", type=float)
    ap.add_argument("--trim", type=float)
    ap.add_argument("--scale", type=float, default=1.0,
                    help="multiply KP and KD for this run only (e.g. 0.3)")
    ap.add_argument("--stand", action="store_true",
                    help="wheels-up test (9.2): no tilt trips, short run")
    ap.add_argument("--seconds", type=float, default=60.0)
    ap.add_argument("--notes", default="")
    ap.add_argument("--force", action="store_true",
                    help="run even if KP looks implausibly low")
    args = ap.parse_args()

    call("stop")
    time.sleep(0.3)
    base = balance_status()
    # Safety settings must be the defaults at the start of every run (a run
    # that was interrupted earlier could have left test values behind).
    wrong = {k: DEFAULTS[k] for k in SAFETY_KEYS
             if abs(float(base[k]) - DEFAULTS[k]) > 1e-3}
    if wrong:
        print("Resetting safety settings to defaults: "
              + ", ".join(f"{k} {base[k]} -> {v:g}" for k, v in wrong.items()))
        call("bal_set", set=wrong)
        base = balance_status()
    saved = {k: float(base[k]) for k in
             ("KP", "KD", "TRIM", "TILT_FWD", "TILT_BACK", "ARM_WINDOW", "VMAX",
              "RUNAWAY")}
    new = {}
    if args.kp is not None:
        new["KP"] = args.kp
    if args.kd is not None:
        new["KD"] = args.kd
    if args.trim is not None:
        new["TRIM"] = args.trim
    if new:
        call("bal_set", set=new)
        saved.update({k: v for k, v in new.items() if k in saved})
    temp = {}
    if args.scale != 1.0:
        temp["KP"] = saved["KP"] * args.scale
        temp["KD"] = saved["KD"] * args.scale
    if args.stand:
        # On the stand the robot can't fall. Zero the error at the stand's
        # angle, allow any tilt, keep the speed low and the runaway trip slow;
        # tilting the robot a little then makes the wheels accelerate that way.
        theta = float(base["THETA"])
        temp.update(TRIM=round(theta, 2), TILT_FWD=round(theta + 30, 1),
                    TILT_BACK=round(theta - 30, 1), ARM_WINDOW=10.0, VMAX=0.2,
                    RUNAWAY=5.0)
    try:
        return run(args, base, saved, temp)
    finally:
        # Always put back anything this run changed temporarily.
        try:
            call("bal_stop")
            time.sleep(0.7)
            if temp:
                call("bal_set", set={k: saved[k] for k in temp})
        except Exception as error:  # noqa: BLE001
            print(f"WARNING: could not restore settings ({error}); "
                  "run: robotdctl.py bal-status")


def run(args, base, saved, temp) -> int:
    if temp:
        call("bal_set", set=temp)
    st = balance_status()
    if not args.stand and float(st["KP"]) < 15 and not args.force:
        print(f"KP={st['KP']} looks too low for a floor run (default 39). "
              "Set it with --kp, or use --force.")
        return 1
    print(f"Gains for this run: KP={st['KP']} KD={st['KD']} TRIM={st['TRIM']} "
          f"AMAX={st['AMAX']} VMAX={st['VMAX']} tilt {st['TILT_BACK']}..{st['TILT_FWD']}")
    print(__doc__.split("How to use")[1].split("Kill switch")[0] if not args.stand else
          "Stand test: tilt the robot by hand; the wheels should spin FORWARD when "
          "you tilt it forward. Ctrl-C to stop.")
    input("Kill switch in reach? Press Enter to arm...")

    run = call("log_start", name="balance_stand" if args.stand else "balance",
               div=1, duration_s=args.seconds + 30,
               notes=f"balance_run.py {args.notes} gains {st['KP']}/{st['KD']} "
                     f"trim {st['TRIM']} scale {args.scale}")["run"]
    call("bal_arm")
    print("Armed. Lift towards upright..." if not args.stand else "Armed.")
    state, engaged_at = "", None
    t0 = time.monotonic()
    try:
        while time.monotonic() - t0 < args.seconds:
            s = balance_status()
            if s["BAL"] != state:
                state = s["BAL"]
                print(f"\n[{time.monotonic() - t0:5.1f} s] {state}"
                      + (f" ({s['REASON']})" if state in ("TRIPPED", "OFF") else ""))
                if state == "RUNNING" and engaged_at is None:
                    engaged_at = time.monotonic()
            if state in ("TRIPPED", "OFF"):
                break
            print(f"\r   err {float(s['ERR']):+6.2f}°  u {float(s['U']):+5.2f}  "
                  f"v {float(s['V']):+5.2f} m/s  x {float(s['X']):+5.2f} m   ",
                  end="", flush=True)
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\nStopping.")
    finally:
        call("bal_stop")
        time.sleep(0.7)
    final = balance_status()
    call("log_stop")
    stood = (time.monotonic() - engaged_at) if engaged_at else 0.0
    bb = None
    try:
        lst = call("log_status")["service"]["log"]["firmware"]
        if lst.get("BB") == "FROZEN":
            bb = call("bb_dump")["run"]
    except RuntimeError:
        pass

    rows = load(run)
    est = l_eff_estimate(rows, float(final["TRIM"]))
    summary = {
        "run": run, "blackbox": bb, "stood_s": round(stood, 2),
        "end_reason": final["REASON"], "max_err_deg": final["MAX_ERR"],
        "max_u": final["MAX_U"], "l_eff": est,
        "gains": {k: st[k] for k in ("KP", "KD", "KV", "KX", "TRIM", "AMAX", "VMAX")},
    }
    print(f"\nRun {run}")
    print(f"  stood {stood:.1f} s, ended: {final['REASON']}, max |err| {final['MAX_ERR']}°, "
          f"max |u| {final['MAX_U']} m/s²")
    if est["l_eff_mm"]:
        print(f"  l_eff estimate {est['l_eff_mm']} mm (r = {est['r']}, {est['samples']} samples)")
    if bb:
        print(f"  black box saved: {bb}")
    (RUNS_DIR / run / "balance_summary.json").write_text(json.dumps(summary, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
