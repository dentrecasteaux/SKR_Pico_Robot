#!/usr/bin/env python3
"""Phase 6 pass test: telemetry stream and black box.

Robot ON THE STAND, wheels clear. Steps:

1. Start a LOG run (default 250 Hz) and drive both wheels with a sine-wave
   velocity (default ±200 mm/s, 0.25 Hz) for 30 s.
2. Stop the run and check data.csv: no missing records, no time gaps, the
   wheel speed follows the command.
3. Freeze the black box, dump it, and check it holds about 2 s of
   consecutive 500 Hz records.
4. Re-arm the black box.

    cd ~/r2w && python3 log_test.py
    python3 log_test.py --seconds 10 --div 1      # shorter, 500 Hz

Prints PASS/FAIL for each check. Results stay in ~/runs/.
"""
from __future__ import annotations

import argparse
import csv
import math
import time
from pathlib import Path

from robotd import default_socket_path
from robotdctl import request
from runlog import RUNS_DIR

SOCK = default_socket_path()


def call(action: str, **kw) -> dict:
    reply = request(SOCK, {"action": action, **kw})
    if not reply.get("ok"):
        raise RuntimeError(f"{action}: {reply.get('error', reply)}")
    return reply


def check(name: str, ok: bool, detail: str) -> bool:
    print(f"  {'PASS' if ok else 'FAIL'}  {name}: {detail}")
    return ok


def read_csv(run: str) -> list[dict[str, str]]:
    with open(RUNS_DIR / run / "data.csv", newline="") as fh:
        return list(csv.DictReader(fh))


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--seconds", type=float, default=30.0)
    ap.add_argument("--div", type=int, default=2)
    ap.add_argument("--amplitude", type=float, default=200.0, help="mm/s")
    ap.add_argument("--freq", type=float, default=0.25, help="Hz")
    args = ap.parse_args()

    results = []
    call("stop")
    time.sleep(0.6)
    call("configure", current_ma=1000, microsteps=16,
         acceleration_mm_s2=2000.0, tmc_mode="STEALTHCHOP")
    call("bb_arm")

    print(f"1. LOG run: {args.seconds:.0f} s sine, ±{args.amplitude:.0f} mm/s, "
          f"{args.freq} Hz, {500 / args.div:.0f} Hz logging")
    run = call("log_start", name="phase6_sine", div=args.div,
               notes=f"log_test.py sine ±{args.amplitude} mm/s {args.freq} Hz")["run"]
    t0 = time.monotonic()
    try:
        while (t := time.monotonic() - t0) < args.seconds:
            v = args.amplitude * math.sin(2 * math.pi * args.freq * t)
            call("velocity", linear=round(v, 1), angular=0.0, lease_ms=500,
                 hold_ms=500)
            time.sleep(0.05)
    finally:
        call("stop")
    time.sleep(0.5)
    stopped = call("log_stop")
    summary = stopped["run"]
    fw = stopped["service"]["log"]["firmware"]
    print(f"   run {run}")

    rows = read_csv(run)
    expected = args.seconds * 500 / args.div
    results.append(check(
        "rows", len(rows) >= 0.95 * expected,
        f"{len(rows)} rows (expected about {expected:.0f})"))
    results.append(check(
        "no missing records", summary.get("seq_gaps") == 0
        and fw.get("STREAM_DROPS") == "0" and fw.get("IMU_DROPS") == "0",
        f"seq_gaps={summary.get('seq_gaps')} stream_drops={fw.get('STREAM_DROPS')} "
        f"imu_drops={fw.get('IMU_DROPS')} bad_lines={summary.get('bad_lines')}"))
    period_us = 2000 * args.div
    results.append(check(
        "no time gaps", summary.get("max_dt_us", 1e9) < 1.6 * period_us,
        f"max dt {summary.get('max_dt_us')} us (period {period_us} us)"))
    peak = max((abs(float(r["v_left_mm_s"] or 0)) for r in rows), default=0)
    results.append(check(
        "wheel speed follows command", 0.8 * args.amplitude < peak < 1.2 * args.amplitude,
        f"peak |v_left| {peak:.0f} mm/s (commanded {args.amplitude:.0f})"))

    print("2. Black box")
    call("bb_freeze")
    bb = call("bb_dump")["run"]
    deadline = time.monotonic() + 20
    last = {}
    while time.monotonic() < deadline:
        last = call("service_status")["service"]["log"].get("last_blackbox", {})
        if last.get("run") == bb:
            break
        time.sleep(0.3)
    bb_rows = read_csv(bb) if (RUNS_DIR / bb / "data.csv").exists() else []
    seqs = [int(r["seq"]) for r in bb_rows]
    consecutive = all(b - a == 1 for a, b in zip(seqs, seqs[1:]))
    span = (float(bb_rows[-1]["t_s"]) if bb_rows else 0.0)
    results.append(check(
        "black box dumped", last.get("complete") is True,
        f"{len(bb_rows)} records, {span:.2f} s, run {bb}"))
    results.append(check(
        "black box continuous", len(bb_rows) >= 1000 and consecutive,
        f"consecutive={consecutive}"))
    call("bb_arm")
    call("configure", current_ma=1000, microsteps=16,
         acceleration_mm_s2=150.0, tmc_mode="STEALTHCHOP")

    ok = all(results)
    print(f"\nPhase 6 {'PASS' if ok else 'FAIL'}  ({sum(results)}/{len(results)} checks)")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
