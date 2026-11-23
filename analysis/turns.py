#!/usr/bin/env python3
"""Turn symmetry: for every turn on the spot (|r| > 20 deg/s for > 1 s) in the
given runs, print the turn rate, the wheel turn speed (vR - vL)/2, the forward
speed (vL + vR)/2 by quarter of the turn, and ratio = |forward| / |turn|.
ratio 0 = both wheels equal and opposite; ratio 1 = one wheel stationary.

    R2W_RUNS=../runs python3 analysis/turns.py RUN [RUN ...]
"""
import csv
import os
import sys

RUNS = os.environ.get("R2W_RUNS", os.path.join(os.path.dirname(__file__), "..", "..", "runs"))
KEYS = ("t_s", "r_dps", "v_left_mm_s", "v_right_mm_s", "flags")

for name in sys.argv[1:]:
    path = os.path.join(RUNS, name, "data.csv") if not os.path.isdir(name) else os.path.join(name, "data.csv")
    if not os.path.exists(path):
        print(f"{name}: no data.csv")
        continue
    rows = [{k: float(r[k]) for k in KEYS} for r in csv.DictReader(open(path))]
    rows = [r for r in rows if int(r["flags"]) & 0x20]
    segs, cur = [], []
    for r in rows + [{"r_dps": 0.0}]:
        if abs(r["r_dps"]) > 20:
            cur.append(r)
        else:
            if len(cur) > 250:
                segs.append(cur)
            cur = []
    for s in segs:
        n, q = len(s), len(s) // 4
        t0 = s[0]["t_s"]
        mr = sum(r["r_dps"] for r in s) / n
        turn = sum((r["v_right_mm_s"] - r["v_left_mm_s"]) / 2 for r in s) / n
        v = [(r["v_left_mm_s"] + r["v_right_mm_s"]) / 2 for r in s]
        quarters = [sum(v[i * q:(i + 1) * q if i < 3 else n]) / max(1, len(v[i * q:(i + 1) * q if i < 3 else n])) for i in range(4)]
        ratio = abs(sum(v) / n) / abs(turn) if turn else 0.0
        print(f"{os.path.basename(name)[:28]:28s} t {t0:6.1f} dur {s[-1]['t_s'] - t0:4.1f} s  r {mr:+6.1f}  "
              f"turn {turn:+5.0f}  forward by quarter " + " ".join(f"{x:+5.0f}" for x in quarters)
              + f"  ratio {ratio:.2f}")
