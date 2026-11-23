#!/usr/bin/env python3
"""Check the robot is in its best known configuration (23 Nov 2026).

Compares the live firmware settings (BAL_STATUS, STATUS, TMC_PWM) with the
expected values and prints OK / DIFF for each, plus any saved web tuning.
Read-only: it changes nothing.

    python3 check_config.py
"""
from __future__ import annotations

import sys

from balance_tools import DEFAULTS, call
import tuning

# Best known configuration. Balance parameters: firmware defaults.
EXPECTED_BAL = {**DEFAULTS, **tuning.TUNING_DEFAULTS, "ACCEL_COMP": 2, "NOTCH_HZ": 20.5, "NOTCH_Q": 2.0}
# Drivers: StealthChop, 1.0 A, 1/16, automatic PWM scaling and gradient on.
EXPECTED_STATUS = {"RUN_CURRENT_MA": 1000, "MICROSTEPS": 16,
                   "X_TMC_MODE": "STEALTHCHOP", "Y_TMC_MODE": "STEALTHCHOP",
                   "ESTOP": "0", "FAULTS": "NONE", "IMU": "OK"}
EXPECTED_PWM = {"X_AUTOSCALE": 1, "Y_AUTOSCALE": 1,
                "X_AUTOGRAD": 1, "Y_AUTOGRAD": 1}


def same(live: str, want) -> bool:
    if isinstance(want, str):
        return live == want
    try:
        return abs(float(live) - float(want)) <= max(1e-3, abs(float(want)) * 1e-3)
    except ValueError:
        return False


def section(title: str, live: dict, expected: dict) -> int:
    print(f"\n{title}")
    bad = 0
    for key, want in expected.items():
        value = live.get(key, "(missing)")
        ok = same(value, want)
        bad += not ok
        print(f"  {'OK  ' if ok else 'DIFF'} {key:16s} {value:>12}   (expected {want})")
    return bad


def main() -> int:
    bal = call("bal_status")["balance"]
    status = call("status")["service"]["status"]
    pwm = call("tmc_pwm").get("pwm", {})
    saved = call("tuning").get("saved", {})

    bad = section("Balance parameters (BAL_STATUS)", bal, EXPECTED_BAL)
    bad += section("Robot and drivers (STATUS)", status, EXPECTED_STATUS)
    bad += section("StealthChop PWM (TMC_PWM)", pwm, EXPECTED_PWM)
    print(f"\n  info: learned PWM  X OFS_AUTO {pwm.get('X_OFS_AUTO')} GRAD_AUTO "
          f"{pwm.get('X_GRAD_AUTO')} | Y OFS_AUTO {pwm.get('Y_OFS_AUTO')} "
          f"GRAD_AUTO {pwm.get('Y_GRAD_AUTO')}")
    print(f"  info: balance state {bal.get('BAL')}, drive lock "
          f"{status.get('DRIVE_LOCK')}, gyro yaw coupling {bal.get('GYRO_QR')}")
    if saved:
        print("\n  Saved web tuning (re-applied at every SKR start): " +
              ", ".join(f"{k} {v:g}" for k, v in saved.items()))
        differs = {k: v for k, v in saved.items()
                   if abs(v - tuning.TUNING_DEFAULTS[k]) > 1e-6}
        if differs:
            print("  NOTE: saved values differ from the best known defaults: "
                  + ", ".join(f"{k} {v:g}" for k, v in differs.items())
                  + "  (clear with: python3 robotdctl.py tuning-clear, "
                  "or 'Firmware defaults' + Apply + Save on the web page)")
            bad += len(differs)
    else:
        print("\n  Saved web tuning: none (firmware defaults at start-up)")
    print(f"\n{'ALL OK' if bad == 0 else f'{bad} difference(s)'}")
    return 0 if bad == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
