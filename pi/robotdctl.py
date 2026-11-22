#!/usr/bin/env python3
"""Small local client for testing robotd."""

from __future__ import annotations

import argparse
import json
import socket
from pathlib import Path
from typing import Any

from robotd import default_socket_path


def request(path: Path, payload: dict[str, Any]) -> dict[str, Any]:
    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as connection:
        connection.connect(str(path))
        connection.sendall((json.dumps(payload) + "\n").encode("utf-8"))
        response = connection.makefile("rb").readline()
    return json.loads(response.decode("utf-8"))


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Send a command to robotd")
    parser.add_argument("--socket", type=Path, default=default_socket_path())
    subparsers = parser.add_subparsers(dest="action", required=True)
    subparsers.add_parser("ping")
    subparsers.add_parser("status")
    subparsers.add_parser("service-status")
    subparsers.add_parser("stop")
    subparsers.add_parser("estop")
    subparsers.add_parser("clear-estop")

    configure = subparsers.add_parser("configure")
    configure.add_argument("current_ma", type=int)
    configure.add_argument("microsteps", type=int)
    configure.add_argument("acceleration_mm_s2", type=float)
    configure.add_argument(
        "tmc_mode", choices=("STEALTHCHOP", "SPREADCYCLE")
    )

    velocity = subparsers.add_parser("velocity")
    velocity.add_argument("linear", type=float)
    velocity.add_argument("angular", type=float)
    velocity.add_argument("--lease", type=int, default=1000)
    velocity.add_argument("--hold", type=int, default=1000)

    move = subparsers.add_parser("move")
    move.add_argument("distance", type=float)
    turn = subparsers.add_parser("turn")
    turn.add_argument("angle", type=float)

    bal_set = subparsers.add_parser(
        "bal-set", help="set balance parameters, e.g. KP=39 KD=2.9 TRIM=-3.9"
    )
    bal_set.add_argument("pairs", nargs="+", metavar="KEY=VALUE")
    bal_arm = subparsers.add_parser(
        "bal-arm", help="arm the balance controller (lift the robot upright)"
    )
    bal_arm.add_argument("--dry", action="store_true",
                         help="controller only, motors off (sign checks)")
    subparsers.add_parser("bal-stop")
    subparsers.add_parser("bal-status")

    hold = subparsers.add_parser(
        "hold", help="lock the wheels (drivers on, no motion) for a while"
    )
    hold.add_argument("--lease", type=int, default=30000, help="ms, 500-120000")

    log_start = subparsers.add_parser(
        "log-start", help="start recording a telemetry run on the Pi"
    )
    log_start.add_argument("--name", default="run")
    log_start.add_argument("--notes", default="")
    log_start.add_argument(
        "--div", type=int, default=2, help="1 = 500 Hz, 2 = 250 Hz, ..."
    )
    log_start.add_argument(
        "--duration", type=float, help="stop automatically after N seconds"
    )
    subparsers.add_parser("log-stop", help="stop the run and close its files")
    subparsers.add_parser("log-status")
    subparsers.add_parser("bb-freeze", help="freeze the black box")
    subparsers.add_parser("bb-arm", help="clear and restart the black box")
    subparsers.add_parser("bb-dump", help="save the frozen black box to a run")
    subparsers.add_parser("runs", help="list recorded runs")
    return parser


def payload(args: argparse.Namespace) -> dict[str, Any]:
    action = args.action.replace("-", "_")
    result: dict[str, Any] = {"action": action}
    if args.action == "velocity":
        result.update(
            linear=args.linear,
            angular=args.angular,
            lease_ms=args.lease,
            hold_ms=args.hold,
        )
    elif args.action == "move":
        result["distance"] = args.distance
    elif args.action == "turn":
        result["angle"] = args.angle
    elif args.action == "bal-set":
        values = {}
        for pair in args.pairs:
            key, _, value = pair.partition("=")
            values[key] = float(value)
        result["set"] = values
    elif args.action == "bal-arm":
        result["dry"] = args.dry
    elif args.action == "hold":
        result["lease_ms"] = args.lease
    elif args.action == "log-start":
        result.update(name=args.name, notes=args.notes, div=args.div)
        if args.duration is not None:
            result["duration_s"] = args.duration
    elif args.action == "configure":
        result.update(
            current_ma=args.current_ma,
            microsteps=args.microsteps,
            acceleration_mm_s2=args.acceleration_mm_s2,
            tmc_mode=args.tmc_mode,
        )
    return result


def main() -> int:
    args = build_parser().parse_args()
    try:
        response = request(args.socket, payload(args))
    except OSError as error:
        print(f"robotdctl: {error}")
        return 1
    print(json.dumps(response, indent=2, sort_keys=True))
    return 0 if response.get("ok") else 1


if __name__ == "__main__":
    raise SystemExit(main())
