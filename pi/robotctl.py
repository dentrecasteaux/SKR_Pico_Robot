#!/usr/bin/env python3
"""Command-line client for the R2W/1 SKR Pico robot protocol."""

from __future__ import annotations

import argparse
import glob
import os
import sys
import time
from dataclasses import dataclass
from typing import Optional

import serial


PROTOCOL = "R2W/1"
COMMAND_ATTEMPTS = 2
DEFAULT_DEVICE_PATTERN = (
    "/dev/serial/by-id/usb-Arduino_RaspberryPi_Pico_*-if00"
)


class RobotLinkError(RuntimeError):
    """Raised when the Pico link or protocol returns an error."""


@dataclass(frozen=True)
class Reply:
    kind: str
    sequence: Optional[int]
    fields: tuple[str, ...]
    line: str


class RobotClient:
    def __init__(self, port: str, timeout: float = 2.0) -> None:
        self._serial = serial.Serial(
            port=port,
            baudrate=115200,
            timeout=timeout,
            write_timeout=timeout,
        )
        self._next_sequence = max(1, int(time.monotonic() * 1000) & 0xFFFFFFFF)
        time.sleep(0.2)
        self._serial.reset_input_buffer()

    def close(self) -> None:
        self._serial.close()

    def __enter__(self) -> RobotClient:
        return self

    def __exit__(self, *_: object) -> None:
        self.close()

    def next_sequence(self) -> int:
        sequence = self._next_sequence
        self._next_sequence = (self._next_sequence + 1) & 0xFFFFFFFF
        if self._next_sequence == 0:
            self._next_sequence = 1
        return sequence

    def command(self, name: str, *fields: str) -> Reply:
        sequence = self.next_sequence()
        parts = [PROTOCOL, "CMD", str(sequence), name, *fields]
        line = " ".join(parts)

        for attempt in range(COMMAND_ATTEMPTS):
            self._serial.write((line + "\n").encode("ascii"))
            self._serial.flush()

            try:
                while True:
                    reply = self.read_reply()
                    if reply.sequence == sequence:
                        if reply.kind == "ERR":
                            raise RobotLinkError(reply.line)
                        return reply
                    print(reply.line)
            except RobotLinkError as error:
                timed_out = str(error) == "Timed out waiting for the Pico"
                if not timed_out or attempt == COMMAND_ATTEMPTS - 1:
                    raise

        raise RobotLinkError("Command retry failed")

    def read_reply(self) -> Reply:
        raw = self._serial.readline()
        if not raw:
            raise RobotLinkError("Timed out waiting for the Pico")

        return self._parse_reply(raw)

    @staticmethod
    def _parse_reply(raw: bytes) -> Reply:
        try:
            line = raw.decode("ascii").strip()
        except UnicodeDecodeError as error:
            raise RobotLinkError("Received non-ASCII data from the Pico") from error

        parts = line.split()
        if len(parts) < 2 or parts[0] != PROTOCOL:
            return Reply("OTHER", None, tuple(parts[1:]), line)

        kind = parts[1]
        sequence: Optional[int] = None
        fields_start = 2
        if kind in {"ACK", "ERR", "PONG", "STAT"} and len(parts) >= 3:
            try:
                sequence = int(parts[2])
            except ValueError:
                sequence = None
            fields_start = 3

        return Reply(kind, sequence, tuple(parts[fields_start:]), line)

    def wait_for_job(self, job_id: int) -> Reply:
        previous_timeout = self._serial.timeout
        self._serial.timeout = 0.5
        try:
            while True:
                raw = self._serial.readline()
                if not raw:
                    continue
                reply = self._parse_reply(raw)
                print(reply.line)
                if (
                    reply.kind == "DONE"
                    and f"JOB={job_id}" in reply.fields
                ):
                    return reply
        finally:
            self._serial.timeout = previous_timeout


def resolve_port(requested_port: Optional[str]) -> str:
    if requested_port:
        return requested_port

    environment_port = os.environ.get("R2W_PORT")
    if environment_port:
        return environment_port

    matches = sorted(glob.glob(DEFAULT_DEVICE_PATTERN))
    if len(matches) == 1:
        return matches[0]
    if not matches:
        raise RobotLinkError(
            "No SKR Pico found under /dev/serial/by-id; connect its USB cable"
        )
    raise RobotLinkError(
        "Multiple matching Pico devices found; select one with --port"
    )


def field_value(reply: Reply, name: str) -> Optional[str]:
    prefix = name + "="
    for field in reply.fields:
        if field.startswith(prefix):
            return field[len(prefix) :]
    return None


def send_velocity(
    client: RobotClient,
    linear: float,
    angular: float,
    lease_ms: int,
    duration: Optional[float],
) -> None:
    fields = (f"V={linear:g}", f"W={angular:g}", f"LEASE={lease_ms}")
    reply = client.command("SET_VELOCITY", *fields)
    print(reply.line)

    if duration is None:
        return

    renewal_period = lease_ms / 3000.0
    finish_at = time.monotonic() + duration
    try:
        while True:
            remaining = finish_at - time.monotonic()
            if remaining <= 0:
                break
            time.sleep(min(renewal_period, remaining))
            if time.monotonic() < finish_at:
                print(client.command("SET_VELOCITY", *fields).line)
    except KeyboardInterrupt:
        print("\nInterrupted; sending STOP", file=sys.stderr)
    finally:
        print(client.command("STOP").line)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Control and inspect the R2W robot over USB serial"
    )
    parser.add_argument(
        "--port",
        help="serial device; defaults to the unique Pico under /dev/serial/by-id",
    )
    subparsers = parser.add_subparsers(dest="action", required=True)

    subparsers.add_parser("ping", help="check the Pico link")
    subparsers.add_parser("status", help="show robot and driver status")
    subparsers.add_parser("stop", help="stop immediately without latching")
    subparsers.add_parser("estop", help="stop immediately and latch")
    subparsers.add_parser("clear-estop", help="clear a safe estop latch")

    velocity = subparsers.add_parser(
        "velocity", help="set leased linear and angular velocity"
    )
    velocity.add_argument("linear", type=float, help="millimetres per second")
    velocity.add_argument("angular", type=float, help="degrees per second")
    velocity.add_argument("--lease", type=int, default=500, help="lease in ms")
    velocity.add_argument(
        "--duration",
        type=float,
        help="renew for this many seconds, then send STOP",
    )

    move = subparsers.add_parser("move", help="start a finite straight move")
    move.add_argument("distance", type=float, help="millimetres")
    move.add_argument("--wait", action="store_true", help="wait for DONE")

    turn = subparsers.add_parser("turn", help="start a finite in-place turn")
    turn.add_argument("angle", type=float, help="degrees")
    turn.add_argument("--wait", action="store_true", help="wait for DONE")
    return parser


def run(args: argparse.Namespace) -> None:
    port = resolve_port(args.port)
    with RobotClient(port) as client:
        if args.action == "ping":
            print(client.command("PING").line)
        elif args.action == "status":
            print(client.command("STATUS").line)
        elif args.action == "stop":
            print(client.command("STOP").line)
        elif args.action == "estop":
            print(client.command("ESTOP").line)
        elif args.action == "clear-estop":
            print(client.command("CLEAR_ESTOP").line)
        elif args.action == "velocity":
            send_velocity(
                client,
                args.linear,
                args.angular,
                args.lease,
                args.duration,
            )
        elif args.action in {"move", "turn"}:
            if args.action == "move":
                reply = client.command("MOVE", f"DIST={args.distance:g}")
            else:
                reply = client.command("TURN", f"ANGLE={args.angle:g}")
            print(reply.line)
            if args.wait:
                job_text = field_value(reply, "JOB")
                if job_text is None:
                    raise RobotLinkError("Pico accepted a finite job without JOB")
                try:
                    client.wait_for_job(int(job_text))
                except KeyboardInterrupt:
                    print("\nInterrupted; sending STOP", file=sys.stderr)
                    print(client.command("STOP").line)


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()
    try:
        run(args)
    except (RobotLinkError, serial.SerialException) as error:
        print(f"robotctl: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
