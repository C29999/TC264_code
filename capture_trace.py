#!/usr/bin/env python3
"""Receive smart-car control telemetry over TCP and save it as CSV."""

import argparse
import csv
import socket
from datetime import datetime
from pathlib import Path


FIELDS = [
    "system_ms",
    "pure_angle_deg",
    "servo_angle_deg",
    "gyro_z_dps",
    "servo_p",
    "servo_d",
    "left_goal",
    "right_goal",
    "left_encoder",
    "right_encoder",
    "left_pwm",
    "right_pwm",
    "motor_diff",
    "left_line_count",
    "right_line_count",
    "state_flags",
    "mid_x",
    "fps",
    "stop_flag",
]


def decode_trace(line: bytes):
    if not line.startswith(b"$TRACE,"):
        return None
    parts = line.strip().split(b",")[1:]
    if len(parts) != len(FIELDS):
        return None
    values = [int(value) for value in parts]
    for index, scale in ((1, 100.0), (2, 100.0), (3, 10.0), (4, 100.0), (5, 100.0)):
        values[index] /= scale
    return values


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", type=int, default=8080)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()

    output = args.output or Path(
        "trace_" + datetime.now().strftime("%Y%m%d_%H%M%S") + ".csv"
    )
    received = 0
    buffer = bytearray()

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server:
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind(("0.0.0.0", args.port))
        server.listen(1)
        print(f"Listening on TCP 0.0.0.0:{args.port}")
        print("Power on or reset the car now. Press Ctrl+C after the test run.")
        connection, address = server.accept()
        print(f"Connected: {address[0]}:{address[1]}")

        with connection, output.open("w", newline="", encoding="utf-8") as stream:
            writer = csv.writer(stream)
            writer.writerow(FIELDS)
            try:
                while True:
                    chunk = connection.recv(4096)
                    if not chunk:
                        break
                    buffer.extend(chunk)
                    while b"\n" in buffer:
                        raw_line, _, buffer = buffer.partition(b"\n")
                        try:
                            row = decode_trace(raw_line)
                        except ValueError:
                            row = None
                        if row is None:
                            continue
                        writer.writerow(row)
                        stream.flush()
                        received += 1
                        if received % 50 == 0:
                            print(
                                f"samples={received}  t={row[0]}ms  "
                                f"error={row[1]:+.2f}  servo={row[2]:+.2f}  "
                                f"gyro={row[3]:+.1f}"
                            )
            except KeyboardInterrupt:
                pass

    print(f"Saved {received} samples to {output.resolve()}")


if __name__ == "__main__":
    main()
