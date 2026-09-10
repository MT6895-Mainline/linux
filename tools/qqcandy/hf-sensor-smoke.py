#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-only
"""Bounded sampling through the ported sensor/2.0 hf_sensor_io.h ABI."""

import argparse
import collections
import fcntl
import json
import os
import select
import struct
import time


PACKET_SIZE = 68
EVENT = struct.Struct("<q4B16i")
COMMAND = struct.Struct("<4B48s")
SENSOR_INFO = struct.Struct("<B3xI16s16s")
REGISTER_STATUS = 1
SENSOR_INFO_REQUEST = 6
READY_STATUS = 8
ENABLE = 1
DISABLE = 0
DATA_ACTION = 0
SENSORS = {1: "accel", 2: "mag", 4: "gyro", 5: "light", 8: "proximity",
           11: "rotation", 18: "step_detector", 19: "step_counter"}


def request(fd, number, sensor=0):
    packet = bytearray(PACKET_SIZE)
    packet[0] = sensor
    ioctl = (3 << 30) | (PACKET_SIZE << 16) | (ord("a") << 8) | number
    fcntl.ioctl(fd, ioctl, packet, True)
    return packet


def control(fd, sensor, action):
    batch = struct.pack("<qq", 50_000_000, 0)
    command = COMMAND.pack(sensor, action, len(batch), 0, batch.ljust(48, b"\0"))
    # This vendor ABI returns zero, rather than the byte count, on success.
    result = os.write(fd, command)
    if result != 0:
        raise RuntimeError(f"unexpected command result: {result}")


def run(seconds):
    enabled = []
    info = {}
    counts = collections.Counter()
    last_timestamp = {}
    last_sample = {}
    backwards = collections.Counter()
    fd = os.open("/dev/hf_manager", os.O_RDWR | os.O_NONBLOCK)
    try:
        if not request(fd, READY_STATUS)[4]:
            raise RuntimeError("sensor manager is not ready")
        for sensor, label in SENSORS.items():
            if not request(fd, REGISTER_STATUS, sensor)[4]:
                continue
            packet = request(fd, SENSOR_INFO_REQUEST, sensor)
            sensor_type, gain, name, vendor = SENSOR_INFO.unpack_from(packet, 4)
            if sensor_type != sensor or not gain:
                raise RuntimeError(f"invalid sensor info for {sensor}")
            info[sensor] = {
                "label": label, "gain": gain,
                "name": name.split(b"\0", 1)[0].decode("ascii", "replace"),
                "vendor": vendor.split(b"\0", 1)[0].decode("ascii", "replace"),
            }
            control(fd, sensor, ENABLE)
            enabled.append(sensor)
        poller = select.poll()
        poller.register(fd, select.POLLIN)
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            if not poller.poll(min(200, max(1, int((deadline - time.monotonic()) * 1000)))):
                continue
            data = os.read(fd, EVENT.size * 64)
            if len(data) % EVENT.size:
                raise RuntimeError(f"partial event: {len(data)} bytes")
            for event in EVENT.iter_unpack(data):
                timestamp, sensor, accuracy, action, _ = event[:5]
                if action != DATA_ACTION:
                    continue
                if timestamp < last_timestamp.get(sensor, timestamp):
                    backwards[sensor] += 1
                last_timestamp[sensor] = timestamp
                counts[sensor] += 1
                last_sample[sensor] = {"timestamp": timestamp, "accuracy": accuracy,
                                       "word": event[5:11]}
        print(json.dumps({"seconds": seconds, "sensors": info, "samples": counts,
                          "last": last_sample, "backwards_timestamps": backwards},
                         indent=2), flush=True)
        if any(counts[sensor] < 2 for sensor in (1, 2, 4)) or backwards:
            raise RuntimeError("missing continuous IMU samples or nonmonotonic timestamps")
    finally:
        for sensor in reversed(enabled):
            try:
                control(fd, sensor, DISABLE)
            except OSError as exc:
                print(f"disable {sensor}: {exc}", flush=True)
        os.close(fd)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--seconds", type=float, default=8.0)
    args = parser.parse_args()
    if not 0 < args.seconds <= 60:
        parser.error("seconds must be within (0, 60]")
    run(args.seconds)
