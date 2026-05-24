#!/usr/bin/env python3
"""
Show nearest radar-point velocity for each lane on an I2C 1602 LCD.

This file is intentionally standalone. It imports the existing AWR6843 parser,
serial, and config modules, but does not modify the original files.

Raspberry Pi I2C wiring:
    GPIO 2: SDA
    GPIO 3: SCL

Examples:
    python3 lane_speed_lcd.py --data-port /dev/ttyUSB1 --cli-port /dev/ttyUSB0 --cfg-file profiles/profile_2d.cfg
    python3 lane_speed_lcd.py --data-port /dev/ttyUSB1 --cli-port /dev/ttyUSB0 --skip-cfg
    python3 lane_speed_lcd.py --lcd-address 0x27 --absolute-speed
"""

import argparse
import math
import os
import sys
import time

from config import (
    EGO_LANE_INDEX,
    LANE_COUNT,
    LANE_WIDTH_M,
    MAX_RANGE,
    MIN_RANGE,
    build_config,
)
from parser import parse_tlv_points, read_packet_buffer
from serial_io import open_serial_ports, send_cfg


LCD_ADDRESS_CANDIDATES = (0x27, 0x3F)
LCD_LINE_LEN = 16


def load_smbus():
    try:
        from smbus2 import SMBus

        return SMBus
    except ImportError:
        try:
            from smbus import SMBus

            return SMBus
        except ImportError as exc:
            raise SystemExit(
                "I2C package is missing. Install it on Raspberry Pi:\n"
                "  sudo apt install python3-smbus i2c-tools\n"
                "or:\n"
                "  python3 -m pip install smbus2"
            ) from exc


def parse_int_auto(value):
    return int(value, 0)


def scan_i2c(bus):
    found = []
    for address in range(0x03, 0x78):
        try:
            if hasattr(bus, "write_quick"):
                bus.write_quick(address)
            else:
                bus.read_byte(address)
            found.append(address)
        except OSError:
            try:
                bus.read_byte(address)
                found.append(address)
            except OSError:
                pass
    return found


def choose_lcd_address(bus, requested_address):
    if requested_address is not None:
        return requested_address

    found = scan_i2c(bus)
    for address in LCD_ADDRESS_CANDIDATES:
        if address in found:
            return address

    found_text = ", ".join("0x%02X" % address for address in found) or "none"
    raise SystemExit(
        "No common I2C 1602 LCD address was found. "
        "Detected I2C addresses: %s. Try --lcd-address 0x27 or 0x3F." % found_text
    )


class I2c1602Lcd:
    ENABLE = 0x04
    BACKLIGHT = 0x08
    LCD_CHR = 0x01
    LCD_CMD = 0x00

    def __init__(self, bus, address, columns=16):
        self.bus = bus
        self.address = address
        self.columns = columns
        self._init_display()

    def _write_byte(self, value):
        self.bus.write_byte(self.address, value)

    def _pulse(self, value):
        self._write_byte(value | self.ENABLE)
        time.sleep(0.0005)
        self._write_byte(value & ~self.ENABLE)
        time.sleep(0.0001)

    def _write4(self, value):
        value |= self.BACKLIGHT
        self._write_byte(value)
        self._pulse(value)

    def _send(self, value, mode):
        self._write4(mode | (value & 0xF0))
        self._write4(mode | ((value << 4) & 0xF0))

    def _init_display(self):
        time.sleep(0.05)
        for command in (0x33, 0x32, 0x28, 0x0C, 0x06, 0x01):
            self._send(command, self.LCD_CMD)
            time.sleep(0.005)

    def clear(self):
        self._send(0x01, self.LCD_CMD)
        time.sleep(0.002)

    def write_line(self, row, text):
        row_offsets = (0x80, 0xC0)
        self._send(row_offsets[row], self.LCD_CMD)
        text = text.ljust(self.columns)[: self.columns]
        for char in text:
            self._send(ord(char), self.LCD_CHR)

    def write_lines(self, line1, line2):
        self.write_line(0, line1)
        self.write_line(1, line2)


def lane_edges(lane_width_m, lane_count, ego_lane_index):
    return [
        (lane_index - ego_lane_index - 0.5) * lane_width_m
        for lane_index in range(1, lane_count + 2)
    ]


def point_lane_index(x, edges):
    for lane_zero_index in range(len(edges) - 1):
        left = edges[lane_zero_index]
        right = edges[lane_zero_index + 1]
        is_last_lane = lane_zero_index == len(edges) - 2

        if left <= x < right or (is_last_lane and left <= x <= right):
            return lane_zero_index

    return None


def nearest_lane_velocities(points, lane_count, lane_width_m, ego_lane_index):
    edges = lane_edges(lane_width_m, lane_count, ego_lane_index)
    best_distances = [float("inf")] * lane_count
    velocities = [None] * lane_count

    for point in points:
        if len(point) < 4:
            continue

        x = float(point[0])
        y = float(point[1])
        z = float(point[2])
        velocity = float(point[3])
        distance = math.sqrt((x * x) + (y * y) + (z * z))

        if distance < MIN_RANGE or distance > MAX_RANGE:
            continue

        lane_zero_index = point_lane_index(x, edges)
        if lane_zero_index is None:
            continue

        if distance < best_distances[lane_zero_index]:
            best_distances[lane_zero_index] = distance
            velocities[lane_zero_index] = velocity

    return velocities


def format_velocity(value, absolute_speed):
    if value is None:
        return "--.-"

    value = abs(value) if absolute_speed else value
    value = max(-99.9, min(99.9, value))
    return "%+05.1f" % value


def lcd_lines_for_velocities(velocities, absolute_speed):
    padded = list(velocities[:3])
    while len(padded) < 3:
        padded.append(None)

    lane1 = format_velocity(padded[0], absolute_speed)
    lane2 = format_velocity(padded[1], absolute_speed)
    lane3 = format_velocity(padded[2], absolute_speed)

    line1 = "L1%s L2%s" % (lane1, lane2)
    line2 = "L3%s m/s" % lane3
    return line1[:LCD_LINE_LEN], line2[:LCD_LINE_LEN]


def apply_runtime_overrides(cfg, args):
    if args.cli_port:
        cfg["cli_port"] = args.cli_port
    if args.data_port:
        cfg["data_port"] = args.data_port
    if args.cfg_file:
        cfg["cfg_file"] = args.cfg_file

    if os.name == "posix":
        if str(cfg.get("cli_port", "")).upper().startswith("COM"):
            cfg["cli_port"] = "/dev/ttyUSB0"
        if str(cfg.get("data_port", "")).upper().startswith("COM"):
            cfg["data_port"] = "/dev/ttyUSB1"

    return cfg


def parse_args():
    parser = argparse.ArgumentParser(
        description="Display nearest point velocity for each lane on an I2C 1602 LCD."
    )
    parser.add_argument("--lcd-bus", type=int, default=1, help="Raspberry Pi I2C bus number.")
    parser.add_argument(
        "--lcd-address",
        type=parse_int_auto,
        default=None,
        help="LCD I2C address, usually 0x27 or 0x3F. Auto-detect when omitted.",
    )
    parser.add_argument("--cli-port", default=None, help="AWR6843 CLI serial port.")
    parser.add_argument("--data-port", default=None, help="AWR6843 data serial port.")
    parser.add_argument("--cfg-file", default=None, help="AWR6843 mmWave cfg file path.")
    parser.add_argument(
        "--skip-cfg",
        action="store_true",
        help="Do not send the cfg file. Use this if the radar is already streaming.",
    )
    parser.add_argument(
        "--absolute-speed",
        action="store_true",
        help="Show absolute speed instead of signed radar velocity.",
    )
    parser.add_argument(
        "--update-interval",
        type=float,
        default=0.2,
        help="Minimum LCD update interval in seconds.",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    cfg = apply_runtime_overrides(build_config(), args)

    SMBus = load_smbus()
    lcd_bus = SMBus(args.lcd_bus)
    data = None
    cli = None

    try:
        lcd_address = choose_lcd_address(lcd_bus, args.lcd_address)
        lcd = I2c1602Lcd(lcd_bus, lcd_address)
        lcd.write_lines("AWR6843 LCD", "Waiting radar")

        data, cli = open_serial_ports(cfg)
        if not args.skip_cfg:
            if os.path.exists(cfg["cfg_file"]):
                send_cfg(cli, cfg["cfg_file"])
            else:
                print("cfg file not found, skipping cfg send: %s" % cfg["cfg_file"])

        print("LCD address: 0x%02X" % lcd_address)
        print(
            "Lane setup: %d lanes x %.2f m, ego lane %d"
            % (LANE_COUNT, LANE_WIDTH_M, EGO_LANE_INDEX)
        )

        buffer = bytearray()
        next_lcd_update = 0.0
        last_lines = None

        while True:
            packet = read_packet_buffer(data, buffer)
            if packet is None:
                continue

            points, _num_detected_obj = parse_tlv_points(packet)
            if not points:
                velocities = [None] * LANE_COUNT
            else:
                velocities = nearest_lane_velocities(
                    points,
                    LANE_COUNT,
                    LANE_WIDTH_M,
                    EGO_LANE_INDEX,
                )

            now = time.monotonic()
            if now >= next_lcd_update:
                lines = lcd_lines_for_velocities(velocities, args.absolute_speed)
                if lines != last_lines:
                    lcd.write_lines(lines[0], lines[1])
                    last_lines = lines
                next_lcd_update = now + max(0.05, args.update_interval)

    except KeyboardInterrupt:
        print("\nStopped by user.")
    finally:
        if data is not None:
            data.close()
        if cli is not None:
            cli.close()
        if hasattr(lcd_bus, "close"):
            lcd_bus.close()

    return 0


if __name__ == "__main__":
    sys.exit(main())
