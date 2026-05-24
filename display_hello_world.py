#!/usr/bin/env python3
"""
Temporary I2C display smoke test for Raspberry Pi GPIO 2/3.

GPIO 2 is SDA and GPIO 3 is SCL on Raspberry Pi, so this script talks to
common I2C displays without importing or changing the AWR6843 project code.

Examples:
    python3 display_hello_world.py
    python3 display_hello_world.py --type lcd --address 0x27
    python3 display_hello_world.py --type oled --address 0x3c
"""

from __future__ import annotations

import argparse
import sys
import time


LCD_ADDRS = (0x27, 0x3F, 0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26)
OLED_ADDRS = (0x3C, 0x3D)


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
                "Missing I2C Python package. Install one of these on the Pi:\n"
                "  sudo apt install python3-smbus\n"
                "  python3 -m pip install smbus2"
            ) from exc


def parse_address(value: str | None) -> int | None:
    if value is None:
        return None
    return int(value, 0)


def scan_i2c(bus) -> list[int]:
    found: list[int] = []
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


class I2cLcd:
    """HD44780 16x2/20x4 LCD through a PCF8574 I2C backpack."""

    ENABLE = 0x04
    BACKLIGHT = 0x08
    LCD_CHR = 0x01
    LCD_CMD = 0x00

    def __init__(self, bus, address: int, columns: int = 16):
        self.bus = bus
        self.address = address
        self.columns = columns
        self._init_display()

    def _write_byte(self, value: int) -> None:
        self.bus.write_byte(self.address, value)

    def _pulse(self, value: int) -> None:
        self._write_byte(value | self.ENABLE)
        time.sleep(0.0005)
        self._write_byte(value & ~self.ENABLE)
        time.sleep(0.0001)

    def _write4(self, value: int) -> None:
        value |= self.BACKLIGHT
        self._write_byte(value)
        self._pulse(value)

    def _send(self, value: int, mode: int) -> None:
        self._write4(mode | (value & 0xF0))
        self._write4(mode | ((value << 4) & 0xF0))

    def _init_display(self) -> None:
        time.sleep(0.05)
        for command in (0x33, 0x32, 0x28, 0x0C, 0x06, 0x01):
            self._send(command, self.LCD_CMD)
            time.sleep(0.005)

    def clear(self) -> None:
        self._send(0x01, self.LCD_CMD)
        time.sleep(0.002)

    def write_line(self, row: int, text: str) -> None:
        row_offsets = (0x80, 0xC0, 0x94, 0xD4)
        self._send(row_offsets[row], self.LCD_CMD)
        for char in text.ljust(self.columns)[: self.columns]:
            self._send(ord(char), self.LCD_CHR)

    def show_hello(self, message: str) -> None:
        self.clear()
        self.write_line(0, message)
        self.write_line(1, "AWR6843")


FONT_5X7 = {
    " ": [0x00, 0x00, 0x00, 0x00, 0x00],
    "!": [0x00, 0x00, 0x5F, 0x00, 0x00],
    "-": [0x08, 0x08, 0x08, 0x08, 0x08],
    ".": [0x00, 0x60, 0x60, 0x00, 0x00],
    "0": [0x3E, 0x51, 0x49, 0x45, 0x3E],
    "1": [0x00, 0x42, 0x7F, 0x40, 0x00],
    "2": [0x42, 0x61, 0x51, 0x49, 0x46],
    "3": [0x21, 0x41, 0x45, 0x4B, 0x31],
    "4": [0x18, 0x14, 0x12, 0x7F, 0x10],
    "5": [0x27, 0x45, 0x45, 0x45, 0x39],
    "6": [0x3C, 0x4A, 0x49, 0x49, 0x30],
    "7": [0x01, 0x71, 0x09, 0x05, 0x03],
    "8": [0x36, 0x49, 0x49, 0x49, 0x36],
    "9": [0x06, 0x49, 0x49, 0x29, 0x1E],
    "A": [0x7E, 0x09, 0x09, 0x09, 0x7E],
    "D": [0x7F, 0x41, 0x41, 0x22, 0x1C],
    "E": [0x7F, 0x49, 0x49, 0x49, 0x41],
    "H": [0x7F, 0x08, 0x08, 0x08, 0x7F],
    "L": [0x7F, 0x40, 0x40, 0x40, 0x40],
    "O": [0x3E, 0x41, 0x41, 0x41, 0x3E],
    "R": [0x7F, 0x09, 0x19, 0x29, 0x46],
    "W": [0x7F, 0x20, 0x18, 0x20, 0x7F],
    "a": [0x20, 0x54, 0x54, 0x54, 0x78],
    "d": [0x38, 0x44, 0x44, 0x48, 0x7F],
    "e": [0x38, 0x54, 0x54, 0x54, 0x18],
    "l": [0x00, 0x41, 0x7F, 0x40, 0x00],
    "o": [0x38, 0x44, 0x44, 0x44, 0x38],
    "r": [0x7C, 0x08, 0x04, 0x04, 0x08],
}


class Ssd1306Oled:
    """Small SSD1306 128x64 OLED over I2C."""

    WIDTH = 128
    HEIGHT = 64
    PAGES = HEIGHT // 8

    def __init__(self, bus, address: int):
        self.bus = bus
        self.address = address
        self.buffer = bytearray(self.WIDTH * self.PAGES)
        self._init_display()

    def _command(self, *commands: int) -> None:
        for command in commands:
            self.bus.write_i2c_block_data(self.address, 0x00, [command])

    def _data(self, values) -> None:
        values = list(values)
        for index in range(0, len(values), 16):
            self.bus.write_i2c_block_data(self.address, 0x40, values[index : index + 16])

    def _init_display(self) -> None:
        self._command(
            0xAE,
            0xD5,
            0x80,
            0xA8,
            0x3F,
            0xD3,
            0x00,
            0x40,
            0x8D,
            0x14,
            0x20,
            0x00,
            0xA1,
            0xC8,
            0xDA,
            0x12,
            0x81,
            0xCF,
            0xD9,
            0xF1,
            0xDB,
            0x40,
            0xA4,
            0xA6,
            0xAF,
        )

    def clear(self) -> None:
        self.buffer = bytearray(self.WIDTH * self.PAGES)
        self.flush()

    def draw_text(self, page: int, text: str) -> None:
        text_width = max(0, len(text) * 6 - 1)
        x = max(0, (self.WIDTH - text_width) // 2)
        offset = page * self.WIDTH + x
        for char in text:
            glyph = FONT_5X7.get(char, FONT_5X7[" "])
            for column in glyph:
                if offset < len(self.buffer):
                    self.buffer[offset] = column
                offset += 1
            if offset < len(self.buffer):
                self.buffer[offset] = 0x00
            offset += 1

    def flush(self) -> None:
        self._command(0x21, 0, self.WIDTH - 1, 0x22, 0, self.PAGES - 1)
        self._data(self.buffer)

    def show_hello(self, message: str) -> None:
        self.clear()
        self.draw_text(2, message)
        self.draw_text(4, "AWR6843")
        self.flush()


def choose_display(kind: str, address: int | None, found: list[int]) -> tuple[str, int]:
    if address is not None:
        if kind == "auto":
            if address in OLED_ADDRS:
                return "oled", address
            return "lcd", address
        return kind, address

    if kind in ("auto", "oled"):
        for candidate in OLED_ADDRS:
            if candidate in found:
                return "oled", candidate

    if kind in ("auto", "lcd"):
        for candidate in LCD_ADDRS:
            if candidate in found:
                return "lcd", candidate

    if kind == "oled":
        return "oled", OLED_ADDRS[0]
    return "lcd", LCD_ADDRS[0]


def main() -> int:
    parser = argparse.ArgumentParser(description="Show Hello World on an I2C display.")
    parser.add_argument("--message", default="Hello World")
    parser.add_argument("--bus", type=int, default=1)
    parser.add_argument("--address", type=parse_address)
    parser.add_argument("--type", choices=("auto", "lcd", "oled"), default="auto")
    args = parser.parse_args()

    SMBus = load_smbus()
    bus = SMBus(args.bus)
    try:
        found = scan_i2c(bus)
        display_type, address = choose_display(args.type, args.address, found)

        if display_type == "oled":
            display = Ssd1306Oled(bus, address)
        else:
            display = I2cLcd(bus, address)

        display.show_hello(args.message)
    finally:
        if hasattr(bus, "close"):
            bus.close()

    print(f"Displayed {args.message!r} on {display_type.upper()} at 0x{address:02X}.")
    if not found:
        print("No I2C devices were detected during scan; check wiring and I2C enablement.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
