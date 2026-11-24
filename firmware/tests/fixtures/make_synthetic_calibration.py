#!/usr/bin/env python3
"""Generate tests/fixtures/synthetic-calibration.bin: a 4096-byte SYNTHETIC
calibration image in the dense 64-segment layout accepted by
research/calibration/xl330_calibration.c (xl330_calibration_load).

Nothing here is derived from any real servo. All values come from the formulas
below, so the output is byte-for-byte reproducible:

  0x000..0xFFF  erased (0xFF) except the fields listed here
  0x008  u16 0xFFFF   no 4-segment "alternative" table -> dense layout is used
  0x800  u16 0x0002   dense table marker (low byte must be 2)
  0x808  u16 AUX      preserved auxiliary metadata (synthetic, signed)
  0x810  u16 ORIGIN   dense origin (0..4095)
  0x810 + 8*(64-i), i=1..63  u16 knot[i], strictly increasing in (0,4096)
  0xA10  u16 TRIM     timing trim, valid range -600..600
  0xA18  u16 0x0000   trim-valid flag

knot[i] = round(64*i + A*sin(2*pi*i/64) + B*sin(2*pi*3*i/64 + 0.7)),
a smooth, clearly artificial distortion of the ideal 64*i grid.

Usage: make_synthetic_calibration.py OUT.bin   (prints SHA-256)
"""
import hashlib, math, struct, sys

ORIGIN = 1234      # tests/test_fractional_oracle.py asserts this value
AUX = -321
TRIM = 42
A, B = 23.0, 7.0

def build() -> bytes:
    img = bytearray(b'\xff' * 4096)
    put = lambda off, v: struct.pack_into('<H', img, off, v & 0xFFFF)
    put(0x008, 0xFFFF)
    put(0x800, 0x0002)
    put(0x808, AUX)
    put(0x810, ORIGIN)
    knots = [0]
    for i in range(1, 64):
        k = round(64 * i + A * math.sin(2 * math.pi * i / 64) + B * math.sin(2 * math.pi * 3 * i / 64 + 0.7))
        assert knots[-1] < k < 4096
        knots.append(k)
        put(0x810 + 8 * (64 - i), k)
    put(0xA10, TRIM)
    put(0xA18, 0)
    return bytes(img)

if __name__ == '__main__':
    data = build()
    open(sys.argv[1], 'wb').write(data)
    print(f'{sys.argv[1]}: {len(data)} B sha256={hashlib.sha256(data).hexdigest()}')
