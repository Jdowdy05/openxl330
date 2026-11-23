# Derived from the project's hardware-proven Protocol 2.0 framing/CRC module,
# sha256 f55552930f5fb5beaaf11f5ccad3d720bc2130bd0a71e430a2d937aaceb4f7ed.
# Modified: only frame building, the CRC-16, byte stuffing and status parsing are kept (the FA request is framed
# inside wire.Loader._fa); docstrings and messages are reworded for this package.
# Frame construction and strict receive checking share the byte-stuffing operation.
"""Protocol 2.0 frames: CRC-16, byte stuffing, request frames and status-packet parsing. Pure functions, no I/O."""
from __future__ import annotations


def _entry(byte):
    value = byte << 8
    for _ in range(8):
        value = ((value << 1) ^ (0x8005 if value & 0x8000 else 0)) & 0xFFFF
    return value


TABLE = tuple(_entry(byte) for byte in range(256))


def crc16(data, initial=0):
    value = initial
    for byte in data:
        value = ((value << 8) & 0xFFFF) ^ TABLE[((value >> 8) ^ byte) & 0xFF]
    return value


def stuff(payload):
    """Protocol 2.0 byte stuffing for the instruction and parameter bytes."""
    return payload.replace(b'\xff\xff\xfd', b'\xff\xff\xfd\xfd')


def frame(reserved, identifier, instruction, parameters=b''):
    payload = stuff(bytes([instruction]) + parameters)
    body = bytes([255, 255, 253, reserved, identifier]) + (len(payload)+2).to_bytes(2, 'little') + payload
    return body + crc16(body).to_bytes(2, 'little')


def _body(raw):
    if (len(raw) < 10 or raw[:3] != b'\xff\xff\xfd' or raw[3] not in (0, 10)
            or len(raw) != 7 + int.from_bytes(raw[5:7], 'little')
            or crc16(raw[:-2]) != int.from_bytes(raw[-2:], 'little')):
        raise ValueError('Invalid frame, length or CRC')
    payload = raw[7:-2].replace(b'\xff\xff\xfd\xfd', b'\xff\xff\xfd')
    return raw[3], raw[4], payload


def parse_status(raw):
    reserved, identifier, payload = _body(raw)
    if len(payload) < 2 or payload[0] != 0x55:
        raise ValueError('Expected a status packet')
    return {'reserved': reserved, 'id': identifier, 'error': payload[1],
            'data': bytes(payload[2:]), 'raw_hex': raw.hex()}
