# Derived from the project's hardware-proven F7 upload packet builder,
# sha256 c0c6a3eef43aa8ce049ba6831890148f66d8c6419b81d397b74ee1a0e2df6dd9.
# Modified: the annotations import, the shared Protocol 2.0 CRC implementation, and docstrings and messages reworded
# for this package.
"""Build the retained loader's F7 upload packets offline from opaque upload bytes.

This module contains no serial transport, erase command, execute command or image encoder. It splits the upload into
the loader's size request, 128-byte chunk requests and final-chunk request, and applies Protocol 2.0 byte stuffing to
payloads containing FF FF FD. It does not make an upload valid or flashable.
"""
from __future__ import annotations

from .framing import crc16


HEADER = bytes.fromhex("FF FF FD 0A")
TARGET_ID = 0xFE
INSTRUCTION = 0xF7
CHUNK_BYTES = 128


def _frame(parameters: bytes) -> bytes:
    stuffed = bytearray()
    for byte in bytes((INSTRUCTION,)) + parameters:
        stuffed.append(byte)
        if stuffed[-3:] == b"\xff\xff\xfd":
            stuffed.append(0xFD)
    # Length includes the instruction, stuffed parameters, and two CRC bytes.
    length = len(stuffed) + 2
    if length > 0xFFFF:
        raise ValueError("Stuffed packet exceeds Protocol 2.0 length field")
    body = HEADER + bytes((TARGET_ID,)) + length.to_bytes(2, "little") + stuffed
    return body + crc16(body).to_bytes(2, "little")


def build_f7_upload_packets(image: bytes) -> list[bytes]:
    """Return the size request, the full-chunk requests and the final-chunk request for `image`.

    An upload that is an exact multiple of 128 bytes is rejected: how the loader marks its final block is not
    established, so it is not guessed.
    """
    if not isinstance(image, bytes):
        raise TypeError("ECP image must be bytes")
    if not image or len(image) % CHUNK_BYTES == 0:
        raise ValueError("Upload size must be non-zero and not a multiple of 128 bytes")
    full_count, remainder = divmod(len(image), CHUNK_BYTES)
    if full_count > 0xFFFF or len(image) > 0xFFFFFFFF:
        raise ValueError("Upload exceeds the packet index and size fields")

    requests = [_frame(bytes((1,)) + len(image).to_bytes(4, "little"))]
    for index in range(full_count):
        chunk = image[index * CHUNK_BYTES : (index + 1) * CHUNK_BYTES]
        requests.append(_frame(bytes((2,)) + index.to_bytes(2, "little") + chunk))
    tail = image[full_count * CHUNK_BYTES :]
    assert len(tail) == remainder
    requests.append(_frame(bytes((3,)) + full_count.to_bytes(2, "little") + tail))
    return requests
