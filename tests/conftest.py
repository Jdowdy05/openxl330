"""Shared test helpers: a synthetic FW231 program, the bundled release's upload, opaque uploads, a simulated bus, and
scanners for codec material and for CRC-inversion / readout helpers.

Nothing here reads files outside the repository, and nothing here encodes: the only upload the simulated loader boots
is a bundled release's shipped firmware.enc (images.plaintext_for); any other bytes are an opaque upload. The FW231
program is synthetic (vectors, a device block at DEVCFG_OFFSET, the FW231 marker and a deterministic fill)."""
from __future__ import annotations

import ast
import hashlib
import os
import re
import struct
import tempfile
from pathlib import Path
from types import SimpleNamespace

import pytest
import serial
from serial.tools import list_ports

from openxl330 import images, registers
from openxl330._protocol import framing, packets
from openxl330.sim import FakeClock, SimPort, SimServo
from openxl330.wire import Wire

MARKER_OFFSET = 0x400
# Loader bytes whose CRC-32 is no known loader's: SimServo(..., loader=UNKNOWN_LOADER) for the unknown-loader paths (a
# SimServo's default loader bytes report the proven loader's CRC-32).
UNKNOWN_LOADER = bytes((i * 7 + 3) & 0xFF for i in range(0x4000))
_BOOT_CODES = {1_000_000: 0xFF, 2_000_000: 4, 3_000_000: 5}


@pytest.fixture(autouse=True)
def isolate_home(monkeypatch, tmp_path_factory):
    """Every test gets its own empty $OPENXL330_HOME, so nothing a test does (records, wire logs, calibration) reaches
    the user's real ~/.openxl330. A test may still point it elsewhere with monkeypatch.setenv. Temporary folders the
    code under test makes (the CLI's --sim records) go to the test's own temporary folder too."""
    monkeypatch.setenv("OPENXL330_HOME", str(tmp_path_factory.mktemp("openxl330-home")))
    monkeypatch.setattr(tempfile, "tempdir", str(tmp_path_factory.mktemp("tmp")))


@pytest.fixture(autouse=True)
def isolate_serial(monkeypatch):
    """Require an explicit test double for port opens and host adapter discovery."""
    def unexpected_open(*args, **kwargs):
        pytest.fail("test attempted to open a real serial port")

    monkeypatch.setattr(serial, "Serial", unexpected_open)
    monkeypatch.setattr(list_ports, "comports", lambda: [])


def make_devcfg(servo_id=None, label="", boot_baud=1_000_000):
    """A 32-byte device block as earlier tools wrote it (the package itself never writes one): magic 'XLDC', version 1,
    ID (0xFF = keep the stock NV journal ID), start-up baud code (0xFF = 1 Mbaud), 0xFF, a 20-byte ASCII label, the
    CRC-16 of those 28 bytes, FF FF. With no arguments: the default block every bundled image carries."""
    head = (registers.DEVCFG_MAGIC + bytes([1, 0xFF if servo_id is None else servo_id, _BOOT_CODES[boot_baud], 0xFF])
            + label.encode("ascii").ljust(20, b"\0"))
    return head + struct.pack("<H", framing.crc16(head)) + b"\xff\xff"


def make_fw231_program(size=40960, *, servo_id=None, label="", boot_baud=1_000_000):
    """A synthetic FW231 program of `size` bytes: plausible vectors (stack 0x20002000, Thumb reset
    handler inside the program), the device block at DEVCFG_OFFSET (the default one unless an ID,
    label or start-up baud is given: servos an earlier tool installed with a device-block override),
    the FW231 marker at MARKER_OFFSET, a deterministic fill. It stands for an image older than the
    bundled release (no read 1120 / 1136; A0 reads the loader region)."""
    raw = bytearray((i * 151 + 7) & 0xFF for i in range(size))
    raw[0:8] = struct.pack("<II", 0x20002000, 0x08004101)
    block = make_devcfg(servo_id, label, boot_baud)
    raw[registers.DEVCFG_OFFSET:registers.DEVCFG_OFFSET + registers.DEVCFG_BYTES] = block
    raw[MARKER_OFFSET:MARKER_OFFSET + len(registers.FW231_MARKER)] = registers.FW231_MARKER
    return bytes(raw)


def make_upload(encoded):
    """What Loader.upload sends for these upload bytes, taken as opaque (nothing is encoded here): F7 packets, the
    expected F7 progress, the CRC."""
    encoded = bytes(encoded)
    requests = tuple(packets.build_f7_upload_packets(encoded))
    progress = ((0, 0),) + tuple((i + 1, min((i + 1) * 128, len(encoded))) for i in range((len(encoded) + 127) // 128))
    assert len(requests) == len(progress)
    return SimpleNamespace(encoded=encoded, packets=requests, progress=progress, crc16=framing.crc16(encoded),
                           encoded_sha256=hashlib.sha256(encoded).hexdigest())


def make_packaged(release=images.DEFAULT_RELEASE):
    """The upload of a bundled release, built from its shipped encoded bytes (images.Release.packaged)."""
    return images.Release(release).packaged()


def make_bus(servos, baud=1_000_000):
    """(Wire, SimPort) on one FakeClock, the host at `baud`, no wire log."""
    clock = FakeClock()
    port = SimPort(servos, clock)
    port.baudrate = baud
    return Wire(port, clock=clock, sleep=clock.sleep), port


def make_fw231_servos(n=6):
    """Servos already running FW231 (a synthetic, older image) with the default device block, at their stock NV IDs
    1..n."""
    return [SimServo(i + 1, mode="app", program=make_fw231_program(), seed=i + 1) for i in range(n)]


def flash_snapshot(servo):
    """(loader, own calibration/NVM) bytes: what an install must never change."""
    return bytes(servo.flash[0:0x4000]), bytes(servo.flash[0xE000:0x10000])


# ---------------------------------------------------------------------------------------------- codec material
# The three constants of the upload transform (ECP) that the retained loader undoes. Their digests are NOT in this
# repository: digests of 4-byte values are reversible by a 2^32 search, so publishing them would publish the constants.
# Maintainers keep the list (SHA-256 of each constant's little- and big-endian forms, one hex digest per line, '#'
# comments) outside the repository, at $OPENXL330_CODEC_DIGESTS or ../private/codec-digests.txt next to the checkout.
# Without it the codec-constant scan finds nothing and test_codec_digests_available is skipped.
def _load_codec_digests():
    beside = Path(__file__).resolve().parents[2] / "private" / "codec-digests.txt"
    path = os.environ.get("OPENXL330_CODEC_DIGESTS") or beside
    try:
        lines = Path(path).read_text().splitlines()
    except OSError:
        return frozenset()
    return frozenset(line.strip() for line in lines if re.fullmatch(r"[0-9a-f]{64}", line.strip()))


CODEC_CONSTANT_DIGESTS = _load_codec_digests()
_NUMBER = re.compile(rb"0[xX][0-9a-fA-F_]+|\b[0-9a-fA-F]{8}\b|\b[0-9][0-9_]*\b")


def _is_codec_constant(value):
    return 0 <= value < 1 << 32 and hashlib.sha256(struct.pack("<I", value)).hexdigest() in CODEC_CONSTANT_DIGESTS


def codec_constants_in(data):
    """Where `data` holds one of the codec constants: [(offset, how)]. Text (valid UTF-8) is searched for integer
    literals (hex with or without 0x, decimal); anything else for the 4-byte values in either byte order."""
    data = bytes(data)
    try:
        data.decode("utf-8")
    except UnicodeDecodeError:
        return [(i, "bytes") for i in range(len(data) - 3)
                if hashlib.sha256(data[i:i + 4]).hexdigest() in CODEC_CONSTANT_DIGESTS]
    hits = []
    for m in _NUMBER.finditer(data):
        token = m.group().replace(b"_", b"").decode("ascii")
        if token[:2].lower() == "0x":
            values = [int(token, 16)] if len(token) > 2 else []
        else:
            values = [int(token, 10)] if token.isdigit() else []
            if len(token) == 8:
                values.append(int(token, 16))
        if any(_is_codec_constant(v) for v in values):
            hits.append((m.start(), "text " + token))
    return hits


# ---------------------------------------------------------------------------------------------- readout helpers
# The package must not carry a kit for reading program bytes back out of the retained loader's CRC replies: no CRC
# inversion (brute force over byte values, an inverse table), no status-stream collector of such replies, no request
# allow-list of readout operations. These names were removed and must not come back.
REMOVED_READOUT_NAMES = frozenset({"recover_one", "recover_two", "_inverse_pairs", "StatusStream",
                                   "validate_read_request"})
_READOUT_NAME = re.compile(r"(?i)invers|invert|readout|brute|unfold|recover_(one|two)|status_?stream|read_request")
_READOUT_TEXT = ("read-only xl330 interface", "application readout", "readout")
_CRC_SPACES = frozenset({256, 65536})
_CRC_CONSTANTS = frozenset({0x8005, 0xA001, 0xEDB88320, 0x04C11DB7})


def _call_names(node):
    for sub in ast.walk(node):
        if isinstance(sub, ast.Call):
            func = sub.func
            yield func.id if isinstance(func, ast.Name) else getattr(func, "attr", None)


def _over_byte_or_crc_space(iterator):
    return (isinstance(iterator, ast.Call) and isinstance(iterator.func, ast.Name) and iterator.func.id == "range"
            and iterator.args and isinstance(iterator.args[-1], ast.Constant)
            and iterator.args[-1].value in _CRC_SPACES)


def readout_helpers_in(source, *, crc_home=False, text=False):
    """Why `source` (Python) looks like a CRC-inversion or readout helper: [reason]. Flags a function or class named
    like one, a loop over every byte or CRC value (range(256) / range(65536)) that calls crc16, and, unless `crc_home`
    (the module that implements crc16), any use of the CRC table or a CRC polynomial. text=True also flags readout
    wording."""
    source = source.decode("utf-8") if isinstance(source, bytes) else source
    tree, found = ast.parse(source), []
    for node in ast.walk(tree):
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef)):
            if node.name in REMOVED_READOUT_NAMES or _READOUT_NAME.search(node.name):
                found.append(f"readout helper name {node.name!r}")
        loops = []
        if isinstance(node, (ast.For, ast.AsyncFor)):
            loops.append((node.iter, node))
        elif isinstance(node, (ast.ListComp, ast.SetComp, ast.GeneratorExp, ast.DictComp)):
            loops.extend((g.iter, node) for g in node.generators)
        for iterator, body in loops:
            if _over_byte_or_crc_space(iterator) and "crc16" in set(_call_names(body)):
                found.append(f"crc16 over every value of range({iterator.args[-1].value}) at line {node.lineno}")
        if not crc_home:
            if isinstance(node, ast.Name) and node.id == "TABLE" or isinstance(node, ast.Attribute) and node.attr == "TABLE":
                found.append(f"CRC table used outside crc16 at line {node.lineno}")
            if isinstance(node, ast.Constant) and type(node.value) is int and node.value in _CRC_CONSTANTS:
                found.append(f"CRC polynomial {node.value:#x} outside crc16 at line {node.lineno}")
    if text:
        folded = source.lower()
        found += [f"readout wording {phrase!r}" for phrase in _READOUT_TEXT if phrase in folded]
    return found


@pytest.fixture
def devcfg():
    return make_devcfg


@pytest.fixture
def fw231_program():
    return make_fw231_program


@pytest.fixture
def packaged():
    return make_packaged


@pytest.fixture
def upload_of():
    return make_upload


@pytest.fixture
def sim_bus():
    return make_bus


@pytest.fixture
def fw231_servos():
    return make_fw231_servos


@pytest.fixture
def flash_snapshot_of():
    return flash_snapshot


@pytest.fixture
def codec_scan():
    return codec_constants_in


@pytest.fixture
def readout_scan():
    return readout_helpers_in
