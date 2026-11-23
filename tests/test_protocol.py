"""Golden-vector tests for the vendored Protocol 2.0 framing and F7 packet builder (vectors are seeded synthetic data),
and the no-codec rule: the package ships Protocol 2.0 framing and the F7 packet builder, never the upload transform
(ECP), and no helper that reads program bytes back out of the loader's CRC replies."""
from __future__ import annotations

import __future__
import ast
import importlib
import json
import os
import pkgutil
import random
from pathlib import Path

import pytest

import openxl330
from conftest import REMOVED_READOUT_NAMES
from openxl330._protocol import framing, packets
from openxl330.wire import Loader

GOLDEN = json.loads((Path(__file__).parent / "golden" / "protocol_vectors.json").read_text())
PACKAGE = Path(openxl330.__file__).resolve().parent
REPO = Path(__file__).resolve().parent.parent
NOT_PUBLISHED = {".venv", ".pytest_cache", "dist", "build", "__pycache__", ".git"}


def test_frames_match_golden():
    assert len(GOLDEN["frames"]) == 200
    for v in GOLDEN["frames"]:
        got = framing.frame(v["reserved"], v["id"], v["instruction"], bytes.fromhex(v["params"]))
        assert got.hex() == v["frame"]


def test_crc_matches_golden():
    assert len(GOLDEN["crcs"]) == 200
    for v in GOLDEN["crcs"]:
        assert framing.crc16(bytes.fromhex(v["data"])) == v["crc"]


def test_stuffing_round_trip_matches_golden():
    assert any("fffffd" in v["data"] for v in GOLDEN["stuffing"])
    for v in GOLDEN["stuffing"]:
        data = bytes.fromhex(v["data"])
        status = framing.frame(0, 7, 0x55, b"\x00" + data)
        assert status.hex() == v["frame"]
        parsed = framing.parse_status(status)
        assert parsed["data"] == data and parsed["id"] == 7 and parsed["error"] == 0


def test_f7_packets_match_golden():
    """The F7 builder takes the upload as opaque bytes (here seeded synthetic bytes, not an encoded image)."""
    v = GOLDEN["f7"]
    upload = random.Random(v["seed"]).randbytes(v["size"])
    built = packets.build_f7_upload_packets(upload)
    assert len(built) == v["count"]
    assert [p.hex() for p in built[:3]] == v["first3"]


def test_golden_vectors_hold_no_upload_transform():
    assert set(GOLDEN) == {"frames", "crcs", "stuffing", "f7"}


def test_f7_rejects_exact_chunk_multiple():
    with pytest.raises(ValueError):
        packets.build_f7_upload_packets(b"\x00" * 128)


FRAMING_NAMES = frozenset({"TABLE", "crc16", "stuff", "frame", "parse_status"})
PACKETS_NAMES = frozenset({"HEADER", "TARGET_ID", "INSTRUCTION", "CHUNK_BYTES", "crc16", "build_f7_upload_packets"})


def _public_names(module):
    return {name for name, value in vars(module).items()
            if not name.startswith("_") and value is not __future__.annotations}


def test_framing_exports_only_allowed_names():
    """The vendored protocol modules expose exactly these names: frames, CRC-16, stuffing, status parsing and the F7
    packet builder. Nothing else (no readout helper, no FA request builder: only Loader._fa frames FA) may be added
    without updating this list."""
    assert _public_names(framing) == FRAMING_NAMES
    assert _public_names(packets) == PACKETS_NAMES
    assert not REMOVED_READOUT_NAMES & (set(vars(framing)) | set(vars(packets)))
    protocol = importlib.import_module("openxl330._protocol")
    assert {m.name for m in pkgutil.iter_modules(protocol.__path__)} == {"framing", "packets"}
    assert _public_names(protocol) <= {"framing", "packets"}


def test_readout_scanner_flags_the_removed_shapes(readout_scan):
    """The scanner used below flags the shapes of a CRC-inversion / readout helper (shapes only, no working helper)."""
    assert readout_scan("def recover_two(a, b):\n    pass\n")
    assert readout_scan("class StatusStream:\n    pass\n")
    assert readout_scan("def validate_read_request(raw):\n    pass\n")
    assert readout_scan("x = [crc16(b'') for v in range(65536)]\n")
    assert readout_scan("for b in range(256):\n    crc16(b'')\n")
    assert readout_scan("from x import TABLE\nTABLE[0]\n")
    assert readout_scan("p = 0x8005\n")
    assert readout_scan('"""a measured, read-only XL330 interface"""\n', text=True)
    assert not readout_scan("def recover(wire, servo_id):\n    return crc16(b'')\n", text=True)
    assert not readout_scan("p = 0x8005\n", crc_home=True)


def test_no_crc_inversion_or_readout_helper_in_package(readout_scan):
    """No module under openxl330 holds a CRC-inversion or readout helper (by name, by shape or by wording), only the
    framing module does CRC arithmetic, and the loader's CRC check is not public (only Loader.upload runs it)."""
    home = PACKAGE / "_protocol" / "framing.py"
    sources = [p for p in _package_files() if p.suffix == ".py"]
    assert home in sources
    found = {str(p.relative_to(PACKAGE)): readout_scan(p.read_bytes(), crc_home=p == home, text=True)
             for p in sources}
    assert not {p: h for p, h in found.items() if h}
    assert not hasattr(Loader, "fa") and not hasattr(framing, "recover_one") and not hasattr(framing, "fa_request")


def _package_files():
    return [p for p in PACKAGE.rglob("*") if p.is_file() and "__pycache__" not in p.parts]


def test_no_codec_in_package(codec_scan):
    """No module under openxl330 defines an encode or decode function or holds the upload transform's constants, no
    bundled file holds them, and the module that used to carry the encoder is gone."""
    with pytest.raises(ImportError):
        importlib.import_module("openxl330._protocol.ecp")
    modules = [m.name for m in pkgutil.walk_packages([str(PACKAGE)], "openxl330.")]
    assert "openxl330._protocol.ecp" not in modules and "openxl330._packaged" not in modules
    sources = [p for p in _package_files() if p.suffix == ".py"]
    assert sources and not (PACKAGE / "_protocol" / "ecp.py").exists() and not (PACKAGE / "_packaged.py").exists()
    for path in sources:
        names = {n.name for n in ast.walk(ast.parse(path.read_text()))
                 if isinstance(n, (ast.FunctionDef, ast.AsyncFunctionDef))}
        assert not names & {"encode", "decode"}, path
    hits = {str(p.relative_to(PACKAGE)): codec_scan(p.read_bytes()) for p in _package_files()}
    assert not {p: h for p, h in hits.items() if h}


def test_codec_digests_available():
    """The codec-constant scan needs the maintainers' private digest list (see conftest.py); without it the scans above
    and below find nothing, so this test says so instead of passing silently."""
    from conftest import CODEC_CONSTANT_DIGESTS
    if not CODEC_CONSTANT_DIGESTS:
        if os.environ.get("OPENXL330_REQUIRE_CODEC_DIGESTS"):              # the publish workflow: never silent
            pytest.fail("OPENXL330_REQUIRE_CODEC_DIGESTS is set but the private codec digest list did not load")
        pytest.skip("private codec digest list not available: the codec-constant scan is inactive here")
    assert len(CODEC_CONSTANT_DIGESTS) == 6


def test_no_codec_constants_in_the_repository(codec_scan, readout_scan):
    """Everything that gets published (sources, tests, golden vectors, docs) is free of the transform's constants, and
    no Python file in it (tests, examples and docs scripts included) holds a CRC-inversion or readout helper."""
    files = [p for p in REPO.rglob("*") if p.is_file() and not NOT_PUBLISHED & set(p.relative_to(REPO).parts)
             and not any(part.endswith(".egg-info") for part in p.relative_to(REPO).parts)]
    assert any(p.name == "protocol_vectors.json" for p in files)
    hits = {p.relative_to(REPO).as_posix(): codec_scan(p.read_bytes()) for p in files}
    assert not {p: h for p, h in hits.items() if h}
    home = (PACKAGE / "_protocol" / "framing.py").resolve()
    scanners = {REPO / "tests" / "conftest.py", Path(__file__).resolve()}   # they spell out the patterns they flag
    found = {p.relative_to(REPO).as_posix(): readout_scan(p.read_bytes(), crc_home=p.resolve() == home)
             for p in files if p.suffix == ".py" and p.resolve() not in scanners}
    assert any(name.startswith("examples/") for name in found) and any(name.startswith("tests/") for name in found)
    assert not {p: h for p, h in found.items() if h}
