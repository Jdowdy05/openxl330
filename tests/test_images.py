"""The bundled release: one identical, pre-encoded image for every servo (firmware.enc) plus our own firmware's
plaintext (firmware.bin), both pinned by manifest.json. The package has no codec: uploads are the shipped bytes."""
from __future__ import annotations

import hashlib
import importlib.util
import json
import shutil
import subprocess
import sys
import tarfile
import zipfile
from email.parser import BytesParser
from pathlib import Path, PurePosixPath

import pytest

from openxl330 import errors, images
from openxl330._protocol import framing, packets
from openxl330.registers import DEVCFG_BYTES, DEVCFG_OFFSET, encode_release_id, parse_devcfg

NAMES = ("fw231-comp2-fast16",)
ROOT = Path(__file__).resolve().parent.parent
IMAGES = ROOT / "src" / "openxl330" / "images"


def _manifest(name):
    return json.loads((IMAGES / name / "manifest.json").read_text())


def _sha256(data):
    return hashlib.sha256(data).hexdigest()


def _copy_release(name, tmp_path):
    folder = tmp_path / name
    shutil.copytree(IMAGES / name, folder)
    return folder


def test_constants():
    assert images.DEFAULT_RELEASE == "fw231-comp2-fast16"
    assert not hasattr(images, "RELEASE_IDS"), "the release ID comes from the manifest only"
    assert images.RELEASES == NAMES
    assert images.release_names() == list(NAMES)
    assert images.PROGRAM_BASE == 0x08004000 and images.PROGRAM_MAX == 40960
    assert not hasattr(images, "package") and not hasattr(images.Release, "for_servo"), "no per-servo images"


def test_only_the_validated_release_is_bundled():
    assert sorted(p.name for p in IMAGES.iterdir() if p.is_dir()) == list(NAMES)
    for name in NAMES:
        assert sorted(p.name for p in (IMAGES / name).iterdir()) == ["firmware.bin", "firmware.enc", "manifest.json"]


@pytest.mark.parametrize("name", NAMES)
def test_bundled_releases_verify(name):
    release = images.Release(name)
    assert len(release.raw) <= 40960 and len(release.raw) == _manifest(name)["raw_bytes"]
    assert release.name == name and release.marker == "FW231-PD-BUS" and release.version == 231
    assert parse_devcfg(release.raw[DEVCFG_OFFSET:DEVCFG_OFFSET + DEVCFG_BYTES])["state"] == "default"


@pytest.mark.parametrize("name", NAMES)
def test_release_encoded_matches_manifest(name):
    m = _manifest(name)
    release = images.Release(name)
    assert (m["raw_file"], m["encoded_file"]) == ("firmware.bin", "firmware.enc")
    assert release.encoded == (IMAGES / name / "firmware.enc").read_bytes()
    assert _sha256(release.encoded) == m["encoded_sha256"] and len(release.encoded) == m["encoded_bytes"]
    assert _sha256(release.raw) == m["raw_sha256"]
    padded = images.pad_program(release.raw)
    assert (len(padded), _sha256(padded)) == (m["padded_program_bytes"], m["padded_program_sha256"])
    assert len(release.encoded) == len(padded), "the upload transform is length-preserving"


def test_fast16_pins_release_id_and_known_loaders():
    m = _manifest("fw231-comp2-fast16")
    assert (m["raw_bytes"], m["raw_sha256"]) == (
        40948, "c012a5ec2865f5a27b60345a0517adfbc82e4f417e99d49d97a76a7152ba1413")
    assert (m["encoded_bytes"], m["encoded_sha256"]) == (
        40952, "42f23953b9ea0c398d4686a10d9726841a9007718b0a0568867e108d158179d8")
    assert (m["padded_program_bytes"], m["padded_program_sha256"]) == (
        40952, "4aadb09ec9681e75c5760ed04e446f13c9a5c393632c975712e6670d238a8101")
    assert m["source_tree_sha256"].startswith("adbf24ee")
    release = images.Release("fw231-comp2-fast16")
    assert release.packaged().crc16 == 0xE76F
    rid = release.release_id
    assert rid == dict(number=16, variant="comp", abi=4, source_id=0xADBF24EE)
    assert (m["release_number"], m["abi"]) == (16, 4)
    assert encode_release_id(**rid)[:8] in release.raw, "the program carries the words read 1120 reports"
    assert m["known_loaders"] == {"b6934188": "retained ROBOTIS loader v6"}
    assert release.known_loaders == {0xB6934188: "retained ROBOTIS loader v6"}
    assert images.known_loaders() == {0xB6934188: "retained ROBOTIS loader v6"}


@pytest.mark.parametrize("name", NAMES)
def test_packaged_from_shipped_bytes(name, devcfg):
    m = _manifest(name)
    release = images.Release(name)
    shipped = (IMAGES / name / "firmware.enc").read_bytes()
    pack = release.packaged()
    assert pack.encoded == shipped and pack.encoded_sha256 == m["encoded_sha256"]
    assert pack.packets == tuple(packets.build_f7_upload_packets(shipped))
    assert pack.crc16 == framing.crc16(shipped)
    assert len(pack.progress) == len(pack.packets) and pack.progress[0] == (0, 0)
    assert pack.progress[-1] == (len(pack.packets) - 1, len(shipped))
    assert [p for _, p in pack.progress[1:-1]] == [128 * (k + 1) for k in range(len(pack.progress) - 2)]
    assert pack.raw == images.pad_program(release.raw) and pack.raw_sha256 == m["padded_program_sha256"]
    assert parse_devcfg(pack.devcfg)["state"] == "default" and pack.devcfg == devcfg()
    assert release.packaged() == pack


def test_plaintext_for_looks_up_the_bundled_uploads():
    pack = images.Release(images.DEFAULT_RELEASE).packaged()
    assert images.plaintext_for(pack.encoded) == pack.raw
    assert images.plaintext_for(bytearray(pack.encoded)) == pack.raw
    tampered = bytearray(pack.encoded)
    tampered[100] ^= 1
    for other in (pack.raw, pack.encoded[:-8], bytes(tampered), b"", b"\xff" * len(pack.encoded)):
        assert images.plaintext_for(other) is None


@pytest.mark.parametrize("bad", ["../x", "", "fw231-comp2-fast9", "fw231-comp2-fast10", "fw231-comp2-fast11",
                                 "fw231-comp2-fast12", "fw231-comp2-fast13", "fw231-comp2-fast14/../fast14", 5])
def test_unknown_release_refused(bad):
    with pytest.raises(ValueError):
        images.Release(bad)
    with pytest.raises(errors.InvalidArgument):
        images.Release(bad)


@pytest.mark.parametrize("file, offset", [("firmware.bin", 1000), ("firmware.enc", 1000), ("firmware.enc", -1)])
def test_corrupt_release_is_an_error(tmp_path, file, offset):
    folder = _copy_release(NAMES[0], tmp_path)
    data = bytearray((folder / file).read_bytes())
    data[offset] ^= 1
    (folder / file).write_bytes(bytes(data))
    with pytest.raises(errors.Error, match="reinstall openxl330"):
        images.Release(NAMES[0], root=tmp_path)


def test_missing_upload_keeps_diagnostics_and_off_available(tmp_path):
    """A broken local bundle must block installation without blocking connected-servo diagnostics."""
    shutil.copytree(IMAGES.parent, tmp_path / "openxl330", ignore=shutil.ignore_patterns("__pycache__"))
    (tmp_path / "openxl330" / "images" / NAMES[0] / "firmware.enc").unlink()
    code = """
from openxl330 import Bus, Calibration, Error, cli, install
from openxl330 import bus as bus_module
from openxl330.registers import FW231_MARKER
from openxl330.sim import SimPort, SimServo

SimServo._release_id = staticmethod(
    lambda program: dict(number=16, variant="comp", abi=4, source_id=0xADBF24EE)
)
device = SimServo(1, mode="app", program=FW231_MARKER)
device.enabled = 1
bus_module._open_serial = lambda *args: SimPort([device])
bus_module.adapters.ftdi_latency_warning = lambda port: None
bus_module.Calibration.load = classmethod(lambda cls: cls())
assert cli.main(["version"]) == 0
assert cli.main(["status", "--port", "SIM", "--ids", "1"]) == 0
assert cli.main(["off", "--port", "SIM", "--ids", "1"]) == 0 and not device.enabled
with Bus(SimPort([device]), calibration=Calibration(), check_adapter=False) as bus:
    try:
        install.update(bus.wire, servo_id=1)
    except Error as exc:
        assert "reinstall openxl330" in str(exc)
    else:
        raise AssertionError("update accepted a missing upload")
    assert not bus.port.tx_log
"""
    result = subprocess.run([sys.executable, "-c", code], cwd=tmp_path, capture_output=True, text=True)
    assert result.returncode == 0, result.stdout + result.stderr
    assert "FW231 release 16" in result.stdout


@pytest.mark.parametrize("change", ["truncated upload", "missing upload", "wrong name", "manifest not JSON",
                                    "device block not default", "no release number", "no ABI",
                                    "release ID not in the program", "source hash not the program's",
                                    "known loaders not a table", "no known loaders"])
def test_inconsistent_release_is_an_error(tmp_path, change, devcfg):
    """Every bundled release must be complete: a release ID missing from its manifest, or one the program does not
    carry, is a corrupt package (never a release that skips the read-1120 check)."""
    folder = _copy_release(NAMES[0], tmp_path)
    m = _manifest(NAMES[0])
    if change == "truncated upload":
        short = (folder / "firmware.enc").read_bytes()[:-8]
        (folder / "firmware.enc").write_bytes(short)
        m.update(encoded_sha256=_sha256(short))
    elif change == "missing upload":
        (folder / "firmware.enc").unlink()
    elif change == "wrong name":
        m["name"] = "fw231-comp2-fast11"
    elif change == "device block not default":
        raw = bytearray((folder / "firmware.bin").read_bytes())
        raw[DEVCFG_OFFSET:DEVCFG_OFFSET + DEVCFG_BYTES] = devcfg(7, "wrist")
        (folder / "firmware.bin").write_bytes(bytes(raw))
        padded = images.pad_program(bytes(raw))
        m.update(raw_sha256=_sha256(bytes(raw)), padded_program_sha256=_sha256(padded))
    elif change == "no release number":
        del m["release_number"]
    elif change == "no ABI":
        del m["abi"]
    elif change == "release ID not in the program":
        m["release_number"] += 1
    elif change == "source hash not the program's":
        m["source_tree_sha256"] = "0" * 64
    elif change == "known loaders not a table":
        m["known_loaders"] = {"not hex": "x"}
    elif change == "no known loaders":
        del m["known_loaders"]
    (folder / "manifest.json").write_text("{" if change == "manifest not JSON" else json.dumps(m))
    with pytest.raises(errors.Error, match="reinstall openxl330"):
        images.Release(NAMES[0], root=tmp_path)


def test_manifests_have_no_private_names_or_patching_notes():
    for name in NAMES:
        m = _manifest(name)
        text = json.dumps(m)
        assert "firmware/" not in text and "package_release" not in text and "research" not in text
        assert m["known_loaders"] and all(name == name.split(" (")[0] for name in m["known_loaders"].values())
        assert not {"build", "note", "sources", "default_encoded_sha256", "symbols_file", "symbols_sha256"} & set(m)
        assert m["source_hashes"] and all(len(h) == 64 and int(h, 16) >= 0 for h in m["source_hashes"])
        assert m["name"] == name and (m["raw_file"], m["encoded_file"]) == ("firmware.bin", "firmware.enc")


@pytest.fixture(scope="module")
def release_artifacts(tmp_path_factory):
    out = tmp_path_factory.mktemp("release")
    command = [sys.executable, "-m", "build", "--outdir", str(out), str(ROOT)]
    if importlib.util.find_spec("hatchling") is not None:
        command.append("--no-isolation")
    result = subprocess.run(command, capture_output=True, text=True, encoding="utf-8", errors="replace")
    assert result.returncode == 0, result.stdout + result.stderr
    return next(out.glob("openxl330-*.whl")), next(out.glob("openxl330-*.tar.gz"))


def _check_archive(files, images_prefix, codec_scan, readout_scan, *, banner=None):
    blocked = {
        ".git", ".venv", ".pytest_cache", "__pycache__", "records", "dev", "sdd",
        "calibration.json", "loader.bin", "nvm.bin", "ecp.py", "_packaged.py",
    }
    private_markers = [
        b"/" + b"users/",
        b"xl330-firmware" + b"-flash",
        b"neuro" + b"adaptive",
        b"unit" + b"b",
        b"jor" + b"dan",
    ]
    images_expected = {
        f"{images_prefix}/{release}/{filename}"
        for release in NAMES
        for filename in ("firmware.bin", "firmware.enc", "manifest.json")
    }
    assert {name for name in files if name.startswith(images_prefix + "/")} == images_expected
    for name, content in files.items():
        path = PurePosixPath(name)
        assert not path.is_absolute() and ".." not in path.parts and "\\" not in name, name
        assert not blocked.intersection(part.lower() for part in path.parts), name
        assert path.suffix.lower() not in (".pyc", ".pyo", ".ecp"), name
        folded_name, folded_content = name.lower().encode(), content.lower()
        assert not any(marker in folded_name or marker in folded_content for marker in private_markers), name
        if name in images_expected:
            assert content == (IMAGES / path.parent.name / path.name).read_bytes(), name
        elif name != banner:
            assert b"\0" not in content, name
            try:
                content.decode("utf-8")
            except UnicodeDecodeError:
                pytest.fail(f"unexpected binary file in distribution: {name}")
        assert not codec_scan(content), name
        if name.endswith(".py") and PurePosixPath(name).name not in ("conftest.py", "test_protocol.py"):
            home = name.endswith("openxl330/_protocol/framing.py")
            in_package = "/openxl330/" in "/" + name and "tests/" not in name
            assert not readout_scan(content, crc_home=home, text=in_package), name


def test_wheel_has_no_codec(release_artifacts, codec_scan, readout_scan):
    """The built wheel carries the expected image files and no private or codec material."""
    wheel_path, _ = release_artifacts
    with zipfile.ZipFile(wheel_path) as wheel:
        names = [name for name in wheel.namelist() if not name.endswith("/")]
        assert len(set(names)) == len(names)
        files = {name: wheel.read(name) for name in names}
    assert all(name.startswith("openxl330/") or ".dist-info/" in name for name in files)
    _check_archive(files, "openxl330/images", codec_scan, readout_scan)
    metadata_bytes = next(data for name, data in files.items() if name.endswith(".dist-info/METADATA"))
    metadata = BytesParser().parsebytes(metadata_bytes, headersonly=True)
    assert metadata["Requires-Python"] == ">=3.10"
    assert metadata["License-Expression"] == "MIT"
    assert metadata.get_all("License-File") == ["LICENSE"]
    licence = next(data for name, data in files.items() if name.endswith(".dist-info/licenses/LICENSE"))
    assert licence == (ROOT / "LICENSE").read_bytes()


def test_sdist_has_no_codec_or_private_files(release_artifacts, codec_scan, readout_scan):
    """Check the source archive independently, including its docs, examples, and tests."""
    _, sdist_path = release_artifacts
    files = {}
    with tarfile.open(sdist_path, "r:gz") as sdist:
        for member in sdist.getmembers():
            path = PurePosixPath(member.name)
            assert not path.is_absolute() and ".." not in path.parts
            assert member.isfile() or member.isdir(), member.name
            if member.isfile():
                relative = "/".join(path.parts[1:])
                assert relative not in files
                files[relative] = sdist.extractfile(member).read()
    _check_archive(files, "src/openxl330/images", codec_scan, readout_scan, banner="assets/banner.png")
    expected = {
        "LICENSE", "README.md", "CHANGELOG.md", "pyproject.toml", "assets/banner.png",
        ".github/workflows/ci.yml", "docs/generate_registers.py", "tests/test_docs.py",
    }
    expected.update(f"docs/{name}.md" for name in
                    ("quickstart", "install", "wiring", "adapters", "api", "registers", "torque", "troubleshooting"))
    expected.update(f"examples/{name}.py" for name in ("scan", "install_one", "arm_six", "read_torque", "calibrate"))
    assert expected <= files.keys()
    assert "CLI_NOTES.md" not in files, "internal notes are not published"
    metadata = BytesParser().parsebytes(files["PKG-INFO"], headersonly=True)
    assert metadata["License-Expression"] == "MIT"
    assert metadata.get_all("License-File") == ["LICENSE"]
    assert files["LICENSE"] == (ROOT / "LICENSE").read_bytes()
