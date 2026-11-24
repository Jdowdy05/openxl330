"""The installer against the simulated bus: first install from stock, update, recover, the shared-bus census rules,
the untested-servo policy, the loader digest, the release ID and the failure records. Every servo gets the one
identical bundled image and keeps the ID it has on its stock firmware (its NV journal ID). No port is opened; records
go to tmp dirs."""
from __future__ import annotations

import json
import struct
import zlib
from pathlib import Path

import pytest

from conftest import UNKNOWN_LOADER
from openxl330 import images, install, registers
from openxl330.errors import (BusUncertain, Error, InstallError, InvalidArgument, NoReply, UnsupportedServo,
                              UntestedServo)
from openxl330.images import DEFAULT_RELEASE, Release
from openxl330.sim import SimServo, default_eeprom
from openxl330.wire import APP_BAUD, LOADER_BAUD, Loader

RELEASE = Release(DEFAULT_RELEASE)
PACK = RELEASE.packaged()
DEVBLOCK = slice(0x4000 + registers.DEVCFG_OFFSET, 0x4000 + registers.DEVCFG_OFFSET + registers.DEVCFG_BYTES)
RELEASE_ID = dict(RELEASE.release_id, source_id=f"{RELEASE.release_id['source_id']:08x}")   # as receipts record it


# ---------------------------------------------------------------------------------------------- helpers
@pytest.fixture
def records(tmp_path):
    return tmp_path / "records"


@pytest.fixture
def fw231_at(sim_bus, records):
    """A factory servo whose ID was set to `sid` on its stock firmware, given FW231 by a first install (alone on its
    bus, records under `records`)."""
    def make(sid, seed=50):
        s = SimServo(sid, mode="stock", seed=seed)
        wire, _ = sim_bus([s], LOADER_BAUD)
        assert install.from_stock(wire, stock_id=sid, records=records)["completed"]
        return s
    return make


@pytest.fixture
def installed(fw231_at):
    """n factory servos given IDs 1..n on stock firmware, then FW231 (one at a time)."""
    return lambda n=6: [fw231_at(i + 1, seed=i + 1) for i in range(n)]


def stock_servo(nv_id=1, *, seed=1, version=50, model=1200, baud_index=1):
    eeprom = default_eeprom(nv_id, baud_index=baud_index)
    eeprom[0:2] = struct.pack("<H", model)
    eeprom[6] = version
    return SimServo(nv_id, mode="stock", seed=seed, eeprom=eeprom)


def older_fw231(nv_id, *, seed, fw231_program, **block):
    """A servo on an older FW231 image (no read 1120 / 1136), optionally with a device-block override (`servo_id`,
    `label`, `boot_baud`) that an earlier tool wrote."""
    return SimServo(nv_id, mode="app", program=fw231_program(**block), seed=seed)


def snapshot(servo):
    return bytes(servo.flash[0:0x4000]), bytes(servo.flash[0xE000:0x10000])


def devblock(servo):
    return registers.parse_devcfg(bytes(servo.flash[DEVBLOCK]))


def last_receipt(root):
    return json.loads((sorted((Path(root) / "_runs").iterdir())[-1] / "receipt.json").read_text())


def tx_frames(port, instruction):
    """Host frames with this instruction byte in the simulated port's TX log (instruction = byte 7)."""
    return [d for _, d in port.tx_log if d[:3] == b"\xff\xff\xfd" and len(d) > 7 and d[7] == instruction]


def error_on_ping(servo, error=2):
    """A device that answers a ping with an error status (still carrying model/firmware)."""
    real = servo.on_frame

    def reply(f, clock):
        r = real(f, clock)
        if r and f["instruction"] == 1:
            return (r[0], servo.status(error, struct.pack("<HB", 1200, 52)))
        return r
    servo.on_frame = reply
    return servo


def tamper_after_launch(servo, fn):
    real = servo.reset

    def reset(mode=None):
        real(mode)
        if mode == "app" and servo.max_write:
            fn(servo)
    servo.reset = reset


def recovery(servo_id):
    """While the servo may be in the retained loader."""
    return f"openxl330 recover --port SIM --id {servo_id}"


def reinstall(servo_id):
    """Once F8 was acknowledged: the servo runs the new firmware."""
    return f"openxl330 update --port SIM --id {servo_id}"


def in_place(servo):
    """The bundled release is in the servo's program region, byte for byte."""
    return bytes(servo.flash[0x4000:0x4000 + len(PACK.raw)]) == PACK.raw


SIM_LOADER_CRC = zlib.crc32(bytes(SimServo(1).flash[0:0x4000]))      # the proven loader's (sim default)
UNKNOWN_CRC = zlib.crc32(UNKNOWN_LOADER)


# ---------------------------------------------------------------------------------------------- tables
def test_proven_tables_and_records_root(monkeypatch, tmp_path):
    assert install.PROVEN_STOCK == {(1200, 50), (1200, 52), (1200, 53)}
    assert install.PROVEN_LOADERS == {0xB6934188: "retained ROBOTIS loader v6"} == images.known_loaders()
    assert install.SCHEMA == "xlbus-install-v1"
    monkeypatch.setenv("OPENXL330_HOME", str(tmp_path / "home"))
    assert install.records_root() == tmp_path / "home" / "records"


def test_records_default_to_records_root(monkeypatch, tmp_path, sim_bus):
    monkeypatch.setenv("OPENXL330_HOME", str(tmp_path / "home"))
    s = SimServo(1, mode="stock", seed=3)
    wire, _ = sim_bus([s], LOADER_BAUD)
    r = install.from_stock(wire)
    root = tmp_path / "home" / "records"
    assert r["completed"] and r["run_dir"].startswith("_runs/") and (root / r["run_dir"] / "receipt.json").is_file()
    assert r["servo_dir"] == r["fingerprint"] and (root / r["servo_dir"] / "nvm.bin").is_file()
    assert str(tmp_path) not in (root / r["run_dir"] / "receipt.json").read_text(), "shareable: no local paths"


def test_commands():
    class Port:
        port = "/dev/ttyUSB0"
    wire = type("W", (), {"port": Port()})()
    assert install.recovery_command(wire, 4) == "openxl330 recover --port /dev/ttyUSB0 --id 4"
    assert install.update_command(wire, 4) == "openxl330 update --port /dev/ttyUSB0 --id 4"


# ---------------------------------------------------------------------------------------------- first install
def test_from_stock_keeps_nv_id(sim_bus, records):
    """A factory servo given ID 4 on its stock firmware comes up as ID 4 at 1 Mbaud: the image is identical for every
    servo (default device block), and the firmware answers at the stock NV journal ID."""
    s = stock_servo(4, seed=20)
    wire, port = sim_bus([s], LOADER_BAUD)
    r = install.from_stock(wire, stock_id=4, records=records)
    assert r["completed"] and r["servo_id"] == 4 and r["release_id"] == RELEASE_ID
    assert (s.mode, s.version, s.effective_id(), s.baud, port.baudrate) == ("app", 231, 4, APP_BAUD, APP_BAUD)
    assert in_place(s) and devblock(s)["state"] == "default"
    assert (r["program_sha256"], r["encoded_sha256"]) == (PACK.raw_sha256, PACK.encoded_sha256)
    assert wire.ping(4)["firmware"] == 231 and sorted(wire.scan()) == [4]
    for gone in ("label", "boot_baud", "new_id"):
        assert gone not in r
    verified = [x for x in r["steps"] if x["step"] == "verified"][0]
    assert verified["release_id"] == RELEASE_ID and verified["loader_digest"] == "read 1136"


def test_six_factory_servos_one_at_a_time(sim_bus, records):
    servos = []
    for i in range(6):
        s = SimServo(i + 1, mode="stock", seed=i + 1)                # ID i+1 set on the stock firmware
        loader0, nvm0 = snapshot(s)
        wire, _ = sim_bus([s], LOADER_BAUD)
        receipt = install.from_stock(wire, stock_id=i + 1, records=records)
        assert receipt["completed"]
        assert snapshot(s) == (loader0, nvm0), "loader and own NVM must be untouched"
        assert s.writes_outside_program == 0
        assert s.max_write <= 0xE000
        assert in_place(s), "every servo gets the identical image"
        assert (s.mode, s.version, s.effective_id(), s.baud) == ("app", 231, i + 1, 1_000_000)
        assert (receipt["release"], receipt["program_sha256"]) == (DEFAULT_RELEASE, PACK.raw_sha256)
        servos.append(s)
    wire, _ = sim_bus(servos)
    census = wire.scan()
    assert sorted(census) == [1, 2, 3, 4, 5, 6] and {p["firmware"] for p in census.values()} == {231}
    prints = {p.name for p in records.iterdir() if not p.name.startswith("_")}
    assert len(prints) == 6, "one record folder per physical servo"


def test_refuses_more_than_one_servo(sim_bus, records):
    a, b = SimServo(1, seed=1), SimServo(2, seed=2)
    wire, port = sim_bus([a, b], LOADER_BAUD)
    with pytest.raises(InstallError) as e:
        install.from_stock(wire, records=records)
    assert e.value.recovery is None and e.value.step == "census"
    assert (a.mode, b.mode, tx_frames(port, 0xF9)) == ("stock", "stock", [])


def test_two_factory_servos_at_one_id_name_dynamixel_wizard(sim_bus, records):
    a, b = SimServo(1, seed=1), SimServo(1, seed=2)                 # both still at the factory ID 1
    wire, port = sim_bus([a, b], LOADER_BAUD)
    with pytest.raises(InstallError) as e:
        install.from_stock(wire, records=records)
    assert isinstance(e.value.__cause__, BusUncertain) and e.value.step == "census" and e.value.recovery is None
    assert "Dynamixel Wizard" in str(e.value) and tx_frames(port, 0xF9) == []


def test_nv_journal_compare_detects_a_difference(sim_bus, records):
    s = SimServo(1, seed=9)
    wire, _ = sim_bus([s], LOADER_BAUD)
    real_reset = s.reset

    def reset(mode=None):
        real_reset(mode)
        if mode in (None, "app"):
            s.eeprom[20] ^= 0x40       # our firmware would see a different homing offset
    s.reset = reset
    with pytest.raises(InstallError) as e:
        install.from_stock(wire, records=records)
    assert "[20]" in str(e.value) and "unexpected" in str(e.value) and "share" in str(e.value)
    assert e.value.recovery is None and "openxl330 update" not in str(e.value)    # re-installing would not fix it
    r = last_receipt(records)
    assert (r["phase"], r["in_loader"], r["share_receipt"], "recovery" in r) == ("launched", False, True, False)


def test_v52_at_1m_is_proven(sim_bus, records):
    s = stock_servo(4, version=52, baud_index=3, seed=12)
    wire, _ = sim_bus([s])
    r = install.from_stock(wire, stock_id=4, stock_baud=APP_BAUD, records=records)
    assert r["completed"] and (r["stock_firmware"], r["allow_untested"], r["untested"]) == (52, False, None)
    assert (s.mode, s.effective_id()) == ("app", 4)


def test_already_ours_is_refused_by_from_stock(sim_bus, records, fw231_servos):
    s = fw231_servos(1)[0]
    wire, port = sim_bus([s])
    with pytest.raises(InstallError) as e:
        install.from_stock(wire, stock_id=1, stock_baud=APP_BAUD, records=records)
    assert "update" in str(e.value) and tx_frames(port, 0xF9) == []


def test_from_stock_refuses_taken_id(sim_bus, records, fw231_at):
    """A FW231 servo already answers at ID 4 (1 Mbaud): a factory servo whose stock ID is also 4 would collide with it
    once installed, so the install stops before F9 and says to set a unique ID in Dynamixel Wizard first."""
    taken = fw231_at(4, seed=41)                                    # FW231 at ID 4, 1 Mbaud
    new = stock_servo(4, seed=40)                                   # stock ID 4 at 57600
    wire, port = sim_bus([taken, new], LOADER_BAUD)
    with pytest.raises(InstallError) as e:
        install.from_stock(wire, stock_id=4, records=records)
    msg = str(e.value)
    assert "ID 4 is already used" in msg and "Dynamixel Wizard" in msg and "1000000" in msg
    assert e.value.step == "census_other_bauds" and e.value.recovery is None
    assert new.mode == "stock" and tx_frames(port, 0xF9) == [], "refused before any loader traffic"
    assert (taken.mode, taken.faults, port.baudrate) == ("app", 0, LOADER_BAUD)
    # a stock servo at 1 Mbaud (a V52) answering at the same ID as the FW231 one: both reply to the census ping
    v52 = stock_servo(4, version=52, baud_index=3, seed=42)
    wire, port = sim_bus([taken, v52])
    with pytest.raises(InstallError) as e:
        install.from_stock(wire, stock_id=4, stock_baud=APP_BAUD, records=records)
    assert isinstance(e.value.__cause__, BusUncertain) and e.value.step == "census" and e.value.recovery is None
    assert "Dynamixel Wizard" in str(e.value) and tx_frames(port, 0xF9) == [] and v52.mode == "stock"


# ---------------------------------------------------------------------------------------------- untested-servo policy
def test_untested_stock_version_refused_before_f9(sim_bus, records):
    s = stock_servo(version=51)
    wire, port = sim_bus([s], LOADER_BAUD)
    with pytest.raises(UntestedServo) as e:
        install.from_stock(wire, records=records)
    msg = str(e.value)
    assert "V51" in msg and "proven: V50, V52, V53" in msg and "--allow-untested" in msg
    assert isinstance(e.value, InstallError) and e.value.step == "identify" and e.value.recovery is None
    assert tx_frames(port, 0xF9) == [] and s.mode == "stock"
    r = last_receipt(records)
    assert (r["completed"], r["in_loader"], r["phase"], r["stock_firmware"]) == (False, False, None, 51)


def test_untested_allowed_with_flag_records_it(sim_bus, records):
    s = stock_servo(version=51, seed=13)
    wire, _ = sim_bus([s], LOADER_BAUD)
    r = install.from_stock(wire, allow_untested=True, records=records)
    assert r["completed"] and r["allow_untested"] is True and "V51" in r["untested"]
    assert r["share_receipt"] is True
    assert last_receipt(records)["allow_untested"] is True
    assert (s.mode, s.version, s.effective_id()) == ("app", 231, 1)


def test_unknown_model_refused(sim_bus, records):
    for allow in (False, True):
        s = stock_servo(model=1190)
        wire, port = sim_bus([s], LOADER_BAUD)
        with pytest.raises(UnsupportedServo) as e:
            install.from_stock(wire, allow_untested=allow, records=records)
        assert "1190" in str(e.value) and "XL330-M288" in str(e.value)
        assert tx_frames(port, 0xF9) == [] and s.mode == "stock"


def test_unknown_loader_reported_after_install(sim_bus, records):
    s = SimServo(1, mode="stock", seed=14, loader=UNKNOWN_LOADER)
    wire, _ = sim_bus([s], LOADER_BAUD)
    r = install.from_stock(wire, records=records)
    assert r["completed"] and r["loader_known"] is False and r["share_receipt"] is True
    assert r["loader_crc32"] == f"{UNKNOWN_CRC:08x}" and r["share_why"] and r["loader_digest"] == "read 1136"
    assert [w for w in r["warnings"] if w.startswith("unknown loader version, untested")] and len(r["warnings"]) == 1
    on_disk = last_receipt(records)
    assert (on_disk["loader_known"], on_disk["share_receipt"], on_disk["warnings"]) == (False, True, r["warnings"])


def test_known_loader_needs_no_report(sim_bus, records):
    assert SIM_LOADER_CRC == 0xB6934188 and SIM_LOADER_CRC in install.PROVEN_LOADERS
    s = SimServo(1, mode="stock", seed=15)
    wire, _ = sim_bus([s], LOADER_BAUD)
    r = install.from_stock(wire, records=records)
    assert (r["loader_known"], r["loader_name"], r["share_receipt"], r["share_why"], r["warnings"]) == (
        True, "retained ROBOTIS loader v6", False, [], [])
    wire, _ = sim_bus([s])
    r = install.update(wire, servo_id=1, records=records)
    assert (r["loader_known"], r["share_receipt"], r["warnings"]) == (True, False, [])


def history(records, receipt):
    """The servo's loader CRC history: [(crc32 hex, utc)], one line per install."""
    lines = (records / receipt["servo_dir"] / "loader.crc32").read_text().splitlines()
    return [tuple(line.split()) for line in lines]


def test_loader_history_has_one_line_per_install(sim_bus, records):
    s = SimServo(1, mode="stock", seed=17)
    wire, _ = sim_bus([s], LOADER_BAUD)
    first = install.from_stock(wire, records=records)
    wire, _ = sim_bus([s])
    install.update(wire, servo_id=1, records=records)
    r = install.update(wire, servo_id=1, records=records)
    lines = history(records, r)
    assert [crc for crc, _ in lines] == [f"{SIM_LOADER_CRC:08x}"] * 3 and r["servo_dir"] == first["servo_dir"]
    assert all(utc.endswith("Z") and len(utc) == 23 for _, utc in lines) and r["warnings"] == []


def test_loader_change_between_installs_is_only_a_warning(sim_bus, records):
    """The loader differs from the one this servo's last install recorded: the install goes on, with a warning. A change
    during one install is still an error (test_changed_loader_detected)."""
    s = SimServo(1, mode="stock", seed=18)
    wire, _ = sim_bus([s], LOADER_BAUD)
    install.from_stock(wire, records=records)
    s.flash[0x100] ^= 1                                             # between two installs
    changed = zlib.crc32(bytes(s.flash[0:0x4000]))
    wire, _ = sim_bus([s])
    r = install.update(wire, servo_id=1, records=records)
    assert r["completed"] and in_place(s)
    assert r["warnings"] == [f"unknown loader version, untested (CRC-32 {changed:08x})",
                             f"loader CRC changed since the last install ({SIM_LOADER_CRC:08x} -> {changed:08x})"]
    assert r["share_receipt"] is True and last_receipt(records)["warnings"] == r["warnings"]
    assert [crc for crc, _ in history(records, r)] == [f"{SIM_LOADER_CRC:08x}", f"{changed:08x}"]
    r = install.update(wire, servo_id=1, records=records)          # compared with the LAST line: no change now
    assert r["warnings"] == [f"unknown loader version, untested (CRC-32 {changed:08x})"]


# ---------------------------------------------------------------------------------------------- loader digest
def test_loader_crc32_prefers_read_1136(sim_bus, fw231_servos):
    s = fw231_servos(1)[0]
    crc = zlib.crc32(bytes(s.flash[0:0x4000]))
    real = s.read
    wire, port = sim_bus([s])
    assert install.loader_crc32(wire, 1) == (crc, "A0 read, hashed in memory (image without read 1136)")
    sent = len(tx_frames(port, 0xA0))
    s.read = lambda a, n: (0, struct.pack("<II", crc, 16384)) if (a, n) == (1136, 8) else real(a, n)
    assert install.loader_crc32(wire, 1) == (crc, "read 1136")
    assert len(tx_frames(port, 0xA0)) == sent, "no A0 read of the loader when the firmware gives its digest"
    s.read = lambda a, n: (0, struct.pack("<II", crc, 8192)) if (a, n) == (1136, 8) else real(a, n)
    with pytest.raises(InstallError) as e:
        install.loader_crc32(wire, 1)
    assert "8192" in str(e.value)


def test_loader_digest_read_waits_long_enough(sim_bus, installed):
    """Read 1136 computes a CRC-32 over 16 KB on the servo (about 12.7 ms measured): its timeout is at least 100 ms."""
    s = installed(1)[0]
    wire, port = sim_bus([s])
    calls, real = [], wire.read

    def read(servo_id, address, size, timeout=0.1):
        calls.append((address, timeout))
        return real(servo_id, address, size, timeout)
    wire.read = read
    assert install.loader_crc32(wire, 1) == (SIM_LOADER_CRC, "read 1136")
    assert [t for a, t in calls if a == 1136] and all(t >= 0.1 for a, t in calls if a == 1136)
    assert not [f for f in tx_frames(port, 0xA0) if struct.unpack_from("<I", f, 8)[0] < 0x08004000]


def test_records_hold_no_loader_bytes(sim_bus, records, fw231_program):
    """An older image gives its loader digest only through an A0 read (hashed in memory, redacted in every wire log);
    the release answers read 1136. Neither leaves loader bytes on disk."""
    s = older_fw231(2, seed=16, fw231_program=fw231_program)
    loader = bytes(s.flash[0:0x4000])
    wire, port = sim_bus([s])
    events = []
    wire.log = events.append                                       # the caller's own wire log is redacted too
    r = install.update(wire, servo_id=2, records=records)
    assert r["completed"]
    backed_up = [x for x in r["steps"] if x["step"] == "backed_up"][0]
    verified = [x for x in r["steps"] if x["step"] == "verified"][0]
    assert backed_up["loader_digest"].startswith("A0 read") and verified["loader_digest"] == "read 1136"
    wire, _ = sim_bus([s])
    install.update(wire, servo_id=2, records=records)               # on the release: read 1136 only
    servo_dir = records / r["servo_dir"]
    assert [line.split()[0] for line in (servo_dir / "loader.crc32").read_text().splitlines()] == [
        f"{SIM_LOADER_CRC:08x}"] * 2
    assert not list(records.rglob("loader.bin"))
    chunks = [loader[k:k + 24] for k in range(0, len(loader) - 24, 96)]
    for path in records.rglob("*"):
        if path.is_file():
            data = path.read_bytes()
            assert not any(c in data or c.hex().encode() in data for c in chunks), path
    assert any(e.get("redacted_bytes") for e in events)
    assert not any(c.hex() in json.dumps(events) for c in chunks)


# ---------------------------------------------------------------------------------------------- shared bus
def test_reinstall_one_servo_among_six(sim_bus, records, installed):
    servos = installed()
    before = [snapshot(s) for s in servos]
    wire, _ = sim_bus(servos)
    receipt = install.update(wire, servo_id=4, records=records)
    assert receipt["completed"] and receipt["servo_id"] == 4
    assert [snapshot(s) for s in servos] == before
    assert in_place(servos[3]) and devblock(servos[3])["state"] == "default"
    for k, s in enumerate(servos):
        assert (s.mode, s.faults) == ("app", 0)
        if k != 3:
            assert s.counts["fe"] > 0, "bystanders heard the 57600 loader traffic as noise"
    history = (records / receipt["servo_dir"] / "history.jsonl").read_text().splitlines()
    assert len(history) == 2 and json.loads(history[-1])["run"] == receipt["run_dir"]


def test_update_keeps_id(sim_bus, records, installed, fw231_program):
    """update() re-installs the identical image, and the servo answers at the same ID afterwards: on the release, and
    from an older image."""
    servos = installed(3)
    wire, port = sim_bus(servos)
    r = install.update(wire, servo_id=2, records=records)
    assert r["completed"] and r["servo_id"] == 2 and r["release_id"] == RELEASE_ID
    assert sorted(wire.scan()) == [1, 2, 3] and servos[1].effective_id() == 2 and in_place(servos[1])
    older = older_fw231(5, seed=30, fw231_program=fw231_program)
    wire, _ = sim_bus(servos + [older])
    r = install.update(wire, servo_id=5, records=records)
    assert r["completed"] and r["servo_id"] == 5 and in_place(older)
    assert sorted(wire.scan()) == [1, 2, 3, 5]
    for gone in ("new_id", "label", "boot_baud"):
        assert gone not in r


def test_update_refuses_an_id_from_a_device_block(sim_bus, records, fw231_program):
    """An earlier tool gave this servo ID 6 in its device block; its stock NV ID is 5. The identical image keeps the NV
    ID, so the servo would move to ID 5: refused before F9 (IDs are not changed by the installer in this version)."""
    s = older_fw231(5, seed=4, fw231_program=fw231_program, servo_id=6, label="six")
    assert s.effective_id() == 6
    wire, port = sim_bus([s])
    with pytest.raises(InstallError) as e:
        install.update(wire, servo_id=6, records=records)
    msg = str(e.value)
    assert "ID 5" in msg and "device block" in msg and e.value.step == "identify" and e.value.recovery is None
    assert "Dynamixel Wizard cannot" in msg and "keep the firmware" in msg
    assert "allow_id_change=True" in msg and "--allow-id-change" in msg
    assert tx_frames(port, 0xF9) == [] and (s.mode, s.effective_id()) == ("app", 6)
    assert last_receipt(records)["allow_id_change"] is False


def test_update_allow_id_change_moves_to_the_nv_id(sim_bus, records, fw231_program, fw231_at):
    """With allow_id_change=True the servo is updated and answers at its stock NV ID afterwards; the receipt records
    the change, and the recovery and verification use the NV ID. Refused before F9 when the NV ID is taken."""
    s = older_fw231(5, seed=4, fw231_program=fw231_program, servo_id=6, label="six")
    wire, port = sim_bus([s])
    r = install.update(wire, servo_id=6, allow_id_change=True, records=records)
    assert r["completed"] and (r["servo_id"], r["previous_id"], r["allow_id_change"]) == (5, 6, True)
    assert s.effective_id() == 5 and in_place(s) and sorted(wire.scan()) == [5]
    assert [x for x in r["steps"] if x["step"] == "census_after"][0]["census"].keys() == {5}
    # the NV ID is taken by another servo: refused before F9
    other = older_fw231(7, seed=5, fw231_program=fw231_program, servo_id=8)
    taken = fw231_at(7, seed=7)                                     # FW231 at ID 7
    wire, port = sim_bus([other, taken])
    with pytest.raises(InstallError) as e:
        install.update(wire, servo_id=8, allow_id_change=True, records=records)
    assert "ID 7" in str(e.value) and "already used" in str(e.value) and e.value.recovery is None
    assert tx_frames(port, 0xF9) == [] and other.effective_id() == 8
    # a failure in the loader: the recovery command names the NV ID the servo will answer at
    lone = older_fw231(9, seed=6, fw231_program=fw231_program, servo_id=10)
    real = lone.flash_write
    lone.flash_write = lambda address, data: real(address, bytes([data[0] ^ 1]) + bytes(data[1:]))
    wire, _ = sim_bus([lone])
    with pytest.raises(InstallError) as e:
        install.update(wire, servo_id=10, allow_id_change=True, records=records)
    assert e.value.recovery == recovery(9) and lone.mode == "loader"
    lone.flash_write = real
    wire, _ = sim_bus([lone], LOADER_BAUD)
    assert install.recover(wire, servo_id=9, records=records)["completed"] and lone.effective_id() == 9


def test_h2_update_replaces_an_earlier_device_block(sim_bus, records, fw231_program):
    """An earlier tool's device block with the servo's own NV ID, a label and a 3 Mbaud start-up baud: update() keeps
    the ID; the label and start-up baud go (the image's default block: start-up at 1 Mbaud, no label), and the receipt
    records what the block said before."""
    s = older_fw231(3, seed=31, fw231_program=fw231_program, servo_id=3, label="joint3", boot_baud=3_000_000)
    wire, _ = sim_bus([s])
    s.power_cycle()                                                 # talked to at 1 Mbaud inside the rescue window
    r = install.update(wire, servo_id=3, records=records)
    assert r["completed"] and s.effective_id() == 3
    assert devblock(s) == dict(state="default", id=None, label="", boot_baud=1_000_000)
    assert r["device_block_before"] == dict(state="override", id=3, label="joint3", boot_baud=3_000_000)
    s.power_cycle()
    wire.sleep(2.0)
    assert s.baud == APP_BAUD and wire.ping(3)["firmware"] == 231


def test_update_refuses_a_stock_servo(sim_bus, records):
    s = stock_servo(4, version=52, baud_index=3)
    wire, port = sim_bus([s])
    with pytest.raises(InstallError) as e:
        install.update(wire, servo_id=4, records=records)
    assert "V52" in str(e.value) and "from_stock" in str(e.value)
    assert tx_frames(port, 0xF9) == [] and s.mode == "stock"


def test_not_at_1m_refused(sim_bus, records, fw231_servos):
    servos = fw231_servos(2)
    wire, port = sim_bus(servos)
    wire.write_broadcast(registers.BAUD, struct.pack("<I", 3_000_000))
    wire.set_baud(3_000_000)
    assert {wire.ping(1)["firmware"], wire.ping(2)["firmware"]} == {231}
    with pytest.raises(InstallError) as e:
        install.update(wire, servo_id=1, records=records)
    assert "1 Mbaud" in str(e.value) and e.value.step == "precheck"
    assert tx_frames(port, 0xF9) == []


# ---------------------------------------------------------------------------------------------- release ID
@pytest.mark.parametrize("changed", [
    pytest.param({"source_id": 0x3C2E5B73}, id="source"),
    pytest.param({"number": 13}, id="number"),
    pytest.param({"variant": "pd"}, id="variant"),
    pytest.param({"abi": 2}, id="abi"),
    pytest.param(None, id="no-read-1120"),
])
def test_release_id_mismatch_is_an_install_error(sim_bus, records, fw231_servos, changed):
    """The program bytes match, but read 1120 disagrees with the bundled manifest: an InstallError after the launch that
    says to share the receipt (re-installing the same package would not change it)."""
    s = fw231_servos(1)[0]
    real = s.read
    reported = None if changed is None else registers.encode_release_id(**dict(RELEASE.release_id, **changed))

    def read(a, n):
        if (a, n) == registers.RELEASE_ID and s.release_id:
            return (4, b"") if reported is None else (0, reported)
        return real(a, n)
    s.read = read
    wire, _ = sim_bus([s])
    with pytest.raises(InstallError) as e:
        install.update(wire, servo_id=1, records=records)
    assert e.value.step == "verify" and "release" in str(e.value) and "share" in str(e.value)
    assert e.value.recovery is None and in_place(s)
    r = last_receipt(records)
    assert (r["completed"], r["phase"], r["share_receipt"]) == (False, "launched", True)


def test_release_id_mismatch_after_a_first_install_leaves_the_records(sim_bus, records):
    """The release ID is checked after the loader and calibration comparisons, once the servo's first records are
    written: a mismatch still leaves nvm.bin and the loader CRC history."""
    s = SimServo(1, mode="stock", seed=19)
    real = s.read
    wrong = struct.pack("<4I", 12 | 1 << 16 | 3 << 24, 1, 0, 0)
    s.read = lambda a, n: (0, wrong) if (a, n) == (1120, 16) else real(a, n)
    wire, _ = sim_bus([s], LOADER_BAUD)
    with pytest.raises(InstallError) as e:
        install.from_stock(wire, records=records)
    assert e.value.step == "verify" and "release ID" in str(e.value) and e.value.recovery is None
    r = last_receipt(records)
    folder = records / r["servo_dir"]
    assert (folder / "nvm.bin").read_bytes() == bytes(s.flash[0xE000:0x10000])
    assert history(records, r)[0][0] == f"{SIM_LOADER_CRC:08x}" and r["release_id"]["number"] == 12


# ---------------------------------------------------------------------------------------------- failures
def test_fa_mismatch_stops_before_launch(sim_bus, records, fw231_servos):
    s = fw231_servos(1)[0]
    real = s.flash_write

    def corrupt(address, data):
        data = bytearray(data)
        data[1000] ^= 0xFF
        real(address, bytes(data))
    s.flash_write = corrupt
    wire, _ = sim_bus([s])
    with pytest.raises(InstallError) as e:
        install.update(wire, servo_id=1, records=records)
    assert isinstance(e.value.__cause__, BusUncertain) and e.value.step == "upload"
    assert e.value.recovery == recovery(1) and recovery(1) in str(e.value)
    assert s.mode == "loader", "no F8 after a failed FA: the servo stays in the loader"
    with pytest.raises(BusUncertain):
        wire.ping(1)
    r = last_receipt(records)
    assert not r["completed"] and r["in_loader"] and r["phase"] == "uploading" and "FA CRC" in r["error"]
    assert (r["recovery"], r["failed_step"]) == (recovery(1), "upload")
    # recovery on a fresh Wire (the old one stays latched): the loader is still at its prompt
    s.flash_write = real
    wire, _ = sim_bus([s], LOADER_BAUD)
    rec = install.recover(wire, servo_id=1, records=records)
    assert rec["completed"] and rec["known_servo"]
    assert (s.mode, s.version, s.effective_id()) == ("app", 231, 1) and in_place(s)


def test_recover_after_power_cycle_in_loader(sim_bus, records):
    s = SimServo(3, seed=11)
    s.reset("loader")
    s.pending_banner = False                                       # waiting for '#'
    wire, _ = sim_bus([s], LOADER_BAUD)
    with pytest.raises(InstallError) as e:
        install.recover(wire, servo_id=3, records=records)         # F1 before '#': no answer
    assert isinstance(e.value.__cause__, NoReply) and e.value.recovery == "openxl330 scan --port SIM --all-bauds"
    assert f"run `{recovery(3)} --loader-fresh`" in str(e.value)
    wire, _ = sim_bus([s], LOADER_BAUD)
    rec = install.recover(wire, servo_id=3, loader_fresh=True, records=records)
    assert not (rec["completed"] or rec["known_servo"])           # no earlier records: preservation unverified
    assert "unverified" in rec["preservation"] and rec["release_id"] == RELEASE_ID
    assert s.effective_id() == 3 and in_place(s)
    wire, _ = sim_bus([s])
    install.update(wire, servo_id=3, records=records)              # now it has records
    s.reset("loader")
    s.pending_banner = False
    wire, _ = sim_bus([s], LOADER_BAUD)
    rec = install.recover(wire, servo_id=3, loader_fresh=True, records=records)
    assert rec["completed"] and rec["known_servo"]


@pytest.mark.parametrize("fresh", [False, True])
def test_recover_attach_failure_never_repeats_the_same_command(sim_bus, records, fresh):
    """Nothing answers the loader (here the servo runs the firmware at 1 Mbaud): the advice depends on whether the servo
    was power-cycled since the failed install, and never repeats the command that just failed."""
    s = SimServo(1, mode="app", program=PACK.raw)
    wire, _ = sim_bus([s], LOADER_BAUD)
    with pytest.raises(InstallError) as e:
        install.recover(wire, servo_id=1, loader_fresh=fresh, records=records)
    msg, scan = str(e.value), "openxl330 scan --port SIM --all-bauds"
    assert e.value.step == "attach" and e.value.recovery == scan
    assert f"run `{scan}` to find it" in msg and "To recover the servo, run:" not in msg
    if fresh:
        assert ("If the servo was not power-cycled since the failed install, run `openxl330 recover --port SIM --id 1`"
                " (without --loader-fresh).") in msg
        assert ("If the servo was power-cycled again or its prompt failed, isolate it, power-cycle it, and run "
                "`openxl330 recover --port SIM --id 1 --loader-fresh` again.") in msg
        assert msg.count("--loader-fresh`") == 1, "the fresh command again only after a new power cycle"
    else:
        assert ("If the servo was power-cycled since the failed install, run `openxl330 recover --port SIM --id 1 "
                "--loader-fresh`. Otherwise") in msg
        assert "openxl330 recover --port SIM --id 1`" not in msg and msg.count("openxl330 recover") == 1
        assert "and baud. If the servo" in msg, "the reply error and the advice are separate sentences"
    r = last_receipt(records)
    assert (r["failed_step"], r["recovery"], r["in_loader"]) == ("attach", scan, True)
    assert r["recovery_note"] in msg and s.mode == "app"


def test_interrupt_during_recover_attach_keeps_the_prompt_guidance(monkeypatch, sim_bus, records):
    """Ctrl-C while recover() runs the fresh prompt is not a silent loader: the guidance stays the interrupted-prompt
    one (power-cycle, then --loader-fresh), not the attach-failure advice."""
    s = SimServo(1, seed=81)
    s.reset("loader")
    s.pending_banner = False
    wire, _ = sim_bus([s], LOADER_BAUD)
    wait_text = Loader._wait_text

    def interrupt(loader, token, *args, **kwargs):
        if token == b"#":
            raise KeyboardInterrupt
        return wait_text(loader, token, *args, **kwargs)
    monkeypatch.setattr(Loader, "_wait_text", interrupt)
    with pytest.raises(KeyboardInterrupt) as stopped:
        install.recover(wire, servo_id=1, loader_fresh=True, records=records)
    assert stopped.value.recovery == recovery(1) + " --loader-fresh" == last_receipt(records)["recovery"]
    assert "power-cycle" in stopped.value.recovery_note and "scan" not in stopped.value.recovery_note


def test_recover_reports_upload_progress(sim_bus, records):
    s = SimServo(3, seed=11)
    s.reset("loader")
    s.pending_banner = False
    wire, _ = sim_bus([s], LOADER_BAUD)
    seen = []
    install.recover(wire, servo_id=3, loader_fresh=True, records=records,
                    on_progress=lambda done, total: seen.append((done, total)))
    assert seen[0] == (0, len(PACK.encoded)) and seen[-1] == (len(PACK.encoded), len(PACK.encoded))
    with pytest.raises(InvalidArgument):
        install.recover(wire, servo_id=3, records=records, on_progress="progress")


def test_recover_with_the_wrong_id_says_how_to_find_the_servo(sim_bus, records):
    """The servo comes up at its stock NV ID (3) whatever recover() was told (5): no reply at 5 after the launch."""
    s = SimServo(3, seed=12)
    s.reset("loader")
    s.pending_banner = False
    wire, _ = sim_bus([s], LOADER_BAUD)
    with pytest.raises(InstallError) as e:
        install.recover(wire, servo_id=5, loader_fresh=True, records=records)
    msg = str(e.value)
    assert e.value.step == "verify" and "no reply from ID 5 after the launch." in msg
    assert e.value.recovery == "openxl330 scan --port SIM" and (s.mode, s.effective_id()) == ("app", 3)
    assert "openxl330 update --port SIM --id <that ID>" in msg and reinstall(5) not in msg
    r = last_receipt(records)
    assert (r["in_loader"], r["recovery"]) == (False, "openxl330 scan --port SIM")


def test_changed_nvm_detected(sim_bus, records, fw231_servos):
    s = fw231_servos(1)[0]
    tamper_after_launch(s, lambda v: v.flash.__setitem__(0xF800, v.flash[0xF800] ^ 1))
    wire, _ = sim_bus([s])
    with pytest.raises(InstallError) as e:
        install.update(wire, servo_id=1, records=records)
    assert "NVM changed" in str(e.value) and e.value.step == "verify"
    assert e.value.recovery is None
    assert "unexpected" in str(e.value) and "share" in str(e.value) and "openxl330 update" not in str(e.value)


def test_changed_loader_detected(sim_bus, records, fw231_servos):
    s = fw231_servos(1)[0]
    tamper_after_launch(s, lambda v: v.flash.__setitem__(0x100, v.flash[0x100] ^ 1))
    wire, _ = sim_bus([s])
    with pytest.raises(InstallError) as e:
        install.update(wire, servo_id=1, records=records)
    assert "loader changed" in str(e.value) and "CRC-32" in str(e.value) and "unexpected" in str(e.value)
    assert e.value.recovery is None


def test_program_bytes_proved_over_a0(sim_bus, records, fw231_servos):
    s = fw231_servos(1)[0]
    tamper_after_launch(s, lambda v: v.flash.__setitem__(0x9000, v.flash[0x9000] ^ 1))
    wire, _ = sim_bus([s])
    with pytest.raises(InstallError) as e:
        install.update(wire, servo_id=1, records=records)
    assert "A0 SHA-256" in str(e.value) and e.value.step == "verify"
    assert e.value.recovery == reinstall(1) and reinstall(1) in str(e.value)   # re-installing fixes it
    assert "unexpected" not in str(e.value) and "recover --port" not in str(e.value)
    r = last_receipt(records)
    assert (r["recovery"], r["in_loader"], r["phase"]) == (reinstall(1), False, "launched")


def test_f9_acknowledged_then_silent_loader_is_recorded_in_loader(sim_bus, records, fw231_servos):
    s = fw231_servos(1)[0]
    real = s._loader_frame
    s._loader_frame = lambda f: None if f["instruction"] == 0xF1 else real(f)
    wire, _ = sim_bus([s])
    with pytest.raises(InstallError) as e:
        install.update(wire, servo_id=1, records=records)
    assert isinstance(e.value.__cause__, (NoReply, BusUncertain)) and e.value.recovery == recovery(1)
    assert last_receipt(records)["in_loader"] is True


def test_refusal_and_wire_failure_before_f9_are_install_errors(sim_bus, records, fw231_program, tmp_path):
    a = SimServo(1, mode="app", program=fw231_program(), seed=1)
    b = SimServo(1, mode="app", program=fw231_program(), seed=2)   # two servos at ID 1
    wire, port = sim_bus([a, b])
    with pytest.raises(InstallError) as e:
        install.update(wire, servo_id=1, records=records)
    assert isinstance(e.value.__cause__, BusUncertain) and e.value.step == "census" and e.value.recovery is None
    assert str(e.value.__cause__) in str(e.value) and tx_frames(port, 0xF9) == []    # the cause's advice is kept
    blocked = tmp_path / "file"
    blocked.write_text("not a folder")
    wire, port = sim_bus([SimServo(1, mode="stock", seed=5)], LOADER_BAUD)
    with pytest.raises(InstallError) as e:
        install.from_stock(wire, records=blocked)
    assert e.value.step == "records" and port.tx_log == []


# ---------------------------------------------------------------------------------------------- review H-1..H-7
def test_h1_bad_arguments_refused_before_any_tx(sim_bus, records):
    new = SimServo(1, mode="stock", seed=60)
    wire, port = sim_bus([new], LOADER_BAUD)
    for bad in (None, "nv", 253, True, "1"):
        with pytest.raises(InvalidArgument):
            install.from_stock(wire, stock_id=bad, records=records)
        with pytest.raises(InvalidArgument):
            install.update(wire, servo_id=bad, records=records)
        with pytest.raises(InvalidArgument):
            install.recover(wire, servo_id=bad, records=records)
    with pytest.raises(InvalidArgument):
        install.from_stock(wire, stock_baud=12345, records=records)
    with pytest.raises(InvalidArgument):
        install.from_stock(wire, stock_baud=57_600.0, records=records)
    with pytest.raises(InvalidArgument):
        install.from_stock(wire, allow_untested="false", records=records)
    with pytest.raises(InvalidArgument):
        install.from_stock(wire, on_progress="show progress", records=records)
    with pytest.raises(InvalidArgument):
        install.from_stock(wire, records=123)
    with pytest.raises(InvalidArgument):
        install.from_stock(wire, release="fw999", records=records)
    with pytest.raises(InvalidArgument):
        install.update(wire, servo_id=1, release="fw231-comp2-fast12", records=records)
    with pytest.raises(InvalidArgument):
        install.update(wire, servo_id=1, allow_id_change="yes", records=records)
    with pytest.raises(InvalidArgument):
        install.update(wire, servo_id=1, on_progress=False, records=records)
    with pytest.raises(InvalidArgument):
        install.recover(wire, servo_id=1, loader_fresh="false", records=records)
    with pytest.raises(InvalidArgument):
        install.recover(wire, servo_id=1, receipt=123, records=records)
    for gone in (lambda: install.from_stock(wire, new_id=4, records=records),
                 lambda: install.from_stock(wire, label="wrist", records=records),
                 lambda: install.update(wire, servo_id=1, boot_baud=3_000_000, records=records),
                 lambda: install.recover(wire, new_id=4, records=records)):
        with pytest.raises(TypeError):                              # no ID, label or start-up baud settings in v1
            gone()
    assert not hasattr(install, "run_settings")
    assert (port.tx_log, new.mode) == ([], "stock"), "refused before any bus traffic"
    assert not (records / "_runs").exists(), "no run was started"


def test_h2_recover_records_the_failed_run(sim_bus, records, installed):
    s = installed(1)[0]
    real = s.flash_write

    def corrupt(address, data):
        data = bytearray(data)
        data[1000] ^= 0xFF
        real(address, bytes(data))
    s.flash_write = corrupt
    wire, _ = sim_bus([s])
    with pytest.raises(InstallError):
        install.update(wire, servo_id=1, records=records)          # FA mismatch: stays in the loader
    failed = last_receipt(records)
    assert (failed["in_loader"], failed["phase"], failed["servo_id"]) == (True, "uploading", 1)
    s.flash_write = real
    wire, _ = sim_bus([s], LOADER_BAUD)
    rec = install.recover(wire, servo_id=1, records=records)       # no receipt given: the newest run for ID 1
    assert rec["completed"] and rec["failed_run"] == failed["run_dir"] + "/receipt.json"
    # an explicit receipt folder, relative as receipts store it
    for given in (failed["run_dir"], records / failed["run_dir"] / "receipt.json"):
        s.reset("loader")
        s.pending_banner = False
        wire, _ = sim_bus([s], LOADER_BAUD)
        rec = install.recover(wire, servo_id=1, receipt=given, loader_fresh=True, records=records)
        assert rec["completed"] and rec["failed_run"] == failed["run_dir"] + "/receipt.json"
    # the newest run for ID 1 is now a completed recover: nothing to point at
    s.reset("loader")
    s.pending_banner = False
    wire, _ = sim_bus([s], LOADER_BAUD)
    assert install.recover(wire, servo_id=1, loader_fresh=True, records=records)["failed_run"] is None
    # refused before anything is sent: a receipt for another ID, one from before F9, one that cannot be read
    wire, port = sim_bus([s], LOADER_BAUD)
    with pytest.raises(InstallError) as e:
        install.recover(wire, servo_id=7, receipt=failed["run_dir"], records=records)
    assert e.value.step == "receipt" and "ID 1, not ID 7" in str(e.value)
    with pytest.raises(InstallError) as e:
        install.recover(wire, servo_id=1, receipt=records / "_runs" / "nope.json", records=records)
    assert e.value.step == "receipt"
    a, b = SimServo(1, seed=1), SimServo(2, seed=2)                # refused at the census: never reached F9
    refusing, _ = sim_bus([a, b], LOADER_BAUD)
    with pytest.raises(InstallError):
        install.from_stock(refusing, records=records)
    refused = last_receipt(records)
    assert "servo_id" not in refused
    with pytest.raises(InstallError) as e:
        install.recover(wire, servo_id=1, receipt=refused["run_dir"], records=records)
    assert e.value.step == "receipt" and "before F9" in str(e.value)
    completed = [p for p in sorted((records / "_runs").iterdir()) if "recover-id1" in p.name][-1]
    assert json.loads((completed / "receipt.json").read_text())["completed"] is True
    with pytest.raises(InstallError) as e:                         # a run that did not leave the servo in the loader
        install.recover(wire, servo_id=1, receipt=completed, records=records)
    assert e.value.step == "receipt" and "did not leave" in str(e.value)
    assert port.tx_log == [] and s.mode == "app"


def test_h3_install_refuses_a_servo_at_57600(sim_bus, records, fw231_at):
    ours = fw231_at(1, seed=52)
    factory = SimServo(1, mode="stock", seed=53)                   # ID 1 at 57600
    wire, port = sim_bus([ours, factory])
    with pytest.raises(InstallError) as e:
        install.update(wire, servo_id=1, records=records)
    assert "57600" in str(e.value)
    assert tx_frames(port, 0xF9) == [], "refused before F9"
    assert (ours.mode, factory.mode, port.baudrate, wire.uncertain) == ("app", "stock", 1_000_000, False)
    r = last_receipt(records)
    assert (r["in_loader"], r["phase"]) == (False, None)
    assert sorted(r["steps"][-1]["censuses"]) == ["2000000", "3000000", "57600"]
    assert wire.ping(1)["firmware"] == 231                         # host back at 1 Mbaud, bus usable


def test_h3_ids_at_2m_and_3m_are_taken(sim_bus, records, fw231_at, fw231_program):
    a = fw231_at(1, seed=54)
    same = older_fw231(1, seed=55, fw231_program=fw231_program, boot_baud=3_000_000)   # also ID 1, starts at 3M
    wire, port = sim_bus([a, same])
    same.power_cycle()
    wire.sleep(2.0)                                                # it now runs at its start-up baud
    assert sorted(wire.scan()) == [1]
    with pytest.raises(InstallError) as e:
        install.update(wire, servo_id=1, records=records)
    assert "ID 1 is also used" in str(e.value) and "3000000" in str(e.value) and e.value.recovery is None
    assert tx_frames(port, 0xF9) == []
    other = older_fw231(5, seed=56, fw231_program=fw231_program, boot_baud=3_000_000)
    wire, port = sim_bus([a, other])
    other.power_cycle()
    wire.sleep(2.0)
    r = install.update(wire, servo_id=1, records=records)
    assert r["completed"]
    assert (other.mode, other.baud, other.faults, port.baudrate) == ("app", 3_000_000, 0, 1_000_000), \
        "the 3M servo was only pinged"


def test_h3_first_install_censuses_57600_and_2m_3m(sim_bus, records, fw231_program):
    other = older_fw231(2, seed=56, fw231_program=fw231_program, boot_baud=2_000_000)
    new = stock_servo(2, seed=57)                                   # stock ID 2, like the one at 2M
    wire, port = sim_bus([other, new], LOADER_BAUD)
    other.power_cycle()
    wire.sleep(2.0)                                                # ID 2 at 2 Mbaud
    with pytest.raises(InstallError) as e:
        install.from_stock(wire, stock_id=2, records=records)
    assert "Dynamixel Wizard" in str(e.value) and "2000000" in str(e.value)
    assert (tx_frames(port, 0xF9), new.mode, port.baudrate) == ([], "stock", LOADER_BAUD)
    new3 = stock_servo(3, seed=58)
    wire, port = sim_bus([other, new3], LOADER_BAUD)
    assert install.from_stock(wire, stock_id=3, records=records)["completed"]
    assert (other.baud, other.faults, new3.effective_id()) == (2_000_000, 0, 3)
    # a stock servo at 1 Mbaud (a V52) with a factory servo at 57600 beside it: refused before F9
    v52 = stock_servo(4, version=52, seed=58, baud_index=3)
    fresh = SimServo(1, mode="stock", seed=59)
    wire, port = sim_bus([v52, fresh], APP_BAUD)
    with pytest.raises(InstallError) as e:
        install.from_stock(wire, stock_id=4, stock_baud=APP_BAUD, records=records)
    assert "57600" in str(e.value)
    assert (tx_frames(port, 0xF9), v52.mode, port.baudrate) == ([], "stock", APP_BAUD)


def test_h5_f9_acked_cleared_by_the_f8_ack(sim_bus, records, fw231_at):
    a, b = fw231_at(1, seed=61), fw231_at(2, seed=62)
    wire, _ = sim_bus([a, b])
    install.update(wire, servo_id=1, records=records)
    assert (wire.f9_acked, wire.loader, wire.loader_phase, wire.loader_possible) == (None, None, "launched", False)
    with pytest.raises(InstallError):
        install.update(wire, servo_id=7, records=records)           # refused before F9: no servo at ID 7
    r = last_receipt(records)
    assert (r["in_loader"], r["phase"], r["completed"]) == (False, None, False)


def test_h5_unfinished_loader_session_blocks_the_next_install(sim_bus, records, fw231_at):
    s = fw231_at(1, seed=63)
    real, calls = s._loader_frame, []

    def lose_third_f7(f):
        if f["instruction"] == 0xF7:
            calls.append(1)
            if len(calls) == 3:
                return None
        return real(f)
    s._loader_frame = lose_third_f7
    wire, port = sim_bus([s])
    with pytest.raises(InstallError) as e:
        install.update(wire, servo_id=1, records=records)
    assert isinstance(e.value.__cause__, NoReply) and e.value.recovery == recovery(1)
    r = last_receipt(records)
    assert (r["in_loader"], r["phase"], r["wire_uncertain"]) == (True, "uploading", False)
    assert wire.loader_possible
    sent = len(port.tx_log)
    for attempt in (lambda: install.update(wire, servo_id=1, records=records),
                    lambda: install.from_stock(wire, stock_id=4, records=records)):
        with pytest.raises(InstallError) as e:
            attempt()
        assert "may still be in the retained loader" in str(e.value)
        assert e.value.recovery == recovery(1)                     # the ID the unfinished install was flashing
    assert len(port.tx_log) == sent, "nothing sent while a servo may be in the loader"
    s._loader_frame = real
    rec = install.recover(wire, servo_id=1, records=records)       # same Wire, loader at its prompt
    assert rec["completed"]
    assert (wire.loader_possible, rec["failed_run"]) == (False, r["run_dir"] + "/receipt.json")


def test_h6_keyboard_interrupt_mid_upload_is_recorded_and_latches(sim_bus, records, fw231_at):
    s = fw231_at(1, seed=64)
    wire, _ = sim_bus([s])

    def progress(done, total):
        if done >= 256:
            raise KeyboardInterrupt
    with pytest.raises(KeyboardInterrupt):
        install.update(wire, servo_id=1, records=records, on_progress=progress)
    r = last_receipt(records)
    assert (r["completed"], r["error"], r["in_loader"], r["phase"], r["wire_uncertain"]) == (
        False, "KeyboardInterrupt()", True, "uploading", True)
    assert "finished_utc" in r
    with pytest.raises(BusUncertain):
        wire.ping(1)


def test_h7_scan_keeps_collecting_after_an_error_reply(sim_bus, records, fw231_at):
    odd = error_on_ping(SimServo(3, mode="stock", seed=66, eeprom=default_eeprom(3, baud_index=3)))
    ours = fw231_at(5, seed=67)
    wire, port = sim_bus([odd, ours])
    port.ping_slots = True                                         # replies in their 3 ms slots
    with pytest.raises(InstallError) as e:                         # a target that answers with an error
        install.from_stock(wire, stock_id=3, stock_baud=APP_BAUD, records=records)
    assert "error 2" in str(e.value)
    with pytest.raises(InstallError) as e:                         # ... or a bystander: still refused
        install.update(wire, servo_id=5, records=records)
    assert "ID 3 answered the ping with error 2" in str(e.value)
    assert (tx_frames(port, 0xF9), odd.mode, ours.mode, wire.uncertain) == ([], "stock", "app", False)


# ---------------------------------------------------------------------------------------------- review focus 5
def test_interrupt_during_upload_records_and_explains(sim_bus, records):
    """Ctrl-C at the 10th F7 packet. Documented behaviour: the KeyboardInterrupt itself is re-raised (Ctrl-C stays
    Ctrl-C), carrying .recovery, .step and .receipt; the receipt on disk records the error, phase and in_loader."""
    s = SimServo(1, mode="stock", seed=80)
    real, f7 = s._loader_frame, []

    def tenth_f7_interrupts(f):
        if f["instruction"] == 0xF7:
            f7.append(1)
            if len(f7) == 10:
                raise KeyboardInterrupt
        return real(f)
    s._loader_frame = tenth_f7_interrupts
    wire, _ = sim_bus([s], LOADER_BAUD)
    with pytest.raises((InstallError, KeyboardInterrupt)) as e:
        install.from_stock(wire, records=records)
    assert e.value.recovery == "openxl330 recover --port SIM --id 1"
    assert isinstance(e.value, KeyboardInterrupt) and e.value.step == "upload"
    r = json.loads(Path(e.value.receipt).read_text())
    assert r == last_receipt(records)
    assert r["phase"] == "uploading" and r["in_loader"] is True
    assert (r["error"], r["completed"], r["recovery"]) == ("KeyboardInterrupt()", False, recovery(1))
    assert wire.uncertain and s.mode == "loader"
    # the command it printed works (on a new connection), and its receipt names the run it finished
    s._loader_frame = real
    wire, _ = sim_bus([s], LOADER_BAUD)
    rec = install.recover(wire, servo_id=1, records=records)
    assert (s.mode, s.version, s.effective_id()) == ("app", 231, 1)
    assert records / rec["failed_run"] == Path(e.value.receipt)
    assert not rec["known_servo"] and "unverified" in rec["preservation"]   # no records before a first install


@pytest.mark.parametrize("phase", ["before_prompt", "during_prompt", "after_prompt"])
def test_interrupt_in_initial_loader_handshake_guides_recovery(monkeypatch, sim_bus, records, phase):
    device = SimServo(1, seed=80)
    wire, _ = sim_bus([device], LOADER_BAUD)
    with monkeypatch.context() as m:
        if phase == "before_prompt":
            def interrupt(loader):
                raise KeyboardInterrupt
            m.setattr(Loader, "handshake", interrupt)
        elif phase == "during_prompt":
            wait_text = Loader._wait_text
            def interrupt(loader, token, *args, **kwargs):
                if token == b"#":
                    raise KeyboardInterrupt
                return wait_text(loader, token, *args, **kwargs)
            m.setattr(Loader, "_wait_text", interrupt)
        else:
            def interrupt(loader, *args, **kwargs):
                raise KeyboardInterrupt
            m.setattr(Loader, "_exchange", interrupt)
        with pytest.raises(KeyboardInterrupt) as stopped:
            install.from_stock(wire, records=records)

    fresh = phase != "after_prompt"
    expected = recovery(1) + (" --loader-fresh" if fresh else "")
    assert stopped.value.recovery == expected == last_receipt(records)["recovery"]
    if phase == "during_prompt":
        assert "power-cycle" in stopped.value.recovery_note
        device.reset("loader")
    device.pending_banner = False
    wire, _ = sim_bus([device], LOADER_BAUD)
    result = install.recover(wire, servo_id=1, loader_fresh=fresh, records=records)
    assert device.mode == "app" and result["servo_id"] == 1


def test_exception_during_upload_explains_in_the_message(sim_bus, records):
    s = SimServo(1, mode="stock", seed=81)
    wire, _ = sim_bus([s], LOADER_BAUD)

    def progress(done, total):
        if done >= 1280:
            raise RuntimeError("progress display broke")
    with pytest.raises(InstallError) as e:
        install.from_stock(wire, records=records, on_progress=progress)
    assert isinstance(e.value, Error) and e.value.step == "upload"
    assert recovery(1) in str(e.value) and e.value.recovery == recovery(1)
    assert "progress display broke" in str(e.value)
    r = last_receipt(records)
    assert (r["phase"], r["in_loader"], r["completed"]) == ("uploading", True, False)


# ---------------------------------------------------------------------------------------------- review fix round 1
def garble_loader_reads(servo, on_pass):
    """The servo's A0 replies from the loader region lose their 9-byte header on the `on_pass`-th read of that region
    (pass 1: update's backup before F9; pass 2: the verification after the launch), so the host's buffer starts
    with loader bytes and the Wire latches on them."""
    real, passes = servo.on_frame, []

    def on_frame(f, clock):
        r = real(f, clock)
        if r and f["instruction"] == 0xA0:
            addr = int.from_bytes(bytes(f["params"][:4]), "little")
            if addr == 0x08000000:
                passes.append(1)
            if len(passes) == on_pass and 0x08000000 + 96 * 3 <= addr < 0x08004000:
                return (r[0], r[1][9:])
        return r
    servo.on_frame = on_frame


def without_loader_digest(servo):
    """After the launch the servo rejects read 1136 and reads its loader region over A0, like an image without loader
    privacy, so the verification takes the A0 path too."""
    tamper_after_launch(servo, lambda v: setattr(v, "loader_private", False))


def loader_windows(servo):
    """Hex of every 8-byte window of the servo's loader (the simulated loader repeats every 256 bytes)."""
    loader = bytes(servo.flash[0:0x4000])
    return {loader[k:k + 8].hex() for k in range(256)}


def chain(exc):
    """The exception and everything chained to it."""
    seen = []
    while exc is not None and exc not in seen:
        seen.append(exc)
        exc = exc.__cause__ or exc.__context__
    return seen


@pytest.mark.parametrize("on_pass, step", [(1, "loader_digest"), (2, "loader_digest")])
def test_garbled_loader_read_leaks_no_loader_bytes(sim_bus, records, fw231_servos, on_pass, step):
    s = fw231_servos(1)[0]
    windows = loader_windows(s)
    garble_loader_reads(s, on_pass)
    without_loader_digest(s)
    wire, port = sim_bus([s])
    events, redactions, real_redact = [], [], wire.redact_latch
    wire.log = events.append                                       # a log the caller attached
    wire.redact_latch = lambda reason: (redactions.append(reason), real_redact(reason))
    with pytest.raises(InstallError) as e:
        install.update(wire, servo_id=1, records=records)
    assert e.value.step == step and "could not read the loader digest" in str(e.value)
    assert len(redactions) == 1, "the latch reason is replaced through the Wire's public API"
    assert wire.uncertain, "the Wire stays latched"
    assert e.value.recovery == (None if on_pass == 1 else reinstall(1))   # before F9: none; after the launch: update
    texts = [str(x) for x in chain(e.value)] + [repr(x) for x in chain(e.value)]
    with pytest.raises(BusUncertain) as later:                     # the latch reason quoted by later requests
        wire.ping(1)
    texts.append(str(later.value))
    run = sorted((records / "_runs").iterdir())[-1]
    texts += [(run / "receipt.json").read_text(), (run / "wire.jsonl").read_text(), json.dumps(events)]
    leaks = [w for w in windows for t in texts if w in t]
    assert not leaks, leaks[:3]
    assert any(ev.get("event") == "uncertain" and ev.get("redacted") == ["why"] for ev in events)
    assert "bytes not shown" in str(later.value)


def test_no_recovery_offered_when_f9_never_left(sim_bus, records, fw231_servos):
    s = fw231_servos(1)[0]
    wire, port = sim_bus([s])
    real = wire.enter_loader

    def enter_loader(servo_id):                                    # stray bytes arrive just before F9
        port.rx.extend(b"\x55")
        return real(servo_id)
    wire.enter_loader = enter_loader
    with pytest.raises(InstallError) as e:
        install.update(wire, servo_id=1, records=records)
    assert isinstance(e.value.__cause__, BusUncertain) and e.value.step == "enter_loader"
    assert e.value.recovery is None and "recover" not in str(e.value)
    r = last_receipt(records)
    assert (r["in_loader"], r["phase"], "recovery" in r) == (False, None, False)
    assert tx_frames(port, 0xF9) == [] and s.mode == "app"


def test_keyboard_interrupt_before_f9_offers_no_recovery(sim_bus, records):
    s = SimServo(1, mode="stock", seed=82)
    real = s.on_frame

    def on_frame(f, clock):
        if f["instruction"] == 1:
            raise KeyboardInterrupt
        return real(f, clock)
    s.on_frame = on_frame
    wire, port = sim_bus([s], LOADER_BAUD)
    with pytest.raises(KeyboardInterrupt) as e:
        install.from_stock(wire, records=records)
    assert (e.value.recovery, e.value.step) == (None, "census")
    r = last_receipt(records)
    assert (r["in_loader"], r["phase"], r["error"], "recovery" in r) == (False, None, "KeyboardInterrupt()", False)
    assert wire.uncertain and tx_frames(port, 0xF9) == [] and s.mode == "stock"


def test_recover_refuses_a_servo_whose_records_disagree(sim_bus, records, fw231_servos):
    """A calibration page that differs from the servo's record is refused as unexpected; the NV journal half may
    differ (a trip back to stock firmware resets the stock settings there) and is kept beside the record; a loader CRC
    that differs from the last recorded install is a warning (the loader rule for every install)."""
    s = fw231_servos(1)[0]
    wire, _ = sim_bus([s])
    first = install.update(wire, servo_id=1, records=records)      # records: nvm.bin and loader.crc32
    record = records / first["servo_dir"] / "nvm.bin"
    good = record.read_bytes()
    record.write_bytes(bytes([good[0x10] ^ 1]).join([good[:0x10], good[0x11:]]))   # the record's calibration page
    s.reset("loader")
    s.pending_banner = False
    wire, _ = sim_bus([s], LOADER_BAUD)
    with pytest.raises(InstallError) as e:
        install.recover(wire, servo_id=1, loader_fresh=True, records=records)
    assert "calibration page differs" in str(e.value) and "this servo's record" in str(e.value)
    assert e.value.step == "records" and "unexpected" in str(e.value) and "share" in str(e.value)
    assert e.value.recovery is None and last_receipt(records)["completed"] is False
    record.write_bytes(good)
    s.reset("loader")
    s.pending_banner = False
    s.flash[0xF800] ^= 1                                           # the journal half: same fingerprint, accepted
    wire, _ = sim_bus([s], LOADER_BAUD)
    rec = install.recover(wire, servo_id=1, loader_fresh=True, records=records)
    assert rec["completed"] and rec["known_servo"] and rec["kept"]["nvm.bin"].startswith("changed:")
    assert record.read_bytes() == good                             # the first-seen copy is never overwritten
    s.reset("loader")
    s.pending_banner = False
    s.flash[0x100] ^= 1                                            # the loader instead
    wire, _ = sim_bus([s], LOADER_BAUD)
    rec = install.recover(wire, servo_id=1, loader_fresh=True, records=records)
    assert rec["completed"] and rec["known_servo"]
    assert [w for w in rec["warnings"] if w.startswith("loader CRC changed since the last install")]
    assert len(history(records, first)) == 3                       # update and two recovers (the refused one: none)


def test_recover_calls_a_servo_known_only_with_its_calibration_record(sim_bus, records, fw231_servos):
    s = fw231_servos(1)[0]
    wire, _ = sim_bus([s])
    r = install.update(wire, servo_id=1, records=records)
    (records / r["servo_dir"] / "nvm.bin").unlink()                # only loader.crc32 left in the folder
    s.reset("loader")
    s.pending_banner = False
    wire, _ = sim_bus([s], LOADER_BAUD)
    rec = install.recover(wire, servo_id=1, loader_fresh=True, records=records)
    assert (rec["completed"], rec["known_servo"]) == (False, False) and "unverified" in rec["preservation"]
    assert (records / r["servo_dir"] / "nvm.bin").is_file()        # kept again by this run
    s.reset("loader")
    s.pending_banner = False
    wire, _ = sim_bus([s], LOADER_BAUD)
    assert install.recover(wire, servo_id=1, loader_fresh=True, records=records)["completed"]


def test_recover_does_not_count_unreadable_loader_history_as_evidence(sim_bus, records, fw231_servos):
    servo, = fw231_servos(1)
    wire, _ = sim_bus([servo])
    initial = install.update(wire, servo_id=1, records=records)
    history_path = records / initial["servo_dir"] / "loader.crc32"
    history_path.write_bytes(b"partial record")
    servo.reset("loader")
    servo.pending_banner = False
    wire, _ = sim_bus([servo], LOADER_BAUD)

    recovery_receipt = install.recover(wire, servo_id=1, loader_fresh=True, records=records)

    assert (recovery_receipt["completed"], recovery_receipt["known_servo"]) == (False, False)
    assert recovery_receipt["loader_history"]["previous"] is None
    assert history_path.read_text(encoding="utf-8").splitlines()[0] == "partial record"
    assert history_path.read_text(encoding="utf-8").splitlines()[1].startswith(f"{SIM_LOADER_CRC:08x} ")
    assert any("unreadable line" in warning for warning in recovery_receipt["warnings"])


def test_recover_checks_the_failed_runs_calibration_identity(sim_bus, records, fw231_program):
    original = older_fw231(1, seed=55, fw231_program=fw231_program)
    replacement = older_fw231(1, seed=56, fw231_program=fw231_program)
    wire, _ = sim_bus([replacement])
    install.update(wire, servo_id=1, records=records)
    wire, _ = sim_bus([original])

    def interrupt(*_):
        raise KeyboardInterrupt

    with pytest.raises(KeyboardInterrupt) as interrupted:
        install.update(wire, servo_id=1, records=records, on_progress=interrupt)
    replacement.reset("loader")
    replacement.pending_banner = False
    wire, _ = sim_bus([replacement], LOADER_BAUD)
    with pytest.raises(InstallError, match="calibration fingerprint") as error:
        install.recover(wire, servo_id=1, loader_fresh=True, records=records,
                        receipt=interrupted.value.receipt)
    assert error.value.step == "records" and error.value.recovery is None
    assert last_receipt(records)["completed"] is False


@pytest.mark.parametrize("replace", [False, True])
@pytest.mark.parametrize("lose_final_receipt", [False, True])
def test_recovery_retries_preserve_expected_calibration_identity(sim_bus, records, monkeypatch, replace,
                                                                lose_final_receipt):
    original = SimServo(1, mode="app", program=PACK.raw, seed=55)
    replacement = SimServo(1, mode="app", program=PACK.raw, seed=56)
    wire, _ = sim_bus([replacement])
    install.update(wire, servo_id=1, records=records)  # Even a known replacement is the wrong target.
    wire, _ = sim_bus([original])

    def interrupt(*_):
        raise KeyboardInterrupt

    with pytest.raises(KeyboardInterrupt) as failure:
        install.update(wire, servo_id=1, records=records, on_progress=interrupt)
    expected = json.loads(Path(failure.value.receipt).read_text())["fingerprint"]
    target = replacement if replace else original
    target.reset("loader")
    target.pending_banner = False
    save = install._Run.save

    def save_intermediate_only(run):
        if "finished_utc" in run.receipt:
            raise OSError("injected loss of the final receipt write")
        save(run)

    for fresh in (True, False, False):
        wire, _ = sim_bus([target], LOADER_BAUD)
        with monkeypatch.context() as patched:
            patched.setattr(Loader, "upload", interrupt)
            if lose_final_receipt:
                patched.setattr(install._Run, "save", save_intermediate_only)
            with pytest.raises(KeyboardInterrupt):
                install.recover(wire, servo_id=1, loader_fresh=fresh, records=records)
        receipt = last_receipt(records)
        assert receipt["in_loader"]
        assert receipt["expected_fingerprint"] == expected
        assert "fingerprint" not in receipt

    wire, _ = sim_bus([target], LOADER_BAUD)
    if replace:
        with pytest.raises(InstallError, match="calibration fingerprint"):
            install.recover(wire, servo_id=1, records=records)
        assert not last_receipt(records)["completed"]
    else:
        receipt = install.recover(wire, servo_id=1, records=records)
        assert receipt["completed"] and receipt["fingerprint"] == expected
        assert last_receipt(records)["in_loader"] is False


def test_launch_ack_clears_loader_flag_before_final_receipt(sim_bus, records, monkeypatch):
    wire, _ = sim_bus([SimServo(1, mode="app", program=PACK.raw)])

    def interrupt(*_):
        raise KeyboardInterrupt

    monkeypatch.setattr(install, "_wait_boot", interrupt)
    monkeypatch.setattr(install._Run, "finish", lambda run: run.wire_log.close())
    with pytest.raises(KeyboardInterrupt):
        install.update(wire, servo_id=1, records=records)
    receipt = last_receipt(records)
    assert "finished_utc" not in receipt
    assert receipt["in_loader"] is False
    assert not any(step["step"] == "verified" for step in receipt["steps"])


@pytest.mark.parametrize("defect", ["reply error", "model", "firmware", None])
def test_postupdate_census_checks_bystander_identity(sim_bus, records, defect):
    target, other = [SimServo(i, mode="app", program=PACK.raw, seed=i) for i in (1, 2)]
    wire, _ = sim_bus([target, other])
    on_frame = other.on_frame

    def final_reply(frame, clock):
        result = on_frame(frame, clock)
        if result and frame["instruction"] == 1 and wire.loader_phase == "launched":
            error = 2 if defect == "reply error" else 0
            model = 1190 if defect == "model" else 1200
            firmware = 230 if defect == "firmware" else 231
            return (result[0], other.status(error | 0x80, struct.pack("<HB", model, firmware)))
        return result

    other.on_frame = final_reply
    if defect is None:
        result = install.update(wire, servo_id=1, records=records)
        assert result["completed"]  # A changed alert bit alone is compatible with the same identity.
    else:
        with pytest.raises(InstallError, match=r"census.*ID\(s\).*2") as failure:
            install.update(wire, servo_id=1, records=records)
        assert failure.value.step == "census_after"
        assert failure.value.recovery == install.scan_command(wire)
        result = last_receipt(records)
        assert not result["completed"] and not result["in_loader"]
    assert any(step["step"] == "census_after" for step in result["steps"])


@pytest.mark.parametrize("defect", ["marker", "nv_id"])
def test_update_refuses_inconsistent_preinstall_identity(sim_bus, records, installed, defect):
    servo, = installed(1)
    original_read = servo.read

    def inconsistent(address, length):
        payload = original_read(address, length)
        if (address, length) == registers.MARKER and defect == "marker":
            return 0, bytes(length)
        if (address, length) == registers.BUSDIAG and defect == "nv_id":
            err, original = payload
            return err, original[:1] + b"\xfe" + original[2:]
        return payload

    servo.read = inconsistent
    wire, port = sim_bus([servo])
    with pytest.raises(InstallError, match="firmware marker|NV journal ID") as error:
        install.update(wire, servo_id=1, records=records, allow_id_change=True)
    assert error.value.step == "identify" and error.value.recovery is None
    assert tx_frames(port, 0xF9) == []


@pytest.mark.parametrize("mode", ["stock", "app"])
def test_install_requires_torque_off_readback_before_loader_entry(sim_bus, records, installed, mode):
    servo = SimServo(1, mode="stock", seed=57) if mode == "stock" else installed(1)[0]
    servo.enabled = 1
    original_write = servo.write

    def ignore_torque_off(address, data, broadcast=False):
        if address == registers.TORQUE_ENABLE and data == b"\0":
            return 0
        return original_write(address, data, broadcast)

    servo.write = ignore_torque_off
    wire, port = sim_bus([servo], LOADER_BAUD if mode == "stock" else APP_BAUD)
    with pytest.raises(InstallError, match="still reports torque enabled") as error:
        if mode == "stock":
            install.from_stock(wire, records=records)
        else:
            install.update(wire, servo_id=1, records=records)
    assert error.value.recovery is None
    assert tx_frames(port, 0xF9) == [] and servo.mode == mode and servo.enabled
