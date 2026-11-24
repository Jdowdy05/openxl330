"""0.2.0 bundles four fast17 images, {XL330-M288, XL330-M077 (beta)} x {performance (default), logging}, chosen by the
servo's model and the variant asked for (owner, 2026-11-24)."""
from __future__ import annotations

import json
import struct

import pytest

from openxl330 import images, install, models
from openxl330.errors import InstallError, InvalidArgument
from openxl330.sim import SimServo, default_eeprom
from openxl330.wire import LOADER_BAUD

from test_install import records, stock_servo  # noqa: F401  (records is a fixture)

FOUR = {('fw231-comp2-fast17', 1200, 'performance'), ('fw231-comp2-fast17-log', 1200, 'logging'),
        ('fw231-comp2-fast17-m077', 1190, 'performance'), ('fw231-comp2-fast17-m077-log', 1190, 'logging')}


def test_exactly_the_four_fast17_images_are_bundled():
    assert {(n, images.bundled(n).model, images.bundled(n).kind) for n in images.RELEASES} == FOUR
    assert images.DEFAULT_RELEASE == 'fw231-comp2-fast17'
    for name in images.RELEASES:
        r = images.bundled(name)
        assert (r.release_id['number'], r.release_id['abi']) == (17, 5)
        assert r.beta == (r.model == 1190) and r.set_id == (r.kind == 'performance')
        assert models.for_variant(r.release_id['variant']).number == r.model


@pytest.mark.parametrize('name, model, kind', sorted(FOUR))
def test_release_for_picks_by_model_and_variant(name, model, kind):
    assert images.release_for(model, kind) == name


def test_release_for_defaults_to_performance_and_refuses_unknowns():
    assert images.release_for(1200) == 'fw231-comp2-fast17'
    with pytest.raises(InvalidArgument):
        images.release_for(1200, 'debug')
    with pytest.raises(InvalidArgument):
        images.release_for(1060)


def m077_stock(nv_id=1):
    return stock_servo(nv_id, model=1190, version=52)


def test_install_picks_the_m077_image_for_an_m077(sim_bus, records):
    s = m077_stock()
    wire, _ = sim_bus([s], LOADER_BAUD)
    r = install.from_stock(wire, allow_beta=True, records=records)
    assert r['completed'] and r['release'] == 'fw231-comp2-fast17-m077' and s.mode == 'app'


def test_install_logging_variant(sim_bus, records):
    s = stock_servo(version=52)
    wire, _ = sim_bus([s], LOADER_BAUD)
    r = install.from_stock(wire, variant='logging', records=records)
    assert r['completed'] and r['release'] == 'fw231-comp2-fast17-log'


def test_an_explicit_release_must_match_the_model(sim_bus, records):
    s = m077_stock()
    wire, port = sim_bus([s], LOADER_BAUD)
    from openxl330.errors import UnsupportedServo
    with pytest.raises(UnsupportedServo):
        install.from_stock(wire, allow_beta=True, release='fw231-comp2-fast17', records=records)
    assert s.mode == 'stock'


def test_update_keeps_the_running_variant(sim_bus, records):
    s = stock_servo(version=52)
    wire, _ = sim_bus([s], LOADER_BAUD)
    install.from_stock(wire, variant='logging', records=records)
    wire, _ = sim_bus([s])
    assert install.update(wire, servo_id=1, records=records)['release'] == 'fw231-comp2-fast17-log'
    wire, _ = sim_bus([s])
    assert install.update(wire, servo_id=1, variant='performance', records=records)['release'] == 'fw231-comp2-fast17'


def test_recover_uses_the_failed_runs_release(sim_bus, records):
    s = m077_stock()
    real = s.flash_write

    def corrupt(address, data):
        data = bytearray(data)
        data[1000] ^= 0xFF
        real(address, bytes(data))
    s.flash_write = corrupt
    wire, _ = sim_bus([s], LOADER_BAUD)
    with pytest.raises(InstallError):
        install.from_stock(wire, allow_beta=True, records=records)     # FA mismatch: left in the loader
    s.flash_write = real
    wire, _ = sim_bus([s], LOADER_BAUD)
    r = install.recover(wire, servo_id=1, records=records)
    assert r['release'] == 'fw231-comp2-fast17-m077'


def test_cli_install_variant_logging(capsys, monkeypatch):
    from openxl330.cli import main
    from test_cli import use_sim
    use_sim(monkeypatch, stock_servo(version=52))
    assert main(['--sim', '1', 'install', '--stock-id', '1', '--variant', 'logging', '--json']) == 0
    assert json.loads(capsys.readouterr().out)['release'] == 'fw231-comp2-fast17-log'
