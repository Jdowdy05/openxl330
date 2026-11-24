"""Per-model constants (openxl330.models) and the XL330-M077 beta: install policy, bus identity, PD configuration."""
from __future__ import annotations

import struct

import pytest

from openxl330 import Bus, Calibration, images, install, models, registers
from openxl330.errors import UnsupportedServo
from openxl330.sim import SimPort, SimServo, default_eeprom
from openxl330.wire import LOADER_BAUD

from test_install import records, stock_servo, tx_frames  # noqa: F401  (records is a fixture)

RELEASE = images.Release(images.DEFAULT_RELEASE)
RATIO = 77.5 / 288.4


def fast17(variant='comp'):
    """The release ID a fast17 image reports (release 17, ABI 5); 'comp-m077' for the M077 beta image."""
    return dict(number=17, variant=variant, abi=5, source_id=0x17171717)


def app_servo(model, release_id=None, nv_id=1):
    eeprom = default_eeprom(nv_id)
    eeprom[0:2] = struct.pack('<H', model)
    return SimServo(nv_id, mode='app', program=RELEASE.packaged().raw, eeprom=eeprom, release_id=release_id)


def bus_of(*sims):
    return Bus(SimPort(list(sims)), calibration=Calibration(), check_adapter=False)


def config_payloads(port):
    """The data of every host write to 320 (PD config) in the TX log."""
    out = []
    for _, d in port.tx_log:
        if d[:3] == b'\xff\xff\xfd' and len(d) > 9 and d[7] == 3 and struct.unpack_from('<H', d, 8)[0] == 320:
            out.append(d[10:-2])
    return out


def test_m288_constants_match_0_1_0():
    m = models.M288
    assert (m.number, m.name, m.beta) == (1200, 'XL330-M288', False)
    assert m.k_nm_per_v == registers.K_NM_PER_V
    assert (m.kp_comp, m.kd_comp, m.kp_plain, m.kd_plain) == (registers.KP_COMP, registers.KD_COMP,
                                                              registers.KP_PLAIN, registers.KD_PLAIN)
    assert m.proven_stock == frozenset({50, 52, 53})


def test_m077_is_beta_and_ratio_scaled():
    m = models.M077
    assert (m.number, m.name, m.beta, m.gear_ratio) == (1190, 'XL330-M077', True, 77.5)
    assert m.k_nm_per_v == pytest.approx(0.0504 * RATIO)
    assert round(m.k_nm_per_v * 2 ** 24) == 227225               # the firmware session's Q24 starting value
    assert m.kp_comp == pytest.approx(3.5 * RATIO ** 2) and m.kd_comp == pytest.approx(0.10 * RATIO ** 2)
    assert m.kp_plain == pytest.approx(1.71 * RATIO ** 2) and m.kd_plain == pytest.approx(0.043 * RATIO ** 2)
    assert m.proven_stock == frozenset() and 'assum' in m.notes


def test_lookup_by_number_and_name():
    assert models.by_number(1200) is models.M288 and models.by_number(1190) is models.M077
    assert models.by_number(1060) is None and models.by_number(None) is None
    assert models.by_name('XL330-M077') is models.M077 and models.by_name('model 1060') is None


def test_require_refuses_m077_without_beta():
    with pytest.raises(UnsupportedServo) as e:
        models.require(1190, allow_beta=False, what='ID 4')
    assert 'ID 4' in str(e.value) and 'XL330-M077' in str(e.value) and '--beta' in str(e.value)
    assert models.require(1190, allow_beta=True, what='ID 4') is models.M077


def test_require_refuses_unknown_models_even_with_beta():
    with pytest.raises(UnsupportedServo) as e:
        models.require(1060, allow_beta=True, what='ID 2')
    assert 'model 1060' in str(e.value) and 'XL330-M288' in str(e.value)


def test_bundled_release_is_for_the_m288():
    assert RELEASE.model == 1200


def test_install_stock_m077_refused_before_f9_without_beta(sim_bus, records):
    s = stock_servo(model=1190)
    wire, port = sim_bus([s], LOADER_BAUD)
    with pytest.raises(UnsupportedServo) as e:
        install.from_stock(wire, records=records)
    assert '--beta' in str(e.value)
    assert tx_frames(port, 0xF9) == [] and s.mode == 'stock'


def test_install_stock_m077_with_beta_refuses_an_m288_release(sim_bus, records):
    s = stock_servo(model=1190)
    wire, port = sim_bus([s], LOADER_BAUD)
    with pytest.raises(UnsupportedServo) as e:
        install.from_stock(wire, allow_beta=True, release=RELEASE.name, records=records)
    assert 'XL330-M077' in str(e.value) and RELEASE.name in str(e.value) and 'fw231-comp2-fast17-m077' in str(e.value)
    assert tx_frames(port, 0xF9) == [] and s.mode == 'stock'


def test_m288_install_unchanged_with_beta_flag(sim_bus, records):
    s = stock_servo(model=1200, version=52)
    wire, _ = sim_bus([s], LOADER_BAUD)
    r = install.from_stock(wire, allow_beta=True, records=records)
    assert r['completed'] and r['model'] == 1200 and (s.mode, s.version) == ('app', 231)


def test_bus_refuses_m288_image_on_model_1190():
    bus = bus_of(app_servo(1190, fast17('comp')))
    (info,) = bus.scan()
    assert not info.supported and info.model == 'XL330-M077'
    with pytest.raises(UnsupportedServo) as e:
        bus.servo(1)
    assert 'XL330-M077' in str(e.value) and 'XL330-M288' in str(e.value)


def test_bus_refuses_m077_image_on_model_1200():
    bus = bus_of(app_servo(1200, fast17('comp-m077')))
    (info,) = bus.scan()
    assert not info.supported
    with pytest.raises(UnsupportedServo) as e:
        bus.servo(1)
    assert 'comp-m077' in str(e.value) and 'XL330-M288' in str(e.value)


def test_bus_accepts_m077_beta_image_and_marks_it_beta():
    bus = bus_of(app_servo(1190, fast17('comp-m077')))
    info = bus.servo(1).info
    assert info.supported and info.beta and info.model == 'XL330-M077'
    assert not bus_of(app_servo(1200, fast17('comp'))).servo(1).info.beta


def test_older_images_report_1200_and_are_not_beta():
    """Images before release 17 hard-code model 1200 in PING; release 17+ report the servo's NV model."""
    from conftest import make_fw231_program
    eeprom = default_eeprom(1)
    eeprom[0:2] = struct.pack('<H', 1190)
    info = bus_of(SimServo(1, mode='app', program=make_fw231_program(), eeprom=eeprom)).servo(1).info
    assert info.model == 'XL330-M288' and not info.beta


def test_configure_sends_the_model_k():
    for model, release_id, k in ((1200, None, registers.K_NM_PER_V),
                                 (1190, fast17('comp-m077'), models.M077.k_nm_per_v)):
        sim = app_servo(model, release_id)
        bus = bus_of(sim)
        bus.servo(1).configure()
        (payload,) = config_payloads(bus.port)
        assert struct.unpack_from('<I', payload, 12)[0] == round(k * 2 ** 24)


def test_group_configure_uses_each_model_k_and_default_gains():
    m288, m077 = app_servo(1200, fast17('comp'), nv_id=1), app_servo(1190, fast17('comp-m077'), nv_id=2)
    bus = bus_of(m288, m077)
    applied = bus.group([1, 2]).configure()
    gains = {struct.unpack_from('<I', p, 12)[0] for p in config_payloads(bus.port)}
    assert gains == {round(registers.K_NM_PER_V * 2 ** 24), round(models.M077.k_nm_per_v * 2 ** 24)}
    m = models.M077                                  # the bundled program carries a non-zero power-up comp table
    assert applied[2]['comp'] and applied[2]['kp'] == pytest.approx(m.kp_comp) and applied[2]['kd'] == pytest.approx(m.kd_comp)
    assert applied[1]['kp'] == registers.KP_COMP and applied[1]['kd'] == registers.KD_COMP


# ------------------------------------------------------------------------------------------------ gate P1-V1
@pytest.mark.parametrize('word, variant', [(0x04060011, 'comp-m077-log'), (0x04050011, 'comp-log'),
                                           (0x04040011, 'comp-m077'), (0x04010010, 'comp')])
def test_release_variant_codes_5_and_6_decode(word, variant):
    rid = registers.decode_release_id(struct.pack('<4I', word, 0x12345678, 0, 0))
    assert rid['variant'] == variant and rid['number'] == word & 0xFFFF and rid['abi'] == 4


def test_each_variant_names_its_model_and_unknown_codes_name_none():
    assert models.for_variant('comp-m077') is models.M077 and models.for_variant('comp-m077-log') is models.M077
    for v in ('pd', 'comp', 'demo', 'lab', 'comp-log'):
        assert models.for_variant(v) is models.M288
    assert models.for_variant('code 7') is None


def test_bus_accepts_the_m077_logging_bench_image():
    """fw231-comp2-fast17-m077-log-beta reports 1120 = 0x04060011 (release 17, variant 6, ABI 4)."""
    bus = bus_of(app_servo(1190, dict(number=17, variant='comp-m077-log', abi=4, source_id=0xd2b60e4f)))
    info = bus.servo(1).info
    assert info.supported and info.beta and info.model == 'XL330-M077'


def test_bus_refuses_an_unknown_variant_code():
    """Fail closed: a variant this openxl330 does not know is not assumed to be an XL330-M288 image."""
    sim = app_servo(1200, dict(number=17, variant='comp', abi=5, source_id=1))
    real = sim.read

    def read(a, n):
        err, data = real(a, n)
        if (a, n) == (1120, 16) and not err:
            data = bytes(data[:2]) + bytes([7]) + bytes(data[3:])          # variant code 7
        return err, data
    sim.read = read
    bus = bus_of(sim)
    with pytest.raises(UnsupportedServo) as e:
        bus.servo(1)
    assert 'variant' in str(e.value) and 'upgrade openxl330' in str(e.value)
