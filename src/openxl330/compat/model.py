"""Model constants the compatibility layer converts with, in one place, built from openxl330.models.

Stock XL330 control-table facts (unit sizes, factory defaults) are literals here; actuator constants (K, default
gains, model numbers, beta) come from openxl330.models. The approximations are listed in docs/compat.md.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

from .. import models as _models

# Output torque per amp of motor current on the M288, openxl330 bench fit (torque-estimate check: 0.24-0.26 N*m/A;
# the datasheet stall ratio 0.354 overstates it). Other models scale it by their K. Used only to turn stock
# Goal_Current (mA) into a torque command. Needs hardware validation.
M288_TORQUE_PER_AMP_NM = 0.25


@dataclass(frozen=True)
class StockModel:
    model: _models.Model           # openxl330's own model constants
    torque_per_amp_nm: float
    resolution: int = 4096         # encoder counts per turn
    center: int = 2048             # stock "zero" count used by rad-based SDKs (rustypot, Reachy Mini)
    pwm_full: int = 885            # stock Goal_PWM / Present_PWM full scale (100 % duty)
    velocity_unit_rpm: float = 0.229
    accel_unit_rpm2: float = 214.577
    # Supply voltage assumed when turning stock PWM-based gains into N*m/rad (stock gains act on duty, so their
    # stiffness scales with supply voltage).
    nominal_supply_v: float = 5.0
    # Stock factory defaults reported on reads (writes of other values raise NotSupported where unsupported).
    position_p_gain: int = 400
    position_i_gain: int = 0
    position_d_gain: int = 0
    velocity_p_gain: int = 180
    velocity_i_gain: int = 1600
    current_limit_ma: int = 1750
    velocity_limit: int = 445

    @property
    def name(self) -> str:
        return self.model.name

    @property
    def model_number(self) -> int:
        return self.model.number

    @property
    def torque_per_volt_nm(self) -> float:
        return self.model.k_nm_per_v

    @property
    def kd_default(self) -> float:
        """openxl330's default Kd with the gear-train compensation loaded (the mapped-Kd floor)."""
        return self.model.kd_comp

    @property
    def counts_per_rad(self) -> float:
        return self.resolution / (2 * math.pi)

    def velocity_to_stock(self, rad_s: float) -> int:
        return round(rad_s * 60 / (2 * math.pi) / self.velocity_unit_rpm)

    def velocity_from_stock(self, units: float) -> float:
        return units * self.velocity_unit_rpm * 2 * math.pi / 60

    def accel_from_stock(self, units: float) -> float:
        return units * self.accel_unit_rpm2 * 2 * math.pi / 3600

    def _gain_scale(self, supply_v):
        """N*m/rad per (stock PWM unit per count): duty/885 x supply x K, per count -> per rad."""
        v = self.nominal_supply_v if supply_v is None else supply_v
        return v * self.torque_per_volt_nm / self.pwm_full * self.counts_per_rad

    def kp_from_stock(self, p_gain: int, supply_v=None) -> float:
        """Approximation: stock KPP = Position_P_Gain / 128 (PWM units per count of error)."""
        return p_gain / 128 * self._gain_scale(supply_v)

    def kd_from_stock(self, d_gain: int, supply_v=None) -> float:
        """Approximation: stock KPD = Position_D_Gain / 16 (PWM units per count/ms of error rate). The 1 ms rate unit is
        an assumption that needs hardware validation."""
        return d_gain / 16 * self._gain_scale(supply_v) * 1e-3

    def p_to_stock(self, kp: float, supply_v=None) -> int:
        return round(kp / self._gain_scale(supply_v) * 128)

    def d_to_stock(self, kd: float, supply_v=None) -> int:
        return round(kd / (self._gain_scale(supply_v) * 1e-3) * 16)


def _stock(model):
    return StockModel(model, torque_per_amp_nm=M288_TORQUE_PER_AMP_NM * model.k_nm_per_v / _models.M288.k_nm_per_v)


XL330_M288 = _stock(_models.M288)
XL330_M077 = _stock(_models.M077)
MODELS = {m.name: m for m in (XL330_M288, XL330_M077)}
BY_NUMBER = {m.model_number: m for m in MODELS.values()}


def model_for(info) -> StockModel:
    """The StockModel of an openxl330 ServoInfo (or a model name); NotSupported for a model without stock numbers."""
    name = getattr(info, 'model', info)
    if name not in MODELS:
        from ..errors import NotSupported
        raise NotSupported(f'no stock control-table model for {name!r}: compat knows {sorted(MODELS)}')
    return MODELS[name]
