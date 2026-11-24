"""The servo models openxl330 knows, with the per-model constants the host uses. Pure data, no I/O.

  XL330-M288 (model 1200)  supported; constants identified on the bench (0.1.0)
  XL330-M077 (model 1190)  BETA: same motor and electronics, lower gear ratio. Every constant is the M288 value scaled
                           by the gear ratio (torque constant by N, gains by N squared for the same bandwidth), an
                           assumption until it is re-identified on hardware. Installs and the bus accept it only with
                           an M077 image, and installs only with allow_beta=True (CLI: --beta).

Sources (ROBOTIS e-manual, xl330-m077 / xl330-m288 pages): gear ratio 77.5:1 vs 288.4:1, model number 1190 vs 1200,
no-load speed 383 vs 103 rev/min at 5.0 V (ratio 3.72, the gear ratio), stall torque 0.215 vs 0.52 N*m at 5.0 V (ratio
2.42, not the gear ratio), the same stall current: so K for the M077 lies between the ratio-scaled 0.0135 N*m/V and the
stall-scaled 0.0208 N*m/V.
"""
from __future__ import annotations

from dataclasses import dataclass

from .errors import UnsupportedServo
from .registers import RELEASE_VARIANTS


@dataclass(frozen=True)
class Model:
    """number: the model number the servo reports (PING, control table 0/1); gear_ratio: N:1; k_nm_per_v: the
    actuator torque constant at the output, N*m per volt at full duty (written in every PD config, 320); kp_comp /
    kd_comp: default gains with the image's gear-train compensation loaded, kp_plain / kd_plain without it (N*m/rad,
    N*m*s/rad); beta: accepted only when asked for; proven_stock: stock firmware versions a first install was proven
    over; notes: where the constants come from."""
    number: int
    name: str
    gear_ratio: float
    k_nm_per_v: float
    kp_comp: float
    kd_comp: float
    kp_plain: float
    kd_plain: float
    beta: bool
    proven_stock: frozenset
    notes: str


M288 = Model(1200, 'XL330-M288', 288.4, 0.0504, 3.5, 0.10, 1.71, 0.043, False, frozenset({50, 52, 53}),
             'identified on the bench: K from the 2026-11-16 plant fit, gains from the PD-comp tuning')

_N = 77.5 / M288.gear_ratio
M077 = Model(1190, 'XL330-M077', 77.5, M288.k_nm_per_v * _N, M288.kp_comp * _N ** 2, M288.kd_comp * _N ** 2,
             M288.kp_plain * _N ** 2, M288.kd_plain * _N ** 2, True, frozenset(),
             'assumptions scaled from the M288 by the gear ratio (K by N, gains by N squared); the published stall '
             'torque suggests K up to 0.0208 N*m/V; to be re-identified on hardware')

MODELS = {m.number: m for m in (M288, M077)}


def by_number(number) -> Model | None:
    return MODELS.get(number)


def by_name(name) -> Model | None:
    return next((m for m in MODELS.values() if m.name == name), None)


M077_VARIANTS = frozenset({'comp-m077', 'comp-m077-log'})


def for_variant(variant) -> Model | None:
    """The model a firmware variant (read 1120, decode_release_id) is built for; None for a variant this version does
    not know, which must not be assumed to be either."""
    if variant not in RELEASE_VARIANTS:
        return None
    return M077 if variant in M077_VARIANTS else M288


def require(number, *, allow_beta: bool, what: str) -> Model:
    """The Model for `number`, else UnsupportedServo naming `what` (e.g. 'ID 3'); a beta model only with allow_beta."""
    model = by_number(number)
    if model is None:
        raise UnsupportedServo(f'{what} is model {number}: openxl330 supports the {M288.name} (model {M288.number}) '
                               f'and, as a beta, the {M077.name} (model {M077.number}). Nothing was installed')
    if model.beta and not allow_beta:
        raise UnsupportedServo(f'{what} is an {model.name} (model {model.number}), which openxl330 supports only as a '
                               f'beta (its constants are scaled from the {M288.name}, not yet measured): pass '
                               f'allow_beta=True (CLI: --beta) to accept it. Nothing was installed')
    return model
