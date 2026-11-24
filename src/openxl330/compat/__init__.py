"""Opt-in compatibility layer: drive openxl330 servos from software written for stock Dynamixel XL330 servos.

    from openxl330.compat import StockTable          # stock control-table names and units, host-side
    from openxl330.compat import lerobot             # LeRobot SerialMotorsBus/DynamixelMotorsBus (openxl330[lerobot])
    from openxl330.compat import reachy_mini         # Reachy Mini motor-controller API

See docs/compat.md for what maps exactly, what is approximated and what raises NotSupported.
"""
from __future__ import annotations

from .dynamixel_table import REGISTERS, SUPPORTED_MODES, Register, StockTable, register
from .model import MODELS, XL330_M077, XL330_M288, StockModel, model_for

__all__ = ["StockTable", "Register", "REGISTERS", "SUPPORTED_MODES", "register", "StockModel", "MODELS", "XL330_M288",
           "XL330_M077", "model_for"]
