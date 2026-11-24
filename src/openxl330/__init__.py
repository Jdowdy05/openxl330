"""openxl330: library and installer for custom XL330 servo firmware.

    import openxl330
    bus = openxl330.Bus("/dev/ttyUSB0")       # or openxl330.Bus.simulated(3)
    arm = bus.group([1, 2, 3])
    arm.configure()                           # once per power-up: the firmware forgets its PD configuration
    arm.enable()                              # holds where each servo is
    arm.move({1: 0.5, 2: -0.2}, duration=0.3)
    print(arm.read())
"""
from __future__ import annotations

__version__ = "0.2.0"

from .bus import Bus, ServoInfo
from .calibration import Calibration
from .errors import (AdapterEchoes, AdapterError, AdapterWarning, BusUncertain, CalibrationError, Error, Faulted,
                     InstallError, InvalidArgument, NoReply, NoServosFound, NotEnabled, NotSupported, PortBusy, Refused,
                     Rejected, UnsupportedServo, UntestedServo)
from .group import Group
from .servo import Servo, State, Status

__all__ = [
    "__version__", "Bus", "Servo", "Group", "State", "Status", "ServoInfo", "Calibration",
    "Error", "InvalidArgument", "Refused", "BusUncertain", "NoReply", "Rejected", "NotSupported", "NotEnabled",
    "Faulted", "InstallError", "UntestedServo", "UnsupportedServo", "NoServosFound", "CalibrationError",
    "AdapterError", "PortBusy", "AdapterEchoes", "AdapterWarning",
]
