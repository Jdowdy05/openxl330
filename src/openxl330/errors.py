"""Every exception openxl330 raises derives from Error; each message says what happened and what to do next.

AdapterWarning is a warning (warnings.warn), not an Error."""
from __future__ import annotations

# Protocol 2.0 status error codes (the low 7 bits of a status packet's error byte).
PROTOCOL_ERRORS = {
    1: 'result fail',
    2: 'instruction error',
    3: 'CRC error',
    4: 'data range error',
    5: 'data length error',
    6: 'data limit error',
    7: 'access error (busy, OFF-only, or not allowed now)',
}

_NEXT_STEP = {
    1: 'check the servo status and faults, then retry',
    2: 'the firmware on this servo does not support this instruction',
    3: 'the request arrived corrupted: check the wiring and the baud, then retry',
    4: 'a value is out of range: check the argument',
    5: 'the data length does not match this address: check the register size',
    6: 'a value exceeds a limit: check the argument',
    7: 'turn the servo OFF or let the running move finish, then retry',
}


class Error(Exception):
    """Base class of every exception openxl330 raises."""


class InvalidArgument(Error, ValueError):
    """An argument is out of range or of the wrong type. Raised before anything is sent."""


class Refused(Error):
    """Not done in the current state (for example while a servo may be in the retained loader); the message
    says what to do first. Nothing was sent."""


class BusUncertain(Error):
    """The state of the bus is unknown (unexpected, partial, duplicate-ID or undecodable bytes, an interrupted
    exchange, or a failing adapter). Nothing more is sent until Wire.reset() or a new connection."""


class NoReply(Error, TimeoutError):
    """Expected replies did not arrive. `partial`: the replies that did arrive, by ID; `missing`: the IDs without
    one. The bus stays usable."""

    def __init__(self, message, partial=None, missing=None):
        super().__init__(message)
        self.partial = dict(partial or {})
        self.missing = list(missing or [])


class Rejected(Error):
    """A servo answered with a protocol error. `error`: the code (PROTOCOL_ERRORS), `meaning`: its name."""

    def __init__(self, servo_id, error, data=b''):
        self.servo_id, self.error, self.data = servo_id, error, bytes(data)
        self.meaning = PROTOCOL_ERRORS.get(error, f'unknown error {error}')
        step = _NEXT_STEP.get(error, 'check the request against the control table')
        super().__init__(f'ID {servo_id} rejected the request with error {error} ({self.meaning}): {step}')


class NotSupported(Error):
    """The servo's firmware does not have this feature."""


class NotEnabled(Error):
    """The servo is OFF (never enabled, or turned off by a fault)."""


class Faulted(Error):
    """The servo has latched faults."""


class InstallError(Error):
    """An install, update or recover step failed. `step`: where; `recovery`: the exact command to run next (also in
    the message): `openxl330 recover --port P --id N` while the servo may be in the retained loader, `openxl330 update
    --port P --id N` once it runs the new firmware; None before F9 (nothing was installed) and for unexpected failures,
    whose message says to share the receipt. `receipt`: the path of the run's receipt.json when a run was started,
    else None."""

    receipt = None

    def __init__(self, message, *, step, recovery=None):
        if recovery and recovery not in message:
            message = f'{message}\nTo recover the servo, run: {recovery}'
        super().__init__(message)
        self.step, self.recovery = step, recovery


class UntestedServo(InstallError):
    """The servo's stock firmware or loader is not among the proven ones."""


class UnsupportedServo(Error):
    """The servo model is not supported. `receipt`: the path of the run's receipt.json, else None."""

    receipt = None


class NoServosFound(Error):
    """No servo answered."""


class CalibrationError(Error):
    """The calibration file cannot be used."""


class AdapterError(Error):
    """The USB-serial adapter cannot be used."""


class PortBusy(AdapterError):
    """Another program has the serial port open."""


class AdapterEchoes(AdapterError):
    """The adapter echoes our own request bytes back (not a half-duplex Dynamixel adapter)."""


class AdapterWarning(UserWarning):
    """Adapter settings that cost speed (for example a high FTDI latency timer)."""
