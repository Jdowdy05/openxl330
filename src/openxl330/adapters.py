"""USB-serial adapter checks run when a Bus opens: an adapter that echoes our own request bytes is refused, and a slow
FTDI latency timer gets advice (warnings.warn with AdapterWarning, through Bus).

  check_echo(wire)                 one ping to ID 253 (no servo uses it); our own bytes coming back -> AdapterEchoes
  ftdi_latency_warning(port_name)  Linux: the usb-serial latency timer, with the command that sets it to 1 ms;
                                   macOS: an FTDI adapter (0403:6014 FT232H, 0403:6001 FT232R) costs ~15 ms per exchange
"""
from __future__ import annotations

import os
import sys
from pathlib import Path

from ._protocol import framing
from .errors import AdapterEchoes, BusUncertain

SYSFS_USB_SERIAL = Path('/sys/bus/usb-serial/devices')
FTDI_IDS = frozenset({(0x0403, 0x6014), (0x0403, 0x6001)})
ECHO_ID = 253                       # no servo answers it: a reply can only be the adapter's echo (or a fault)
ECHO_QUIET = 0.04                   # s of quiet that ends the wait (an FTDI adapter hands bytes over within ~16 ms)
ECHO_LIMIT = 0.2


def _platform() -> str:
    return sys.platform


def _comports():
    try:
        from serial.tools import list_ports
        return list(list_ports.comports())
    except Exception:                # no listing on this system: no advice, never a failure
        return []


def _port_name(wire) -> str:
    return getattr(wire.port, 'port', None) or 'this port'


def check_echo(wire) -> None:
    """Send one ping to ID 253 and listen briefly. Nothing back: fine. Our own request bytes back: the adapter echoes
    (a full-duplex or echoing RS-485 adapter), which v1 does not support: the Wire is latched (nothing more is sent)
    and AdapterEchoes is raised. Anything else back latches the Wire (BusUncertain)."""
    request = framing.frame(0, ECHO_ID, 1)              # what Wire._echo_probe sends
    got = wire._echo_probe(ECHO_QUIET, ECHO_LIMIT)
    if not got:
        return None
    if got.startswith(request) or request.startswith(got):
        wire.mark_uncertain('the adapter echoed the request back')
        raise AdapterEchoes(f'the adapter on {_port_name(wire)} echoes our own request bytes back (a full-duplex or '
                            f'echoing RS-485 adapter), so replies cannot be told apart from requests: openxl330 needs a '
                            f'half-duplex Dynamixel adapter (for example a U2D2 or a BusLinker-type board). Nothing more '
                            f'is sent on this port')
    why = f'unexpected bytes after a ping to ID {ECHO_ID}, which no servo uses: {got[:16].hex()}'
    wire.mark_uncertain(why)
    raise BusUncertain(f'{why}. The bus state is unknown, so nothing more is sent: make sure only Dynamixel servos and '
                       f'one adapter are on the bus, then reopen the port.')


def ftdi_latency_warning(port_name) -> str | None:
    """Advice for an FTDI adapter whose latency timer slows every exchange, else None.
    Linux: reads /sys/bus/usb-serial/devices/<tty>/latency_timer and, if it is not 1 ms, returns the exact
    `echo 1 | sudo tee ...` command. macOS: for an FTDI VID:PID, returns the ~15 ms per exchange note (the driver's
    timer cannot be set from here). Anything else: None."""
    if not isinstance(port_name, str):
        return None
    platform = _platform()
    if platform.startswith('linux'):
        tty = os.path.basename(os.path.realpath(port_name))
        timer = SYSFS_USB_SERIAL / tty / 'latency_timer'
        try:
            value = int(timer.read_text().strip())
        except (OSError, ValueError):
            return None
        if value == 1:
            return None
        return (f'the FTDI adapter {port_name} has a {value} ms latency timer, so every exchange can wait up to '
                f'{value} ms. Set it to 1 ms (until the adapter is replugged): echo 1 | sudo tee {timer}')
    if platform == 'darwin':
        for p in _comports():
            if getattr(p, 'device', None) == port_name and (getattr(p, 'vid', None), getattr(p, 'pid', None)) in FTDI_IDS:
                return (f'{port_name} is an FTDI adapter: with the macOS driver expect ~15 ms per exchange (its latency '
                        f'timer); a CDC adapter (BusLinker-type) answers in ~1-2 ms')
    return None
