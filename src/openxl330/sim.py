"""Simulated half-duplex bus of XL330 servos for offline tests (no hardware, no port).

Each SimServo has a full 64 KB flash image (retained loader 0x0000..0x3FFF, program from
0x4000, own calibration/NV journal 0xE000..0xFFFF) and runs one of:
  * 'stock'  ROBOTIS firmware (version from its EEPROM, V50 by default) at its EEPROM baud
             (ping, reads within the control table 0..147, torque/LED writes, EEPROM writes 7..63 while torque is
             off: an ID write answers from the old ID, a baud write takes effect after its reply; F9);
  * 'app'    FW231, recognised by its marker in the program bytes, with the FW231 bus behaviour; a bundled
             release (recognised by its read-1120 words in the program, as the firmware keeps them in .rodata)
             also answers read 1120 and read 1136 and refuses A0 reads of the loader region, like fast12+;
             any other FW231 program behaves like an older image (no 1120/1136, A0 reads the loader);
  * 'loader' the retained loader v6 at 57600 (banner, '#', CR prompt, F1/F7/FA/F8);
  * 'dead'   a program the simulator does not recognise (silent).
A servo only understands bytes sent at its own baud; anything else is line noise to it
(FW231 counts it). Two servos answering the same request corrupt each other's bytes.
The loader emulation has no decoder: it boots only a bundled release's upload (images.plaintext_for);
anything else is written as received and the servo comes up 'dead'. Its FA answers only for what an upload wrote.
Read 1120 / 1136 and the A0 loader-region refusal (release 12 and later) are emulated on a bundled release only.

Motion: a first-order position model. While ARMed the position follows the reference with a 10 ms lag and at most
6 rad/s; open-loop duty drives it at duty x 11.1 rad/s. The rules follow FW231 (app.c, joint_control.c):
  * the PD configuration (320) is lost at every boot; ARM (64 <- 1) is refused (7) until it is written, while already
    enabled and with a latched fault; live gains (328) and open-loop duty need it too (4);
  * a target (256) is stored even while OFF (duration >= 1 tick, profile 0/1, word 3 zero); at boot the stored target is
    the boot position over 2000 ticks; read 256 returns {target, duration, 0, 0};
  * ARM plans from the shaft to the STORED target over the stored duration (linear only for a linear target that is not
    the current position); a new target while ARMed starts a new profile from the reference (release 12+; an older
    image answers 7 while a profile runs);
  * 260 INT32_MIN is accepted in any state: open loop off, the current position becomes the target (and, while ARMed,
    the reference); any other duty (-480..480) is accepted while OFF too and stays latched through ARM until OFF or
    INT32_MIN;
  * OFF, a latched fault, a reboot or an expired 'off' lease stop the model where it is (an expiry inside a long wait
    stops it at the expiry instant); an expired 'hold' lease acts like 260 INT32_MIN. Protection settings (848) are
    volatile. Voltage and temperature faults can latch with torque OFF and can be cleared once conditions recover.
  * release 16+ (a bundled release with ABI 4): 264 / 268 take a torque command (N*m Q24, compensated / raw) while
    configured, OFF or ARMed; INT32_MIN to 260, 264 or 268 leaves torque or duty mode (the shaft's position becomes the
    target); duty and torque exclude each other; a target (256) answers 7 in torque mode; OFF, a fault and an expired
    'hold' lease leave it (an expired 'hold' lease also refuses 264/268 with 7). While ARMed in torque mode the shaft
    turns at torque / 0.0214 N*m*s/rad (compensated; raw: less 4 mN*m of friction), at most the no-load speed. Read 640
    reports the mode (word 13 bits 2-3, bit 1 duty), the request after the configured limit (word 4: the torque command,
    or the PD output kp*error + kd*(v_ref - v) while ARMed in position mode) and the controller flags (word 9: 2 saturated, 8
    limited). Unlike the firmware, the simulator leaves the gear-train compensation's share out of word 4 and of the
    limited test (the firmware limits the request plus compensation).
The motion block (1088) and the status block report the model. Tests may still set `enabled`, `motion_pwm`, `motion_v`
and `motion_current` directly: the model only runs after an ARM write.
"""
from __future__ import annotations

import math
import struct
import time
import zlib

from . import images
from ._protocol import framing
from .registers import (CONFIG_MAGIC, DEVCFG_BYTES, DEVCFG_OFFSET, FW231_MARKER, MAX_REFERENCE_TICKS,
                        MAX_TARGET_COUNTS, MIN_TARGET_COUNTS, encode_release_id, motion_model, parse_devcfg)

BANNER = b'\r\nROBOTIS bootloader v6\r\nRDP : Yes\r\n'
COUNTS_PER_RAD = 4096 / (2 * math.pi)
MAX_SPEED = 6.0                   # rad/s: the fastest the model follows a target
NO_LOAD_SPEED = 11.1              # rad/s at full duty (open loop)
LAG = 0.01                        # s: first-order lag of the position behind the reference
VISCOUS = 0.0214                  # N*m*s/rad: the torque-mode model's speed per N*m (the comp's b)
RAW_FRICTION = 0.004              # N*m the raw torque mode loses to friction (no compensation)
STEP = 0.001                      # s: integration step
DUTY_PD = -2 ** 31                # write 260: back to PD
# The synthetic retained-loader region every SimServo starts with (loader= gives another): a fill pattern whose last
# four bytes make the region's CRC-32 (zlib) b6934188, the proven loader's (images.known_loaders()), so a simulated
# install sees a known loader. These are not the loader's bytes.
SIM_LOADER = bytes((i * 7 + 3) & 0xFF for i in range(0x4000 - 4)) + bytes.fromhex('865d109e')
_DEVCFG = slice(0x4000 + DEVCFG_OFFSET, 0x4000 + DEVCFG_OFFSET + DEVCFG_BYTES)
_Q16_MAX = (1 << 31) - 1


def _angle_q16(counts):
    angle = counts * 411775 / 4096
    return int(math.copysign(math.floor(abs(angle) + 0.5), angle))


def _reference_fits(start, goal, ticks, linear):
    """Bounds from fast14's Q16 reference generator for a newly planned move."""
    if not 1 <= ticks <= MAX_REFERENCE_TICKS:
        return False
    delta = _angle_q16(goal) - _angle_q16(start)
    if not -_Q16_MAX - 1 <= delta <= _Q16_MAX:
        return False
    if ticks == 1:
        return not linear
    speed = (abs(delta) * 2000 + ticks // 2) // ticks
    if linear:
        return speed <= _Q16_MAX + (delta < 0)
    ticks_squared = ticks * ticks
    accel = (abs(delta) * 4000000 + ticks_squared // 2) // ticks_squared
    return speed * 2 <= _Q16_MAX and accel * 6 <= _Q16_MAX


def _valid_gains(kp, kd):
    return 0 <= kp <= 50 << 16 and 0 <= kd <= 2 << 16


class FakeClock:
    def __init__(self):
        self.t = 1000.0

    def __call__(self):
        self.t += 0.0001
        return self.t

    def sleep(self, dt):
        self.t += max(0.0, dt)


class SimClock(FakeClock):
    """A FakeClock that also follows real time: the library's own waits are instant (sleep() only advances the clock),
    and the time a caller spends between commands (time.sleep, work) passes on the simulated bus too. Bus.simulated()
    uses it, so an example that moves, sleeps and reads sees the servo move."""

    def __init__(self):
        self._t, self._seen = 1000.0, time.monotonic()

    @property
    def t(self):
        now = time.monotonic()
        self._t += max(0.0, now - self._seen)
        self._seen = now
        return self._t

    @t.setter
    def t(self, value):
        self._t = value


def default_eeprom(nv_id=1, baud_index=1):
    e = bytearray(64)
    e[0:2] = struct.pack('<H', 1200); e[2:6] = b'\0\0\0\0'; e[6] = 50; e[7] = nv_id; e[8] = baud_index; e[9] = 250
    e[10] = 0; e[11] = 3; e[12] = 255; e[13] = 2; e[31] = 70; e[32:34] = struct.pack('<H', 70); e[34:36] = struct.pack('<H', 35)
    e[36:38] = struct.pack('<H', 885); e[38:40] = struct.pack('<H', 1750); e[44:48] = struct.pack('<I', 445)
    e[48:52] = struct.pack('<I', 4095); e[63] = 52
    return e


class FastRow(bytes):
    """One Fast Sync Read row (ERR ID data, CRC added by SimPort) plus the packet shape."""
    def __new__(cls, rows, size, body):
        o = super().__new__(cls, body)
        o.rows, o.size = rows, size
        return o


def fast_packet(rows_by_slot, rows, size):
    """FW231 Fast Sync Read on the wire: header, then rows 0, 1, ... each sealed with the CRC of every byte
    before it; a missing slot ends the packet (later servos never saw their prefix); two servos in one
    slot collide."""
    if 0 not in rows_by_slot:
        return b''
    pkt = bytearray([0xFF, 0xFF, 0xFD, 0x00, 0xFE]) + struct.pack('<H', 1 + rows * (size + 4)) + b'\x55'
    crc = framing.crc16(pkt)
    for k in range(rows):
        got = rows_by_slot.get(k, [])
        if len(got) != 1:
            return bytes(pkt) + (SimPort._collide(got) if got else b'')
        row = got[0]
        crc = framing.crc16(row, crc)
        seal = struct.pack('<H', crc)
        pkt += row
        pkt += seal
        crc = framing.crc16(seal, crc)
    return bytes(pkt)


class SimServo:
    STOCK_BAUDS = {0: 9600, 1: 57600, 2: 115200, 3: 1000000, 4: 2000000, 5: 3000000, 6: 4000000, 7: 4500000}
    STOCK_TABLE = 148               # stock control table bytes 0..147 (EEPROM 0..63, RAM 64..147)

    def __init__(self, nv_id=1, *, mode='stock', program=None, loader=None, seed=1, eeprom=None):
        self.flash = bytearray(b'\xff' * 0x10000)
        self.flash[0:0x4000] = loader if loader is not None else SIM_LOADER
        calib = bytes(((i * 31 + seed * 97) ^ (i >> 3)) & 0xFF for i in range(4096))
        journal = bytes(((i * 13 + seed) & 0xFF) for i in range(4096))
        self.flash[0xE000:0xF000] = calib
        self.flash[0xF000:0x10000] = journal
        self.eeprom = bytearray(eeprom if eeprom is not None else default_eeprom(nv_id))
        self.eeprom[7] = nv_id
        if program is not None:
            self.flash[0x4000:0x4000 + len(program)] = program
        self.writes_outside_program = 0
        self.max_write = None
        self.opaque_upload = None       # an upload that is no bundled release: written as received, boots 'dead'
        self.uploaded = None            # (bytes written at 0x4000, the upload that wrote them): what FA can check
        self.clock = None; self.switches = self.reverts = 0
        self.version = None
        self.mode = mode
        self.reset(mode)

    # ---------------------------------------------------------- state
    def reset(self, mode=None):
        mode = mode or 'app'
        self.boot_t = self.clock.t if self.clock else 0.0
        self.reset_reason, self.next_reset = getattr(self, 'next_reset', None) or 8, None   # read 1112 (8 power-on)
        self.hang_reboots = getattr(self, 'hang_reboots', 0)
        self.enabled = 0; self.faults = 0; self.led = None
        self.counts = dict(pe=0, fe=0, ne=0, ore=0, rx_overflow=0, tx_abort=0)
        self.gains = None; self.pending_banner = False
        self.token = 0; self.config_epoch = 0; self.config = None; self.comp = bytes(40)
        self.lease = dict(ms=0, action=0, state=0, last=0.0, expiries=0)
        self.gap_us, self.fast_start, self.fast_sent, self.fast_abandoned, self.irq_started = 0, 1, 0, 0, 0   # 1080/1084
        self.fast_armed = 0
        self.fsr_seen, self.fsr_rows = 0, 0                                     # read-1080 words 8, 9
        self.motion_pwm, self.motion_v, self.motion_current = 0, 0.0, 0          # what read 1088 reports (tests set it)
        self.position = getattr(self, 'position', 2085.0)                       # counts; a reset does not move the shaft
        self.profile, self.duty, self.model_t = None, None, None                # the motion model (runs after an ARM)
        self.tau_mode, self.tau_q24, self.saturated = 0, 0, False               # 264/268 (release 16+): 1 comp, 2 raw
        self.configured = False                                                 # 320 written since this boot
        self.target_counts, self.duration_ticks, self.target_profile = round(self.position), 2000, 0   # stored target
        self.protect = (1000, 65, 70)                                            # 848: current mA, temperature C, volts x10
        self.temperature_c = getattr(self, 'temperature_c', 31)
        self.supply_mv = getattr(self, 'supply_mv', 5000)
        self.loader_text = 'banner'; self.encoded = bytearray(); self.upload_size = 0
        self._pending_baud = None
        self.release_id, self.loader_private = None, False                      # set below for a bundled release
        if mode == 'stock':
            self.mode, self.version, self.baud = 'stock', self.eeprom[6], self.STOCK_BAUDS[self.eeprom[8]]
        elif mode == 'loader':
            self.mode, self.baud, self.pending_banner = 'loader', 57600, True
        else:
            program = bytes(self.flash[0x4000:0xE000])
            self.version = 231 if self.opaque_upload is None and FW231_MARKER in program else None
            self.mode = 'app' if self.version else 'dead'
            self.comp = self._power_up_comp()
            self.release_id = self._release_id(program) if self.version else None
            self.loader_private = self.release_id is not None                   # fast12+: read 1136, no loader A0
            d = parse_devcfg(bytes(self.flash[_DEVCFG])) if self.version else {}
            self.boot_baud = d.get('boot_baud', 1000000) if d.get('state') in ('default', 'override') else 1000000
            self.boot_state = 'waiting' if self.boot_baud != 1000000 else 'none'
            self.boot_since = self.clock.t if self.clock else 0.0
            self.baud = 1000000
            self.baud_prev, self.baud_since, self.baud_unconfirmed = 1000000, 0.0, False

    COMP2 = struct.pack('<10I', 68451, 20054, 1401, 6375, 3277, 0, 0, 9040, 0x102, 0x31504d43)

    def _power_up_comp(self):
        """The comp image carries its power-up table (app_comp_default) in .rodata."""
        return self.COMP2 if self.COMP2 in bytes(self.flash[0x4000:0xE000]) else bytes(40)

    @staticmethod
    def _release_id(program):
        """The bundled release whose read-1120 words (release, variant, ABI, source ID) the program carries, as
        images.Release.release_id; None for any other program."""
        for name in images.RELEASES:
            rid = images.bundled(name).release_id
            if rid and encode_release_id(**rid)[:8] in program:
                return rid.copy()
        return None

    def effective_id(self):
        if self.mode == 'stock':
            return self.eeprom[7]
        if self.version == 231:
            d = parse_devcfg(bytes(self.flash[_DEVCFG]))
            if d['state'] == 'override':
                return d['id']
        return self.eeprom[7]

    def power_cycle(self):
        """Power-up: stock stays stock; anything else boots the program in flash (the loader
        launches a valid application)."""
        self.reset('stock' if self.mode == 'stock' else 'app')

    @property
    def target(self):
        """The stored target as {counts, duration ticks, profile, 0} (the profile is kept here for tests; read 256
        reports 0 in its place, as the firmware does)."""
        return struct.pack('<iIII', self.target_counts, self.duration_ticks, self.target_profile, 0)

    def tick(self, now):
        L = getattr(self, 'lease', None)
        if self.mode == 'app' and L and L['state'] == 1 and self.enabled and L['ms']:
            expiry = L['last'] + L['ms'] / 1000
            if now >= expiry:
                self._model(expiry)                                              # up to the expiry instant only
                L['state'] = 2; L['expiries'] += 1
                if L['action'] == 0:
                    self._off()
                else:
                    self._here(expiry)                                           # the firmware writes 260 INT32_MIN
        self._model(now)
        if self.mode == 'app' and getattr(self, 'boot_state', 'none') == 'waiting' and now - self.boot_since >= 1.5:
            self.baud_prev, self.baud, self.boot_state = self.baud, self.boot_baud, 'switched'
            self.switches += 1
        if self.mode == 'app' and self.baud_unconfirmed and now - self.baud_since >= 2.0:
            self.baud, self.baud_unconfirmed = self.baud_prev, False
            self.reverts += 1

    def noise(self, nbytes):
        """Bytes at a baud this servo is not listening at: FW231 counts them as framing errors."""
        if self.mode == 'app':
            self.counts['fe'] += nbytes

    # ---------------------------------------------------------- replies
    def status(self, error=0, data=b'', reserved=0, sid=None):
        alert = 0x80 if (self.faults and reserved == 0 and self.mode == 'app') else 0
        return framing.frame(reserved, self.effective_id() if sid is None else sid, 0x55, bytes([error | alert]) + data)

    def low_table(self):
        e = bytearray(self.eeprom)
        if self.mode == 'stock':
            return bytes(e)
        e[0:2] = struct.pack('<H', 1200); e[6] = self.version
        e[7] = self.effective_id(); e[8] = {1000000: 3, 2000000: 4, 3000000: 5}[self.baud]
        return bytes(e)

    def stock_table(self):
        """The stock control table 0..147: EEPROM, then RAM (64 torque enable, 65 LED, 144 voltage 5.1 V, 146
        temperature 24 C; the rest zero)."""
        ram = bytearray(self.STOCK_TABLE - 64)
        ram[0], ram[1] = self.enabled, self.led or 0
        ram[80:83] = struct.pack('<HB', 51, 24)
        return self.low_table() + bytes(ram)

    def _stock_write(self, a, data):
        """Stock firmware: torque (64) and LED (65), and EEPROM 7..63 while torque is off (access error 7 while on;
        0..6 are read-only). ID 7 takes effect at once (the reply still comes from the old ID); baud 8 after the
        reply."""
        if a == 64 and len(data) == 1 and data[0] in (0, 1): self.enabled = data[0]; return 0
        if a == 65 and len(data) == 1 and data[0] in (0, 1): self.led = data[0]; return 0
        if not 7 <= a or a + len(data) > 64 or not data: return 7
        if self.enabled: return 7
        if a <= 7 < a + len(data) and data[7 - a] > 252: return 4
        if a <= 8 < a + len(data) and data[8 - a] not in self.STOCK_BAUDS: return 4
        self.eeprom[a:a + len(data)] = data
        if a <= 8 < a + len(data):
            self._pending_baud = self.STOCK_BAUDS[data[8 - a]]
        return 0

    def status_words(self):
        uptime = int((self.clock.t - self.boot_t) * 1e6) & 0xFFFFFFFF if self.clock else 5678
        pos = round(self.position)
        return struct.pack('<17I', self.enabled, not self.faults, 1, 0, self.faults, 0, 1234, uptime,
                           pos & 0xFFFFFFFF, round(self.motion_v * 1000) & 0xFFFFFFFF, 210, 0, 500 << 16, 3456,
                           pos % 4096, self.supply_mv | (12 << 16), self.temperature_c)

    # ---------------------------------------------------------- the motion model
    def _arm(self, now):
        """ARM plans from the shaft to the STORED target over the stored duration; a latched duty stays."""
        linear = self.target_profile == 1 and round(self.position) != self.target_counts
        if not _reference_fits(round(self.position), self.target_counts, self.duration_ticks, linear):
            return False
        self.enabled, self.model_t = 1, now
        self.profile = (self.position, float(self.target_counts), now, self.duration_ticks / 2000.0, int(linear))
        return True

    def _here(self, now):
        """260 INT32_MIN: open loop off; the current position becomes the target and, while ARMed, the reference."""
        self.duty, self.target_counts, self.target_profile = None, round(self.position), 0
        self.tau_mode, self.tau_q24 = 0, 0
        if self.enabled and self.model_t is not None:
            self.profile = (self.position, self.position, now, 0.0, 0)

    def _off(self):
        self.enabled, self.profile, self.duty, self.model_t = 0, None, None, None
        self.tau_mode, self.tau_q24 = 0, 0
        self.motion_v, self.motion_pwm, self.motion_current = 0.0, 0, 0

    def _reference(self, t):
        start, goal, t0, duration, kind = self.profile
        if duration <= 0 or t >= t0 + duration:
            return goal
        x = max(0.0, (t - t0) / duration)
        return start + (goal - start) * (x if kind == 1 else x ** 3 * (10 - 15 * x + 6 * x * x))

    def _moving(self, now):
        """A profile (not a step, not a hold) that has not reached its end."""
        return self.profile is not None and self.profile[3] > 1 / 2000 and now < self.profile[2] + self.profile[3]

    def _target(self, counts, ticks, kind, now):
        """Write 256 while ARMed and changed (another target, or linear): a new profile from where the reference is
        now (an open-loop duty stays in charge until INT32_MIN). Older images refuse it (7) while a profile runs;
        release 12+ (a bundled release) retargets. While OFF the target is only stored."""
        changed = counts != self.target_counts or kind == 1
        if self.mode != 'app' or not self.enabled or self.model_t is None or not changed:
            return 0
        if self.release_id is None and self._moving(now):
            return 7
        start = self._reference(now) if self.profile is not None else self.position
        if not _reference_fits(start, counts, ticks, kind == 1):
            return 4
        self.profile = (start, float(counts), now, ticks / 2000.0, kind)
        return 0

    def _protection_fault(self):
        """Return the current voltage or temperature fault, in firmware priority order."""
        _, temp_c, vmax_dv = self.protect
        if self.supply_mv > vmax_dv * 100 or self.supply_mv < 3500:
            return 8
        return 16 if self.temperature_c >= temp_c else 0

    def _model(self, now):
        if self.mode != 'app':
            return
        self.faults |= self._protection_fault()
        if self.faults and self.enabled:
            self._off()
        if not self.enabled or self.model_t is None:
            return
        dt = now - self.model_t
        if dt <= 0:
            return
        if self.tau_mode:
            tau = self.tau_q24 / 16777216.0
            if self.tau_mode == 2:
                tau = math.copysign(max(0.0, abs(tau) - RAW_FRICTION), tau)
            v = max(-NO_LOAD_SPEED, min(NO_LOAD_SPEED, tau / VISCOUS))
            self.saturated = abs(tau / VISCOUS) > NO_LOAD_SPEED                   # more than full duty asked for
            self.position += v * COUNTS_PER_RAD * dt
        elif self.duty is not None:
            v = self.duty / 480.0 * NO_LOAD_SPEED
            self.position += v * COUNTS_PER_RAD * dt
        else:
            steps = max(1, math.ceil(dt / STEP))
            h, t, before = dt / steps, self.model_t, self.position
            response = 1.0 - math.exp(-h / LAG)
            limit = MAX_SPEED * COUNTS_PER_RAD * h
            for _ in range(steps):
                t += h
                before = self.position
                step = (self._reference(t) - self.position) * response
                self.position += max(-limit, min(limit, step))
            v = (self.position - before) / h / COUNTS_PER_RAD                  # the rate at the end of the interval
        self.model_t = now
        self.motion_v = 0.0 if abs(v) < 1e-9 else v
        pwm = self.duty if self.duty is not None else round(480 * self.motion_v / NO_LOAD_SPEED)
        self.motion_pwm = max(-480, min(480, pwm))
        self.motion_current = round(abs(self.motion_pwm) * 0.6)

    def read(self, a, n):
        if self.mode == 'stock':
            if n < 1 or a + n > self.STOCK_TABLE: return 4, b''
            return 0, self.stock_table()[a:a + n]
        if a + n <= 64: return 0, self.low_table()[a:a + n]
        if 64 <= a and a + n <= 132: return 0, self.status_words()[a - 64:a + n - 64]
        if (a, n) == (208, 16): return 0, FW231_MARKER.ljust(16, b'\0')
        if a + n <= 256:
            table = self.low_table() + self.status_words() + bytes(76) + FW231_MARKER.ljust(16, b'\0') + bytes(32)
            return 0, table[a:a + n]
        if (a, n) == (844, 4):
            return 0, struct.pack('<I', self.baud | (1 if self.baud_unconfirmed else 0))
        if (a, n) == (1000, 64):
            st = parse_devcfg(bytes(self.flash[_DEVCFG]))['state']
            w0 = (self.effective_id() | (self.eeprom[7] << 8) | (('default', 'override', 'invalid', 'absent').index(st) << 16)
                  | ({1000000: 3, 2000000: 4, 3000000: 5}[self.boot_baud] << 18)
                  | (('none', 'waiting', 'switched', 'rescued').index(self.boot_state) << 21)
                  | ((self.release_id['abi'] if self.release_id else 2) << 24))
            c = self.counts
            return 0, struct.pack('<16I', w0, self.baud | (1 if self.baud_unconfirmed else 0), c['pe'], c['fe'], c['ne'], c['ore'],
                                  c['rx_overflow'], c['tx_abort'], 0, 0, 0, 0, 0, 0, 0, (self.switches & 0xFFFF) | (self.reverts << 16))
        if a == 1064 and n in (16, 24, 32, 48, 52):
            later = self.release_id and self.release_id['number'] >= 13
            details = (0, 0, 0, 0) if later else (175, 2 | (64 << 16), 310, 6)
            timing = struct.pack('<13I', 180, 0 if later else 420, 160, 14, 1900, 0,
                                 *details, 12, 0, 0x0800A1B3)
            return 0, timing[:n]
        if (a, n) == (852, 16): return 0, struct.pack('<4I', 0, 0, 0, 100 | (2000 << 16))
        if (a, n) == (1120, 16) and self.release_id:                           # fast12+ release ID
            return 0, encode_release_id(**self.release_id)
        if (a, n) == (1136, 8) and self.loader_private:                         # fast12+: the loader's CRC-32 only
            return 0, struct.pack('<II', zlib.crc32(bytes(self.flash[0:0x4000])), 0x4000)
        if (a, n) == (1112, 12):
            healthy = self.clock is None or not 0 <= self.clock.t - self.boot_t < 2.0   # (a new bus's clock: booted long ago)
            w0 = self.reset_reason | ((1 << 24) if self.reset_reason in (1, 2, 3, 5) else 0) | ((1 << 25) if healthy else 0)
            return 0, struct.pack('<3I', w0, self.hang_reboots, 0)
        if (a, n) == (1088, 20):
            tm, to = motion_model(self.motion_pwm, self.supply_mv / 1000, self.motion_v, enabled=bool(self.enabled))
            cur = -self.motion_current if self.motion_pwm < 0 else self.motion_current
            return 0, struct.pack('<iiiihh', round(self.position), round(self.motion_v * 65536), round(tm * 2 ** 24),
                                  round(to * 2 ** 24), cur, self.motion_pwm)
        if (a, n) in ((1080, 16), (1080, 24), (1080, 32), (1080, 48)):
            w = struct.pack('<12I', self.gap_us | (self.fast_start << 31), self.fast_sent, self.fast_abandoned, 0, self.irq_started, 14,
                            self.fast_armed, 0, self.fsr_seen, self.fsr_rows, 0, 0)
            return 0, w[:n]
        if (a, n) == (1072, 16):
            L = self.lease; now = self.clock.t if self.clock else 0.0
            return 0, struct.pack('<4I', L['ms'], L['action'], L['state'] | (L['expiries'] << 8), int((now - L['last']) * 1000) if L['state'] == 1 else 0)
        if (a, n) == (856, 24): return 0, bytes(24)
        if (a, n) == (848, 16):
            rms = self.motion_current if self.enabled else 0
            return 0, struct.pack('<4I', *self.protect, rms)
        if (a, n) == (1048, 32): return 0, bytes(self.flash[_DEVCFG])
        if (a, n) == (256, 16): return 0, struct.pack('<iIII', self.target_counts, self.duration_ticks, 0, 0)
        if (a, n) == (704, 56):
            prefix = struct.pack('<3I2i', self.token, self.config_epoch, int(self.configured), *(self.gains or (0, 0)))
            detail = self.config[12:40] + struct.pack('<2I', self.config[40] & 1, 0) if self.config else bytes(36)
            return 0, prefix + detail
        if (a, n) == (324, 40): return 0, self.comp
        if (a, n) == (640, 64):
            w = [0] * 16
            limit = struct.unpack_from('<I', self.config, 16)[0] if self.config else 2 << 24
            if self.tau_mode:
                request = self.tau_q24
            elif self.enabled and self.profile is not None and self.gains:      # the PD output (no compensation share)
                now = self.clock.t if self.clock else 0.0
                error = (self._reference(now) - self.position) / COUNTS_PER_RAD
                v_ref = (self._reference(now + 0.0005) - self._reference(now - 0.0005)) / 0.001 / COUNTS_PER_RAD
                kp, kd = self.gains                                            # firmware: kp*e + kd*(v_ref - v_hat)
                request = round((kp * error + kd * (v_ref - self.motion_v)) * 256)   # Q16 gains x rad -> Q24 N*m
            else:
                request = 0
            w[4] = max(-limit, min(limit, request))                            # after the configured limit
            w[9] = (2 if getattr(self, 'saturated', False) else 0) | (8 if abs(request) > limit else 0)
            w[5] = (self.motion_pwm & 0xFFFF) | (self.motion_pwm & 0xFFFF) << 16       # commanded | applied duty
            w[10], w[11] = self.target_counts & 0xFFFFFFFF, self.duration_ticks
            w[13] = (int(self._moving(self.clock.t if self.clock else 0.0)) | (self.duty is not None) << 1
                     | self.tau_mode << 2 | ((self.duty or 0) & 0xFFFF) << 16)
            return 0, struct.pack('<16I', *(x & 0xFFFFFFFF for x in w))
        return 4, b''

    def write(self, a, data, broadcast=False):
        if self.mode == 'stock':
            return self._stock_write(a, data)
        L, now = self.lease, (self.clock.t if self.clock else 0.0)
        if a == 1072:
            if len(data) != 12: return 5
            ms, act, magic = struct.unpack('<III', data)
            if magic != 0x5341454C or ms > 60000 or act > 1: return 4
            L.update(ms=ms, action=act, state=1 if ms else 0, last=now); return 0
        if a == 1076:
            if len(data) != 1: return 5
            if L['state'] == 1 and not broadcast: L['last'] = now
            return 0
        if L['state'] == 2 and L['action'] == 1 and self.enabled:     # an expired 'hold' lease: only while still holding
            if a in (256, 260, 264, 268): return 7
            if a == 64 and data == b'\1':
                L.update(state=1, last=now); return 0
        if a == 64 and len(data) == 1:
            if data[0] == 0: self._off(); return 0
            if data[0] == 1:
                if self.faults or self.enabled or not self.configured: return 7
                if not MIN_TARGET_COUNTS <= round(self.position) <= MAX_TARGET_COUNTS: return 4
                if not self._arm(now): return 4
                if L['ms']: L.update(state=1, last=now)
                return 0
            return 4
        if a == 65 and len(data) == 1:
            if data[0] > 1: return 4
            self.led = data[0]; return 0
        if a == 126:
            if len(data) != 1: return 5
            if data[0] != 0x5A: return 4
            if self.enabled or self._protection_fault(): return 7
            self.faults = 0; return 0
        if a == 844 and len(data) == 4:
            v = struct.unpack('<I', data)[0]
            if v not in (1000000, 2000000, 3000000): return 4
            if self.baud_unconfirmed: return 7
            return self._switch(v) if v != self.baud else 0
        if a == 256:
            if len(data) != 16: return 5
            counts, ticks, kind, pad = struct.unpack('<iIII', data)
            if not ticks or kind > 1 or pad or not MIN_TARGET_COUNTS <= counts <= MAX_TARGET_COUNTS: return 4
            if self.tau_mode: return 7                                     # torque mode: leave it first
            err = self._target(counts, ticks, kind, now)
            if err: return err
            self.target_counts, self.duration_ticks, self.target_profile = counts, ticks, kind
            if L['state'] == 1 and not broadcast: L['last'] = now
            return 0
        if a == 260:
            if len(data) != 4: return 5
            v = struct.unpack('<i', data)[0]
            if v == DUTY_PD:
                self._here(now)
            elif not -480 <= v <= 480 or not self.configured:
                return 4
            else:
                self.duty = v                                              # also while OFF: latched until OFF / INT32_MIN
                self.tau_mode, self.tau_q24 = 0, 0                         # duty and torque exclude each other
            if L['state'] == 1 and not broadcast: L['last'] = now
            return 0
        if a in (264, 268) and self.release_id and self.release_id['number'] >= 16:
            if len(data) != 4: return 5
            v = struct.unpack('<i', data)[0]
            if v == DUTY_PD:
                self._here(now)
            elif not self.configured:
                return 4
            else:
                self.tau_mode, self.tau_q24, self.duty = (1 if a == 264 else 2), v, None
            if L['state'] == 1 and not broadcast: L['last'] = now
            return 0
        if a == 324 and len(data) == 40: self.comp = bytes(data); return 0
        if a == 848:
            if len(data) != 12: return 5
            if self.enabled: return 7
            cur, temp, vmax = struct.unpack('<3I', data)
            if not (cur <= 100000 and 20 <= temp <= 100 and 50 <= vmax <= 70): return 4
            self.protect = (cur, temp, vmax); return 0
        if a == 328:
            if len(data) != 8 or not self.configured: return 4
            gains = struct.unpack('<ii', data)
            if not _valid_gains(*gains): return 4
            self.gains = gains; return 0
        if a == 1064 and len(data) == 4: return 0 if data == bytes(4) else 4
        if a == 1080:
            if len(data) != 4: return 5
            v = struct.unpack('<I', data)[0]
            if v > 2000: return 4
            self.gap_us = v; return 0
        if a == 1084:
            if len(data) != 1: return 5
            if data[0] > 1: return 4
            self.fast_start = data[0]; return 0
        if a == 320:
            if self.enabled: return 7
            if len(data) != 48: return 4
            token, kp, kd, gain, torque_limit, _, _, _, deadzone, resync, flags, magic = struct.unpack('<IiiIIIIIiiII', data)
            if (token <= self.token or magic != CONFIG_MAGIC or not _valid_gains(kp, kd)
                    or not 0 < gain <= 1 << 24 or not 0 < torque_limit <= 2 << 24 or flags & ~0x101
                    or not 0 <= deadzone < 1 << 24 or not 0 < resync < 1 << 27):
                return 4
            self.gains = (kp, kd)
            self.config_epoch += 1
            self.token, self.config, self.configured = token, bytes(data), True; return 0
        return 7

    def _switch(self, v):
        self._pending_baud = v
        return 0

    def after_reply(self, clock):
        v = getattr(self, '_pending_baud', None)
        if v:
            self.baud_prev, self.baud, self.baud_since, self.baud_unconfirmed = self.baud, v, clock(), True
            self._pending_baud = None; self.switches += 1

    def heard_valid_frame(self):
        if self.mode == 'app':
            self.baud_unconfirmed = False
            if self.boot_state == 'waiting':
                self.boot_state = 'rescued'      # a host at 1 Mbaud within 1.5 s of power-up

    def flash_write(self, address, data):
        end = address + len(data)
        if address < 0x4000 or end > 0xE000:
            self.writes_outside_program += 1
            raise AssertionError(f'simulated loader asked to write 0x{address:x}..0x{end:x} outside the program region')
        self.flash[address:end] = data
        self.max_write = max(self.max_write or 0, end)

    # ---------------------------------------------------------- frames
    def on_frame(self, f, clock):
        """f: parsed request dict. Returns (key, reply bytes) or None. Side effects after reply via .post."""
        self.post = None
        if self.mode == 'loader':
            return self._loader_frame(f)
        if self.mode not in ('app', 'stock') or f['reserved'] != 0:
            return None
        sid, bc, ins, p = f['id'], f['id'] == 0xFE, f['instruction'], f['params']
        me = self.effective_id()
        self.heard_valid_frame()
        if ins == 0x8A and self.mode == 'app':
            self.fsr_seen += 1                                               # CRC-valid 0x8A parsed
        if sid != me and not bc:
            return None
        if ins in (0x82, 0x92, 0x83, 0x93, 0x8A):
            response = self._group(ins, p)
            if ins == 0x83 and response is None:
                self.after_reply(clock)                                   # an addressed baud row has no broadcast ACK
            return response
        if ins == 1:
            t = self.low_table()
            return (me if bc else 0, self.status(0, t[0:2] + t[6:7]))     # model, firmware version
        if bc and ins != 3:
            return None
        if ins == 2:
            a, n = struct.unpack('<HH', p)
            err, data = self.read(a, n)
            return (0, self.status(err, data if not err else b''))
        if ins == 3:
            a = struct.unpack_from('<H', p)[0]
            err = self.write(a, p[2:], bc)
            if bc:
                self.after_reply(clock); return None
            return (0, self.status(err, sid=me))         # an ID write still answers from the old ID
        if ins == 0xA0:
            if self.mode == 'stock': return (0, self.status(2))
            addr, n = struct.unpack('<IH', p)
            if not (0x08000000 <= addr and addr + n <= 0x08010000 and 1 <= n <= 96): return (0, self.status(4))
            if self.loader_private and addr < 0x08004000: return (0, self.status(4))   # never loader bytes
            o = addr - 0x08000000
            return (0, self.status(0, bytes(self.flash[o:o + n])))
        if ins == 0xF9:
            self.post = 'loader'; return (0, self.status(0))
        if ins == 8:
            if self.mode == 'app':
                self.next_reset = 5; self.post = 'reboot'                 # FW231: back into the application
            else:
                self.post = 'loader'
            return (0, self.status(0))
        return (0, self.status(2))

    def _group(self, ins, p):
        me = self.effective_id()
        if ins == 0x82:
            a, n = struct.unpack_from('<HH', p)
            ids = list(p[4:])
            if me not in ids: return None
            err, data = self.read(a, n)
            return (ids.index(me), self.status(err, data if not err else b''))
        if ins == 0x8A:                  # Fast Sync Read: our row only; SimPort builds the one packet
            a, n = struct.unpack_from('<HH', p)
            ids = list(p[4:])
            if me not in ids or ids.count(me) > 1: return None
            err, data = self.read(a, n)
            alert = 0x80 if (self.faults and self.mode == 'app') else 0
            return (ids.index(me), FastRow(len(ids), n, bytes([err | alert, me]) + (bytes(n) if err else data)))
        if ins == 0x92:
            rows = [struct.unpack_from('<BHH', p, k) for k in range(0, len(p), 5)]
            for k, (i, a, n) in enumerate(rows):
                if i == me:
                    err, data = self.read(a, n)
                    return (k, self.status(err, data if not err else b''))
            return None
        if ins == 0x83:
            a, n = struct.unpack_from('<HH', p)
            for k in range(4, len(p), n + 1):
                if p[k] == me: self.write(a, p[k + 1:k + 1 + n], False)   # a Sync Write row is addressed to this servo
            return None
        return None

    def _loader_frame(self, f):
        if f['reserved'] != 10 or f['id'] != 254 or self.loader_text != 'ready':
            return None
        ins, p = f['instruction'], f['params']
        ok = lambda data=b'', err=0: (0, framing.frame(10, 252, 0x55, bytes([err]) + data))
        if ins == 0xF1: return ok(b'\x06')
        if ins == 0xF7:
            sub = p[0]
            if sub == 1:
                self.upload_size = int.from_bytes(p[1:5], 'little'); self.encoded = bytearray()
                self.uploaded = None                                            # a new upload replaces the program
                if self.upload_size > 0xE000 - 0x4000: return ok(err=4)
                return ok(struct.pack('<HI', 0, 0))
            index = int.from_bytes(p[1:3], 'little'); chunk = p[3:]
            if index * 128 != len(self.encoded): return ok(err=1)
            self.encoded += chunk
            if sub == 3:
                if len(self.encoded) != self.upload_size: return ok(err=1)
                upload = bytes(self.encoded)
                raw = images.plaintext_for(upload)
                # No decoder: an upload that is no bundled release is kept as received and boots 'dead'.
                written = raw if raw is not None else upload
                self.opaque_upload = None if raw is not None else upload
                self.uploaded = (written, upload)
                self.flash_write(0x4000, written)
            return ok(struct.pack('<HI', index + 1, len(self.encoded)))
        if ins == 0xFA:
            length, expected = struct.unpack('<IH', p)
            crc = self._fa_crc(length, expected)
            return ok(struct.pack('<H', crc), 0 if crc == expected else 245)
        if ins == 0xF8:
            self.post = 'launch'; return ok()
        return ok(err=2)

    def _fa_crc(self, length, expected):
        """The loader's FA over `length` bytes of the program it wrote. The simulator answers it only for what the
        last upload wrote, with the CRC-16 of that upload's own bytes over `length`; a region that differs from what
        the last upload wrote (changed since, or longer) reads as a mismatch."""
        if self.uploaded is not None:
            written, upload = self.uploaded
            if length <= len(written) and bytes(self.flash[0x4000:0x4000 + length]) == written[:length]:
                return framing.crc16(upload[:length])
        return expected ^ 0xFFFF

    def on_text(self, data):
        """Raw handshake bytes in loader mode."""
        out = b''
        for b in data:
            if self.loader_text == 'banner' and b == ord('#'):
                self.loader_text = 'echo'; out += b'#'
            elif self.loader_text == 'echo' and b == ord('\r'):
                self.loader_text = 'ready'; out += b'\r\n$'
        return out


def parse_requests(data):
    """Split host bytes into frames and raw handshake bytes."""
    out, i = [], 0
    while i < len(data):
        if data[i:i + 3] == b'\xff\xff\xfd' and i + 7 <= len(data):
            total = 7 + int.from_bytes(data[i + 5:i + 7], 'little')
            raw = bytes(data[i:i + total])
            if framing.crc16(raw[:-2]) != int.from_bytes(raw[-2:], 'little'):
                out.append(('bad', raw)); i += total; continue
            payload = raw[7:-2].replace(b'\xff\xff\xfd\xfd', b'\xff\xff\xfd')
            out.append(('frame', dict(reserved=raw[3], id=raw[4], instruction=payload[0], params=payload[1:], raw=raw)))
            i += total
        else:
            j = data.find(b'\xff\xff\xfd', i + 1)
            j = len(data) if j < 0 else j
            out.append(('text', bytes(data[i:j]))); i = j
    return out


class SimPort:
    """pyserial-like port shared by all SimServos. Its name (pyserial's `port`) is "SIM", so recovery commands read
    `openxl330 recover --port SIM ...`."""

    port = "SIM"

    def __init__(self, servos, clock=None):
        self.servos = list(servos)
        self.clock = clock or FakeClock()
        for s in self.servos:
            s.clock = self.clock
        self.baudrate = 1000000
        self.rx = bytearray()
        self.tx_log = []
        self.later = []              # (release time, bytes): line activity that starts after a delay
        # Opt-in: broadcast-ping replies arrive in their slots (ID x 3 ms after the request) instead of at once, so a
        # host window that ends early sees the later ones as unsolicited bytes. Default off.
        self.ping_slots = False
        self.timed = []              # (release time, baud, bytes): slot replies, delivered only if the host is still at that baud

    @property
    def in_waiting(self):
        self._banners()
        return len(self.rx)

    def _banners(self):
        due = [x for x in self.later if x[0] <= self.clock.t]
        for x in due:
            self.later.remove(x)
            if self.baudrate == 1000000:
                self.rx += x[1]
        for x in [x for x in self.timed if x[0] <= self.clock.t]:
            self.timed.remove(x)
            if x[1] == self.baudrate:
                self.rx += x[2]
        for s in self.servos:
            s.tick(self.clock.t)
            if s.mode == 'loader' and s.pending_banner and self.baudrate == 57600:
                s.pending_banner = False
                self.rx += b'\x00\xf0' + BANNER          # first bytes garbled by the baud change

    def read(self, n=1):
        self._banners()
        part = bytes(self.rx[:n]); del self.rx[:n]
        return part

    def flush(self):
        pass

    def reset_input_buffer(self):
        self.rx.clear()

    def close(self):
        pass

    def write(self, data):
        data = bytes(data)
        self.tx_log.append((self.baudrate, data))
        for s in self.servos:
            s.tick(self.clock.t)
        listening = [s for s in self.servos if s.baud == self.baudrate and s.mode != 'dead']
        for s in self.servos:
            if s not in listening:
                s.noise(len(data))
        for kind, item in parse_requests(data):
            if kind == 'text':
                replies = [s.on_text(item) for s in listening if s.mode == 'loader']
                replies = [r for r in replies if r]
                self._emit(replies)
                continue
            if kind == 'bad':
                continue
            keyed = []
            for s in listening:
                r = s.on_frame(item, self.clock)
                if r is not None:
                    keyed.append((r[0], s, r[1]))
            keyed.sort(key=lambda x: x[0])
            fast = [(k, raw) for k, s, raw in keyed if isinstance(raw, FastRow)]
            if fast:
                slots = {}
                for k, raw in fast:
                    slots.setdefault(k, []).append(raw)
                cut = next(k for k in range(fast[0][1].rows + 1) if k not in slots or k == fast[0][1].rows)
                for k, sv, raw in keyed:                 # FW231 read-1080 counters: sent, or abandoned after a gap
                    if isinstance(raw, FastRow):
                        sv.fsr_rows += 1
                    if isinstance(raw, FastRow) and k:
                        sv.fast_armed += 1
                    if isinstance(raw, FastRow) and 0 < k < cut:
                        sv.fast_sent += 1; sv.irq_started += sv.fast_start
                    elif isinstance(raw, FastRow) and k > cut:
                        sv.fast_abandoned += 1
                self._emit([fast_packet(slots, fast[0][1].rows, fast[0][1].size)])
                keyed = [x for x in keyed if not isinstance(x[2], FastRow)]
            groups = {}
            for key, s, raw in keyed:
                groups.setdefault(key, []).append(raw)
            replies = [(key, v[0] if len(v) == 1 else self._collide(v)) for key, v in sorted(groups.items())]
            if self.ping_slots and item['id'] == 0xFE and item['instruction'] == 1:
                self.timed += [(self.clock.t + 0.0005 + key * 0.003, self.baudrate, raw) for key, raw in replies]
            else:
                self._emit([raw for _, raw in replies])
            for _, s, _ in keyed:
                s.after_reply(self.clock)
                if s.post == 'loader':
                    s.reset('loader')
                elif s.post == 'reboot':
                    s.reset('app')
                    # measured on hardware: ~55 ms after the ACK, 128 x 0x00 at 1 Mbaud
                    # (the loader's 57600 start-up output); some of it in a second burst
                    self.later += [(self.clock.t + 0.055, b'\0' * 100), (self.clock.t + 0.07, b'\0' * 28)]
                elif s.post == 'launch':
                    s.reset('app')
                s.post = None
        return len(data)

    def _emit(self, chunks):
        for c in chunks:
            self.rx += c

    @staticmethod
    def _collide(replies):
        n = max(len(r) for r in replies)
        out = bytearray(n)
        for r in replies:
            for i, b in enumerate(r):
                out[i] |= b ^ (i * 37 & 0xFF if r is not replies[0] else 0)
        return bytes(out)
