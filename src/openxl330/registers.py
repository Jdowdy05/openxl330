"""FW231 control-table layout, decoders and encoders. Pure functions, no I/O.

Control-table addresses of the FW231 register map, all little-endian:
   0..63  R   stock EEPROM view of the NV journal; 0/1 model 1200, 6 firmware 231,
              7 effective ID, 8 live baud index (3 = 1M, 4 = 2M, 5 = 3M)
   64     W1  1 = ARM (PD from the current position), 0 = OFF        R 64/68: 17 status words
   65     W1  LED (1 on, 0 off)                                       126 W1 0x5A: fault clear (OFF)
   208/16 R   marker "FW231-PD-BUS"                                   224/32 R first trip
   256/16 RW  target {i32 counts, u32 duration ticks (2 kHz), u32 profile 0 quintic/1 linear, u32 0}
              profile word is reserved and reads zero
   260/4  W   open-loop duty i32 -480..480 (INT32_MIN = back to PD)
   264/4  W   torque command (fast16+, ABI 4): output torque at the horn, i32 N*m Q24, through the gear-train
              compensation; 268/4 W the same as motor torque referred to the output, no compensation. INT32_MIN to
              260, 264 or 268 leaves torque or duty mode (the PD then holds where the shaft is). Only the configured
              torque limit and duty saturation apply. OFF and faults clear the mode; a target (256) answers 7 in it.
   320/48 RW  PD config (OFF only; read back 704/56)   328/8 W live gains {i32 Kp Q16 Nm/rad, i32 Kd Q16 Nm*s/rad}
   324/40, 336/16, 340/12  live compensation          640/64 R snapshot (word 4 commanded output torque Q24,
              word 13 bit 1 duty mode, bits 2-3 torque mode 0 PD / 1 compensated / 2 raw; decode_drive)
   844/4  RW  baud: write 1000000/2000000/3000000 (also broadcast); read = bps | 1 while unconfirmed
   848    protections W 12 / R 16    852/16 deadline   856/24 sample budget   1536/32*n DAQ drain
   1000/64 R  bus diagnostics (decode_busdiag)  1048/32 R device block (parse_devcfg)
   1064/24 R  common reply timing maxima (16 B on older releases, W 4 B zero = reset)
   1072/12 W  command lease {ms 0..60000 (0 = off), action 0 OFF / 1 hold, 'LEAS'}  R 1072/16 status
   1076/1  W  lease heartbeat (unicast or Sync Write row; a plain broadcast write does not count)
   1080/32 R  Fast Sync Read / fast start counters   1080/4 W gap us (RAM)   1084/1 W fast start on/off (RAM)
   1112/12 R  last reset (fast11+): cause (stall / fault + exception / DMA / watchdog / host reboot / software /
              reset pin / power-on), the RCC reset flags, hang reboots since power-on and the quick-hang streak
              (decode_reset)
   1088/20 R  motion block (fast6+): position, v_hat, estimated torque at the motor and at the horn, signed
              current, applied duty; one Sync / Fast Sync Read for all (decode_motion)
   1120/16 R  release ID (fast12+): release number, variant, ABI, source ID (decode_release_id)
   1136/8  R  loader digest (fast12+): {u32 CRC-32 (zlib) of 0x08000000..0x08003FFF, u32 bytes covered}; fast12
              computes it on request (about 13 ms), fast13 and later at boot. From fast12 on, A0 never returns bytes of that
              region
The command lease is opt-in, not a protection: it is off after every boot.

This module sits at the bottom of the dependency graph: it imports nothing from the wire or later layers.
"""
from __future__ import annotations

import struct

from ._protocol import framing
from .errors import InvalidArgument

# --- addresses: single byte/word addresses are ints, ranges are (address, length) ---------------------------
STATUS = (64, 68)
STATUS_CONTROL = (STATUS[0], 20)     # enable, readiness, permission and faults
STATUS_ARM = (STATUS[0], 36)         # through current position
POSITION = (96, 4)
TORQUE_ENABLE = 64
LED = 65
CLEAR_FAULTS = 126
MARKER = (208, 16)
TARGET = (256, 16)
DUTY = 260
TORQUE = 264                         # fast16+: output torque at the horn, N*m Q24, compensated
TORQUE_RAW = 268                     # fast16+: motor torque referred to the output, N*m Q24, no compensation
TORQUE_RELEASE = -2 ** 31            # INT32_MIN to 260, 264 or 268: back to the PD, holding where the shaft is
SNAPSHOT = (640, 64)
PD_CONFIG = 320
PD_CONFIG_READ = (704, 56)
COMP = (324, 40)
GAINS = 328
BAUD = 844
PROTECT = 848
BUSDIAG = (1000, 64)
DEVBLOCK = (1048, 32)
TIMING = (1064, 32)
REPLY_TIMING = (TIMING[0], 24)
LEASE = 1072
HEARTBEAT = 1076
FSR_COUNTERS = (1080, 48)
MOTION = (1088, 20)
LAST_RESET = (1112, 12)
RELEASE_ID = (1120, 16)
LOADER_DIGEST = (1136, 8)

# --- constants ----------------------------------------------------------------------------------------------
COUNTS_PER_REV = 4096
CENTER_COUNTS = 2048
# The firmware rounds counts * 411775 / 4096 to a signed Q16 reference angle.
MIN_TARGET_COUNTS = -((2 ** 31 * 4096 + 2047) // 411775)
MAX_TARGET_COUNTS = ((2 ** 31 - 1) * 4096 + 2047) // 411775
TICK_HZ = 2000
MAX_REFERENCE_TICKS = 3037000499     # largest duration whose square fits signed int64
DUTY_FULL = 480
K_NM_PER_V = 0.0504            # actuator torque constant, N*m per volt at full duty
# Gains for the gear-train-compensated controller: quintic tracking RMS 0.14-1.09 deg, quiet hold.
# Without the gear-train cancellation these gains buzz at hold.
KP_COMP, KD_COMP = 3.5, 0.10
KP_PLAIN, KD_PLAIN = 1.71, 0.043
FW231_MARKER = b"FW231-PD-BUS"
MIN_ABI = 2

FAULT_NAMES = {1: 'sensor', 2: 'timeout', 4: 'current', 8: 'voltage', 16: 'temperature', 32: 'timing',
               64: 'config', 128: 'travel', 256: 'speed', 512: 'daq', 1024: 'energy'}

# PD config (write 320, OFF only), KF30 lead-1 estimator.
CONFIG_MAGIC = 0x31424450
EST_KF30 = (737857423, 69489982, 3272216)
LEASE_MAGIC = 0x5341454C
LEASE_ACTIONS = {'off': 0, 'hold': 1}
LEASE_MAX_MS = 60000

RESET_REASONS = {0: 'none', 1: 'stall', 2: 'fault', 3: 'DMA', 4: 'watchdog', 5: 'host reboot', 6: 'software',
                 7: 'reset pin', 8: 'power-on'}
HANG_RESETS = ('stall', 'fault', 'DMA', 'watchdog')
RCC_FLAGS = ((1, 'OBL'), (2, 'PIN'), (3, 'POWER'), (4, 'SOFTWARE'), (5, 'IWDG'), (6, 'WWDG'), (7, 'LOWPOWER'))
RELEASE_VARIANTS = ('pd', 'comp', 'demo', 'lab')    # read 1120 word 0 bits 16..23: plain PD, PD-comp, demo, lab

# --- device block (read 1048/32, at DEVCFG_OFFSET in the program; the default one in every bundled image) ---
# 'XLDC', version 1, ID (0xFF = the stock NV journal ID), start-up baud code (0xFF = 1 Mbaud, 4 = 2M, 5 = 3M), 0xFF,
# 20-byte ASCII label, CRC-16 of the 28 bytes before it, FF FF. Only read here: v1 writes no device block, but a
# servo an earlier tool installed may carry one with an ID, label or start-up baud (parse_devcfg reports it).
DEVCFG_OFFSET = 0xC0                # 0x080040C0, right after the 192-byte vector table
DEVCFG_BYTES = 32
DEVCFG_MAGIC = b'XLDC'


def parse_devcfg(block):
    """Mirror of firmware bus_devcfg_resolve: returns dict(state, id, label, boot_baud)."""
    if len(block) != DEVCFG_BYTES or block[:4] != DEVCFG_MAGIC:
        return dict(state='absent', id=None, label='', boot_baud=1000000)
    ok = (block[4] == 1 and (block[6] == 0xFF or 3 <= block[6] <= 5) and block[7] == 0xFF and block[30:] == b'\xff\xff'
          and struct.unpack_from('<H', block, 28)[0] == framing.crc16(block[:28])
          and (block[5] <= 252 or block[5] == 0xFF))
    if not ok:
        return dict(state='invalid', id=None, label='', boot_baud=1000000)
    label = block[8:28].rstrip(b'\0').decode('ascii', 'replace')
    boot = {5: 3000000, 4: 2000000}.get(block[6], 1000000)
    return dict(state='default' if block[5] == 0xFF else 'override', id=None if block[5] == 0xFF else block[5],
                label=label, boot_baud=boot)


# --- decoders -----------------------------------------------------------------------------------------------
def decode_status(raw):
    """Read 64/68: 17 little-endian words."""
    v = struct.unpack('<8I2i7I', raw)
    names = ('enabled', 'ready', 'allowed', 'reserved', 'faults', 'hardware_faults', 'samples', 'time_us',
             'position_counts', 'velocity_mrad_s', 'worst_tick_us', 'drops', 'queue_period', 'origin', 'raw',
             'supply', 'temperature')
    d = dict(zip(names, v))
    d['voltage_mV'] = d['supply'] & 0xFFFF
    return d


DRIVE_MODES = ('position', 'torque', 'raw_torque')
JC_SATURATED, JC_LIMITED = 2, 8       # read 640 word 9 controller flags (firmware joint_control.h)


def decode_drive(raw):
    """Read 640/64 (the controller snapshot): what drives the motor. mode 'position' (the PD), 'duty' (open loop,
    write 260), 'torque' (write 264) or 'raw_torque' (write 268); torque_nm = the commanded output torque after the
    configured limit (N*m, hardware direction; the PD's own output in position mode); duty = the applied duty -1..1
    (word 5 high half, hardware direction); from the controller flags (word 9): saturated = the request needed more
    than full duty, so the motor delivers less than asked (with the default 2 N*m limit this, not the limit, bounds the
    torque), limited = the configured torque limit clipped the request."""
    if len(raw) != SNAPSHOT[1]:
        raise InvalidArgument(f'controller snapshot received {len(raw)} bytes, expected {SNAPSHOT[1]}')
    words = struct.unpack('<16i', raw)
    flags = words[13] & 0xFFFF
    tau_mode = (flags >> 2) & 3
    mode = DRIVE_MODES[tau_mode] if tau_mode < len(DRIVE_MODES) else f'mode {tau_mode}'
    if flags & 2 and not tau_mode:
        mode = 'duty'
    applied = struct.unpack_from('<h', raw, 5 * 4 + 2)[0]
    controller = words[9] & 0xFFFF
    return dict(mode=mode, torque_nm=words[4] / 16777216.0, duty=applied / DUTY_FULL,
                saturated=bool(controller & JC_SATURATED), limited=bool(controller & JC_LIMITED))


def encode_torque(nm):
    """The 4 bytes of a torque command (264 / 268): N*m as Q24. |nm| must be below 128 (the Q24 range)."""
    q = round(nm * 16777216)
    if not -2 ** 31 < q < 2 ** 31:
        raise InvalidArgument(f'torque {nm!r} N*m: use -128 < torque < 128 (the command is N*m in Q24)')
    return _pack('torque', '<i', q)


def decode_control_status(raw):
    """Decode the compact status prefix. The ARM read also covers the current position."""
    enabled, ready, allowed, faults = struct.unpack_from('<3I4xI', raw)
    status = dict(enabled=enabled, ready=ready, allowed=allowed, faults=faults)
    if len(raw) == STATUS_ARM[1]:
        status['position_counts'] = struct.unpack_from('<i', raw, POSITION[0] - STATUS[0])[0]
    return status


def decode_busdiag(raw):
    """Read 1000/64: bus diagnostics."""
    v = struct.unpack('<16I', raw)
    abi = v[0] >> 24
    boot = {3: 1000000, 4: 2000000, 5: 3000000}.get((v[0] >> 18) & 7, 1000000) if abi >= 2 else 1000000
    boot_state = ('none', 'waiting', 'switched', 'rescued')[(v[0] >> 21) & 3] if abi >= 2 else 'none'
    return dict(id=v[0] & 0xFF, nv_id=(v[0] >> 8) & 0xFF,
                devcfg_state=('default', 'override', 'invalid', 'absent')[(v[0] >> 16) & 3],
                abi=abi, boot_baud=boot, boot_state=boot_state, baud=v[1] & ~1, baud_unconfirmed=bool(v[1] & 1),
                uart=dict(pe=v[2], fe=v[3], ne=v[4], ore=v[5], rx_overflow=v[6], tx_abort=v[7]),
                replies=dict(direct=v[8], chained=v[9], fallback=v[10], slotted=v[11], superseded=v[12]),
                statuses_seen=v[13], deferrals=v[14], baud_switches=v[15] & 0xFFFF, baud_reverts=v[15] >> 16)


def decode_reset(raw):
    """Read 1112 (fast11+). A hang the firmware caught (main loop stalled > 250 ms, a fault, a DMA stop
    failure) or a watchdog reset reboots the APPLICATION once it has run healthy for 2 s; it comes back OFF. Before
    that (start-up), or at the 5th hang in a row within 60 s of boot, it goes to the loader instead (recover).
    detail = the exception number of a fault. app_reboots_armed = the boot-time loader request is cleared, i.e. a hang
    now reboots the application. record = the cause came from the firmware's own reset record (else from RCC flags).
    iwdg_update_timeout (fast15+; always False on older images) = the independent watchdog never confirmed its settings
    at start-up, so it runs at its reset default (about 0.5 s) instead of its configured timeout; diagnostic only."""
    w0, hangs, quick = struct.unpack('<3I', raw)
    reason = RESET_REASONS.get(w0 & 0xFF, f'code {w0 & 0xFF}')
    return dict(reason=reason, hang=reason in HANG_RESETS, detail=(w0 >> 8) & 0xFF,
                rcc_flags=[n for b, n in RCC_FLAGS if (w0 >> 16) & (1 << b)], record=bool(w0 >> 24 & 1),
                app_reboots_armed=bool(w0 >> 25 & 1), iwdg_update_timeout=bool(w0 >> 26 & 1), hang_reboots=hangs,
                quick_hangs=quick)


def decode_reply_timing(raw: bytes) -> dict[str, int]:
    """Decode the firmware's shared 16- or 24-byte reply timing block."""
    if len(raw) not in (16, 24):
        raise InvalidArgument(f'reply timing received {len(raw)} bytes, expected 16 or 24')
    direct, armed, chained, primask, *extra = struct.unpack(f'<{len(raw) // 4}I', raw)
    timing = dict(direct_max_us=direct, chained_max_us=chained, primask_max_us=primask)
    if armed:                                  # later releases reserve this word and return zero
        timing['direct_armed_max_us'] = armed
    if extra:
        timing['predecessor_done_max_us'], timing['late_chains'] = extra
    return timing


def decode_motion(raw):
    """Read 1088 (firmware/motion.h). Torques in Nm (Q24), velocity in rad/s (v_hat, Q16), duty -1..1.
    tau_motor: electromagnetic torque referred to the output, K*V*d - b(d)*v_hat, fixed constants (fit to
    hardware runs).
    tau_out: torque at the horn, tau_motor - J_rotor*a_ref - loss(v, |tau_out|), with a_ref the PLANNED acceleration
    of the running profile (quintic; 0 for linear) and 0 otherwise (hold, steps, open-loop duty, OFF, BAM). During
    planned moves it approximates the load torque at the horn; during steps, open-loop or external pushes it includes
    the rotor's inertia torque J_rotor*alpha, and a tracking error shows as J_rotor*(a_ref - a). Unreliable near zero
    speed (stiction, backlash).
    USE tau_out while holding, at slow or steady speed, and on planned (quintic) moves the motor can follow
    (constant speed within a few mNm at 35 % duty, -24..+14 mNm at 85 %; 450 ms quintic rms 22-27 mNm).
    NOT during steps, saturated moves (planned acceleration not achieved; 150 ms quintic rms 144-165 mNm) or external
    pushes in fast motion: there J_rotor*alpha dominates (peaks 0.4-0.8 Nm). Braking (tau_out against v_hat) is
    amplified 1/(1 - C1) = 1.56x by the load-dependent loss term.
    current_ma: |I| with the sign of the duty, an approximation (regeneration can reverse it).
    Both torques are 0 while OFF."""
    pos, v, tm, to, cur, pwm = struct.unpack('<iiiihh', raw)
    return dict(position_counts=pos, velocity_rad_s=v / 65536.0, tau_motor_nm=tm / 16777216.0,
                tau_out_nm=to / 16777216.0, current_ma=cur, pwm=pwm, duty=pwm / 480.0)


def decode_release_id(raw):
    """Read 1120 (fast12+): word 0 = release number (bits 0..15) | variant << 16 (RELEASE_VARIANTS) | ABI << 24
    (the same ABI byte as read 1000), word 1 = source ID (the first 4 bytes of the release's source-tree SHA-256),
    words 2 and 3 reserved (0)."""
    w0, w1, _, _ = struct.unpack('<4I', raw)
    code = (w0 >> 16) & 0xFF
    variant = RELEASE_VARIANTS[code] if code < len(RELEASE_VARIANTS) else f'code {code}'
    return dict(number=w0 & 0xFFFF, variant=variant, abi=w0 >> 24, source_id=w1)


def decode_lease(raw):
    """Read 1072/16: {ms, action word, state | expiries << 8, ms since the last renewal}."""
    ms, act, st, age = struct.unpack('<4I', raw)
    state = st & 0xFF
    if act not in (0, 1) or state not in (0, 1, 2):
        raise InvalidArgument(f'lease readback contains an unknown action or state ({act}, {state})')
    return dict(ms=ms, action='hold' if act else 'off', state=('off', 'running', 'expired')[state],
                expiries=st >> 8, since_renewal_ms=age)


# --- the firmware's motion model in floating point, for analysis and the simulator --------------------------
# Specified: K 0.0620 Nm/V, b0 0.0278, bdrop 0.0254 Nm s/rad, kq 0.00211 Nm/(rad/s)^2, C0 0.0048 Nm, C1 0.36,
# J_rotor 0.90e-3 kg m^2, knee 0.05 rad/s (fit to hardware runs, horn only, <= 50 % duty at constant speed).
# Below are the firmware's fixed-point values of those (firmware/motion.h), so this mirror reproduces what
# the servos report (within the firmware's rounding, a few tens of uNm).
MOTION_K = 2219 * 480000 / 1024 / 2 ** 24
MOTION_B0, MOTION_BDROP, MOTION_KQ = 1822 / 65536, 1665 / 65536, 35400 / 2 ** 24
MOTION_C0, MOTION_C1, MOTION_J_ROTOR, MOTION_VEPS = 80531 / 2 ** 24, 23593 / 65536, 15099 / 2 ** 24, 3277 / 65536


def motion_model(pwm, volts, v_rad_s, a_ref_rad_s2=0.0, enabled=True):
    """(tau_motor_nm, tau_out_nm) the firmware reports for this duty (-480..480), supply, velocity and PLANNED
    acceleration (the running profile's, else 0). tau_out = tau_motor - J_rotor*a_ref - s*(C0 + KQ*v^2 + C1*|tau_out|),
    solved exactly."""
    if not enabled:
        return 0.0, 0.0
    d = pwm / 480.0
    v = max(-32.0, min(32.0, v_rad_s))
    tm = MOTION_K * min(volts, 9.0) * d - (MOTION_B0 - MOTION_BDROP * abs(d)) * v
    s = (1.0 if v > 0 else -1.0) if abs(v) >= MOTION_VEPS else v / MOTION_VEPS
    t = tm - MOTION_J_ROTOR * a_ref_rad_s2 - s * (MOTION_C0 + MOTION_KQ * v * v)
    return tm, t / (1.0 + s * MOTION_C1 * (-1.0 if t < 0 else 1.0))


# --- encoders -----------------------------------------------------------------------------------------------
def _pack(what, fmt, *values):
    try:
        return struct.pack(fmt, *values)
    except struct.error as e:
        raise InvalidArgument(f'{what} value out of range ({e}): check the arguments') from None


def encode_target(counts, duration_ticks, profile):
    """Write 256/16: profile 0 = quintic, 1 = linear; duration in 2 kHz ticks (1 = step)."""
    return _pack('target', '<iIII', counts, duration_ticks, profile, 0)


def encode_release_id(number, variant, abi, source_id):
    """The 16 bytes read 1120 returns for this release (see decode_release_id; the simulator answers with them)."""
    if variant not in RELEASE_VARIANTS:
        raise InvalidArgument(f'release variant {variant!r}: use one of {", ".join(RELEASE_VARIANTS)}')
    if not (type(number) is int and 0 <= number <= 0xFFFF and type(abi) is int and 0 <= abi <= 0xFF):
        raise InvalidArgument(f'release number {number!r} / ABI {abi!r}: use 0..65535 and 0..255')
    return _pack('release ID', '<4I', number | RELEASE_VARIANTS.index(variant) << 16 | abi << 24, source_id, 0, 0)


def encode_gains(kp, kd):
    """Write 328/8: live gains, Q16 (N*m/rad, N*m*s/rad)."""
    try:
        q = round(kp * 65536), round(kd * 65536)
    except (TypeError, ValueError, OverflowError):
        raise InvalidArgument(f'gains kp {kp!r}, kd {kd!r}: use finite numbers') from None
    return _pack('gains', '<ii', *q)


def encode_lease(ms, action):
    """Write 1072/12: ms 0..60000 (0 = off); action 'off' or 'hold'."""
    if not isinstance(action, str) or action not in LEASE_ACTIONS:
        raise InvalidArgument(f"lease action {action!r}: use 'off' or 'hold'")
    if type(ms) is not int or not 0 <= ms <= LEASE_MAX_MS:
        raise InvalidArgument(f'lease ms {ms!r}: use an integer 0..60000 (0 = no lease)')
    return struct.pack('<III', ms, LEASE_ACTIONS[action], LEASE_MAGIC)


def pd_config(kp, kd, token, gain=K_NM_PER_V, limit=2.0):
    """Write 320/48 (OFF only). token: non-zero u32 that must differ from the previous accepted one."""
    if not (0 <= kp <= 50 and 0 <= kd <= 2 and 0 < gain <= 1 and 0 < limit <= 2):
        raise InvalidArgument(f'PD config kp {kp!r}, kd {kd!r}, gain {gain!r}, limit {limit!r}: the firmware accepts '
                              f'0 <= kp <= 50, 0 <= kd <= 2, 0 < gain <= 1, 0 < limit <= 2')
    if type(token) is not int or not 0 < token < 2**32:
        raise InvalidArgument(f'PD config token {token!r}: use a non-zero 32-bit integer')
    return struct.pack('<IiiIIIIIiiII', token, round(kp * 65536), round(kd * 65536), round(gain * 2**24),
                       round(limit * 2**24), *EST_KF30, 0, 16 << 20, 1, CONFIG_MAGIC)
