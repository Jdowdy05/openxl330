"""Dynamixel SDK compatibility: the official ROBOTIS Python SDK (dynamixel_sdk, Protocol 2.0)
talks over a pseudo-terminal to six virtual servos that run the FIRMWARE's own protocol.c
(built as a shared library, called through ctypes). Group replies are written in the order
the firmware's response_slot gives (the wire timing of that order is covered by test_bus.c).

Covers ping, broadcast ping, 1/2/4-byte read/write, GroupSyncRead/Write, GroupBulkRead/Write,
reboot, status error and alert reporting, byte stuffing, and GroupSyncRead.fastSyncRead() (0x8A,
FW231): each follower's row CRC is sealed by the firmware's own bus_fast_* code over the bytes
before it, exactly as main.c does on the wire; a missing servo ends the packet (SDK timeout).
"""
import ctypes as C, os, select, struct, subprocess, sys, threading, time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
LIB = ROOT / 'build/native/libdxl_protocol.dylib'
subprocess.run(['clang', '-std=c11', '-O1', '-shared', '-fPIC', '-Wall', '-Wextra', '-Werror', '-I', str(ROOT / 'firmware'),
                str(ROOT / 'firmware/protocol.c'), str(ROOT / 'firmware/bus.c'), '-o', str(LIB)], check=True)
lib = C.CDLL(str(LIB))
import dynamixel_sdk as sdk

READ = C.CFUNCTYPE(C.c_uint8, C.c_void_p, C.c_uint16, C.c_uint16, C.POINTER(C.c_uint8))
WRITE = C.CFUNCTYPE(C.c_uint8, C.c_void_p, C.c_uint16, C.c_uint16, C.POINTER(C.c_uint8))
FLASH = C.CFUNCTYPE(C.c_uint8, C.c_void_p, C.c_uint32, C.c_uint16, C.POINTER(C.c_uint8))


class Io(C.Structure):
    _fields_ = [('id', C.c_uint8), ('firmware', C.c_uint8), ('model', C.c_uint16), ('context', C.c_void_p),
                ('read', READ), ('write', WRITE), ('flash_read', FLASH), ('alert', C.c_uint8)]


class Result(C.Structure):
    _fields_ = [('reply_bytes', C.c_uint16), ('valid_for_us', C.c_bool), ('enter_loader', C.c_bool), ('group_reply', C.c_bool),
                ('response_slot', C.c_uint8), ('reboot', C.c_bool), ('prev_id', C.c_uint8), ('prior_bytes', C.c_uint32),
                ('fast', C.c_bool), ('fast_len', C.c_uint16), ('fast_seg', C.c_uint16)]


class Fast(C.Structure):                  # bus.h bus_fast
    _fields_ = [('count', C.c_uint32), ('need', C.c_uint32), ('crc', C.c_uint16), ('seg', C.c_uint16), ('len', C.c_uint16),
                ('field', C.c_uint16), ('prev_id', C.c_uint8), ('state', C.c_uint8)]


lib.dxl_process.restype = Result
lib.dxl_process.argtypes = [C.POINTER(Io), C.POINTER(C.c_uint8), C.c_uint16, C.POINTER(C.c_uint8), C.c_uint16]
lib.bus_fast_begin.argtypes = [C.POINTER(Fast), C.c_uint32, C.c_uint16, C.c_uint16, C.c_uint8]
lib.bus_fast_feed.argtypes = [C.POINTER(Fast), C.c_uint8]; lib.bus_fast_feed.restype = C.c_uint8
lib.bus_fast_seal.argtypes = [C.POINTER(Fast), C.POINTER(C.c_uint8), C.c_uint16]
FAST_READY = 2


def fast_packet(rows):
    """rows: [(slot, result, out)] of a Fast Sync Read. Slot 0 sends header + row; each later servo checks
    everything before its row with the firmware's verifier and seals its CRC, or stays silent."""
    pkt = bytearray()
    for k, (slot, res, out) in enumerate(sorted(rows, key=lambda r: r[0])):
        if slot != k:
            break                                                        # a missing servo ends the packet
        if k:
            f = Fast(); lib.bus_fast_begin(C.byref(f), res.prior_bytes, res.fast_seg, res.fast_len, res.prev_id)
            st = 0
            for b in pkt: st = lib.bus_fast_feed(C.byref(f), b)
            if st != FAST_READY:
                break
            lib.bus_fast_seal(C.byref(f), out, res.reply_bytes)
        pkt += bytes(out[:res.reply_bytes])
    return bytes(pkt)


class Servo:
    def __init__(self, sid):
        self.table = bytearray(2048)
        for a in range(2048):
            self.table[a] = (a * 7 + sid * 13) & 0xFF
        self.table[96:100] = struct.pack('<i', 1000 * sid - 2500)
        self.table[1200:1206] = b'\xff\xff\xfd\xff\xff\xfd'          # forces byte stuffing
        self.reboots = self.loaders = 0
        self._read, self._write, self._flash = READ(self.read), WRITE(self.write), FLASH(lambda c, a, n, o: 7)
        self.io = Io(sid, 231, 1200, None, self._read, self._write, self._flash, 0)

    def read(self, ctx, a, n, out):
        if a + n > 1536:
            return 4                                                     # like app_read: unknown address
        for i in range(n):
            out[i] = self.table[a + i]
        return 0

    def write(self, ctx, a, n, data):
        if a + n > 1536:
            return 4
        self.table[a:a + n] = bytes(data[i] for i in range(n))
        return 0


class Emulator(threading.Thread):
    """Reads host frames from the pty master; every virtual servo runs dxl_process on each."""

    def __init__(self, master, servos):
        super().__init__(daemon=True)
        self.master, self.servos, self.stop, self.frames = master, servos, False, 0

    def run(self):
        buf = bytearray()
        while not self.stop:
            r, _, _ = select.select([self.master], [], [], 0.02)
            if not r:
                continue
            buf += os.read(self.master, 4096)
            while True:
                i = buf.find(b'\xff\xff\xfd\x00')
                if i < 0 or len(buf) < i + 7:
                    break
                total = 7 + int.from_bytes(buf[i + 5:i + 7], 'little')
                if len(buf) < i + total:
                    break
                frame = bytes(buf[i:i + total]); del buf[:i + total]
                self.frames += 1
                replies, fast = [], []
                for s in self.servos:
                    wire = (C.c_uint8 * 256)(*frame) if len(frame) <= 256 else None
                    if wire is None:
                        continue
                    out = (C.c_uint8 * 640)()
                    res = lib.dxl_process(C.byref(s.io), wire, len(frame), out, 640)
                    if res.reboot: s.reboots += 1
                    if res.enter_loader: s.loaders += 1
                    if res.reply_bytes and res.fast:
                        fast.append((res.response_slot, res, out))
                    elif res.reply_bytes:
                        replies.append((res.response_slot if res.group_reply else 0, bytes(out[:res.reply_bytes])))
                replies.sort(key=lambda x: x[0])
                if fast:
                    replies.append((0, fast_packet(fast)))
                if replies:
                    os.write(self.master, b''.join(r for _, r in replies))


failures = []


def check(ok, what):
    print(('PASS ' if ok else 'FAIL ') + what)
    if not ok:
        failures.append(what)


def main():
    master, slave = os.openpty()
    servos = [Servo(i) for i in range(1, 7)]
    servos[4].io.alert = 0x80
    emu = Emulator(master, servos); emu.start()
    port = sdk.PortHandler(os.ttyname(slave))
    ph = sdk.PacketHandler(2.0)
    port.baudrate = 57600                     # a pty rejects the custom-speed ioctl; the rate is irrelevant here
    check(port.openPort(), 'SDK opens the pseudo-terminal')
    try:
        ok = all(ph.ping(port, i)[0:2] == (1200, sdk.COMM_SUCCESS) for i in range(1, 7))
        check(ok, 'ping IDs 1..6 -> model 1200')
        found, res = ph.broadcastPing(port)
        check(res == sdk.COMM_SUCCESS and found == {i: [1200, 231] for i in range(1, 7)}, 'broadcastPing finds 1..6 (model 1200, firmware 231)')
        v, res, err = ph.read4ByteTxRx(port, 3, 96)
        check(res == sdk.COMM_SUCCESS and err == 0 and v == (500 & 0xFFFFFFFF), 'read4ByteTxRx')
        res, err = ph.write4ByteTxRx(port, 2, 256, 123456)
        check(res == sdk.COMM_SUCCESS and err == 0 and servos[1].table[256:260] == struct.pack('<I', 123456), 'write4ByteTxRx')
        res, err = ph.write1ByteTxRx(port, 6, 65, 1)
        check(res == sdk.COMM_SUCCESS and servos[5].table[65] == 1, 'write1ByteTxRx (LED address 65)')
        v, res, err = ph.read2ByteTxRx(port, 1, 0)
        check(res == sdk.COMM_SUCCESS and v == servos[0].table[0] | servos[0].table[1] << 8, 'read2ByteTxRx')

        g = sdk.GroupSyncRead(port, ph, 64, 68)
        for i in range(1, 7): g.addParam(i)
        res = g.txRxPacket()
        ok = res == sdk.COMM_SUCCESS and all(g.isAvailable(i, 96, 4) and g.getData(i, 96, 4) == (1000 * i - 2500) & 0xFFFFFFFF for i in range(1, 7))
        check(ok, 'GroupSyncRead 6 x 68 B (status block) incl. position word')
        g = sdk.GroupSyncRead(port, ph, 1200, 6)
        for i in range(1, 7): g.addParam(i)
        res = g.txRxPacket()
        check(res == sdk.COMM_SUCCESS and all(g.getData(i, 1200, 4) == 0xFFFDFFFF for i in range(1, 7)), 'GroupSyncRead with FF FF FD data (byte stuffing)')

        w = sdk.GroupSyncWrite(port, ph, 256, 16)
        for i in range(1, 7): w.addParam(i, list(struct.pack('<iIII', -100 * i, 1000, 0, 0)))
        res = w.txPacket(); time.sleep(0.05)
        check(res == sdk.COMM_SUCCESS and all(struct.unpack_from('<i', servos[i - 1].table, 256)[0] == -100 * i for i in range(1, 7)), 'GroupSyncWrite targets 6 x 16 B')

        b = sdk.GroupBulkRead(port, ph)
        layout = {1: (96, 4), 2: (64, 68), 3: (208, 16), 4: (0, 64), 5: (844, 4), 6: (1000, 64)}
        for i, (a, n) in layout.items(): b.addParam(i, a, n)
        res = b.txRxPacket()
        check(res == sdk.COMM_SUCCESS and all(b.isAvailable(i, a, min(n, 4)) for i, (a, n) in layout.items())
              and b.getData(1, 96, 4) == (1000 - 2500) & 0xFFFFFFFF, 'GroupBulkRead with different address/size per servo')
        bw = sdk.GroupBulkWrite(port, ph)
        for i in range(1, 7): bw.addParam(i, 328, 8, list(struct.pack('<ii', 1000 * i, 10 * i)))
        res = bw.txPacket(); time.sleep(0.05)
        check(res == sdk.COMM_SUCCESS and all(struct.unpack_from('<ii', servos[i - 1].table, 328) == (1000 * i, 10 * i) for i in range(1, 7)), 'GroupBulkWrite gains')

        res, err = ph.reboot(port, 4)
        check(res == sdk.COMM_SUCCESS and err == 0 and servos[3].reboots == 1 and servos[3].loaders == 0, 'reboot -> reset into the application (not the loader)')
        # (read4ByteTxRx itself indexes the empty data of any error status and raises: an SDK quirk.)
        data, res, err = ph.readTxRx(port, 2, 3000, 4)
        check(res == sdk.COMM_SUCCESS and err == 4, f'status error reported (data range error 4; SDK result {res}, error {err})')
        model, res, err = ph.ping(port, 5)
        check(res == sdk.COMM_SUCCESS and err & 0x80, 'alert bit (fault latched) reported in the error byte')
        g = sdk.GroupSyncRead(port, ph, 96, 4)
        for i in range(1, 7): g.addParam(i)
        res = g.fastSyncRead()
        check(res == sdk.COMM_SUCCESS and all(g.isAvailable(i, 96, 4) and g.getData(i, 96, 4) == (1000 * i - 2500) & 0xFFFFFFFF for i in range(1, 7)),
              f'GroupSyncRead.fastSyncRead() 6 x 4 B position (0x8A, one combined packet; SDK result {res})')
        g = sdk.GroupSyncRead(port, ph, 64, 68)
        for i in (6, 2, 4, 1, 5, 3): g.addParam(i)
        res = g.fastSyncRead()
        check(res == sdk.COMM_SUCCESS and all(g.getData(i, 96, 4) == (1000 * i - 2500) & 0xFFFFFFFF for i in range(1, 7)),
              'fastSyncRead 6 x 68 B status block, rows in any ID order')
        g = sdk.GroupSyncRead(port, ph, 1200, 6)
        for i in range(1, 7): g.addParam(i)
        res = g.fastSyncRead()
        check(res == sdk.COMM_SUCCESS and all(g.getData(i, 1200, 4) == 0xFFFDFFFF for i in range(1, 7)), 'fastSyncRead with FF FF FD data (unstuffed, as the SDK expects)')
        g = sdk.GroupSyncRead(port, ph, 96, 4)
        for i in (1, 2, 9, 4): g.addParam(i)
        res = g.fastSyncRead()
        check(res != sdk.COMM_SUCCESS, f'fastSyncRead with an absent ID 9: rows after it stay silent, the SDK reports {ph.getTxRxResult(res)}')
        g = sdk.GroupSyncRead(port, ph, 96, 4)
        for i in range(1, 7): g.addParam(i)
        res = g.txRxPacket()
        check(res == sdk.COMM_SUCCESS, 'ordinary GroupSyncRead still works after the failed fast read')
    finally:
        port.closePort(); emu.stop = True; emu.join(1)
        os.close(slave); os.close(master)
    print(f'SDK compatibility (dynamixel_sdk Protocol 2.0 vs firmware protocol.c): {len(failures)} failure(s), {emu.frames} host frames')
    sys.exit(1 if failures else 0)


if __name__ == '__main__':
    main()
