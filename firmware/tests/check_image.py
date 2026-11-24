"""FW231 linked-image check (run by build.sh): program fits 0x08004000..0x0800DFFF, vectors
point into the image, and the release device block at 0x080040C0 is the default
(magic XLDC, layout 1, ID 0xFF = NV journal ID, empty label, valid CRC)."""
import struct, sys
def crc16(data):
    v = 0
    for b in data:
        v ^= b << 8
        for _ in range(8): v = ((v << 1) ^ (0x8005 if v & 0x8000 else 0)) & 0xFFFF
    return v
raw = open(sys.argv[1], 'rb').read()
# fast17: with the ID log (build.sh IDLOG=1, argument --idlog) the image must end below its page, 0x0800D800 (38912 B;
# the same 8 B margin below it as below the NVM).
end = 0x9800 if '--idlog' in sys.argv[2:] else 40960
assert 192 + 32 <= len(raw) <= end - 8, f'image size {len(raw)} outside the program region' + (' (below the ID log page)' if end < 40960 else '')
stack, reset = struct.unpack_from('<II', raw)
assert 0x20000000 < stack <= 0x20002000 and reset & 1 and 0x08004000 <= (reset & ~1) < 0x08004000 + len(raw), 'vectors'
block = raw[0xC0:0xE0]
assert block[:8] == b'XLDC\x01\xff\xff\xff' and block[8:28] == bytes(20) and block[30:] == b'\xff\xff', block.hex()
assert struct.unpack_from('<H', block, 28)[0] == crc16(block[:28]) == 0x7EB7, 'device block CRC'
print(f'image check: {len(raw)} B ({end - len(raw)} B below 0x{0x08004000 + end:08X}), default device block at 0x080040C0 PASS')
