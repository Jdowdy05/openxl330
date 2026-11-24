#include "bus.h"
/* FW231 device block (layout in bus.h), linked at exactly 0x080040C0 right after the
 * vector table (controller.ld asserts it). Release default: magic "XLDC", layout 1,
 * ID 0xFF = use the stock NV journal ID, empty label, CRC16 0x7EB7. The flasher patches
 * byte 5 (ID) and bytes 8..27 (label) per servo and rewrites the CRC at 28..29.
 * Firmware reads it through a volatile pointer, never through this symbol. */
__attribute__((used,section(".devcfg"),aligned(4)))
const uint8_t bus_devcfg_block[BUS_DEVCFG_BYTES] = {
    0x58,0x4c,0x44,0x43, 0x01,0xff,0xff,0xff,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    0xb7,0x7e,0xff,0xff
};
