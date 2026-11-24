#ifndef XL330_OWN_PROTOCOL_H
#define XL330_OWN_PROTOCOL_H

#include <stdbool.h>
#include <stdint.h>

enum {
    DXL_MAX_WIRE = 256,
    DXL_MAX_READ = 384,
    DXL_REPLY_CAP = 640
};

typedef struct {
    uint8_t id;
    uint8_t firmware;
    uint16_t model;
    void *context;
    uint8_t (*read)(void *context, uint16_t address, uint16_t size,
                    uint8_t *out);
    uint8_t (*write)(void *context, uint16_t address, uint16_t size,
                     const uint8_t *data);
    uint8_t (*flash_read)(void *context, uint32_t address, uint16_t size,
                          uint8_t *out);
    uint8_t alert; /* Only bit 7 is emitted, ORed into each status error. */
} dxl_io;

typedef struct {
    uint16_t reply_bytes;
    bool valid_for_us;
    bool enter_loader;
    bool group_reply;
    uint8_t response_slot;
    bool reboot;          /* FW231: 0x08 Reboot -> reset back into the application */
    uint8_t prev_id;      /* FW231: ID of the previous Sync/Bulk Read row (valid when response_slot>0) */
    uint32_t prior_bytes; /* FW231: worst-case wire bytes of all earlier rows' status packets
                           * (Fast Sync Read: exact bytes before our segment, 8 + slot*(X+4)) */
    bool fast;            /* FW231: Fast Sync Read (0x8A) segment; the reply buffer holds
                           * header+segment (slot 0) or ERR ID data + 2 CRC bytes to seal (slot>0) */
    uint16_t fast_len;    /* LEN field of the combined packet: 1 + n*(X+4) */
    uint16_t fast_seg;    /* X + 4 */
} dxl_result;

/* Worst-case Protocol2 status packet bytes for n data bytes: 11 + n + one
 * stuffing byte per three data bytes (55/error never start an FF FF FD run). */
uint32_t dxl_worst_status_bytes(uint32_t n);

/* CRC-16, polynomial 0x8005, initial value zero, no reflection/xor-out. */
uint16_t dxl_crc(const uint8_t *data, uint16_t size);
uint16_t dxl_crc_update(uint16_t crc, uint8_t byte);   /* one byte; continue a CRC over a stream */

/* Single foreground caller; not reentrant (fixed read scratch).
 * Process exactly one complete Protocol2 wire frame. No allocation, I/O,
 * persistent writes or hidden state. io->id must be 0..252. Buffers must not
 * overlap; out may be NULL only when cap is zero. Invalid framing is silent.
 *
 * Callbacks operate on the complete owned range once, after full packet/row
 * validation. A write callback MUST validate its entire range before changing
 * any state and either apply it atomically or return an error without changes;
 * the parser cannot roll back a callback. Callback errors are Protocol2 status
 * errors (zero means success). Missing callbacks return access error 7.
 *
 * Output-capacity failure cannot call a write callback or request the loader
 * when that operation requires an ACK. Broadcast writes need no output buffer.
 * A RAM read callback may run before insufficient reply capacity is known.
 * A0 flash reads preflight the worst-case stuffed reply before their callback;
 * they accept only 0x08000000..0x0800ffff and sizes 1..96, without writes.
 *
 * valid_for_us means a framed command addressed this device, or a complete
 * group command included its ID; it can also accompany a semantic error reply.
 * group_reply/response_slot specify the scheduling position for broadcast Ping
 * (slot = ID) and Sync/Bulk Read (slot = zero-based request row). At most 256
 * Sync Read IDs can be represented. Caller schedules/transmits the reply and
 * only enters the loader (F9) or resets (0x08 Reboot) after the ACK has
 * completely left the UART. For group reads prev_id/prior_bytes describe the
 * rows before ours so the caller can chain its reply after the predecessor.
 */
/* Foreground-only/nonreentrant: callbacks finish synchronously and must not
 * recurse into dxl_process. Internal read/A0 scratch is shared; encoded reply
 * is caller-owned after return and must remain immutable through transmit. */
dxl_result dxl_process(const dxl_io *io, uint8_t *wire, uint16_t n,
                       uint8_t *out, uint16_t cap);

/* Single foreground owner; read callback and write-plan lifetimes disjoint. */
void *dxl_foreground_scratch(void);
#endif
