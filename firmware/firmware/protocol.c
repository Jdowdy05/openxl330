#include "protocol.h"

/* CRC-16 (poly 0x8005, init 0, MSB first) with a 16-entry nibble table: two lookups per byte instead
 * of eight shift steps (a review performance note; 32 B of table). Same values as the bit loop. */
static const uint16_t crc_nibble[16] = {0x0000u, 0x8005u, 0x800fu, 0x000au, 0x801bu, 0x001eu, 0x0014u, 0x8011u, 0x8033u, 0x0036u, 0x003cu, 0x8039u, 0x0028u, 0x802du, 0x8027u, 0x0022u};
uint16_t dxl_crc_update(uint16_t crc, uint8_t byte) {
    crc = (uint16_t)((uint16_t)(crc << 4) ^ crc_nibble[(crc >> 12) ^ (byte >> 4)]);
    return (uint16_t)((uint16_t)(crc << 4) ^ crc_nibble[(crc >> 12) ^ (byte & 15u)]);
}

uint16_t dxl_crc(const uint8_t *data, uint16_t size) {
    uint16_t crc = 0;
    if (!data) return 0;
    for (uint16_t i = 0; i < size; i++) crc = dxl_crc_update(crc, data[i]);
    return crc;
}

static uint16_t little16(const uint8_t *p) {
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static uint32_t little32(const uint8_t *p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

/* One pass: header, 55, ERR, data with Protocol2 byte stuffing (FF FF FD -> FF FF FD FD over the body
 * 55 ERR data), LEN, CRC. 0 if the exact stuffed packet does not fit in cap (FW231: one pass for flash;
 * same output and the same exact-fit rule as the former two-pass version). */
static uint16_t status(uint8_t id, uint8_t alert, uint8_t error,
                        const uint8_t *data, uint16_t n,
                        uint8_t *out, uint16_t cap) {
    if (!out || n > DXL_MAX_READ || cap < 11u) return 0;
    out[0] = 0xff; out[1] = 0xff; out[2] = 0xfd; out[3] = 0; out[4] = id;
    out[7] = 0x55; out[8] = (uint8_t)(error | alert);
    uint16_t at = 9;
    for (uint16_t i = 0; i < n; i++) {
        if (at >= cap) return 0;
        out[at++] = data[i];
        if (out[at - 3] == 0xff && out[at - 2] == 0xff && out[at - 1] == 0xfd) {
            if (at >= cap) return 0;
            out[at++] = 0xfd;
        }
    }
    if ((uint32_t)at + 2u > cap) return 0;
    const uint16_t length = (uint16_t)(at - 5u);
    out[5] = (uint8_t)length; out[6] = (uint8_t)(length >> 8);
    const uint16_t crc = dxl_crc(out, at);
    out[at++] = (uint8_t)crc; out[at++] = (uint8_t)(crc >> 8);
    return at;
}

typedef struct {
    uint16_t address;
    uint16_t size;
    uint16_t data_offset;
    uint8_t slot;
    bool found;
    bool duplicate;
    uint8_t prev_id;
    uint32_t prior_bytes;
} owned_row;

uint32_t dxl_worst_status_bytes(uint32_t n) {
    return 11u + n + n / 3u;
}

/* Rows before ours: remember the previous row's ID and the worst-case bytes of
 * every earlier row's status packet (FW231 chained group replies). */
static void account_row(owned_row *row, uint8_t id, uint16_t size, uint8_t *last) {
    if (row->found) return;
    row->prior_bytes += dxl_worst_status_bytes(size);
    *last = id;
}

static void select_row(owned_row *row, uint8_t id, uint8_t ours,
                        uint16_t address, uint16_t size, uint16_t data,
                        uint16_t slot) {
    if (id != ours) return;
    if (row->found) {
        row->duplicate = true;
        return;
    }
    row->found = true;
    row->address = address;
    row->size = size;
    row->data_offset = data;
    row->slot = (uint8_t)slot;
}

/* Selection never invokes callbacks, including when later rows are invalid. Sync rows: ID [data]
 * after one address/size; Bulk rows: ID address size [data]. (FW231: one walker for both, flash.) */
static bool select_rows(const uint8_t *body, uint16_t used, bool bulk, bool writing,
                         uint8_t ours, owned_row *row) {
    if (used < 6) return false;
    uint16_t address = little16(body + 1), size = little16(body + 3);
    const uint32_t head = bulk ? 5u : 1u;
    if (!bulk && ((writing && !size) || (!writing && used - 5u > 256u))) return false;
    uint16_t at = bulk ? 1 : 5;
    uint16_t slot = 0;
    uint8_t last = 0xff;
    while (at < used) {
        if (bulk) {
            if (used - at < 5) return false;
            address = little16(body + at + 1);
            size = little16(body + at + 3);
            if (writing && !size) return false;
        }
        const uint32_t width = writing ? (uint32_t)size + head : head;
        if (width > (uint32_t)(used - at)) return false;
        if (!row->found && body[at] == ours) row->prev_id = last;
        select_row(row, body[at], ours, address, size,
                   (uint16_t)(at + head), slot);
        account_row(row, body[at], size, &last);
        at = (uint16_t)(at + width);
        slot++;
    }
    return true;
}

static uint8_t range_error(uint16_t address, uint16_t size, bool reading) {
    if (!size || (reading && size > DXL_MAX_READ)) return 5;
    if ((uint32_t)address + size > 65536u) return 4;
    return 0;
}

/* One foreground protocol owner; never touched by timer/UART ISR. */
static _Alignas(8) uint8_t protocol_read_scratch[DXL_MAX_READ];
void *dxl_foreground_scratch(void){return protocol_read_scratch;}
static dxl_result finish_range(const dxl_io *io, uint8_t id, uint8_t alert,
                                dxl_result result, const uint8_t *body,
                                const owned_row *row, bool writing,
                                bool broadcast, uint8_t error,
                                uint8_t *out, uint16_t cap) {
    if (!error) error = range_error(row->address, row->size, !writing);
    const bool reply_needed = !writing || !broadcast;
    /* Every ACK/error status is exactly 11 bytes, independent of error code. */
    if (reply_needed && (!out || cap < 11)) return result;
    if (error) {
        if (reply_needed) result.reply_bytes = status(id, alert, error, 0, 0, out, cap);
        return result;
    }
    if (writing) {
        error = io->write ? io->write(io->context, row->address, row->size,
                                      body + row->data_offset) : 7;
        if (reply_needed) result.reply_bytes = status(id, alert, error, 0, 0, out, cap);
    } else {
        error = io->read ? io->read(io->context, row->address, row->size, protocol_read_scratch) : 7;
        result.reply_bytes = status(id, alert, error, error ? 0 : protocol_read_scratch,
                                    error ? 0 : row->size, out, cap);
    }
    return result;
}

/* FW231 Fast Sync Read (0x8A, owner 2026-11-20 "Yes, with fast start"; the Fast Sync Read
 * design, 2026-11-20). One combined status packet for all rows, unstuffed:
 * FF FF FD 00 FE LEN_L LEN_H 55, then per row ERR ID D1..DX CRC_L CRC_H, LEN = 1 + n*(X+4). Each
 * row's CRC covers every packet byte before it (earlier CRC bytes included), so the last one is the
 * packet CRC. Row 0 builds header + segment + CRC; row k>0 builds ERR ID data and leaves the CRC to
 * be sealed when the bytes before it have been heard and verified (bus_fast_seal). A row that cannot
 * form a valid segment (duplicate ID, bad size) stays silent: the packet is broken anyway. */
static dxl_result finish_fast(const dxl_io *io, uint8_t id, uint8_t alert, dxl_result result,
                              const owned_row *row, uint16_t rows, uint8_t *out, uint16_t cap) {
    const uint16_t x = row->size;
    const uint32_t seg = (uint32_t)x + 4u, len = 1u + (uint32_t)rows * seg;
    const uint32_t head = row->slot ? 0u : 8u;
    if (row->duplicate || !x || x > DXL_MAX_READ || len > 0xffffu || !out || head + seg > cap)
        return result;                                   /* silent */
    uint8_t error = range_error(row->address, x, true);
    if (!error) error = io->read ? io->read(io->context, row->address, x, protocol_read_scratch) : 7;
    uint16_t at = 0;
    if (!row->slot) {
        out[0] = 0xff; out[1] = 0xff; out[2] = 0xfd; out[3] = 0; out[4] = 0xfe;
        out[5] = (uint8_t)len; out[6] = (uint8_t)(len >> 8); out[7] = 0x55; at = 8;
    }
    out[at++] = (uint8_t)(error | alert);
    out[at++] = id;
    for (uint16_t i = 0; i < x; i++) out[at++] = error ? 0u : protocol_read_scratch[i];
    uint16_t crc = row->slot ? 0u : dxl_crc(out, at);    /* slot>0: sealed later from the heard prefix */
    out[at++] = (uint8_t)crc; out[at++] = (uint8_t)(crc >> 8);
    result.reply_bytes = at;
    result.fast = true;
    result.fast_len = (uint16_t)len;
    result.fast_seg = (uint16_t)seg;
    result.prior_bytes = 8u + (uint32_t)row->slot * seg;
    return result;
}

dxl_result dxl_process(const dxl_io *io, uint8_t *wire, uint16_t n,
                       uint8_t *out, uint16_t cap) {
    dxl_result result = {0, false, false, false, 0, false, 0xff, 0, false, 0, 0};
    if (!io || !wire || (!out && cap) || io->id > 252 || n < 10 ||
        n > DXL_MAX_WIRE) return result;
    if (wire[0] != 0xff || wire[1] != 0xff || wire[2] != 0xfd ||
        wire[3] != 0) return result;
    const uint16_t declared = little16(wire + 5);
    if (declared < 3 || (uint32_t)declared + 7u != n) return result;
    if (dxl_crc(wire, (uint16_t)(n - 2)) != little16(wire + n - 2)) return result;
    const uint8_t id = io->id;
    const uint8_t alert = io->alert & 0x80u;
    const bool broadcast = wire[4] == 0xfe;
    if (wire[4] != id && !broadcast) return result;
    uint8_t *body=wire+7; /* CRC checked before leftward mutation. */
    uint16_t used = 0;
    for (uint16_t at = 7; at < n - 2; at++) {
        /* Wire size has already bounded the maximum possible decoded body. */
        body[used++] = wire[at];
        if (used >= 3 && body[used - 3] == 0xff && body[used - 2] == 0xff &&
            body[used - 1] == 0xfd) {
            at++;
            if (at >= n - 2 || wire[at] != 0xfd) return result;
        }
    }
    const uint8_t instruction = body[0];
    const bool group_read = instruction == 0x82 || instruction == 0x92 || instruction == 0x8a;
    const bool group_write = instruction == 0x83 || instruction == 0x93;
    if (group_read || group_write) {
        owned_row row = {0,0,0,0,false,false,0xff,0};
        const bool is_sync = instruction == 0x82 || instruction == 0x83 || instruction == 0x8a;
        const bool complete = select_rows(body, used, !is_sync, group_write, id, &row);
        if (!complete) {
            if (!broadcast) {
                result.valid_for_us = true;
                result.reply_bytes = status(id, alert, 5, 0, 0, out, cap);
            }
            return result;
        }
        if (!row.found) return result;
        /* fast10 (review F-3): our ID twice in a Sync/Bulk Read stays silent, like 0x8A. An error-5 reply here was the
         * chain predecessor of BOTH later rows (e.g. [1,3,1,2]: 3 and 2 followed 1's status and collided). */
        if (row.duplicate && (instruction == 0x82 || instruction == 0x92)) return result;
        result.valid_for_us = true;
        result.group_reply = group_read;
        result.response_slot = row.slot;
        result.prev_id = row.prev_id;
        result.prior_bytes = row.prior_bytes;
        if (instruction == 0x8a)
            return finish_fast(io, id, alert, result, &row, (uint16_t)(used - 5u), out, cap);
        return finish_range(io, id, alert, result, body, &row, group_write,
                             broadcast, row.duplicate ? 5 : 0, out, cap);
    }
    /* Protocol2 only permits broadcast Ping, ordinary Write, and group work. */
    if (broadcast && instruction != 1 && instruction != 3) return result;
    result.valid_for_us = true;
    switch (instruction) {
    case 1: {
        if (used != 1) {
            if (broadcast) return (dxl_result){0,false,false,false,0,false,0xff,0,false,0,0};
            result.reply_bytes = status(id, alert, 5, 0, 0, out, cap);
            return result;
        }
        const uint8_t identity[3] = {(uint8_t)io->model,
                                     (uint8_t)(io->model >> 8), io->firmware};
        result.group_reply = broadcast;
        result.response_slot = broadcast ? id : 0;
        result.reply_bytes = status(id, alert, 0, identity, 3, out, cap);
        return result;
    }
    case 2:
    case 3: {
        const bool writing = instruction == 3;
        if ((!writing && used != 5) || (writing && used < 4)) {
            if (!broadcast) result.reply_bytes = status(id, alert, 5, 0, 0, out, cap);
            return result;
        }
        const owned_row row = {little16(body + 1),
                               writing ? (uint16_t)(used - 3) : little16(body + 3),
                               3,0,true,false,0xff,0};
        return finish_range(io, id, alert, result, body, &row, writing,
                             broadcast, 0, out, cap);
    }
    case 0xa0: {
        uint8_t error = 0;
        if (used != 7) error = 5;
        uint32_t address = 0;
        uint16_t size = 0;
        if (!error) {
            address = little32(body + 1);
            size = little16(body + 5);
            if (!size || size > 96) error = 5;
            else if (address < 0x08000000u || address >= 0x08010000u ||
                     size > 0x08010000u - address) error = 4;
            else if (!io->flash_read) error = 7;
        }
        if (error) {
            result.reply_bytes = status(id, alert, error, 0, 0, out, cap);
            return result;
        }
        /* Success starts 55 00/80, so at most floor(size/3) stuffed bytes. */
        if (!out || (uint32_t)size + size / 3u + 11u > cap) return result;
        uint8_t *data = protocol_read_scratch;
        error = io->flash_read(io->context, address, size, data);
        result.reply_bytes = status(id, alert, error, error ? 0 : data,
                                    error ? 0 : size, out, cap);
        return result;
    }
    case 8:
    case 0xf9:
        /* FW231: F9 enters the retained loader; 0x08 Reboot (Dynamixel SDK
         * reboot()) resets back into this application, like stock. */
        result.reply_bytes = status(id, alert, used == 1 ? 0 : 5, 0, 0, out, cap);
        if (instruction == 0xf9) result.enter_loader = used == 1 && result.reply_bytes != 0;
        else result.reboot = used == 1 && result.reply_bytes != 0;
        return result;
    default:
        result.reply_bytes = status(id, alert, 2, 0, 0, out, cap);
        return result;
    }
}
