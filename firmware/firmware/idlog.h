#ifndef IDLOG_H
#define IDLOG_H
/* fast17 (v1.1 Stage 2, owner 2026-11-24): the servo ID set over the bus (write 1144), kept in an append-only log in the
 * last program page, 0x0800D800..0x0800DFFF (never the loader, never the servo's own NVM at 0x0800E000..). Images end
 * below it (tests/check_image.py). 8-byte records, one STM32G0 double word each:
 *   slot 0     header  'X' 'L' 'I' 'D', u16 format 1, u16 CRC-16 (dxl_crc) of the 6 bytes before
 *   slot 1..   record  u8 type 1, u8 id (0..252, 0xFF = no override), u8 id^0xFF, u8 0, u16 seq, u16 CRC of the 6 before
 * The last record with a valid CRC before the first erased slot wins; a torn record fails its CRC (or raised an ECC NMI,
 * which platform.c turns into a flag for the log page) and is skipped. A page without a valid header, a full page, or one
 * that showed an ECC error is erased and re-formatted by the next write, which then holds only the new record; so does a
 * page where a program or its read-back failed (review F2). A power cut between that erase and the new record loses the
 * logged ID: the servo then answers at its device-block / NV journal ID (review F3; on a bus of factory servos that is
 * ID 1 for all of them, so re-apply the ID with that servo alone). */
#include <stdbool.h>
#include <stdint.h>
enum { IDLOG_ADDRESS = 0x0800D800u, IDLOG_BYTES = 2048u, IDLOG_SLOTS = IDLOG_BYTES / 8u, IDLOG_NONE = 0xFFu,
       IDLOG_RECORDS = IDLOG_SLOTS - 1u };
/* The flash access is platform.c's (the log page only): platform_flash_idlog(offset, record) programs one double word,
 * record NULL erases the page; platform_idlog_ecc_take() reports (once) an ECC error the NMI saw in the page;
 * platform_idlog_page() is the mapped page. Native tests provide their own. */
bool platform_flash_idlog(uint32_t offset, const uint8_t record[8]);
bool platform_idlog_ecc_take(void);
const volatile uint8_t *platform_idlog_page(void);
typedef struct {
    uint16_t next;       /* first free slot (IDLOG_SLOTS = full); 0 when not formatted */
    uint16_t seq;        /* sequence number of the newest valid record */
    uint8_t id;          /* the newest valid record's ID, IDLOG_NONE without one */
    bool formatted, ecc_seen;
} idlog;
/* Scan the page. */
void idlog_scan(idlog *l);
/* Append a record for `id` (0..252 or IDLOG_NONE), erasing/formatting first when needed. 0 = written and read back,
 * 7 = a flash operation failed (the log is then re-scanned). */
uint8_t idlog_write(idlog *l, uint8_t id);
#endif
