/* fast17: ID log (firmware/idlog.c) against a simulated STM32G0 page: program only on erased double words (else the
 * write fails, like PROGERR), whole-page erase, torn programs, ECC errors on read, power loss between erase and rewrite. */
#include "idlog.h"
#include "protocol.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
static volatile uint8_t page[IDLOG_BYTES];
static unsigned erases, programs, fail_program_at = ~0u, torn_bytes, ecc_calls, ecc_slot = ~0u;
static bool fail_erase;
static void fill(uint8_t v) { for (unsigned i = 0; i < IDLOG_BYTES; i++) page[i] = v; }
static bool f_erase(void) { erases++; if (fail_erase) return false; fill(0xff); return true; }
static bool f_program(uint32_t off, const uint8_t r[8]) {
    assert(off % 8u == 0 && off < IDLOG_BYTES);
    for (unsigned i = 0; i < 8u; i++) if (page[off + i] != 0xffu) return false;      /* PROGERR: not erased */
    if (programs++ == fail_program_at) {                                              /* power cut mid-program */
        for (unsigned i = 0; i < torn_bytes; i++) page[off + i] = r[i];
        return false;
    }
    for (unsigned i = 0; i < 8u; i++) page[off + i] = r[i];
    return true;
}
/* idlog_scan calls ecc_take() once to discard a stale latch, then after each slot it reads: call k (k >= 1) follows slot k-1 */
bool platform_idlog_ecc_take(void) { unsigned k = ecc_calls++; return ecc_slot != ~0u && k >= 1u && k - 1u == ecc_slot; }
bool platform_flash_idlog(uint32_t offset, const uint8_t record[8]) { return record ? f_program(offset, record) : f_erase(); }
const volatile uint8_t *platform_idlog_page(void) { return page; }
static idlog scan(void) { idlog l; ecc_calls = 0; idlog_scan(&l); return l; }
static uint8_t wr(idlog *l, uint8_t id) { ecc_calls = 1000; return idlog_write(l, id); }

int main(void)
{
    fill(0xff);
    /* blank page and a page of old program bytes: not formatted, no ID; the first write formats it (one erase) */
    idlog l = scan(); assert(!l.formatted && l.id == IDLOG_NONE);
    fill(0x00); l = scan(); assert(!l.formatted && l.id == IDLOG_NONE);
    erases = 0; assert(wr(&l, 7) == 0 && erases == 1 && l.id == 7 && l.next == 2);
    l = scan(); assert(l.formatted && l.id == 7 && l.next == 2 && l.seq == 1);
    /* appends without erasing; the last valid record wins; 0xFF = no override */
    assert(wr(&l, 9) == 0 && wr(&l, IDLOG_NONE) == 0 && wr(&l, 252) == 0 && erases == 1);
    l = scan(); assert(l.id == 252 && l.seq == 4 && l.next == 5);
    assert(wr(&l, IDLOG_NONE) == 0); l = scan(); assert(l.id == IDLOG_NONE && l.seq == 5);
    /* torn program (power cut after 1..7 bytes): the partial record fails its CRC, the previous one wins */
    assert(wr(&l, 3) == 0);
    for (unsigned t = 1; t < 8; t++) {
        fail_program_at = programs; torn_bytes = t;
        assert(wr(&l, 4) == 7);
        fail_program_at = ~0u;
        l = scan(); assert(l.id == 3 && l.formatted);
    }
    assert(wr(&l, 5) == 0); l = scan(); assert(l.id == 5);       /* writing goes on after the torn slots */
    /* fill the page (255 records); the next write erases and keeps only the new one */
    while (l.next < IDLOG_SLOTS) assert(wr(&l, (uint8_t)(l.next % 200u)) == 0);
    l = scan(); assert(l.next == IDLOG_SLOTS);
    unsigned e0 = erases; assert(wr(&l, 42) == 0 && erases == e0 + 1 && l.next == 2);
    l = scan(); assert(l.id == 42 && l.next == 2);
    /* power loss between erase and the header: the page is blank -> no ID (devcfg/NV fallback); the next write formats */
    fail_program_at = programs; torn_bytes = 0; l.next = IDLOG_SLOTS;
    assert(wr(&l, 43) == 7); fail_program_at = ~0u;
    l = scan(); assert(!l.formatted && l.id == IDLOG_NONE);
    assert(wr(&l, 44) == 0); l = scan(); assert(l.id == 44);
    /* erase failure: 7, the log is re-scanned and keeps its last record */
    fail_erase = true; l.formatted = false; assert(wr(&l, 45) == 7); fail_erase = false; l = scan(); assert(l.id == 44);
    /* a corrupted header (bit flip) = not formatted */
    page[2] ^= 1u; l = scan(); assert(!l.formatted && l.id == IDLOG_NONE); page[2] ^= 1u;
    /* a record with ID 253 is ignored even with a matching CRC; so is one with a bad complement */
    l = scan(); assert(l.id == 44 && l.next == 2);
    { uint8_t r[8] = {1, 253, 253 ^ 0xff, 0, 9, 0, 0, 0}; uint16_t c = dxl_crc(r, 6); r[6] = (uint8_t)c; r[7] = (uint8_t)(c >> 8);
      for (unsigned i = 0; i < 8u; i++) page[16 + i] = r[i]; }
    { uint8_t r[8] = {1, 12, 0x00, 0, 9, 0, 0, 0}; uint16_t c = dxl_crc(r, 6); r[6] = (uint8_t)c; r[7] = (uint8_t)(c >> 8);
      for (unsigned i = 0; i < 8u; i++) page[24 + i] = r[i]; }
    l = scan(); assert(l.id == 44 && l.next == 4);
    /* ECC error while reading a record: that slot is skipped (the last good one wins), the next write re-formats */
    assert(wr(&l, 46) == 0 && wr(&l, 47) == 0); l = scan(); assert(l.id == 47 && l.next == 6);
    ecc_slot = 5; l = scan(); assert(l.ecc_seen && l.id == 46 && l.formatted);
    ecc_slot = ~0u; e0 = erases; assert(wr(&l, 48) == 0 && erases == e0 + 1); l = scan(); assert(l.id == 48 && l.next == 2 && !l.ecc_seen);
    /* ECC error on the header: not formatted, no ID */
    ecc_slot = 0; l = scan(); assert(l.ecc_seen && !l.formatted && l.id == IDLOG_NONE); ecc_slot = ~0u;
    puts("idlog: format on first write, append, last valid wins, torn records ignored, full page re-format, power loss between erase and rewrite, erase failure, header/record validation, ECC on record/header PASS");
    return 0;
}
