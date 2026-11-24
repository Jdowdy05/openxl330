#include "idlog.h"
#include "protocol.h"

static const uint8_t header[6] = {'X', 'L', 'I', 'D', 1, 0};
static void seal(uint8_t r[8]) { uint16_t c = dxl_crc(r, 6); r[6] = (uint8_t)c; r[7] = (uint8_t)(c >> 8); }
/* one slot into r; true when that read raised an ECC error */
static bool get(unsigned slot, uint8_t r[8])
{
    const volatile uint8_t *p = platform_idlog_page() + 8u * slot;
    for (unsigned i = 0; i < 8u; i++) r[i] = p[i];
    return platform_idlog_ecc_take();
}
static bool sealed(const uint8_t r[8]) { return dxl_crc(r, 6) == (uint16_t)(r[6] | (uint16_t)r[7] << 8); }
static bool blank(const uint8_t r[8]) { unsigned a = 0xffu; for (unsigned i = 0; i < 8u; i++) a &= r[i]; return a == 0xffu; }

void idlog_scan(idlog *l)
{
    *l = (idlog){.id = IDLOG_NONE};
    uint8_t r[8];
    (void)platform_idlog_ecc_take();                                   /* discard a stale latch */
    if (get(0, r)) { l->ecc_seen = true; return; }
    for (unsigned i = 0; i < 6u; i++) if (r[i] != header[i]) return;
    if (!sealed(r)) return;
    l->formatted = true; l->next = IDLOG_SLOTS;
    for (unsigned s = 1; s < IDLOG_SLOTS; s++) {
        if (get(s, r)) { l->ecc_seen = true; continue; }               /* skipped; the next write re-formats */
        if (blank(r)) { l->next = (uint16_t)s; break; }
        if (r[0] == 1u && (uint8_t)(r[1] ^ r[2]) == 0xffu && !r[3] && (r[1] <= 252u || r[1] == IDLOG_NONE) && sealed(r)) {
            l->id = r[1]; l->seq = (uint16_t)(r[4] | (uint16_t)r[5] << 8);
        }
    }
}

uint8_t idlog_write(idlog *l, uint8_t id)
{
    uint8_t r[8], back[8];
    (void)platform_idlog_ecc_take();                                   /* review nit: a stale latch (A0 read) is not ours */
    if (!l->formatted || l->ecc_seen || l->next >= IDLOG_SLOTS) {
        for (unsigned i = 0; i < 6u; i++) r[i] = header[i];
        seal(r);
        if (!platform_flash_idlog(0, 0) || !platform_flash_idlog(0, r)) goto fail;
        l->formatted = true; l->ecc_seen = false; l->next = 1;
    }
    const uint16_t seq = (uint16_t)(l->seq + 1u), slot = l->next;
    r[0] = 1u; r[1] = id; r[2] = (uint8_t)(id ^ 0xffu); r[3] = 0; r[4] = (uint8_t)seq; r[5] = (uint8_t)(seq >> 8); seal(r);
    if (!platform_flash_idlog(8u * slot, r) || get(slot, back)) goto fail;
    for (unsigned i = 0; i < 8u; i++) if (back[i] != r[i]) goto fail;
    l->next = (uint16_t)(slot + 1u); l->id = id; l->seq = seq;
    return 0;
fail:
    idlog_scan(l);
    l->formatted = false;   /* review F2: a slot that reads blank but will not program (interrupted erase) -> erase next time */
    return 7;
}
