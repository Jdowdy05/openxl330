#ifndef XL330_NV_READ_H
#define XL330_NV_READ_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
/* Read-only snapshot of the servo's settings journal: flash 0x0800F000..0x0800FFFF, two 2 KB banks, each with a
 * 16-byte header (ABAB marker, generation) followed by 8-byte {key, value} records. No repair, erase/program
 * routines, MMIO, or implicit factory defaults. */
typedef struct {
    uint8_t bytes[64];
    uint64_t present;
    uint16_t generation;
    uint8_t bank;
    bool ready;
} xl330_nv_snapshot;
typedef struct {
    int32_t homing_offset_raw;
    uint16_t pwm_limit;
    uint16_t current_limit; /* Stock register counts, not engineering units. */
    uint8_t id;
    uint8_t baud_index;
    uint8_t drive_mode;
    uint8_t operating_mode;
} xl330_nv_config;
/* size>=4096, bytes map F000..10000. Exact ABAB 64-bit header required.
 * If both banks carry the ABAB header, the first bank (0x0800F000) is used.
 * Torn/malformed selected records fail closed; ready remains false. */
bool xl330_nv_open(xl330_nv_snapshot *snapshot,const uint8_t *nvm,size_t size);
/* Stock lookup: missing key returns zero; present bitmap distinguishes it. */
bool xl330_nv_get(const xl330_nv_snapshot *snapshot,uint8_t key,uint8_t *value);
/* Requires every returned field's bytes to exist and legal mode/drive/id/baud.
 * Returns raw EEPROM homing offset; resolve mode/drive before tracker_begin. */
bool xl330_nv_config_read(const xl330_nv_snapshot *snapshot,xl330_nv_config *config);
#endif
