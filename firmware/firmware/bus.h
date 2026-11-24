#ifndef XL330_BUS_H
#define XL330_BUS_H
/* FW231 six-servo bus support (2026-11-20). Pure: no MMIO, no clock reads, no
 * allocation. The single foreground owner (main loop) passes hw_micros() times.
 *
 * 1. Chained group replies. Sync/Bulk Read row k (k>0) answers once the complete
 *    status packet of the row k-1 ID has been seen on the bus, plus BUS_CHAIN_GAP_US
 *    (the predecessor releases its driver enable in its UART IRQ, which its own
 *    masked control-tick sections can delay). If that predecessor never answers, a fallback deadline fires:
 *    the worst-case wire time of every earlier row's status packet plus
 *    BUS_SLOT_LATENCY_US per earlier row, measured from the end of the request,
 *    and only once the bus has been idle for bus_idle_us(). Row 0 and every
 *    unicast reply go out at once. Broadcast Ping keeps the stock ID * 3 ms slot.
 *    Any later instruction packet cancels a pending group reply.
 * 2. Baud: boot is always 1 Mbaud. A write of 844 (also broadcast) requests
 *    1M/2M/3M; the switch happens once the ACK (if any) has left. The first
 *    CRC-valid frame at the new baud confirms it, otherwise it reverts to the
 *    previous baud after BUS_BAUD_CONFIRM_US.
 * 3. Device block: 32 bytes at 0x080040C0 (right after the vector table) that the
 *    flasher patches per servo: magic "XLDC", layout 1, ID (0xFF = stock NV
 *    journal ID), start-up baud (0xFF or 3 = 1M, 4 = 2M, 5 = 3M; stock EEPROM 8
 *    codes), one reserved 0xFF, 20-byte label, CRC16 (Protocol2 CRC over bytes
 *    0..27), two reserved 0xFF.
 * 4. Start-up baud with a rescue window: the UART always comes up at 1 Mbaud. If
 *    the block asks for 2M/3M, the servo keeps listening at 1 Mbaud for
 *    BUS_RESCUE_US and then switches; any CRC-valid frame heard at 1 Mbaud in that
 *    window keeps 1 Mbaud for this power cycle. So a servo is never stranded: power
 *    it up while a host pings at 1 Mbaud (the loader is reachable only through the
 *    application). */
#include <stdbool.h>
#include <stdint.h>
#include "protocol.h"

enum {
    BUS_BOOT_BAUD = 1000000u,
    /* Per earlier row: the latest a present predecessor may START its reply. 600 us was too tight
     * on hardware (2026-11-20, servo ID 1 on older firmware, armed, at slot 0 + ID3: CRC collision); 3 ms = stock slot. */
    BUS_SLOT_LATENCY_US = 3000u,
    BUS_CHAIN_GAP_US = 40u,         /* min gap after the predecessor's last byte (its DE release). Chosen
                                     * 2026-11-20: >= 3x the smallest clean value of the hardware sweep (fsr3-04:
                                     * clean at 150..10 us, 1000 sync + 1000 fast reads per step, 1M and 3M) AND >=
                                     * the longest masked section + 10 us (fsr4: 28 us while armed, the control
                                     * tick's output phase delays the TC IRQ that releases DE). Was 150, then 30.
                                     * Floor: one byte time. RAM knob: write 1080. */
    BUS_PING_SLOT_US = 3000u,       /* stock broadcast-ping slot per ID */
    BUS_BAUD_CONFIRM_US = 2000000u,
    BUS_DEVCFG_BYTES = 32u,
    BUS_DEVCFG_LABEL = 20u,
    BUS_DEVCFG_MAGIC = 0x43444c58u, /* "XLDC" little-endian */
    BUS_DEVCFG_ADDRESS = 0x080040c0u,
    BUS_RESCUE_US = 1500000u
};

bool bus_baud_supported(uint32_t bps);
uint32_t bus_byte_ns(uint32_t bps);                 /* 10 bits per byte; 0 if unsupported */
uint32_t bus_wire_us(uint32_t bytes, uint32_t bps); /* rounded up */
uint32_t bus_idle_us(uint32_t bps);                 /* 3 byte times, at least 20 us */
uint8_t bus_baud_index(uint32_t bps);               /* stock EEPROM address 8 code: 3=1M 4=2M 5=3M */

enum { BUS_REPLY_NONE = 0, BUS_REPLY_DIRECT, BUS_REPLY_CHAIN, BUS_REPLY_SLOT, BUS_REPLY_FAST };
typedef struct {
    uint8_t kind, prev_id;
    bool triggered, deferred;
    uint32_t armed_us, fallback_us, trigger_us, gap_us;
    /* counters (read 1000) */
    uint32_t direct, chained, fallback, slotted, superseded, statuses, deferrals;
    /* latency maxima (read 1064): request end -> TX start for direct replies, predecessor status end ->
     * TX start for chained replies. (FW231 fast5+: the armed / slot-0 / slowest-request diagnostics of the
     * closed tail-latency hunt are gone for flash; read 1064 words 1, 6, 7 read 0.) */
    uint32_t lat_direct_max, lat_chain_max;
    /* request end -> predecessor status seen (max), and how often that was past our deadline */
    uint32_t pred_done_max, late_chains;
    uint32_t gap_min_us;           /* chain/segment gap floor; 0 = BUS_CHAIN_GAP_US (RAM test write 1080) */
    uint32_t fast_sent, fast_dropped;
} bus_reply;
/* The predecessor (status from prev_id, or verified Fast Sync Read prefix) is complete at `now`. */
void bus_reply_trigger(bus_reply *, uint32_t now);
/* Fast Sync Read follower past its deadline without a verified prefix: drop it (never a late segment).
 * Only once the foreground has caught up with the wire: no unread bytes, and nothing received for the
 * chain gap (a lagging foreground must check the prefix first; fsr-02 lost 71/200 rows to this). */
bool bus_reply_abandon(bus_reply *, uint32_t now, uint32_t last_rx_us, bool rx_pending);

/* Fast Sync Read follower: streaming check of the bytes heard before our segment. */
enum { BUS_FAST_IDLE = 0, BUS_FAST_CAPTURING, BUS_FAST_READY, BUS_FAST_FAILED };
typedef struct {
    uint32_t count, need;          /* bytes heard / bytes before our segment (8 + k*(X+4)) */
    uint16_t crc, seg, len, field; /* running CRC over every byte; segment size; LEN; CRC before a field */
    uint16_t off, segno, pred;     /* position in the current row, its index, the predecessor's row index:
                                    * counted, never divided (M0+ has no divider: 3 divisions per byte cost
                                    * ~4.7 us/byte on hardware, fsr-02 2026-11-20) */
    uint8_t prev_id, state;
} bus_fast;
void bus_fast_begin(bus_fast *, uint32_t need, uint16_t seg, uint16_t len, uint8_t prev_id);
uint8_t bus_fast_feed(bus_fast *, uint8_t byte);           /* returns the state */
/* Write our segment's CRC (continuing the verified prefix) into its last two bytes. */
void bus_fast_seal(const bus_fast *, uint8_t *seg, uint16_t seg_len);

/* FW231 fast start (owner 2026-11-20): the priority-0 USART IRQ runs this watcher over the RX ring when the
 * receiver timeout says the line has been idle for the chain gap. It mirrors what the foreground does
 * (CHAIN: a CRC-valid status from prev_id; FAST: the verified prefix) on its own cursor, so the reply can
 * start from the IRQ while the foreground lags. DEAD: a CRC-valid instruction went by (the foreground
 * supersedes) or the prefix failed. Pure; no allocation; any frame length (only counts and a CRC). */
enum { BUS_WATCH_CAPTURING = 0, BUS_WATCH_READY, BUS_WATCH_DEAD };
typedef struct {
    bus_fast fast;                 /* FAST: copy of the follower's verifier */
    uint32_t at;                   /* CHAIN: bytes of the current frame (frames back to back from the request) */
    uint16_t total, crc;
    uint8_t kind, prev_id, id, inst, low;
} bus_watch;
void bus_watch_begin(bus_watch *, uint8_t kind, uint8_t prev_id, const bus_fast *fast);
uint8_t bus_watch_feed(bus_watch *, uint8_t byte);
/* Same result as bus_watch_feed on ring[*at .. end) (index & mask), stopping at the first decision; *at = the
 * next unread index. Plain row/status-body bytes go through a tight CRC loop; the header, the predecessor's ID
 * byte and the CRC fields get the full per-byte checks (hardware fsr2-02: ~3.3 us/byte per call chain). */
uint8_t bus_watch_run(bus_watch *, const volatile uint8_t *ring, uint32_t mask, uint32_t *at, uint32_t end);
void bus_reply_reset_maxima(bus_reply *);
void bus_reply_init(bus_reply *);
/* Arm for the reply described by r to `instruction` (call only when
 * r->reply_bytes != 0). */
void bus_reply_arm(bus_reply *, const dxl_result *r, uint8_t instruction, uint32_t now, uint32_t bps);
/* True when the armed reply should be transmitted now. last_rx_us = time the
 * latest byte was taken from the RX ring; rx_pending = bytes are waiting in the ring
 * unread (the bus was active while the foreground was held). Pure decision: the path
 * is counted by bus_reply_sent once the reply actually goes out. */
bool bus_reply_due(bus_reply *, uint32_t now, uint32_t last_rx_us, bool rx_pending, uint32_t bps);
void bus_reply_done(bus_reply *); /* reply dropped */
/* Reply is being handed to the UART now: records its latency, then clears it. */
void bus_reply_sent(bus_reply *, uint32_t now);
bool bus_reply_pending(const bus_reply *);
bool bus_reply_watching(const bus_reply *); /* a chained reply waits for a status packet */
/* A CRC-valid status packet from `id` completed at `now`. */
void bus_observe_status(bus_reply *, uint8_t id, uint32_t now);
/* A CRC-valid instruction packet (any ID) completed: cancels a pending group
 * reply (returns true if one was cancelled). */
bool bus_observe_instruction(bus_reply *);

enum { BUS_BAUD_STABLE = 0, BUS_BAUD_PENDING, BUS_BAUD_UNCONFIRMED };
enum { BUS_BOOT_NONE = 0, BUS_BOOT_WAITING, BUS_BOOT_SWITCHED, BUS_BOOT_RESCUED };
typedef struct {
    uint32_t current, previous, target, since_us;
    uint8_t state, boot_state;
    uint32_t boot_target, boot_since_us;
    uint32_t switches, confirms, reverts;
} bus_baud;
void bus_baud_init(bus_baud *);
/* 0 accepted (or already at that baud), 4 unsupported value, 7 a switch is in progress. */
uint8_t bus_baud_request(bus_baud *, uint32_t bps);
bool bus_baud_switch_ready(const bus_baud *); /* PENDING: caller switches the UART to ->target */
void bus_baud_switched(bus_baud *, bool ok, uint32_t now);
void bus_baud_frame(bus_baud *);              /* a CRC-valid frame arrived: confirms */
bool bus_baud_revert_due(const bus_baud *, uint32_t now); /* caller switches the UART to ->previous */
void bus_baud_reverted(bus_baud *, bool ok, uint32_t now);
uint32_t bus_baud_word(const bus_baud *);     /* read 844: current bps | 1 while unconfirmed */
/* Start-up baud (item 4). boot_bps 1000000 (the default) arms nothing. */
void bus_baud_boot(bus_baud *, uint32_t boot_bps, uint32_t boot_us);
bool bus_baud_boot_due(const bus_baud *, uint32_t now);   /* caller switches the UART to ->boot_target */
void bus_baud_boot_done(bus_baud *, bool ok);
bool bus_baud_wants_frames(const bus_baud *);              /* unconfirmed switch or rescue window open */

/* 5. Command lease: an opt-in DEPLOYMENT OPTION, not a protection (owner, 2026-11-20: "Yes, opt-in
 *    only"). Off after every boot, so the normal behaviour has no comms cut-off. When the host sets
 *    it (write 1072), the servo must keep receiving accepted motion commands (256 target, 260 duty,
 *    64 ARM) or heartbeats (write 1076) addressed to it (unicast or its Sync/Bulk Write row; a plain
 *    broadcast write does not count, telemetry reads never count). If none arrives for lease_ms
 *    while armed, the configured response runs once: 0 = OFF, 1 = hold (back to PD at the current
 *    position). Recovery is an explicit re-arm (write 64 <- 1; after a hold the servo is still armed,
 *    so that write only re-arms the lease). While expired-in-hold, 256/260 are refused (7). */
enum { BUS_LEASE_OFF = 0, BUS_LEASE_RUNNING, BUS_LEASE_EXPIRED };
/* The motion commands for the lease: 256 goal, 260 duty and (fast16) 264/268 torque. An accepted one renews a running
 * lease; while an expired HOLD lease blocks motion (armed), they answer 7. */
static inline bool bus_lease_motion(uint16_t address) { return (address & ~12u) == 256u; }
enum { BUS_LEASE_ACT_OFF = 0, BUS_LEASE_ACT_HOLD = 1, BUS_LEASE_MAX_MS = 60000u, BUS_LEASE_MAGIC = 0x5341454cu /* "LEAS" */ };
typedef struct {
    uint32_t ms, action, last_us, expiries;
    uint8_t state;
} bus_lease;
/* write 1072, 12 B {lease_ms 0..60000 (0 = off), action 0/1, magic}; 0 ok, 4 bad value, 5 bad size */
uint8_t bus_lease_config(bus_lease *, const uint8_t *in, uint16_t n, uint32_t now);
void bus_lease_renew(bus_lease *, uint32_t now);   /* accepted motion command / heartbeat (no effect when expired) */
void bus_lease_rearm(bus_lease *, uint32_t now);   /* explicit ARM: expired -> running */
bool bus_lease_expire(bus_lease *, uint32_t now, bool armed);   /* true once, at the moment it expires */
bool bus_lease_blocks_motion(const bus_lease *);   /* expired with the hold response */
/* fast10 (review F-2): what an expired HOLD lease does to a write: 0 proceed, 7 refuse (a new target while still holding),
 * 1 an ARM while holding (re-arm the lease only). Disarmed (OFF written, or a fault) nothing is blocked. */
uint8_t bus_lease_gate(const bus_lease *, uint16_t address, bool arm, bool armed);

enum { BUS_DEVCFG_DEFAULT = 0, BUS_DEVCFG_OVERRIDE = 1, BUS_DEVCFG_INVALID = 2, BUS_DEVCFG_ABSENT = 3 };
/* Effective ID: the block's ID when the block is valid and its ID is 0..252,
 * otherwise nv_id. *state reports which case applied. */
uint8_t bus_devcfg_resolve(const uint8_t block[BUS_DEVCFG_BYTES], uint8_t nv_id, uint8_t *state);
/* Start-up baud of a valid block (1000000 for a default, invalid or absent block). */
uint32_t bus_devcfg_boot_baud(const uint8_t block[BUS_DEVCFG_BYTES]);
#endif
