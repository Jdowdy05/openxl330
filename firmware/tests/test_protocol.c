#include "protocol.h"
/* The production decoder now owns mutable frame storage. Preserve each test's
 * immutable/reusable wire fixture by modeling main's fresh receive buffer. */
static dxl_result fresh_process(const dxl_io *io,const uint8_t *wire,uint16_t n,uint8_t *out,uint16_t cap){uint8_t copy[DXL_MAX_WIRE];if(!wire)return dxl_process(io,0,n,out,cap);for(unsigned i=0;i<n&&i<sizeof copy;i++)copy[i]=wire[i];return dxl_process(io,copy,n,out,cap);}
#define dxl_process fresh_process

#include <assert.h>
#include <stdio.h>
#include <string.h>

static unsigned failures;
static unsigned checks;
#define CHECK(expr) do { \
    checks++; \
    if (!(expr)) { \
        fprintf(stderr, "%s:%d: FAIL %s\n", __func__, __LINE__, #expr); \
        failures++; \
        return; \
    } \
} while (0)

typedef struct {
    uint8_t ram[65536];
    unsigned reads;
    unsigned writes;
    uint16_t address;
    uint16_t size;
    uint8_t read_error;
    uint8_t write_error;
    unsigned flash_reads;
    uint8_t flash_error;
} registers;

static registers regs;
static dxl_io io;
/* Fixed fixture storage also permits a meaningful RED run before cap changes. */
static uint8_t out[640];

static uint8_t read_ram(void *context, uint16_t address, uint16_t size,
                        uint8_t *data) {
    registers *r = context;
    assert(size > 0 && size <= DXL_MAX_READ);
    assert((uint32_t)address + size <= sizeof r->ram);
    r->reads++;
    r->address = address;
    r->size = size;
    if (r->read_error) return r->read_error;
    memcpy(data, r->ram + address, size);
    return 0;
}

static uint8_t write_ram(void *context, uint16_t address, uint16_t size,
                         const uint8_t *data) {
    registers *r = context;
    assert(size > 0);
    assert((uint32_t)address + size <= sizeof r->ram);
    r->writes++;
    r->address = address;
    r->size = size;
    if (r->write_error) return r->write_error;
    memcpy(r->ram + address, data, size);
    return 0;
}

static void reset(void) {
    memset(&regs, 0, sizeof regs);
    memset(out, 0xcc, sizeof out);
    io = (dxl_io){.id=1, .firmware=0xfe, .model=1200, .context=&regs,
                  .read=read_ram, .write=write_ram};
}

static uint8_t read_flash(void *context, uint32_t address, uint16_t size,
                          uint8_t *data) {
    registers *r = context;
    assert(address >= 0x08000000u && address < 0x08010000u);
    assert(size > 0 && size <= 96 && size <= 0x08010000u - address);
    r->flash_reads++;
    if (r->flash_error) return r->flash_error;
    memcpy(data, r->ram + (address - 0x08000000u), size);
    return 0;
}

/* Independent long-division oracle; literals below pin it to real wire CRCs. */
static uint16_t reference_crc(const uint8_t *data, uint16_t n) {
    uint32_t remainder = 0;
    for (uint16_t i = 0; i < n; i++) {
        for (unsigned bit = 0; bit < 8; bit++) {
            const uint32_t incoming = (data[i] >> (7u - bit)) & 1u;
            const uint32_t top = (remainder >> 15) ^ incoming;
            remainder = (remainder << 1) & 0xffffu;
            if (top) remainder ^= 0x8005u;
        }
    }
    return (uint16_t)remainder;
}

static uint16_t frame(uint8_t id, const uint8_t *body, uint16_t n,
                      uint8_t *wire) {
    wire[0] = 0xff; wire[1] = 0xff; wire[2] = 0xfd;
    wire[3] = 0; wire[4] = id;
    uint16_t at = 7;
    for (uint16_t i = 0; i < n; i++) {
        wire[at++] = body[i];
        if (i >= 2 && body[i - 2] == 0xff && body[i - 1] == 0xff &&
            body[i] == 0xfd) wire[at++] = 0xfd;
    }
    const uint16_t length = (uint16_t)(at - 7u + 2u);
    wire[5] = (uint8_t)length; wire[6] = (uint8_t)(length >> 8);
    const uint16_t crc = reference_crc(wire, at);
    wire[at++] = (uint8_t)crc; wire[at++] = (uint8_t)(crc >> 8);
    return at;
}

static void reseal(uint8_t *wire, uint16_t n) {
    const uint16_t crc = reference_crc(wire, (uint16_t)(n - 2));
    wire[n - 2] = (uint8_t)crc;
    wire[n - 1] = (uint8_t)(crc >> 8);
}

/* Only verifies replies; request builder does not derive expected payloads. */
static bool reply_is(const dxl_result *r, uint8_t error,
                      const uint8_t *data, uint16_t size) {
    if (r->reply_bytes < 11 || r->reply_bytes > sizeof out) return false;
    if (out[0] != 0xff || out[1] != 0xff || out[2] != 0xfd || out[3] != 0 ||
        out[4] != io.id) return false;
    const uint16_t declared = (uint16_t)(out[5] | ((uint16_t)out[6] << 8));
    if ((uint32_t)declared + 7u != r->reply_bytes) return false;
    const uint16_t crc = reference_crc(out, (uint16_t)(r->reply_bytes - 2));
    if (out[r->reply_bytes - 2] != (uint8_t)crc ||
        out[r->reply_bytes - 1] != (uint8_t)(crc >> 8)) return false;
    uint8_t body[DXL_MAX_READ + 2];
    uint16_t used = 0;
    for (uint16_t i = 7; i < r->reply_bytes - 2; i++) {
        if (used >= sizeof body) return false;
        body[used++] = out[i];
        if (used >= 3 && body[used - 3] == 0xff && body[used - 2] == 0xff &&
            body[used - 1] == 0xfd) {
            i++;
            if (i >= r->reply_bytes - 2 || out[i] != 0xfd) return false;
        }
    }
    return used == (uint16_t)(size + 2) && body[0] == 0x55 && body[1] == error &&
           (!size || memcmp(body + 2, data, size) == 0);
}

static const uint8_t ping[] = {0xff,0xff,0xfd,0,1,3,0,1,0x19,0x4e};
static const uint8_t broadcast_ping[] = {0xff,0xff,0xfd,0,0xfe,3,0,1,0x31,0x42};
static const uint8_t ping_reply[] = {0xff,0xff,0xfd,0,1,7,0,0x55,0,0xb0,4,0xfe,0x0e,0xd6};
static const uint8_t ack[] = {0xff,0xff,0xfd,0,1,4,0,0x55,0,0xa1,0x0c};
static const uint8_t torque_write[] = {0xff,0xff,0xfd,0,1,6,0,3,0x40,0,1,0xdb,0x66};
static const uint8_t f9[] = {0xff,0xff,0xfd,0,1,3,0,0xf9,0x0a,0xcc};
static const uint8_t reboot[] = {0xff,0xff,0xfd,0,1,3,0,8,0x2f,0x4e};

static void literal_identity_and_crc(void) {
    reset();
    CHECK(reference_crc(ping, 8) == 0x4e19);
    CHECK(dxl_crc(ping, 8) == 0x4e19);
    CHECK(dxl_crc(NULL, 0) == 0);
    dxl_result r = dxl_process(&io, ping, sizeof ping, out, sizeof out);
    CHECK(r.reply_bytes == sizeof ping_reply);
    CHECK(memcmp(out, ping_reply, sizeof ping_reply) == 0);
    CHECK(r.valid_for_us && !r.group_reply && !r.enter_loader);
    CHECK(regs.reads == 0 && regs.writes == 0);
    io.id = 27; io.model = 0x1234; io.firmware = 0x61;
    uint8_t wire[32]; const uint8_t cmd[] = {1};
    const uint16_t n = frame(27, cmd, sizeof cmd, wire);
    const uint8_t identity[] = {0x34, 0x12, 0x61};
    r = dxl_process(&io, wire, n, out, sizeof out);
    CHECK(reply_is(&r, 0, identity, sizeof identity));
}

static void broadcast_identity_and_foreign_ids(void) {
    reset();
    dxl_result r = dxl_process(&io, broadcast_ping, sizeof broadcast_ping,
                               out, sizeof out);
    CHECK(r.reply_bytes == sizeof ping_reply && r.valid_for_us);
    CHECK(r.group_reply && r.response_slot == 1 && !r.enter_loader);
    uint8_t wire[32]; const uint8_t cmd[] = {1};
    uint16_t n = frame(2, cmd, sizeof cmd, wire);
    r = dxl_process(&io, wire, n, out, sizeof out);
    CHECK(!r.reply_bytes && !r.valid_for_us && !r.group_reply);
    n = frame(0xfd, cmd, sizeof cmd, wire);
    r = dxl_process(&io, wire, n, out, sizeof out);
    CHECK(!r.reply_bytes && !r.valid_for_us);
    io.id = 252;
    r = dxl_process(&io, broadcast_ping, sizeof broadcast_ping, out, sizeof out);
    CHECK(r.valid_for_us && r.group_reply && r.response_slot == 252);
}

static void unicast_read_write_and_errors(void) {
    reset();
    dxl_result r = dxl_process(&io, torque_write, sizeof torque_write, out, sizeof out);
    CHECK(r.reply_bytes == sizeof ack && memcmp(out, ack, sizeof ack) == 0);
    CHECK(r.valid_for_us && regs.writes == 1 && regs.ram[64] == 1);
    uint8_t wire[512]; const uint8_t read_cmd[] = {2,64,0,1,0};
    uint16_t n = frame(1, read_cmd, sizeof read_cmd, wire);
    const uint8_t one[] = {1};
    r = dxl_process(&io, wire, n, out, sizeof out);
    CHECK(reply_is(&r, 0, one, sizeof one) && regs.reads == 1);
    regs.read_error = 4;
    r = dxl_process(&io, wire, n, out, sizeof out);
    CHECK(reply_is(&r, 4, NULL, 0));
    regs.write_error = 7;
    r = dxl_process(&io, torque_write, sizeof torque_write, out, sizeof out);
    CHECK(reply_is(&r, 7, NULL, 0) && regs.ram[64] == 1);
    io.read = NULL;
    r = dxl_process(&io, wire, n, out, sizeof out);
    CHECK(reply_is(&r, 7, NULL, 0));
    io.write = NULL;
    r = dxl_process(&io, torque_write, sizeof torque_write, out, sizeof out);
    CHECK(reply_is(&r, 7, NULL, 0));
    const uint8_t unknown[] = {0xa1}; n = frame(1, unknown, sizeof unknown, wire);
    r = dxl_process(&io, wire, n, out, sizeof out);
    CHECK(reply_is(&r, 2, NULL, 0) && !r.enter_loader);
}

static void literal_read_matches_existing_wire(void) {
    reset();
    static const uint8_t request[] = {0xff,0xff,0xfd,0,1,7,0,2,0x50,0,0x10,0,0x3a,0xfd};
    static const uint8_t marker[] = {'C','O','D','E','X','-','M','A','R','K','E','R','-','V','1',0};
    static const uint8_t reply[] = {0xff,0xff,0xfd,0,1,0x14,0,0x55,0,'C','O','D','E','X','-','M','A','R','K','E','R','-','V','1',0,0xe0,0xe6};
    memcpy(regs.ram + 80, marker, sizeof marker);
    dxl_result r=dxl_process(&io,request,sizeof request,out,sizeof out);
    CHECK(r.reply_bytes == sizeof reply && memcmp(out,reply,sizeof reply) == 0);
    CHECK(regs.reads == 1 && regs.address == 80 && regs.size == 16);
}

static void broadcast_write_and_loader_rules(void) {
    reset(); uint8_t wire[64]; const uint8_t write_cmd[] = {3,64,0,0x5a};
    uint16_t n = frame(0xfe, write_cmd, sizeof write_cmd, wire);
    dxl_result r = dxl_process(&io, wire, n, NULL, 0);
    CHECK(!r.reply_bytes && r.valid_for_us && regs.writes == 1 && regs.ram[64] == 0x5a);
    r = dxl_process(&io, f9, sizeof f9, out, sizeof out);
    CHECK(r.enter_loader && r.valid_for_us && r.reply_bytes == sizeof ack);
    CHECK(memcmp(out, ack, sizeof ack) == 0);
    r = dxl_process(&io, reboot, sizeof reboot, out, sizeof out);
    /* FW231: Reboot resets into the application (Dynamixel SDK reboot()), never the loader. */
    CHECK(r.reboot && !r.enter_loader && r.reply_bytes == sizeof ack);
    r = dxl_process(&io, f9, sizeof f9, out, sizeof out);
    CHECK(r.enter_loader && !r.reboot);
    const uint8_t commands[] = {8,0xf9,0xa0,2};
    for (unsigned i = 0; i < sizeof commands; i++) {
        n = frame(0xfe, commands + i, 1, wire);
        r = dxl_process(&io, wire, n, out, sizeof out);
        CHECK(!r.reply_bytes && !r.enter_loader && !r.reboot && !r.valid_for_us);
    }
    const uint8_t extra[] = {0xf9,0}; n = frame(1, extra, sizeof extra, wire);
    r = dxl_process(&io, wire, n, out, sizeof out);
    CHECK(!r.enter_loader && reply_is(&r, 5, NULL, 0));
    const uint8_t extra8[] = {8,0}; n = frame(1, extra8, sizeof extra8, wire);
    r = dxl_process(&io, wire, n, out, sizeof out);
    CHECK(!r.reboot && reply_is(&r, 5, NULL, 0));
}

static void output_capacity_precedes_mutation(void) {
    reset();
    for (uint16_t cap = 0; cap < 11; cap++) {
        dxl_result r = dxl_process(&io, torque_write, sizeof torque_write, out, cap);
        CHECK(!r.reply_bytes && !r.enter_loader && regs.writes == 0);
        r = dxl_process(&io, f9, sizeof f9, out, cap);
        CHECK(!r.reply_bytes && !r.enter_loader);
        r = dxl_process(&io, reboot, sizeof reboot, out, cap);
        CHECK(!r.reply_bytes && !r.enter_loader && !r.reboot);
    }
    dxl_result r = dxl_process(&io, torque_write, sizeof torque_write, out, 11);
    CHECK(r.reply_bytes == 11 && regs.writes == 1 && regs.ram[64] == 1);
    r = dxl_process(&io, torque_write, sizeof torque_write, NULL, 20);
    CHECK(!r.reply_bytes && regs.writes == 1);
}

static void strict_framing_no_side_effects(void) {
    reset(); uint8_t wire[600]; dxl_result r;
    for (uint16_t n = 0; n < sizeof torque_write; n++) {
        r = dxl_process(&io, torque_write, n, out, sizeof out);
        CHECK(!r.reply_bytes && !r.valid_for_us && regs.writes == 0);
    }
    memcpy(wire, torque_write, sizeof torque_write); wire[12] ^= 1;
    r = dxl_process(&io, wire, sizeof torque_write, out, sizeof out);
    CHECK(!r.reply_bytes && regs.writes == 0);
    memcpy(wire, torque_write, sizeof torque_write); wire[3] = 1; reseal(wire, sizeof torque_write);
    r = dxl_process(&io, wire, sizeof torque_write, out, sizeof out);
    CHECK(!r.reply_bytes && regs.writes == 0);
    memcpy(wire, torque_write, sizeof torque_write); wire[5] = 0xff; wire[6] = 0xff;
    reseal(wire, sizeof torque_write);
    r = dxl_process(&io, wire, sizeof torque_write, out, sizeof out);
    CHECK(!r.reply_bytes && regs.writes == 0);
    const uint8_t bad_stuff[] = {0xff,0xff,0xfd,0,1,8,0,3,0,0,0xff,0xff,0xfd,0,0};
    memcpy(wire, bad_stuff, sizeof bad_stuff); reseal(wire, sizeof bad_stuff);
    r = dxl_process(&io, wire, sizeof bad_stuff, out, sizeof out);
    CHECK(!r.reply_bytes && !r.valid_for_us && regs.writes == 0);
    memset(wire, 0, sizeof wire); wire[0]=0xff;wire[1]=0xff;wire[2]=0xfd;
    wire[4]=1;wire[5]=0xfa;wire[6]=0;wire[7]=3;reseal(wire, 257);
    r = dxl_process(&io, wire, 257, out, sizeof out);
    CHECK(!r.reply_bytes && regs.writes == 0);
    r = dxl_process(NULL, ping, sizeof ping, out, sizeof out);
    CHECK(!r.reply_bytes);
    r = dxl_process(&io, NULL, sizeof ping, out, sizeof out);
    CHECK(!r.reply_bytes);
}

static void range_and_length_boundaries(void) {
    reset(); uint8_t wire[600];
    const uint8_t bad[][5] = {{2,0,0,0,0},{2,0,0,0x81,1},{2,0xff,0xff,2,0}};
    const uint8_t errors[] = {5,5,4};
    for (unsigned i = 0; i < 3; i++) {
        const uint16_t n = frame(1, bad[i], 5, wire);
        dxl_result r = dxl_process(&io, wire, n, out, sizeof out);
        CHECK(reply_is(&r, errors[i], NULL, 0) && regs.reads == 0);
    }
    const uint8_t end[] = {3,0xff,0xff,0x5c};
    uint16_t n = frame(1, end, sizeof end, wire);
    dxl_result r = dxl_process(&io, wire, n, out, sizeof out);
    CHECK(reply_is(&r, 0, NULL, 0) && regs.ram[65535] == 0x5c && regs.writes == 1);
    const uint8_t overflow[] = {3,0xff,0xff,1,2};
    n = frame(1, overflow, sizeof overflow, wire);
    r = dxl_process(&io, wire, n, out, sizeof out);
    CHECK(reply_is(&r, 4, NULL, 0) && regs.writes == 1);
    const uint8_t empty[] = {3,1,0}; n = frame(1, empty, sizeof empty, wire);
    r = dxl_process(&io, wire, n, out, sizeof out);
    CHECK(reply_is(&r, 5, NULL, 0) && regs.writes == 1);
    const uint8_t wrong_ping[] = {1,0}; n = frame(1, wrong_ping, sizeof wrong_ping, wire);
    r = dxl_process(&io, wire, n, out, sizeof out);
    CHECK(reply_is(&r, 5, NULL, 0));
    const uint8_t stuffed_read[] = {0xff,0xff,0xfd,0,1,8,0,2,0xff,0xff,0xfd,0xfd,0,0xc7,0x76};
    r = dxl_process(&io, stuffed_read, sizeof stuffed_read, out, sizeof out);
    CHECK(reply_is(&r, 4, NULL, 0) && regs.reads == 0);
}

static void largest_frame_and_group_range_boundaries(void) {
    reset(); uint8_t wire[600]; uint8_t body[247] = {3,0,1};
    memset(body + 3,0x5a,sizeof body - 3);
    uint16_t n=frame(1,body,sizeof body,wire);
    CHECK(n == DXL_MAX_WIRE);
    dxl_result r=dxl_process(&io,wire,n,out,DXL_REPLY_CAP);
    CHECK(reply_is(&r,0,NULL,0) && regs.writes == 1 && regs.size == 244);
    CHECK(regs.ram[256] == 0x5a && regs.ram[499] == 0x5a);
    reset();
    const uint8_t cases[][8] = {
        {0x82,0,0,0,0,2,1,3},
        {0x82,0xff,0xff,2,0,2,1,3},
        {0x82,0,0,0x81,1,2,1,3},
        {0x92,1,0xff,0xff,2,0,0,0},
        {0x83,0xff,0xff,2,0,1,0x31,0x32},
        {0x83,0,0,0xff,0xff,1,0x31,0x32}
    };
    const uint16_t lengths[]={8,8,8,6,8,8};
    const uint8_t errors[]={5,4,5,4,4,5};
    for(unsigned i=0;i<6;i++) {
        n=frame(1,cases[i],lengths[i],wire);
        r=dxl_process(&io,wire,n,out,sizeof out);
        CHECK(reply_is(&r,errors[i],NULL,0) && regs.reads == 0 && regs.writes == 0);
    }
    const uint8_t bulk_zero[] = {0x93,1,64,0,0,0};
    n=frame(1,bulk_zero,sizeof bulk_zero,wire);
    r=dxl_process(&io,wire,n,out,sizeof out);
    CHECK(reply_is(&r,5,NULL,0) && regs.writes == 0);
    const uint8_t foreign_range[] = {0x92,2,0xff,0xff,0xff,0xff,1,0,0,1,0};
    n=frame(0xfe,foreign_range,sizeof foreign_range,wire);
    const uint8_t zero[]={0};
    r=dxl_process(&io,wire,n,out,sizeof out);
    CHECK(reply_is(&r,0,zero,1) && r.group_reply && r.response_slot == 1);
    uint8_t many[247]={0x82,0,0,1,0};memset(many+5,2,sizeof many-5);many[246]=1;
    n=frame(0xfe,many,sizeof many,wire);
    r=dxl_process(&io,wire,n,out,sizeof out);
    CHECK(reply_is(&r,0,zero,1) && r.group_reply && r.response_slot == 241);
}

static void full_logging_read_survives_worst_case_stuffing(void) {
    reset(); uint8_t wire[32];
    /* 384 data bytes, each FF FF FD triplet requiring one inserted FD. */
    for (unsigned i=0;i<384;i++) regs.ram[1000+i]=(const uint8_t[]){0xff,0xff,0xfd}[i%3];
    const uint8_t read_cmd[]={2,0xe8,3,0x80,1};
    const uint16_t n=frame(1,read_cmd,sizeof read_cmd,wire);
    dxl_result r=dxl_process(&io,wire,n,out,DXL_REPLY_CAP);
    CHECK(r.reply_bytes == 523 && r.valid_for_us && regs.reads == 1 && regs.size == 384);
    CHECK(reply_is(&r,0,regs.ram+1000,384));
    CHECK(out[5] == 4 && out[6] == 2); /* stuffed packet length = 516 */
    memset(out,0xcc,sizeof out);
    r=dxl_process(&io,wire,n,out,523);
    CHECK(r.reply_bytes == 523 && reply_is(&r,0,regs.ram+1000,384));
    CHECK(out[523] == 0xcc);
    memset(out,0xcc,sizeof out);
    r=dxl_process(&io,wire,n,out,522);
    CHECK(!r.reply_bytes && out[522] == 0xcc && !r.enter_loader);
}

static void max_receive_frame_truncations_and_stuffing_are_safe(void) {
    reset(); uint8_t body[247]={3,0,1};uint8_t wire[260];
    memset(body+3,0x5a,sizeof body-3);
    const uint16_t n=frame(1,body,sizeof body,wire);
    CHECK(n == 256);
    for(uint16_t short_size=0;short_size<n;short_size++) {
        dxl_result r=dxl_process(&io,wire,short_size,out,sizeof out);
        CHECK(!r.valid_for_us && !r.reply_bytes && !r.enter_loader && regs.writes == 0);
    }
    wire[255]^=1;
    dxl_result r=dxl_process(&io,wire,n,out,sizeof out);
    CHECK(!r.valid_for_us && !r.reply_bytes && regs.writes == 0);
    wire[255]^=1;
    /* CRC-valid, exactly 256 bytes, missing required terminal stuffing FD. */
    wire[251]=0xff;wire[252]=0xff;wire[253]=0xfd;reseal(wire,n);
    r=dxl_process(&io,wire,n,out,sizeof out);
    CHECK(!r.valid_for_us && !r.reply_bytes && regs.writes == 0);
}

static void stuffing_round_trip_and_read_capacity(void) {
    reset(); uint8_t wire[600];
    const uint8_t cmd[] = {3,0,1,0xff,0xff,0xfd,0xfd,0xff,0xff,0xfd};
    uint16_t n = frame(1, cmd, sizeof cmd, wire);
    dxl_result r = dxl_process(&io, wire, n, out, sizeof out);
    const uint8_t expected[] = {0xff,0xff,0xfd,0xfd,0xff,0xff,0xfd};
    CHECK(reply_is(&r, 0, NULL, 0) && regs.writes == 1);
    CHECK(regs.size == sizeof expected && memcmp(regs.ram + 256, expected, sizeof expected) == 0);
    for (unsigned i = 0; i < DXL_MAX_READ; i++) regs.ram[1000+i] = (const uint8_t[]){0xff,0xff,0xfd}[i % 3];
    const uint8_t read_cmd[] = {2,0xe8,3,192,0}; n = frame(1, read_cmd, sizeof read_cmd, wire);
    r = dxl_process(&io, wire, n, out, sizeof out);
    CHECK(r.reply_bytes == 267 && reply_is(&r, 0, regs.ram + 1000, 192));
    r = dxl_process(&io, wire, n, out, 267);
    CHECK(r.reply_bytes == 267 && reply_is(&r, 0, regs.ram + 1000, 192));
    r = dxl_process(&io, wire, n, out, 266);
    CHECK(!r.reply_bytes && !r.enter_loader);
}

static void sync_write_validates_all_rows(void) {
    reset(); uint8_t wire[600];
    const uint8_t cmd[] = {0x83,64,0,2,0,2,0x21,0x22,1,0x31,0x32,3,0x41,0x42};
    uint16_t n = frame(0xfe, cmd, sizeof cmd, wire);
    dxl_result r = dxl_process(&io, wire, n, NULL, 0);
    CHECK(r.valid_for_us && !r.reply_bytes && regs.writes == 1);
    CHECK(regs.address == 64 && regs.size == 2 && regs.ram[64] == 0x31 && regs.ram[65] == 0x32);
    reset(); const uint8_t duplicate[] = {0x83,64,0,1,0,1,0x31,2,0x41,1,0x51};
    n = frame(0xfe, duplicate, sizeof duplicate, wire);
    r = dxl_process(&io, wire, n, out, sizeof out);
    CHECK(!r.reply_bytes && regs.writes == 0 && regs.ram[64] == 0);
    n = frame(1, duplicate, sizeof duplicate, wire);
    r = dxl_process(&io, wire, n, out, sizeof out);
    CHECK(reply_is(&r, 5, NULL, 0) && regs.writes == 0);
    const uint8_t truncated[] = {0x83,64,0,2,0,1,0x31,0x32,2,0x41};
    n = frame(0xfe, truncated, sizeof truncated, wire);
    r = dxl_process(&io, wire, n, out, sizeof out);
    CHECK(!r.reply_bytes && regs.writes == 0);
    const uint8_t foreign[] = {0x83,64,0,1,0,2,0x31};
    n = frame(0xfe, foreign, sizeof foreign, wire);
    r = dxl_process(&io, wire, n, out, sizeof out);
    CHECK(!r.valid_for_us && !r.reply_bytes && regs.writes == 0);
    n = frame(1, cmd, sizeof cmd, wire);
    r = dxl_process(&io, wire, n, out, 10);
    CHECK(!r.reply_bytes && regs.writes == 0);
}

static void bulk_write_validates_all_rows(void) {
    reset(); uint8_t wire[600];
    const uint8_t cmd[] = {0x93,2,0x50,0,1,0,0x21,1,0x40,0,2,0,0x31,0x32,3,0x60,0,1,0,0x41};
    uint16_t n = frame(0xfe, cmd, sizeof cmd, wire);
    dxl_result r = dxl_process(&io, wire, n, NULL, 0);
    CHECK(r.valid_for_us && !r.reply_bytes && regs.writes == 1);
    CHECK(regs.address == 64 && regs.size == 2 && regs.ram[64] == 0x31 && regs.ram[65] == 0x32);
    reset(); const uint8_t duplicate[] = {0x93,1,64,0,1,0,0x21,1,64,0,1,0,0x31};
    n = frame(1, duplicate, sizeof duplicate, wire);
    r = dxl_process(&io, wire, n, out, sizeof out);
    CHECK(reply_is(&r, 5, NULL, 0) && regs.writes == 0);
    const uint8_t malformed[] = {0x93,1,64,0,1,0,0x21,2,64,0,0xff,0xff};
    n = frame(0xfe, malformed, sizeof malformed, wire);
    r = dxl_process(&io, wire, n, out, sizeof out);
    CHECK(!r.reply_bytes && regs.writes == 0);
    const uint8_t short_header[] = {0x93,1,64,0,1,0,0x21,2,64};
    n = frame(0xfe, short_header, sizeof short_header, wire);
    r = dxl_process(&io, wire, n, out, sizeof out);
    CHECK(!r.reply_bytes && regs.writes == 0);
    n = frame(1, cmd, sizeof cmd, wire);
    r = dxl_process(&io, wire, n, out, 10);
    CHECK(!r.reply_bytes && regs.writes == 0);
    const uint8_t overflow[] = {0x93,1,0xff,0xff,2,0,0x21,0x22};
    n = frame(1, overflow, sizeof overflow, wire);
    r = dxl_process(&io, wire, n, out, sizeof out);
    CHECK(reply_is(&r, 4, NULL, 0) && regs.writes == 0);
}

static void chained_slot_metadata(void) {
    reset(); uint8_t wire[64];
    const uint8_t first[] = {0x82,64,0,2,0,1,2,3};
    uint16_t n = frame(0xfe, first, sizeof first, wire);
    dxl_result r = dxl_process(&io, wire, n, out, sizeof out);
    CHECK(r.group_reply && r.response_slot == 0 && r.prev_id == 0xff && r.prior_bytes == 0);
    const uint8_t big[] = {0x82,0,1,0x30,0,7,9,1};   /* 48-byte rows: worst 11+48+16 = 75 each */
    n = frame(0xfe, big, sizeof big, wire);
    r = dxl_process(&io, wire, n, out, sizeof out);
    CHECK(r.response_slot == 2 && r.prev_id == 9 && r.prior_bytes == 150);
    const uint8_t bulk[] = {0x92,5,0,1,100,0,6,0,1,4,0,1,64,0,2,0};   /* rows 100 B, 4 B, then ours */
    n = frame(0xfe, bulk, sizeof bulk, wire);
    r = dxl_process(&io, wire, n, out, sizeof out);
    CHECK(r.response_slot == 2 && r.prev_id == 6 && r.prior_bytes == (11u+100u+33u)+(11u+4u+1u));
    CHECK(dxl_worst_status_bytes(0) == 11 && dxl_worst_status_bytes(3) == 15 && dxl_worst_status_bytes(384) == 523);
}

static void group_reads_and_slots(void) {
    reset(); uint8_t wire[600]; regs.ram[64] = 0x31; regs.ram[65] = 0x32;
    const uint8_t expected[] = {0x31,0x32};
    const uint8_t sync[] = {0x82,64,0,2,0,3,2,1,4};
    uint16_t n = frame(0xfe, sync, sizeof sync, wire);
    dxl_result r = dxl_process(&io, wire, n, out, sizeof out);
    CHECK(reply_is(&r, 0, expected, sizeof expected) && r.valid_for_us);
    CHECK(r.group_reply && r.response_slot == 2 && regs.reads == 1);
    /* FW231 chaining: previous row ID 2, two earlier 2-byte rows of worst 11+2+0 bytes. */
    CHECK(r.prev_id == 2 && r.prior_bytes == 26);
    const uint8_t bulk[] = {0x92,2,0x50,0,1,0,1,64,0,2,0,3,0x60,0,1,0};
    n = frame(0xfe, bulk, sizeof bulk, wire);
    r = dxl_process(&io, wire, n, out, sizeof out);
    CHECK(reply_is(&r, 0, expected, sizeof expected) && r.group_reply && r.response_slot == 1);
    CHECK(r.prev_id == 2 && r.prior_bytes == 12);
    /* fast10 (review F-3): our ID twice in a Sync/Bulk Read stays silent (was error 5, which both later rows chained on) */
    const uint8_t duplicate[] = {0x82,64,0,2,0,1,2,1};
    n = frame(0xfe, duplicate, sizeof duplicate, wire);
    r = dxl_process(&io, wire, n, out, sizeof out);
    CHECK(!r.valid_for_us && !r.reply_bytes && regs.reads == 2);
    const uint8_t duplicate_bulk[] = {0x92,1,64,0,2,0,1,64,0,2,0};
    n = frame(0xfe, duplicate_bulk, sizeof duplicate_bulk, wire);
    r = dxl_process(&io, wire, n, out, sizeof out);
    CHECK(!r.valid_for_us && !r.reply_bytes && regs.reads == 2);
    const uint8_t malformed[] = {0x92,1,64,0,2,0,2,64,0,1};
    n = frame(0xfe, malformed, sizeof malformed, wire);
    r = dxl_process(&io, wire, n, out, sizeof out);
    CHECK(regs.reads == 2 && (!r.reply_bytes || reply_is(&r, 5, NULL, 0)));
    const uint8_t foreign[] = {0x82,64,0,2,0,2,3,4};
    n = frame(0xfe, foreign, sizeof foreign, wire);
    r = dxl_process(&io, wire, n, out, sizeof out);
    CHECK(!r.reply_bytes && !r.valid_for_us && regs.reads == 2);
    uint8_t many[262] = {0x82,64,0,2,0};
    memset(many + 5, 2, sizeof many - 5); many[261] = 1;
    n = frame(0xfe, many, sizeof many, wire);
    r = dxl_process(&io, wire, n, out, sizeof out);
    CHECK(regs.reads == 2 && !r.enter_loader);
}

static void bounded_read_only_flash_and_alert(void) {
    reset(); io.flash_read = read_flash;
    uint8_t wire[128];
    const uint8_t cmd[] = {0xa0,0,0,0,8,4,0};
    uint16_t n = frame(1, cmd, sizeof cmd, wire);
    regs.ram[0]=0;regs.ram[1]=1;regs.ram[2]=2;regs.ram[3]=3;
    const uint8_t expected[] = {0,1,2,3};
    const uint8_t literal_reply[] = {0xff,0xff,0xfd,0,1,8,0,0x55,0,0,1,2,3,0xa1,0xb4};
    dxl_result r = dxl_process(&io, wire, n, out, sizeof out);
    CHECK(reply_is(&r, 0, expected, sizeof expected) && regs.flash_reads == 1);
    CHECK(r.reply_bytes == sizeof literal_reply && memcmp(out, literal_reply, sizeof literal_reply) == 0);
    const uint8_t invalid[][7] = {
        {0xa0,0xff,0xff,0xff,7,1,0}, {0xa0,0,0,1,8,1,0},
        {0xa0,0xff,0xff,0,8,2,0}, {0xa0,0xff,0xff,0xff,0xff,96,0},
        {0xa0,0,0,0,8,0,0}, {0xa0,0,0,0,8,97,0}
    };
    for (unsigned i = 0; i < 6; i++) {
        n = frame(1, invalid[i], 7, wire);
        r = dxl_process(&io, wire, n, out, sizeof out);
        CHECK(reply_is(&r, i < 4 ? 4 : 5, NULL, 0) && regs.flash_reads == 1);
    }
    const uint8_t last[] = {0xa0,0xff,0xff,0,8,1,0}; n=frame(1,last,sizeof last,wire);
    regs.ram[65535]=0x3c;const uint8_t last_expected[]={0x3c};
    r=dxl_process(&io,wire,n,out,sizeof out);
    CHECK(reply_is(&r,0,last_expected,1) && regs.flash_reads == 2);
    const uint8_t largest[]={0xa0,0,0,0,8,96,0};n=frame(1,largest,sizeof largest,wire);
    r=dxl_process(&io,wire,n,out,138);
    CHECK(!r.reply_bytes && regs.flash_reads == 2);
    r=dxl_process(&io,wire,n,out,139);
    CHECK(r.reply_bytes && regs.flash_reads == 3);
    wire[n-1]^=1;r=dxl_process(&io,wire,n,out,sizeof out);
    CHECK(!r.reply_bytes && regs.flash_reads == 3);
    n=frame(0xfe,cmd,sizeof cmd,wire);r=dxl_process(&io,wire,n,out,sizeof out);
    CHECK(!r.reply_bytes && !r.valid_for_us && regs.flash_reads == 3);
    io.flash_read=NULL;n=frame(1,cmd,sizeof cmd,wire);
    r=dxl_process(&io,wire,n,out,sizeof out);
    CHECK(reply_is(&r,7,NULL,0) && regs.flash_reads == 3);
    io.flash_read=read_flash;regs.flash_error=4;
    r=dxl_process(&io,wire,n,out,sizeof out);
    CHECK(reply_is(&r,4,NULL,0) && regs.flash_reads == 4);
    io.alert=0xff;
    r=dxl_process(&io,wire,n,out,sizeof out);
    CHECK(reply_is(&r,0x84,NULL,0));
    r=dxl_process(&io,ping,sizeof ping,out,sizeof out);
    const uint8_t identity[]={0xb0,4,0xfe};
    CHECK(reply_is(&r,0x80,identity,3));
}

static void all_single_bit_corruptions_are_silent(void) {
    reset(); uint8_t wire[sizeof torque_write];
    for (unsigned index=0;index<sizeof torque_write;index++) {
        for(unsigned bit=0;bit<8;bit++) {
            memcpy(wire,torque_write,sizeof wire);wire[index]^=(uint8_t)(1u<<bit);
            dxl_result r=dxl_process(&io,wire,sizeof wire,out,sizeof out);
            CHECK(!r.reply_bytes && !r.valid_for_us && !r.enter_loader && regs.writes == 0);
        }
    }
}

static uint32_t random_state = 0xb7e15162u;
static uint32_t random_u32(void) {
    random_state ^= random_state << 13;
    random_state ^= random_state >> 17;
    random_state ^= random_state << 5;
    return random_state;
}

static void fuzz_frames_and_capacities(void) {
    reset(); uint8_t wire[600]={0}; uint8_t guarded[DXL_REPLY_CAP + 2];
    for (unsigned trial = 0; trial < 40000; trial++) {
        const uint16_t n = (uint16_t)(random_u32() % (sizeof wire + 1));
        for (uint16_t i = 0; i < n; i++) wire[i] = (uint8_t)random_u32();
        if (n >= 10 && (trial & 1u)) {
            wire[0]=0xff;wire[1]=0xff;wire[2]=0xfd;wire[3]=0;
            wire[4] = (trial & 2u) ? 1 : 0xfe;
            wire[5]=(uint8_t)(n-7);wire[6]=(uint8_t)((n-7)>>8);
            reseal(wire, n);
        }
        const uint16_t cap = (uint16_t)(random_u32() % (DXL_REPLY_CAP + 1));
        memset(guarded, 0xcc, sizeof guarded);
        regs.reads = 0; regs.writes = 0;
        dxl_result r = dxl_process(&io, wire, n, guarded + 1, cap);
        CHECK(r.reply_bytes <= cap && guarded[0] == 0xcc && guarded[cap + 1] == 0xcc);
        CHECK(!r.enter_loader || (r.reply_bytes >= 11 && wire[4] == 1));
        CHECK(regs.reads <= 1 && regs.writes <= 1);
        CHECK(cap >= 11 || wire[4] == 0xfe || regs.writes == 0);
    }
}

static void fuzz_stuffed_command_bodies(void) {
    reset();uint8_t body[503];uint8_t wire[680];uint8_t guarded[DXL_REPLY_CAP+2];
    const uint8_t instructions[]={1,2,3,8,0xf9,0x82,0x83,0x92,0x93,0xa0,0xa1};
    io.flash_read=read_flash;
    for(unsigned trial=0;trial<20000;trial++) {
        const uint16_t size=(uint16_t)(1u+random_u32()%sizeof body);
        for(uint16_t i=0;i<size;i++) {
            const uint32_t sample=random_u32();
            body[i]=(sample & 3u) ? (const uint8_t[]){0xff,0xff,0xfd}[i%3] : (uint8_t)sample;
        }
        body[0]=instructions[random_u32()%sizeof instructions];
        const uint8_t id=(trial & 1u)?1:0xfe;
        uint16_t n=frame(id,body,size,wire);
        if((trial & 3u)==0 && n>10) {
            /* Removing an arbitrary stuffed-body byte with a repaired CRC
             * reaches strict decoding and row parsing, not only CRC checks. */
            const uint16_t where=(uint16_t)(7u+random_u32()%(n-9u));
            memmove(wire+where,wire+where+1,n-where-1u);n--;
            wire[5]=(uint8_t)(n-7u);wire[6]=(uint8_t)((n-7u)>>8);reseal(wire,n);
        }
        const uint16_t cap=(uint16_t)(random_u32()%(DXL_REPLY_CAP+1));
        memset(guarded,0xcc,sizeof guarded);
        regs.reads=0;regs.writes=0;regs.flash_reads=0;
        dxl_result r=dxl_process(&io,wire,n,guarded+1,cap);
        CHECK(r.reply_bytes<=cap && guarded[0]==0xcc && guarded[cap+1]==0xcc);
        CHECK(regs.reads<=1 && regs.writes<=1 && regs.flash_reads<=1);
        CHECK(!r.enter_loader || (r.reply_bytes>=11 && id==1));
        CHECK(cap>=11 || id==0xfe || regs.writes==0);
    }
}

int main(void) {
    literal_identity_and_crc();
    broadcast_identity_and_foreign_ids();
    unicast_read_write_and_errors();
    literal_read_matches_existing_wire();
    broadcast_write_and_loader_rules();
    output_capacity_precedes_mutation();
    strict_framing_no_side_effects();
    range_and_length_boundaries();
    largest_frame_and_group_range_boundaries();
    full_logging_read_survives_worst_case_stuffing();
    max_receive_frame_truncations_and_stuffing_are_safe();
    stuffing_round_trip_and_read_capacity();
    sync_write_validates_all_rows();
    bulk_write_validates_all_rows();
    group_reads_and_slots();
    chained_slot_metadata();
    bounded_read_only_flash_and_alert();
    all_single_bit_corruptions_are_silent();
    fuzz_frames_and_capacities();
    fuzz_stuffed_command_bodies();
    printf("Protocol2: %u checks; %u failures; 60000 deterministic fuzz cases\n", checks, failures);
    return failures ? 1 : 0;
}
