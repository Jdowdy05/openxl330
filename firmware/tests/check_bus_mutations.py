"""Mutation check for the FW231 bus scheduler: every mutant of firmware/bus.c must make
tests/test_bus.c fail (collision, wrong order, stale reply or a unit check)."""
import pathlib, shutil, subprocess, sys, tempfile
ROOT = pathlib.Path(__file__).resolve().parent.parent
SRC = (ROOT / 'firmware/bus.c').read_text()
MUTANTS = {
    'trigger on any status': ('id == b->prev_id) bus_reply_trigger', 'id != 0xffu) bus_reply_trigger'),
    'no per-slot latency allowance': ('slot_us = (uint32_t)r->response_slot * BUS_SLOT_LATENCY_US', 'slot_us = 0u'),
    'fallback ignores earlier rows': ('uint32_t wire_us = bus_wire_us(r->prior_bytes, bps)', 'uint32_t wire_us = 0u'),
    'group rows reply at once': ('if (!r->group_reply || r->response_slot == 0u)', 'if (instruction != 1u || !r->group_reply || r->response_slot == 0u)'),
    'instruction never cancels': ('if (!b || (b->kind != BUS_REPLY_CHAIN && b->kind != BUS_REPLY_SLOT && b->kind != BUS_REPLY_FAST)) return false;', 'return false;'),
    'ping slot 0': ('(uint32_t)r->response_slot * BUS_PING_SLOT_US', '0u'),
    'no gap after predecessor': ('b->gap_us = gap < floor_us ? floor_us : gap;', 'b->gap_us = 0u;'),
    'one-byte chain gap (FW231 before 2026-11-20)': ('b->gap_us = gap < floor_us ? floor_us : gap;', 'b->gap_us = gap < 5u ? 5u : gap;'),
    'gap knob (write 1080) ignored': ('uint32_t floor_us = b->gap_min_us ? b->gap_min_us : BUS_CHAIN_GAP_US;', 'uint32_t floor_us = BUS_CHAIN_GAP_US;'),
    'fast: segment CRC not checked': ('? byte == (uint8_t)f->field\n           : off + 1u == f->seg ? byte == (uint8_t)(f->field >> 8)', '? true\n           : off + 1u == f->seg ? true'),
    'fast: predecessor ID not checked': ('? byte == f->prev_id : true;', '? true : true;'),
    'fast: header LEN not checked': ('ok = i == 5u ? byte == (uint8_t)f->len :', 'ok = i == 5u ? true :'),
    'fast: CRC skips earlier CRC bytes': ('    f->crc = dxl_crc_update(f->crc, byte);', '    if (i < 8u || (f->off != f->seg - 1u && f->off != 0u)) f->crc = dxl_crc_update(f->crc, byte);'),
    'fast: abandons with unread bytes (fsr-02, 71/200 rows lost)': ('    if (rx_pending || (uint32_t)(now - last_rx_us) < b->gap_us) return false;   /* parse what arrived first */\n', ''),
    'fast: row position never wraps': ('if (++off == f->seg) { off = 0u; f->segno++; }', 'if (++off == f->seg) { f->segno++; }'),
    'fast: predecessor row index off by one': ('f->pred = (uint16_t)((need - 8u) / seg - 1u);', 'f->pred = (uint16_t)((need - 8u) / seg);'),
    'fast: follower falls back without its prefix': ('    if (b->kind == BUS_REPLY_FAST && !b->triggered) return false;', ''),
    'fast: never abandons': ('if (!b || b->kind != BUS_REPLY_FAST || b->triggered || (uint32_t)(now - b->armed_us) < b->fallback_us) return false;', 'return false;'),
    'fast start: any status releases us': ('return w->id == w->prev_id ? BUS_WATCH_READY : BUS_WATCH_CAPTURING;', 'return BUS_WATCH_READY;'),
    'fast start: status CRC not checked': ('    if ((uint16_t)(w->low | (byte << 8)) != w->crc) return BUS_WATCH_CAPTURING;\n', ''),
    'fast start: a new instruction does not stop the watcher': ('    if (w->inst != 0x55u) return BUS_WATCH_DEAD;               /* a new instruction: the foreground supersedes */\n', ''),
    'fast start: Fast Sync Read prefix not verified': ('return st == BUS_FAST_READY ? BUS_WATCH_READY : st == BUS_FAST_FAILED ? BUS_WATCH_DEAD : BUS_WATCH_CAPTURING;', 'return BUS_WATCH_READY;'),
    'fast start: CRC excludes the header': ('if (w->at == 4u) w->crc = 0x0e28u;', 'if (w->at == 4u) w->crc = 0u;'),
    'span runner: plain run over the CRC field': ('uint32_t stop = f->segno == f->pred && f->off <= 1u ? 1u : f->seg - 2u;', 'uint32_t stop = f->segno == f->pred && f->off <= 1u ? 1u : f->seg;'),
    'span runner: predecessor ID never checked': ('uint32_t stop = f->segno == f->pred && f->off <= 1u ? 1u : f->seg - 2u;', 'uint32_t stop = f->seg - 2u;'),
    'span runner: status CRC bytes hashed as body': ('} else if (w->at >= 8u && w->at + 2u < w->total) n = w->total - 2u - w->at;', '} else if (w->at >= 8u && w->at < w->total) n = w->total - w->at;'),
    '600 us per slot (FW231 before 2026-11-20)': ('slot_us = (uint32_t)r->response_slot * BUS_SLOT_LATENCY_US', 'slot_us = (uint32_t)r->response_slot * 600u'),
    'fallback may bypass the chain gap (bus2-02 build)': ('        if ((uint32_t)(now - b->trigger_us) < b->gap_us) return false;\n        if (rx_pending || (uint32_t)(now - last_rx_us) < bus_idle_us(bps)) return false;\n        return true;\n    }',
                                                         '        if ((uint32_t)(now - b->trigger_us) >= b->gap_us) return true;\n    }'),
    'triggered reply ignores later traffic (review P1)': ('        if (rx_pending || (uint32_t)(now - last_rx_us) < bus_idle_us(bps)) return false;\n        return true;', '        return true;'),
    'unread RX bytes ignored (bus2-03 build)': ('if (rx_pending || (uint32_t)(now - last_rx_us) < bus_idle_us(bps)) {', 'if ((uint32_t)(now - last_rx_us) < bus_idle_us(bps)) {'),
    'arming forgets nothing / trigger kept across requests': ('    b->triggered = false; b->deferred = false; b->armed_us = now;', '    b->deferred = false; b->armed_us = now;'),
    'lease renewed while expired': ('void bus_lease_renew(bus_lease *b, uint32_t now) { if (b && b->state == BUS_LEASE_RUNNING) b->last_us = now; }', 'void bus_lease_renew(bus_lease *b, uint32_t now) { if (b) { b->state = b->ms ? BUS_LEASE_RUNNING : BUS_LEASE_OFF; b->last_us = now; } }'),
    'lease expires while OFF': ('if (!b || b->state != BUS_LEASE_RUNNING || !armed) return false;', 'if (!b || b->state != BUS_LEASE_RUNNING) return false;'),
    'revert to boot baud': ('b->current = b->previous; b->state', 'b->current = BUS_BOOT_BAUD; b->state'),
    'devcfg accepts bad CRC': ('dxl_crc(block, 28u) != crc ||', ''),
    'no rescue at 1M': ('if (b->boot_state == BUS_BOOT_WAITING) b->boot_state = BUS_BOOT_RESCUED;', ''),
    'no rescue window': ('(uint32_t)(now - b->boot_since_us) >= BUS_RESCUE_US', '(uint32_t)(now - b->boot_since_us) >= 0u'),
    'start-up baud ignored': ("return block[6] == 5u ? 3000000u : block[6] == 4u ? 2000000u : BUS_BOOT_BAUD;", "return BUS_BOOT_BAUD;"),
    'bad baud code accepted': ("(block[6] != 0xffu && (block[6] < 3u || block[6] > 5u)) ||", ""),
}
cc = ['clang', '-std=c11', '-O1', '-Wall', '-Wextra', '-Werror', '-Wno-misleading-indentation', '-Wno-unused-variable', '-Wno-unused-parameter', '-Wno-type-limits', '-Wno-tautological-unsigned-zero-compare']
survived = []
with tempfile.TemporaryDirectory() as tmp:
    tmp = pathlib.Path(tmp)
    shutil.copytree(ROOT / 'firmware', tmp / 'firmware')
    for name, (old, new) in MUTANTS.items():
        if SRC.count(old) != 1: sys.exit(f'mutation anchor not unique: {name}')
        (tmp / 'firmware/bus.c').write_text(SRC.replace(old, new))
        exe = tmp / 'bus'
        build = subprocess.run(cc + ['-I', str(tmp / 'firmware'), str(ROOT / 'tests/test_bus.c'), str(tmp / 'firmware/bus.c'),
                                     str(tmp / 'firmware/protocol.c'), '-o', str(exe)], capture_output=True, text=True)
        if build.returncode:
            sys.exit(f'mutant failed to build: {name}\n{build.stderr}')
        run = subprocess.run([str(exe)], capture_output=True, text=True, timeout=600)
        killed = run.returncode != 0
        print(('killed   ' if killed else 'SURVIVED ') + name)
        if not killed: survived.append(name)
if survived: sys.exit('bus mutants survived: ' + ', '.join(survived))
print(f'bus scheduler mutations: {len(MUTANTS)}/{len(MUTANTS)} killed')
