/* Behavioral contract tests for the real driver at its MMIO boundary.
 * Mock only real hardware: reads/writes, GPIO latch side effects, TIM1 reset.
 * Removing gate-first/gate-last ordering, bounds, break checks, or reversal
 * blanking must fail the corresponding test below. */
#include "bridge.h"
#include "hw_io.h"
#include <limits.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define A ((uintptr_t)0x50000000u)
#define B ((uintptr_t)0x50000400u)
#define T ((uintptr_t)0x40012c00u)
#define R ((uintptr_t)0x40021000u)
#define GATE 0x800u
#define PWM 0x100u
#define DIR 4u
#define MOE 0x8000u
#define AOE 0x4000u
#define BREAKS 0x180u
#define CHECK(c) do { if (!(c)) { fprintf(stderr, "%s:%d: FAIL %s\n", __func__, __LINE__, #c); return false; } } while (0)
struct reg { uintptr_t a; uint32_t v; };
struct event { uintptr_t a; uint32_t v; };
static struct reg regs[128];
static struct event trace[4096];
static size_t nr, nt;
static uint32_t forced_break, micros_now;
static uintptr_t rejected_read;
static bool activation_violation, unexpected_write, inject_break_at_gate;
static int expected_direction;
static uint16_t expected_cap;

static uint32_t get(uintptr_t a) {
    for (size_t i=0; i<nr; ++i) if (regs[i].a==a) return regs[i].v;
    return 0;
}
static void put(uintptr_t a, uint32_t v) {
    for (size_t i=0; i<nr; ++i) if (regs[i].a==a) { regs[i].v=v; return; }
    if (nr==128) abort();
    regs[nr++]=(struct reg){a,v};
}
uint32_t hw_read32(uintptr_t a) {
    uint32_t v=get(a);
    if (a==T+0x10) v|=forced_break;
    if (a==rejected_read) v^=1u;
    return v;
}
static bool allowed(uintptr_t a) {
    if (a==R+0x30 || a==R+0x34 || a==R+0x40) return true;
    if (a==A || a==A+4 || a==A+8 || a==A+12 || a==A+0x18 || a==A+0x20 || a==A+0x24 || a==A+0x28) return true;
    if (a==B || a==B+4 || a==B+8 || a==B+12 || a==B+0x18 || a==B+0x20 || a==B+0x28) return true;
    return a>=T && a<=T+0x68 && !(a&3u);
}
void hw_write32(uintptr_t a, uint32_t v) {
    if (nt==4096) abort();
    trace[nt++]=(struct event){a,v};
    if (!allowed(a)) unexpected_write=true;
    if (a==A+0x28 && (v&~(GATE|PWM))) unexpected_write=true;
    if (a==A+0x18 && (v&~GATE)) unexpected_write=true;
    if ((a==B+0x28 || a==B+0x18) && (v&~DIR)) unexpected_write=true;
    if (a==T+0x44 && (v&AOE)) activation_violation=true;
    if (a==A+0x18 && (v&GATE)) {
        bool ok=((get(B+0x14)&DIR)!=0)==(expected_direction<0);
        ok=ok && get(T+0x34)>0 && get(T+0x34)<=expected_cap;
        ok=ok && (get(T+0x44)&MOE) && !(get(T+0x44)&AOE);
        ok=ok && (get(T+0x20)&1u) && (get(T)&1u);
        ok=ok && ((get(A)>>16)&3u)==2u && (get(A+0x24)&15u)==2u;
        if (!ok) activation_violation=true;
        if (inject_break_at_gate) forced_break=0x80u;
    }
    put(a,v);
    if (a==A+0x18 || a==B+0x18) {
        uintptr_t p=a-0x18; put(p+0x14,(get(p+0x14)|(v&0xffffu))&~(v>>16));
    }
    if (a==A+0x28 || a==B+0x28) {
        uintptr_t p=a-0x28; put(p+0x14,get(p+0x14)&~v);
    }
    if (a==R+0x30 && (v&0x800u)) {
        for (uintptr_t x=T; x<=T+0x68; x+=4) put(x,0);
        put(T+0x60,1); put(T+0x64,1);
    }
}
uint32_t hw_micros(void) { return micros_now; }
void hw_relax(void) { ++micros_now; }
static void reset_mock(void) {
    nr=nt=0; forced_break=micros_now=0; rejected_read=0;
    activation_violation=unexpected_write=inject_break_at_gate=false;
    expected_direction=1; expected_cap=480;
    /* Seed unrelated GPIO/RCC bits so destructive whole-port writes fail. */
    put(A,0x6c3c5a5au); put(B,0x9b6f33c3u);
    put(A+4,0xa55au); put(B+4,0x5aa5u);
    put(A+8,0x12345678u); put(B+8,0x87654321u);
    put(A+12,0x56789abcu); put(B+12,0x2468ace0u);
    put(A+0x20,0x13579bdfu); put(A+0x24,0xfedcba98u);
    put(B+0x20,0x2468ace0u);
    put(R+0x34,0x24u); put(R+0x40,0x2000u); put(R+0x30,0x20u);
    put(A+0x14,0xa55au); put(B+0x14,0x5aa5u);
    put(T+0x44,0x0300f000u); put(T+0x20,1u); put(T,0x81u); put(T+0x34,250u);
}
static bool off(void) {
    return !(get(A+0x14)&GATE) && get(T+0x34)==0 && !(get(T+0x44)&(MOE|AOE)) && !(get(T+0x20)&1u);
}
static bool initialize(void) { reset_mock(); return bridge_init(); }
static bool test_early_off_orders_gate_before_modes_and_timer(void) {
    reset_mock();
    bridge_early_off();
    size_t gate=nt;
    for (size_t i=0; i<nt; ++i) {
        if (trace[i].a==A+0x28 && (trace[i].v&GATE)) { gate=i; break; }
    }
    CHECK(gate<nt);
    for (size_t i=0; i<nt; ++i) if (trace[i].a==A || (trace[i].a>=T && trace[i].a<=T+0x68)) CHECK(i>gate);
    CHECK(off()); CHECK(!unexpected_write); CHECK(!activation_violation);
    CHECK((get(R+0x34)&0x24u)==0x24u); CHECK((get(R+0x40)&0x2000u)!=0);
    return true;
}
static bool test_nonzero_before_init_fails_closed(void) {
    reset_mock(); bridge_early_off();
    CHECK(!bridge_set(1,1)); CHECK(off()); CHECK(bridge_applied()==0);
    CHECK(bridge_faults()&BRIDGE_FAULT_NOT_INITIALIZED);
    return true;
}
static bool test_init_preserves_ports_and_starts_disabled(void) {
    CHECK(initialize());
    CHECK(off()); CHECK(bridge_applied()==0); CHECK(bridge_faults()==0);
    CHECK(get(T+0x28)==3 && get(T+0x2c)==479);
    CHECK(get(T+0x18)==0x68 && get(T+0x44)==0x03003000u);
    CHECK(get(T+0x60)==1 && get(T+0x64)==1);
    CHECK((get(A)&~((3u<<16)|(3u<<22)))==(0x6c3c5a5au&~((3u<<16)|(3u<<22))));
    CHECK((get(B)&~(3u<<4))==(0x9b6f33c3u&~(3u<<4)));
    CHECK((get(A+4)&~(PWM|GATE))==(0xa55au&~(PWM|GATE)));
    CHECK((get(B+4)&~DIR)==(0x5aa5u&~DIR));
    CHECK((get(A+8)&~((3u<<16)|(3u<<22)))==(0x12345678u&~((3u<<16)|(3u<<22))));
    CHECK((get(B+8)&~(3u<<4))==(0x87654321u&~(3u<<4)));
    CHECK((get(A+12)&~((3u<<16)|(3u<<22)))==(0x56789abcu&~((3u<<16)|(3u<<22))));
    CHECK((get(B+12)&~(3u<<4))==(0x2468ace0u&~(3u<<4)));
    CHECK((get(A+0x24)&~15u)==(0xfedcba98u&~15u));
    CHECK(get(A+0x20)==0x13579bdfu && get(B+0x20)==0x2468ace0u);
    CHECK((get(R+0x30)&0x20u)!=0 && !(get(R+0x30)&0x800u));
    CHECK(!unexpected_write); CHECK(!activation_violation);
    return true;
}
static bool test_signed_commands_are_bounded_and_gate_last(void) {
    const struct { int16_t ticks; uint16_t cap; int dir; } cases[]={{1,1,1},{120,150,1},{480,480,1},{-1,1,-1},{-237,240,-1},{-480,480,-1}};
    for (size_t i=0; i<sizeof(cases)/sizeof(cases[0]); ++i) {
        CHECK(initialize()); expected_direction=cases[i].dir; expected_cap=cases[i].cap;
        CHECK(bridge_set(cases[i].ticks,cases[i].cap));
        CHECK(bridge_applied()==cases[i].ticks);
        CHECK(get(T+0x34)==(uint32_t)abs(cases[i].ticks));
        CHECK((get(A+0x14)&GATE)!=0); CHECK(!activation_violation); CHECK(!unexpected_write);
        bridge_disable(); CHECK(off()); CHECK(bridge_applied()==0);
    }
    return true;
}
static bool test_zero_is_off_even_with_zero_cap(void) {
    CHECK(initialize()); CHECK(bridge_set(150,200));
    nt=0; CHECK(bridge_set(0,0)); CHECK(off()); CHECK(bridge_applied()==0);
    CHECK(trace[0].a==A+0x28 && (trace[0].v&GATE));
    CHECK(!activation_violation);
    return true;
}
static bool test_invalid_requests_fail_closed_including_int16_min(void) {
    const struct { int16_t ticks; uint16_t cap; } bad[]={{481,480},{-481,480},{INT16_MIN,480},{1,0},{6,5},{-6,5},{0,481},{0,UINT16_MAX}};
    for (size_t i=0; i<sizeof(bad)/sizeof(bad[0]); ++i) {
        CHECK(initialize()); CHECK(bridge_set(5,5));
        CHECK(!bridge_set(bad[i].ticks,bad[i].cap)); CHECK(off()); CHECK(bridge_applied()==0);
        CHECK(bridge_faults()&BRIDGE_FAULT_BAD_REQUEST);
        CHECK(!bridge_set(1,1)); CHECK(off());
    }
    return true;
}
static bool test_fast_reversal_drives_low_before_phase_flip(void) {
    CHECK(initialize()); bridge_set_fast_reversal(true); CHECK(bridge_set(120,150));
    size_t mark=nt; expected_direction=-1; expected_cap=150;
    CHECK(bridge_set(-120,150)); CHECK(bridge_applied()==-120); CHECK(bridge_faults()==0);
    CHECK((get(B+0x14)&DIR)!=0); CHECK(get(T+0x34)==120); CHECK(get(A+0x14)&GATE);
    /* order: CCR=0, UG, then the direction write, then the new duty */
    size_t ccr0=nt,ug=nt,dir=nt,duty=nt;
    for (size_t i=mark;i<nt;++i){
        if(trace[i].a==T+0x34&&trace[i].v==0&&ccr0==nt)ccr0=i;
        else if(trace[i].a==T+0x14&&trace[i].v==1u&&ccr0!=nt&&ug==nt)ug=i;
        else if((trace[i].a==B+0x18||trace[i].a==B+0x28)&&(trace[i].v&DIR)&&ug!=nt&&dir==nt)dir=i;
        else if(trace[i].a==T+0x34&&trace[i].v==120&&dir!=nt)duty=i;
    }
    CHECK(ccr0<ug&&ug<dir&&dir<duty&&duty<nt);
    CHECK(bridge_set(120,150)); CHECK(bridge_applied()==120); CHECK(!(get(B+0x14)&DIR));
    CHECK(!activation_violation); CHECK(!unexpected_write);
    return true;
}
static bool test_reversal_blanks_one_call_without_latching_fault(void) {
    CHECK(initialize()); bridge_set_fast_reversal(false); CHECK(bridge_set(120,150));
    CHECK(!bridge_set(-120,150)); CHECK(off()); CHECK(bridge_applied()==0); CHECK(bridge_faults()==0);
    micros_now+=1000; expected_direction=-1; expected_cap=150;
    CHECK(bridge_set(-120,150)); CHECK(bridge_applied()==-120);
    CHECK(!bridge_set(120,150)); CHECK(off()); CHECK(bridge_faults()==0);
    CHECK(!activation_violation);
    bridge_set_fast_reversal(true);
    return true;
}
static bool test_break_flags_latch_and_never_enable(void) {
    const struct { uint32_t sr, fault; } flags[]={{0x80,BRIDGE_FAULT_BREAK},{0x100,BRIDGE_FAULT_BREAK2},{0x180,BRIDGE_FAULT_BREAK|BRIDGE_FAULT_BREAK2}};
    for (size_t i=0; i<sizeof(flags)/sizeof(flags[0]); ++i) {
        CHECK(initialize()); CHECK(bridge_set(50,100)); forced_break=flags[i].sr;
        CHECK(!bridge_set(50,100)); CHECK(off()); CHECK(bridge_applied()==0);
        CHECK((bridge_faults()&flags[i].fault)==flags[i].fault);
        forced_break=0; CHECK(!bridge_set(50,100)); CHECK(off());
        reset_mock(); forced_break=flags[i].sr; CHECK(!bridge_init()); CHECK(off());
    }
    CHECK(initialize()); CHECK(bridge_set(1,1)); forced_break=0x80;
    CHECK(bridge_faults()&BRIDGE_FAULT_BREAK); CHECK(off()); CHECK(bridge_applied()==0);
    return true;
}
static bool test_break_arriving_at_gate_write_is_observed_before_success(void) {
    CHECK(initialize()); inject_break_at_gate=true;
    CHECK(!bridge_set(100,100)); CHECK(off()); CHECK(bridge_applied()==0);
    CHECK(bridge_faults()&BRIDGE_FAULT_BREAK);
    return true;
}
static bool test_readback_failure_never_arms(void) {
    reset_mock(); rejected_read=T+0x2c;
    CHECK(!bridge_init()); CHECK(off()); CHECK(bridge_faults()&BRIDGE_FAULT_READBACK);
    CHECK(initialize()); rejected_read=T+0x34;
    CHECK(!bridge_set(1,1)); CHECK(off()); CHECK(bridge_faults()&BRIDGE_FAULT_READBACK);
    return true;
}
static bool test_early_off_works_after_active_without_resetting_ram(void) {
    CHECK(initialize()); CHECK(bridge_set(120,150)); bridge_early_off(); CHECK(off());
    /* No claim about the pre-BSS RAM mirror; only physical disable is required. */
    return true;
}
int main(void) {
    const struct { const char *name; bool (*fn)(void); } cases[]={
        {"early_off_gate_first",test_early_off_orders_gate_before_modes_and_timer},
        {"reject_before_init",test_nonzero_before_init_fails_closed},
        {"init_preserves_ports_and_disabled",test_init_preserves_ports_and_starts_disabled},
        {"signed_commands_and_gate_last",test_signed_commands_are_bounded_and_gate_last},
        {"zero_off",test_zero_is_off_even_with_zero_cap},
        {"invalid_bounds",test_invalid_requests_fail_closed_including_int16_min},
        {"reversal_blank",test_reversal_blanks_one_call_without_latching_fault},
        {"fast_reversal",test_fast_reversal_drives_low_before_phase_flip},
        {"break_fail_closed",test_break_flags_latch_and_never_enable},
        {"break_during_arm",test_break_arriving_at_gate_write_is_observed_before_success},
        {"readback_fail_closed",test_readback_failure_never_arms},
        {"early_off_without_ram_reset",test_early_off_works_after_active_without_resetting_ram}
    };
    unsigned failed=0;
    for (size_t i=0; i<sizeof(cases)/sizeof(cases[0]); ++i) {
        bool ok=cases[i].fn(); printf("%s %s\n",ok?"PASS":"FAIL",cases[i].name); if (!ok) ++failed;
    }
    printf("bridge tests: %zu cases, %u failed\n",sizeof(cases)/sizeof(cases[0]),failed);
    return failed?1:0;
}
