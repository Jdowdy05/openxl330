/* Offline MMIO fixture. No host device files, serial, or physical MMIO. */
#include "sensing.h"
#include "hw_io.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#define RCC 0x40021000u
#define GA 0x50000000u
#define GB 0x50000400u
#define I2C 0x40005400u
/* Independent ST CMSIS I2C_TypeDef map: ISR18, ICR1C, RXDR24, TXDR28.
 * Unlike ADC_ISR, I2C_ISR does not accept W1C flag clearing. */
#define I2C_ISR (I2C+0x18u)
#define I2C_ICR (I2C+0x1cu)
#define ADC 0x40012400u
#define CCR 0x40012708u
#define SYSCFG 0x40010000u
#ifndef SENSING_FAST_I2C
#define SENSING_FAST_I2C 0
#endif
#if SENSING_FAST_I2C
#define I2C_BYTE_US 12u
#define TEST_ENCODER_LIMIT 130u
#define TEST_STOP_ELAPSED 110u
#else
#define I2C_BYTE_US 24u
#define TEST_ENCODER_LIMIT 225u
#define TEST_STOP_ELAPSED 170u
#endif
static struct { uintptr_t a; uint32_t v; } regs[64];
static unsigned nr, failures, tests, illegal, encoder_reads, adc_starts, stops, resets, adc_mmio;
static uint32_t now, due, adc_due, injected_i2c, adc_conversion_us;
static uint16_t encoder_word, adc_raw[14], conf_word;
static uint8_t encoder_status=0x20u;
static uint8_t pointer, config_high;
static unsigned config_tx, config_writes;
static bool reject_config_write, corrupt_first_config_readback, reject_i2c_timing;
static unsigned phase, channel, selected[16], ns;
static unsigned fault_phase, stall_phase, adc_stall_channel, jump_i2c_phase;
static uint32_t phase_fault, jump_i2c_us, jump_adc_us, jump_adc_dr_us;
static bool i2c_stall, stuck_bus, adc_stall, cal_stall, ready_stall, overrun;
static bool channel_ack_stall, stop_ack_stall;
static unsigned isr_writes, icr_writes, nack_clears;
static uint32_t stop_delay_us;
static const char *test_name;

#define CHECK(c) do { if (!(c)) { fprintf(stderr,"FAIL %s:%d: %s\n",test_name,__LINE__,#c); failures++; return; } } while (0)
static uint32_t get(uintptr_t a) { for(unsigned i=0;i<nr;i++) if(regs[i].a==a) return regs[i].v; return 0; }
static void put(uintptr_t a,uint32_t v) { for(unsigned i=0;i<nr;i++) if(regs[i].a==a) {regs[i].v=v;return;} if(nr==64) abort(); regs[nr].a=a;regs[nr++].v=v; }
static bool allowed(uintptr_t a) {
    static const uintptr_t list[]={RCC+0x34,RCC+0x3c,RCC+0x40,RCC+0x2c,RCC+0x30,RCC+0x54,
        GA,GA+0xc,GB,GB+4,GB+8,GB+0xc,GB+0x20,
        I2C,I2C+4,I2C+0x10,I2C_ICR,I2C+0x28,
        SYSCFG,ADC,ADC+4,ADC+8,ADC+0xc,ADC+0x10,ADC+0x14,ADC+0x28,CCR};
    for(unsigned i=0;i<sizeof list/sizeof *list;i++) if(a==list[i]) return true;
    return false;
}
uint32_t hw_micros(void) { return now; }
void hw_relax(void) { now++; }
uint32_t hw_read32(uintptr_t a) {
    if(a>=ADC && a<=ADC+0x40u)++adc_mmio;
    if(a==I2C_ISR) {
        if(phase==jump_i2c_phase && jump_i2c_us) {now+=jump_i2c_us;jump_i2c_us=0;}
        if(phase==fault_phase) injected_i2c|=phase_fault;
        /* STOPF is sticky even after the bus becomes idle. Only ICR.STOPCF
         * or peripheral reset clears it; starting the next transfer does not. */
        if(phase==5 && !i2c_stall && phase!=stall_phase && (int32_t)(now-due)>=0) {
            injected_i2c|=0x20u;phase=0;
        }
        uint32_t r=injected_i2c;
        if(stuck_bus || phase) r|=0x8000;
        if(!i2c_stall && phase!=stall_phase && (int32_t)(now-due)>=0) {
            if(phase==1 || phase==6) r|=2;
            if(phase==2) r|=0x40;
            if(phase==3 || phase==4 || phase==7 || phase==8) r|=4;
        }
        return r;
    }
    if(a==I2C+0x24) {
        encoder_reads++;
        if(phase==7) {phase=3;due=now+I2C_BYTE_US;return encoder_status;}
        if(phase==8) {phase=5;due=now+stop_delay_us;return encoder_status;}
        if(phase==3) {phase=4;due=now+I2C_BYTE_US;return (pointer==7?(conf_word^((corrupt_first_config_readback && config_writes==1)?0x100u:0u)):encoder_word)>>8;}
        if(phase==4) {phase=5;due=now+stop_delay_us;return (pointer==7?conf_word:encoder_word)&255;}
        illegal++;return 0;
    }
    if(a==ADC+8) {
        uint32_t v=get(a);
        if((v&0x80000000u) && !cal_stall) {v&=~0x80000000u;put(a,v);}
        if((v&4) && !adc_stall && channel!=adc_stall_channel && (int32_t)(now-adc_due)>=0) {
            v&=~4u;put(a,v);put(ADC,get(ADC)|4u|(overrun?16u:0u));put(ADC+0x40,adc_raw[channel]);
        }
        return v;
    }
    if(a==ADC) {
        if(channel==9 && (get(ADC+8)&4) && jump_adc_us) {now+=jump_adc_us;jump_adc_us=0;}
        (void)hw_read32(ADC+8); return get(a);
    }
    if(a==ADC+0x40) {if(jump_adc_dr_us){now+=jump_adc_dr_us;jump_adc_dr_us=0;}uint32_t v=get(a);put(ADC,get(ADC)&~4u);return v;}
    return get(a);
}
void hw_write32(uintptr_t a,uint32_t v) {
    if(a>=ADC && a<=ADC+0x40u)++adc_mmio;
    if(!allowed(a)) illegal++;
    if(a==SYSCFG && !(get(RCC+0x40)&1u))illegal++;
    if(a==I2C+0x10 && reject_i2c_timing)return;
    if(a==I2C_ISR) {isr_writes++;return;} /* Rejected write: no hardware effect. */
    if(a==I2C_ICR) {
        icr_writes++;if(v&~0x3f38u) illegal++;
        if((v&0x10u) && (injected_i2c&0x10u)) nack_clears++;
        injected_i2c&=~v;return;
    }
    if(a==RCC+0x2c && (v&0x200000u)) {phase=0;injected_i2c=0;put(I2C+4,0);resets++;}
    if(a==RCC+0x30 && (v&0x100000u)) {put(ADC+8,0);put(ADC,0);}
    if(a==I2C+4 && (v&0x4000)) {stops++;if(!stuck_bus) {phase=5;due=now+1;}put(a,v);return;}
    if(a==I2C+4 && (v&0x2000)) {
        if(v==0x0001206cu) {phase=1;due=now+I2C_BYTE_US;}
        else if(v==0x0202246cu && phase==2) {phase=3;due=now+2u*I2C_BYTE_US;}
        else if(v==0x0203246cu && phase==2 && pointer==0x0b) {phase=7;due=now+I2C_BYTE_US;}
        else if(v==0x0201246cu && phase==2 && pointer==0x0b) {phase=8;due=now+I2C_BYTE_US;}
        else if(v==0x0203206cu) {phase=6;config_tx=0;due=now+I2C_BYTE_US;}
        else illegal++;
    }
    if(a==I2C+0x28) {
        if(phase==6) {
            if(config_tx==0) {if(v!=7u)illegal++;}
            else if(config_tx==1) config_high=(uint8_t)v;
            else if(config_tx==2) {
                if(!reject_config_write) conf_word=(uint16_t)((config_high<<8)|v);
                ++config_writes;phase=5;
            } else illegal++;
            ++config_tx;due=now+I2C_BYTE_US;
        } else {
            if((v!=0x0c && v!=7u && v!=0x0b) || phase!=1) illegal++;
            pointer=(uint8_t)v;phase=2;due=now+I2C_BYTE_US;
        }
    }
    if(a==ADC) {put(a,get(a)&~v);return;}
    if(a==ADC+0x28) {
        if(v==0 || (v&(v-1)) || (v&~0x2203u)) illegal++;
        for(channel=0;channel<14 && !(v&(1u<<channel));channel++) {}
        if(!channel_ack_stall) put(ADC,get(ADC)|0x2000u); /* CCRDY after accepted channel selection. */
    }
    if(a==ADC+8) {
        if(v&1u) {if(!ready_stall) put(ADC,get(ADC)|1u);}
        if(v&4u) {if(ns<16)selected[ns++]=channel;adc_starts++;adc_due=now+adc_conversion_us;}
        if((v&16u) && !stop_ack_stall) v&=~(4u|16u); /* ADSTP acknowledges stop. */
    }
    put(a,v);
}
static void fixture(void) {
    nr=0;illegal=0;adc_mmio=0;encoder_reads=0;adc_starts=0;stops=0;resets=0;now=100;due=0;adc_due=0;adc_conversion_us=6u;
    phase=0;channel=0;ns=0;injected_i2c=0;encoder_word=0x0abc;
    conf_word=0xedbdu;pointer=0;config_high=0;config_tx=config_writes=0;reject_config_write=corrupt_first_config_readback=reject_i2c_timing=false;
    fault_phase=99;stall_phase=99;adc_stall_channel=99;jump_i2c_phase=99;
    phase_fault=0;jump_i2c_us=0;jump_adc_us=0;jump_adc_dr_us=0;
    i2c_stall=false;stuck_bus=false;adc_stall=false;cal_stall=false;ready_stall=false;overrun=false;
    channel_ack_stall=false;stop_ack_stall=false;
    isr_writes=0;icr_writes=0;nack_clears=0;stop_delay_us=3;
    memset(adc_raw,0,sizeof adc_raw);adc_raw[13]=1500;adc_raw[0]=1800;adc_raw[1]=600;adc_raw[9]=512;
    put(GA,0xa55a5aa5u);put(GA+0xc,0x5aa5a55au);
    put(GB,0xa55a5aa5u);put(GB+4,0x12345678u);put(GB+8,0x87654321u);
    put(GB+0xc,0x5aa5a55au);put(GB+0x20,0x12345678u);
    put(RCC+0x34,0x400);put(RCC+0x3c,0x80);put(RCC+0x40,0x100);
    put(SYSCFG,0x00a40055u);
    put(RCC+0x2c,0x80);put(RCC+0x30,0x100);put(RCC+0x54,0x1234abcd);
}
static bool zero_samples(const sensing_adc *s) {
    return s->raw_current==0 && s->raw_voltage==0 && s->raw_temperature==0 && s->raw_reference==0 &&
        s->current_magnitude_ma==0 && s->voltage_decivolts==0 && s->temperature_c==0 && s->timestamp_us==0;
}
static void test_reject_before_init(void) {
    fixture();uint16_t raw=99;sensing_adc s;
    CHECK(!sensing_read_encoder(&raw));CHECK(raw==0);CHECK(sensing_last_faults()&SENSING_FAULT_NOT_INITIALIZED);
    CHECK(!sensing_read_adc(&s));CHECK(zero_samples(&s));CHECK(s.faults&SENSING_FAULT_NOT_INITIALIZED);
    CHECK(get(RCC+0x34)==0x400);CHECK(illegal==0);
}
static void test_init_pin_isolation(void) {
    fixture();uint32_t start=now;CHECK(sensing_init());
    CHECK(now-start>=34u); /* regulator20us + calibration gap2us + VREF startup12us */
    CHECK((get(GA)&~15u)==(0xa55a5aa5u&~15u));CHECK((get(GA)&15u)==15u);
    CHECK((get(GA+0xc)&~15u)==(0x5aa5a55au&~15u));CHECK((get(GA+0xc)&15u)==0);
    CHECK((get(GB)&~0xf00cu)==(0xa55a5aa5u&~0xf00cu));CHECK((get(GB)&0xf00cu)==0xa00cu);
    CHECK((get(GB+4)&~0xc0u)==(0x12345678u&~0xc0u));CHECK((get(GB+4)&0xc0u)==0xc0u);
    CHECK((get(GB+8)&~0xf000u)==(0x87654321u&~0xf000u));CHECK((get(GB+8)&0xf000u)==0xa000u);
    CHECK((get(GB+0xc)&~0xf00cu)==(0x5aa5a55au&~0xf00cu));CHECK((get(GB+0xc)&0xf00cu)==0x5000u);
    CHECK(get(GB+0x20)==0x66345678u);CHECK(get(RCC+0x34)==0x403u);
    CHECK(get(RCC+0x3c)==0x200080u);
#if SENSING_FAST_I2C
    CHECK(get(RCC+0x40)==0x100101u);CHECK(get(RCC+0x54)==0x12348bcdu);
    CHECK(get(SYSCFG)==0x00a70055u);CHECK(get(I2C+0x10)==0x00c51328u);CHECK(get(I2C)==0x1101u);
#else
    CHECK(get(RCC+0x40)==0x100100u);CHECK(get(RCC+0x54)==0x1234abcdu);
    CHECK(get(SYSCFG)==0x00a40055u);CHECK(get(I2C+0x10)==0x00620d15u);CHECK(get(I2C)==1u);
#endif
    CHECK(get(RCC+0x2c)==0x80);CHECK(get(RCC+0x30)==0x100);
    CHECK(get(ADC+0xc)==0);CHECK(get(ADC+0x10)==0x80000000u);CHECK(get(ADC+0x14)==0x66);
    CHECK((get(CCR)&0x400000u)!=0);CHECK(illegal==0);
}
static void test_encoder_byte_order_and_mask(void) {
    fixture();CHECK(sensing_init());uint16_t raw=99;uint32_t start=now;
    encoder_word=0xfabc;CHECK(sensing_read_encoder(&raw));CHECK(raw==0xabc);CHECK(encoder_reads==2);
    CHECK(now-start<200);CHECK(phase==0);CHECK(sensing_last_faults()==0);CHECK(illegal==0);
}
static void test_encoder_invalid_ff(void) {
    fixture();CHECK(sensing_init());encoder_word=0xffff;uint16_t raw=99;
    CHECK(!sensing_read_encoder(&raw));CHECK(raw==0);CHECK(sensing_last_faults()&SENSING_FAULT_ENCODER_INVALID);CHECK(phase==0);
}
static void test_encoder_status_burst(void) {
    fixture();CHECK(sensing_init());uint16_t raw=0;uint8_t st=0;
    encoder_word=0x0abc;encoder_status=0x20;CHECK(sensing_read_encoder_status(&raw,&st));CHECK(raw==0x0abc&&st==0x20);CHECK(illegal==0);CHECK(phase==0);
    encoder_status=0x08;encoder_word=0x0123;CHECK(sensing_read_encoder_status(&raw,&st));CHECK(raw==0x0123&&st==0x08);
    encoder_status=0xff;encoder_word=0xffff;CHECK(!sensing_read_encoder_status(&raw,&st));CHECK(sensing_last_faults()&SENSING_FAULT_ENCODER_INVALID);
    encoder_status=0x20;encoder_word=0x0abc;CHECK(sensing_read_encoder(&raw)&&raw==0xabc);CHECK(illegal==0);
    encoder_status=0x28;CHECK(sensing_read_encoder_status_byte(&st)&&st==0x28);CHECK(phase==0&&illegal==0);
}
static void test_adc_values_and_sequence(void) {
    fixture();CHECK(sensing_init());sensing_adc s;uint32_t start=now;
    CHECK(sensing_read_adc(&s));CHECK(s.raw_reference==1500);CHECK(s.raw_temperature==1800);
    CHECK(s.raw_voltage==600);CHECK(s.raw_current==512);CHECK(s.current_magnitude_ma==825);
    CHECK(s.voltage_decivolts==37);CHECK(s.temperature_c==26);CHECK(s.faults==0);
    CHECK(s.timestamp_us==now);CHECK(now-start<250);CHECK(ns==4);
    CHECK(selected[0]==13 && selected[1]==0 && selected[2]==1 && selected[3]==9);CHECK(illegal==0);
}
/* The table is the NTC model of tests/make_temperature_lut.py. Pinned here: the hot cap, the 65 C trip boundary
 * (raw 663 -> 65 trips at the default limit, 664 -> 64 does not), room temperature and the cold clamp. */
static void test_temperature_bounds(void) {
    static const struct {uint16_t raw;uint8_t want;} cases[]={{0,125},{8,125},{32,125},{647,66},{648,65},{663,65},{664,64},
                                                             {1800,26},{1848,25},{2423,11},{2424,10},{3000,10},{4095,10}};
    fixture();CHECK(sensing_init());
    for(unsigned i=0;i<sizeof cases/sizeof *cases;i++) {sensing_adc s;adc_raw[0]=cases[i].raw;CHECK(sensing_read_adc(&s));CHECK(s.temperature_c==cases[i].want);}
}
static void test_zero_reference_invalidates_entire_output(void) {
    fixture();CHECK(sensing_init());sensing_adc s;CHECK(sensing_read_adc(&s));adc_raw[13]=0;
    CHECK(!sensing_read_adc(&s));CHECK(zero_samples(&s));CHECK(s.faults&SENSING_FAULT_ADC_REFERENCE);
}
static void test_encoder_timeout_cleanup_recovery(void) {
    fixture();CHECK(sensing_init());i2c_stall=true;uint32_t start=now;uint16_t raw=77;
    CHECK(!sensing_read_encoder(&raw));CHECK(raw==0);CHECK(sensing_last_faults()&SENSING_FAULT_I2C_TIMEOUT);
    CHECK(now-start<=TEST_ENCODER_LIMIT);CHECK(stops>0);CHECK(resets>=2);CHECK((get(I2C)&1)!=0);CHECK(illegal==0);
    i2c_stall=false;CHECK(sensing_read_encoder(&raw));CHECK(raw==0xabc);
}
static void test_i2c_error_flags_and_recovery(void) {
    static const struct {uint32_t flags,want;} cases[]={{0x10,SENSING_FAULT_I2C_NACK},{0x100,SENSING_FAULT_I2C_BUS},{0x200,SENSING_FAULT_I2C_BUS},{0x400,SENSING_FAULT_I2C_BUS},{0x1000,SENSING_FAULT_I2C_BUS}};
    for(unsigned i=0;i<sizeof cases/sizeof *cases;i++) {
        fixture();CHECK(sensing_init());injected_i2c=cases[i].flags;uint16_t raw=88;uint32_t start=now;
        CHECK(!sensing_read_encoder(&raw));CHECK(raw==0);CHECK(sensing_last_faults()&cases[i].want);
        CHECK(injected_i2c==0);CHECK(now-start<=TEST_ENCODER_LIMIT);CHECK(phase==0);CHECK(illegal==0);
        CHECK(sensing_read_encoder(&raw));CHECK(raw==0xabc);
    }
}
static void test_stuck_bus_deadline(void) {
    fixture();CHECK(sensing_init());stuck_bus=true;uint16_t raw=88;uint32_t start=now;
    CHECK(!sensing_read_encoder(&raw));CHECK(raw==0);CHECK(sensing_last_faults()&SENSING_FAULT_I2C_STUCK);
    CHECK(now-start<=TEST_ENCODER_LIMIT);CHECK(stops>0);CHECK(resets>=2);CHECK(illegal==0);
}
static void test_adc_timeout_no_stale_output(void) {
    fixture();CHECK(sensing_init());sensing_adc s;CHECK(sensing_read_adc(&s));adc_stall=true;uint32_t start=now;
    CHECK(!sensing_read_adc(&s));CHECK(zero_samples(&s));CHECK(s.faults&SENSING_FAULT_ADC_TIMEOUT);
    CHECK(now-start<=160);CHECK((get(ADC+8)&4)==0);CHECK(illegal==0);
}
static void test_adc_overrun_rejected(void) {
    fixture();CHECK(sensing_init());overrun=true;sensing_adc s;
    CHECK(!sensing_read_adc(&s));CHECK(zero_samples(&s));CHECK(s.faults&SENSING_FAULT_ADC_OVERRUN);
}
static void test_init_ack_timeouts(void) {
    fixture();cal_stall=true;uint32_t start=now;CHECK(!sensing_init());CHECK(now-start<=2200);
    CHECK(sensing_last_faults()&SENSING_FAULT_ADC_INIT);uint16_t raw=8;CHECK(!sensing_read_encoder(&raw));CHECK(raw==0);
    fixture();ready_stall=true;start=now;CHECK(!sensing_init());CHECK(now-start<=2200);CHECK(sensing_last_faults()&SENSING_FAULT_ADC_INIT);
}
static void test_time_wrap(void) {
    fixture();CHECK(sensing_init());now=0xffffffc0u;uint16_t raw;CHECK(sensing_read_encoder(&raw));CHECK(raw==0xabc);
    now=0xfffffff0u;sensing_adc s;CHECK(sensing_read_adc(&s));CHECK(s.raw_current==512);CHECK(s.timestamp_us==now);
}
static void test_argument_faults(void) {
    fixture();CHECK(sensing_init());CHECK(!sensing_read_encoder(NULL));CHECK(sensing_last_faults()&SENSING_FAULT_ARGUMENT);
    CHECK(!sensing_read_adc(NULL));CHECK(sensing_last_faults()&SENSING_FAULT_ARGUMENT);
}
/* fast17 (review F-7a, a deferred item; the v1.1 plan): the wait checks errors, then completion, then the deadline, so a
 * transfer that completed while the wait was preempted past its budget (1000 us clock jump) is accepted, not a timeout.
 * Before fast17 this test required the opposite (no late success). A bus that never completes still times out
 * (test_stuck_bus_deadline). */
static void test_no_late_success_after_clock_jump(void) {
    fixture();CHECK(sensing_init());jump_i2c_phase=5;jump_i2c_us=1000;uint16_t raw=77;
    CHECK(sensing_read_encoder(&raw));CHECK(raw==0xabc);CHECK(!(sensing_last_faults()&SENSING_FAULT_I2C_TIMEOUT));
}
static void test_no_late_adc_success_after_clock_jump(void) {
    fixture();CHECK(sensing_init());jump_adc_us=300;sensing_adc s;
    CHECK(!sensing_read_adc(&s));CHECK(zero_samples(&s));CHECK(s.faults&SENSING_FAULT_ADC_TIMEOUT);
}
static void test_mid_transaction_errors_never_publish_partial_data(void) {
    for(unsigned p=1;p<=5;p++) {
        fixture();CHECK(sensing_init());fault_phase=p;phase_fault=0x10;uint16_t raw=77;
        CHECK(!sensing_read_encoder(&raw));CHECK(raw==0);CHECK(sensing_last_faults()&SENSING_FAULT_I2C_NACK);
        CHECK(phase==0);CHECK(injected_i2c==0);CHECK(illegal==0);
        fault_phase=99;CHECK(sensing_read_encoder(&raw));CHECK(raw==0xabc);
    }
    fixture();CHECK(sensing_init());adc_stall_channel=9;sensing_adc s;uint32_t start=now;
    CHECK(!sensing_read_adc(&s));CHECK(zero_samples(&s));CHECK(s.faults&SENSING_FAULT_ADC_TIMEOUT);CHECK(now-start<=160u);
}
static void test_full_scale_and_small_reference_saturate(void) {
    fixture();CHECK(sensing_init());adc_raw[9]=4095;adc_raw[1]=4095;adc_raw[13]=1;sensing_adc s;
    CHECK(sensing_read_adc(&s));CHECK(s.raw_current==4095);CHECK(s.current_magnitude_ma==6598);
    CHECK(s.voltage_decivolts==65535);
}
static void test_adc_channel_and_stop_ack_deadlines(void) {
    fixture();CHECK(sensing_init());channel_ack_stall=true;sensing_adc s;uint32_t start=now;
    CHECK(!sensing_read_adc(&s));CHECK(zero_samples(&s));CHECK(s.faults&SENSING_FAULT_ADC_TIMEOUT);
    CHECK(adc_starts==0);CHECK(now-start<=160);
    fixture();CHECK(sensing_init());adc_stall=true;stop_ack_stall=true;start=now;
    CHECK(!sensing_read_adc(&s));CHECK(zero_samples(&s));CHECK(now-start<=160);
    CHECK((get(ADC+8)&5)==0);CHECK(!sensing_read_adc(&s));CHECK(s.faults&SENSING_FAULT_NOT_INITIALIZED);
    uint16_t raw;CHECK(sensing_read_encoder(&raw));CHECK(raw==0xabc);CHECK(illegal==0);
}
static void test_combined_nominal_budget(void) {
    fixture();CHECK(sensing_init());uint32_t start=now;uint16_t raw;sensing_adc s;
    CHECK(sensing_read_encoder(&raw));CHECK(sensing_read_adc(&s));CHECK(now-start<200);
    printf("Nominal fixture encoder+ADC elapsed: %u us\n",now-start);
}
static void test_i2c_w1c_uses_dedicated_icr(void) {
    fixture();CHECK(sensing_init());CHECK(isr_writes==0);CHECK(icr_writes>0);CHECK(illegal==0);
    uint16_t raw;CHECK(sensing_read_encoder(&raw));CHECK(injected_i2c==0);CHECK(isr_writes==0);
}
static void test_second_transaction_waits_for_its_stop(void) {
    fixture();CHECK(sensing_init());uint16_t raw;CHECK(sensing_read_encoder(&raw));
    stop_delay_us=50;uint32_t start=now;
    CHECK(sensing_read_encoder(&raw));CHECK(raw==0xabc);CHECK(now-start>=TEST_STOP_ELAPSED);
    CHECK(phase==0);CHECK(injected_i2c==0);CHECK(isr_writes==0);CHECK(illegal==0);
}
static void test_second_transaction_missing_stop_fails(void) {
    fixture();CHECK(sensing_init());uint16_t raw;CHECK(sensing_read_encoder(&raw));
    stop_delay_us=300;raw=77;uint32_t start=now;
    CHECK(!sensing_read_encoder(&raw));CHECK(raw==0);CHECK(sensing_last_faults()&SENSING_FAULT_I2C_TIMEOUT);
    CHECK(now-start<=TEST_ENCODER_LIMIT);CHECK(phase==0);CHECK(injected_i2c==0);CHECK(isr_writes==0);CHECK(illegal==0);
}
static void test_second_transaction_nack_clears_via_icr(void) {
    fixture();CHECK(sensing_init());uint16_t raw;CHECK(sensing_read_encoder(&raw));
    stop_delay_us=300;fault_phase=5;phase_fault=0x10;raw=77;
    CHECK(!sensing_read_encoder(&raw));CHECK(raw==0);CHECK(sensing_last_faults()&SENSING_FAULT_I2C_NACK);
    CHECK(nack_clears>0);CHECK(phase==0);CHECK(injected_i2c==0);CHECK(isr_writes==0);CHECK(illegal==0);
    fault_phase=99;stop_delay_us=3;CHECK(sensing_read_encoder(&raw));CHECK(raw==0xabc);
}
static void test_encoder_configuration_preserves_fields_without_otp(void) {
    fixture();CHECK(sensing_init());
    uint16_t before=0,after=0;
    CHECK(sensing_encoder_config(&before,&after));
    /* EDBD -> C3BC: preserve HYST=3, OUTS=3, PWMF=2 and reserved14/15. */
    CHECK(before==0xedbdu);CHECK(after==0xc3bcu);CHECK(conf_word==0xc3bcu);
    CHECK(config_writes==1);CHECK(phase==0);CHECK(illegal==0);CHECK(sensing_last_faults()==0);
    uint16_t raw=0;CHECK(sensing_read_encoder(&raw));CHECK(raw==0xabc);
}
static void test_encoder_configuration_rejects_mismatch_and_partial_failure(void) {
    fixture();CHECK(sensing_init());
    uint16_t before=0,after=0;reject_config_write=true;
    CHECK(!sensing_encoder_config(&before,&after));CHECK(before==0xedbdu);CHECK(after==0xedbdu);
    CHECK(sensing_last_faults()&(1u<<11));CHECK(illegal==0);
    fixture();CHECK(sensing_init());before=after=0xbeefu;i2c_stall=true;
    uint32_t start=now;CHECK(!sensing_encoder_config(&before,&after));
    CHECK(before==0 && after==0);CHECK(sensing_last_faults()&SENSING_FAULT_I2C_TIMEOUT);CHECK(now-start<=TEST_ENCODER_LIMIT);
    fixture();CHECK(sensing_init());CHECK(!sensing_encoder_config(NULL,&after));
    CHECK(sensing_last_faults()&SENSING_FAULT_ARGUMENT);CHECK(config_writes==0);
}
static void test_encoder_restore_original_is_verified_and_idempotent(void) {
    fixture();CHECK(sensing_init());
    uint16_t before=0,after=0;
    CHECK(sensing_encoder_config(&before,&after));CHECK(conf_word==0xc3bcu);
    /* A second configuration must not overwrite the first saved original. */
    CHECK(sensing_encoder_config(&before,&after));CHECK(before==0xc3bcu);
    CHECK(sensing_encoder_restore());CHECK(conf_word==0xedbdu);CHECK(sensing_last_faults()==0);
    unsigned writes=config_writes,reads=encoder_reads;
    CHECK(sensing_encoder_restore());CHECK(config_writes==writes && encoder_reads==reads);
}
static void test_encoder_restore_without_original_never_writes_zero(void) {
    fixture();CHECK(sensing_init());
    CHECK(sensing_encoder_restore());CHECK(config_writes==0 && encoder_reads==0);CHECK(conf_word==0xedbdu);
    i2c_stall=true;uint16_t before=99,after=99;
    CHECK(!sensing_encoder_config(&before,&after));CHECK(before==0 && after==0);
    CHECK(sensing_encoder_restore());CHECK(config_writes==0 && conf_word==0xedbdu);
}
static void test_failed_config_verification_rolls_back_saved_original(void) {
    fixture();CHECK(sensing_init());
    corrupt_first_config_readback=true;uint16_t before=0,after=0;
    CHECK(!sensing_encoder_config(&before,&after));
    CHECK(before==0xedbdu && after==0xc2bcu);CHECK(conf_word==0xedbdu);
    CHECK(config_writes==2);CHECK(sensing_last_faults()&SENSING_FAULT_ENCODER_CONFIG);
    unsigned writes=config_writes;CHECK(sensing_encoder_restore());CHECK(config_writes==writes);
}
static void test_failed_restore_retains_original_for_retry(void) {
    fixture();CHECK(sensing_init());
    uint16_t before=0,after=0;CHECK(sensing_encoder_config(&before,&after));
    reject_config_write=true;CHECK(!sensing_encoder_restore());
    CHECK(conf_word==0xc3bcu);CHECK(sensing_last_faults()&(1u<<12));
    reject_config_write=false;CHECK(sensing_encoder_restore());CHECK(conf_word==0xedbdu);
    CHECK(sensing_last_faults()==0);CHECK(illegal==0);
}
static void test_adc_finishes_within_revised_budget_under_interrupt_delay(void) {
    fixture();CHECK(sensing_init());adc_conversion_us=35u;sensing_adc s;uint32_t start=now;
    CHECK(sensing_read_adc(&s));CHECK(s.raw_current==512u && now-start==140u);
    CHECK(sensing_last_faults()==0u);
}
static void test_tick_cache_requires_seed_and_updates_one_channel_in_order(void) {
    fixture();CHECK(sensing_init());sensing_adc s;
    CHECK(!sensing_read_adc_tick(&s));CHECK(zero_samples(&s));CHECK(s.faults&SENSING_FAULT_NOT_INITIALIZED);CHECK(adc_starts==0u);
    CHECK(sensing_read_adc(&s));CHECK(adc_starts==4u);
    adc_raw[13]=1000;adc_raw[0]=1848;adc_raw[1]=1200;adc_raw[9]=1024;
    CHECK(sensing_read_adc_tick(&s));CHECK(adc_starts==5u && selected[4]==13u);
    CHECK(s.raw_reference==1000u && s.raw_temperature==1800u && s.raw_voltage==600u && s.raw_current==512u);
    CHECK(s.voltage_decivolts==56u && s.current_magnitude_ma==825u && s.temperature_c==26u);
    CHECK(sensing_read_adc_tick(&s));CHECK(adc_starts==6u && selected[5]==0u && s.temperature_c==25u);
    CHECK(sensing_read_adc_tick(&s));CHECK(adc_starts==7u && selected[6]==1u && s.voltage_decivolts==111u);
    CHECK(sensing_read_adc_tick(&s));CHECK(adc_starts==8u && selected[7]==9u && s.current_magnitude_ma==1650u);
    adc_raw[13]=1500;CHECK(sensing_read_adc_tick(&s));CHECK(selected[8]==13u && s.voltage_decivolts==74u);
    CHECK(s.timestamp_us==now && s.faults==0u && illegal==0u);
}
static void test_tick_failure_invalidates_seed_until_complete_full_read(void) {
    fixture();CHECK(sensing_init());sensing_adc s;CHECK(sensing_read_adc(&s));
    adc_stall=true;uint32_t start=now;CHECK(!sensing_read_adc_tick(&s));
    CHECK(zero_samples(&s));CHECK(s.faults&SENSING_FAULT_ADC_TIMEOUT);CHECK(now-start<=70u);
    adc_stall=false;unsigned starts=adc_starts;CHECK(!sensing_read_adc_tick(&s));
    CHECK(zero_samples(&s));CHECK(s.faults&SENSING_FAULT_NOT_INITIALIZED);CHECK(adc_starts==starts);
    CHECK(sensing_read_adc(&s));CHECK(sensing_read_adc_tick(&s));CHECK(selected[ns-1]==13u && s.raw_current==512u);
    adc_raw[13]=0;CHECK(!sensing_read_adc(&s));CHECK(s.faults&SENSING_FAULT_ADC_REFERENCE);
    CHECK(!sensing_read_adc_tick(&s));CHECK(zero_samples(&s));
}
static void test_tick_rejects_reference_zero_overrun_and_exact_deadline(void) {
    fixture();CHECK(sensing_init());sensing_adc s;CHECK(sensing_read_adc(&s));
    adc_raw[13]=0;CHECK(!sensing_read_adc_tick(&s));CHECK(zero_samples(&s));CHECK(s.faults&SENSING_FAULT_ADC_REFERENCE);
    adc_raw[13]=1500;CHECK(sensing_read_adc(&s));overrun=true;
    CHECK(!sensing_read_adc_tick(&s));CHECK(zero_samples(&s));CHECK(s.faults&SENSING_FAULT_ADC_OVERRUN);
    overrun=false;CHECK(sensing_read_adc(&s));adc_conversion_us=59u;
    CHECK(sensing_read_adc_tick(&s));CHECK(s.raw_reference==1500u);
    adc_conversion_us=60u;CHECK(!sensing_read_adc_tick(&s));CHECK(zero_samples(&s));CHECK(s.faults&SENSING_FAULT_ADC_TIMEOUT);
    CHECK(!sensing_read_adc_tick(&s));CHECK(s.faults&SENSING_FAULT_NOT_INITIALIZED);
}
static void test_tick_arguments_reinit_and_microsecond_wrap(void) {
    fixture();CHECK(sensing_init());sensing_adc s;CHECK(sensing_read_adc(&s));
    CHECK(!sensing_read_adc_tick(NULL));CHECK(sensing_last_faults()&SENSING_FAULT_ARGUMENT);
    CHECK(!sensing_read_adc_tick(&s));CHECK(zero_samples(&s));
    CHECK(sensing_read_adc(&s));now=0xfffffffcu;CHECK(sensing_read_adc_tick(&s));CHECK(s.timestamp_us==2u);
    CHECK(sensing_read_adc_tick(&s));CHECK(s.timestamp_us==8u && s.raw_temperature==1800u);
    CHECK(sensing_init());CHECK(!sensing_read_adc_tick(&s));CHECK(s.faults&SENSING_FAULT_NOT_INITIALIZED);
    CHECK(sensing_read_adc(&s));CHECK(!sensing_read_adc(NULL));
    CHECK(!sensing_read_adc_tick(&s));CHECK(zero_samples(&s));
}
static void test_tick_performs_one_conversion_and_less_than_half_full_mmio(void) {
    fixture();CHECK(sensing_init());sensing_adc s;
    uint32_t start=now;unsigned io_before=adc_mmio,starts_before=adc_starts;
    CHECK(sensing_read_adc(&s));uint32_t full_us=now-start;unsigned full_io=adc_mmio-io_before;
    CHECK(adc_starts-starts_before==4u && full_us==24u);
    start=now;io_before=adc_mmio;starts_before=adc_starts;
    CHECK(sensing_read_adc_tick(&s));uint32_t tick_us=now-start;unsigned tick_io=adc_mmio-io_before;
    CHECK(adc_starts-starts_before==1u && tick_us==6u);CHECK(tick_io*2u<full_io);
    printf("ADC fixture full:%uus/%u MMIO versus tick:%uus/%u MMIO (not hardware WCET)\n",full_us,full_io,tick_us,tick_io);
}
static void test_tick_never_publishes_conversion_read_after_its_deadline(void) {
    fixture();CHECK(sensing_init());sensing_adc s;CHECK(sensing_read_adc(&s));
    jump_adc_dr_us=61u;CHECK(!sensing_read_adc_tick(&s));CHECK(zero_samples(&s));
    CHECK(s.faults&SENSING_FAULT_ADC_TIMEOUT);CHECK(!sensing_read_adc_tick(&s));
    CHECK(s.faults&SENSING_FAULT_NOT_INITIALIZED);
}
static void test_tick_channel_and_stop_ack_failures_do_not_reuse_cache(void) {
    fixture();CHECK(sensing_init());sensing_adc s;CHECK(sensing_read_adc(&s));
    channel_ack_stall=true;unsigned starts=adc_starts;uint32_t start=now;
    CHECK(!sensing_read_adc_tick(&s));CHECK(zero_samples(&s) && adc_starts==starts);CHECK(now-start<=70u);
    channel_ack_stall=false;CHECK(sensing_read_adc(&s));adc_stall=true;stop_ack_stall=true;start=now;
    CHECK(!sensing_read_adc_tick(&s));CHECK(zero_samples(&s));CHECK(now-start<=70u);
    adc_stall=stop_ack_stall=false;CHECK(!sensing_read_adc_tick(&s));CHECK(s.faults&SENSING_FAULT_NOT_INITIALIZED);
    CHECK(!sensing_read_adc(&s));CHECK(s.faults&SENSING_FAULT_NOT_INITIALIZED);
}
static void test_profile_deadline_and_recovery_restore_matching_configuration(void) {
    fixture();CHECK(sensing_init());uint16_t raw=99;i2c_stall=true;uint32_t start=now;
    CHECK(!sensing_read_encoder(&raw));CHECK(raw==0 && (sensing_last_faults()&SENSING_FAULT_I2C_TIMEOUT));
    CHECK(now-start<=TEST_ENCODER_LIMIT);i2c_stall=false;
    CHECK(sensing_read_encoder(&raw));CHECK(raw==0xabcu);
#if SENSING_FAST_I2C
    CHECK(get(I2C)==0x1101u && get(I2C+0x10)==0x00c51328u);
    CHECK((get(RCC+0x54)&0x3000u)==0u && (get(SYSCFG)&0x30000u)==0x30000u);
#else
    CHECK(get(I2C)==1u && get(I2C+0x10)==0x00620d15u);
    CHECK((get(RCC+0x54)&0x3000u)==0x2000u && get(SYSCFG)==0x00a40055u);
#endif
    CHECK(get(ADC+0x10)==0x80000000u && illegal==0u);
}
#if SENSING_FAST_I2C
static void test_fast_timing_meets_documented_edge_and_clock_envelope(void) {
    fixture();CHECK(sensing_init());uint32_t timing=get(I2C+0x10);
    unsigned presc=(timing>>28)+1u,sdadel=(timing>>16)&15u,scldel=((timing>>20)&15u)+1u;
    unsigned low=(timing&255u)+1u,high=((timing>>8)&255u)+1u;
    double fastest=1e9/(64e6*1.03),slowest=1e9/(64e6*.97);
    CHECK(low*presc*fastest>=500.0);CHECK(high*presc*fastest>=260.0);
    CHECK(scldel*presc*fastest>=170.0);
    CHECK(sdadel*presc*fastest>=120.0-4.0*fastest);
    CHECK(sdadel*presc*slowest<=450.0-120.0-5.0*slowest);
    double minimum=((low+high)*presc+6u)*fastest;
    double maximum=((low+high)*presc+8u)*slowest+240.0;
    CHECK(minimum>=1000.0);
    printf("Fast-profile calculated period bounds: %.3f..%.3f ns; unmeasured edges\n",minimum,maximum);
}
static void test_fast_profile_rejects_unaccepted_timing_configuration(void) {
    fixture();reject_i2c_timing=true;CHECK(!sensing_init());
    CHECK(sensing_last_faults()&SENSING_FAULT_I2C_BUS);uint16_t raw=99;
    CHECK(!sensing_read_encoder(&raw));CHECK(raw==0u);
}
#endif
static void run(void (*fn)(void),const char *name) {test_name=name;tests++;unsigned old=failures;fn();if(old==failures)printf("PASS %s\n",name);}
int main(void) {
    run(test_reject_before_init,"sampling before initialization is rejected without hardware writes");
    run(test_encoder_configuration_preserves_fields_without_otp,"volatile encoder configuration preserves unrelated fields and restores angle pointer");
    run(test_encoder_configuration_rejects_mismatch_and_partial_failure,"configuration readback mismatch and timeout cannot report success");
    run(test_encoder_restore_original_is_verified_and_idempotent,"restore verifies first original after repeated configuration and is idempotent");
    run(test_encoder_restore_without_original_never_writes_zero,"restore without a captured original never writes zero");
    run(test_failed_config_verification_rolls_back_saved_original,"failed configuration verification attempts original restoration");
    run(test_failed_restore_retains_original_for_retry,"failed restore retains original for a successful retry");
    run(test_adc_finishes_within_revised_budget_under_interrupt_delay,"ADC transaction tolerates140us acquisition under interrupt delay");
    run(test_tick_cache_requires_seed_and_updates_one_channel_in_order,"one-channel tick requires seed and cycles13/0/1/9 with cached values");
    run(test_tick_failure_invalidates_seed_until_complete_full_read,"tick/full ADC failure invalidates cache until full reseed");
    run(test_tick_rejects_reference_zero_overrun_and_exact_deadline,"tick ADC rejects zero reference overrun and60us deadline");
    run(test_tick_arguments_reinit_and_microsecond_wrap,"tick cache handles null arguments reinit and32bit time wrap");
    run(test_tick_performs_one_conversion_and_less_than_half_full_mmio,"tick ADC reduces conversion and MMIO work");
    run(test_tick_never_publishes_conversion_read_after_its_deadline,"tick rejects a data-register read completed after60us deadline");
    run(test_tick_channel_and_stop_ack_failures_do_not_reuse_cache,"tick channel/stop failures clear cache and require reinitialization when needed");
    run(test_profile_deadline_and_recovery_restore_matching_configuration,"selected I2C profile deadline and recovery preserve clock/filter/drive configuration");
#if SENSING_FAST_I2C
    run(test_fast_timing_meets_documented_edge_and_clock_envelope,"fast timing meets120ns edge and3percent clock envelope");
    run(test_fast_profile_rejects_unaccepted_timing_configuration,"fast profile refuses unaccepted timing configuration");
#endif
    run(test_init_pin_isolation,"init preserves unrelated GPIO and RCC fields");
    run(test_encoder_byte_order_and_mask,"encoder reads exactly two big-endian bytes and masks12bits");
    run(test_encoder_invalid_ff,"encoder rejects all-FF without stale output");
    run(test_encoder_status_burst,"STATUS+RAW ANGLE 3-byte burst returns magnet bits and raw word");
    run(test_adc_values_and_sequence,"ADC raw values scale and publish in captured channel order");
    run(test_temperature_bounds,"temperature LUT (NTC model): hot cap, 65 C trip at raw <= 663, cold clamp before uint8 overflow");
    run(test_zero_reference_invalidates_entire_output,"zero reference invalidates outputs");
    run(test_encoder_timeout_cleanup_recovery,"encoder timeout cleans up and permits next read");
    run(test_i2c_error_flags_and_recovery,"I2C errors clean state and permit recovery");
    run(test_stuck_bus_deadline,"stuck bus exits inside deadline and resets peripheral");
    run(test_adc_timeout_no_stale_output,"ADC timeout stops conversion and clears stale outputs");
    run(test_adc_overrun_rejected,"ADC overrun invalidates sample");
    run(test_init_ack_timeouts,"calibration and ADRDY handshakes have deadlines");
    run(test_time_wrap,"deadlines work across32bit microsecond wrap");
    run(test_argument_faults,"null output rejects with argument fault");
    run(test_no_late_success_after_clock_jump,"F-7a: a transfer completed late (clock jump) is accepted");
    run(test_no_late_adc_success_after_clock_jump,"late ADC ready flags do not override sample deadline");
    run(test_mid_transaction_errors_never_publish_partial_data,"mid-transfer failures discard partial sensor data");
    run(test_full_scale_and_small_reference_saturate,"full-scale magnitude and low reference cannot wrap output");
    run(test_adc_channel_and_stop_ack_deadlines,"ADC channel configuration and stop acknowledgments have deadlines");
    run(test_combined_nominal_budget,"combined nominal encoder and ADC sampling stays below200us");
    run(test_i2c_w1c_uses_dedicated_icr,"I2C W1C writes target ICR while ISR rejects writes");
    run(test_second_transaction_waits_for_its_stop,"second transaction waits for its own delayed STOP");
    run(test_second_transaction_missing_stop_fails,"second transaction with missing STOP cannot reuse old STOPF");
    run(test_second_transaction_nack_clears_via_icr,"second transaction NACK is cleared through ICR before reset");
    printf("%u tests, %u failures; ASan/UBSan build\n",tests,failures);
    return failures?1:0;
}
