/* Real platform UART runtime, with only CPU/MMIO replaced by a register model.
 * The foreground can be unavailable for1ms while IRQ0 continues to service TX.
 * A peer header arrives100us after the final stop bit: TC handling must release
 * PC6 and enable RX in IRQ0, before foreground resumes. */
#include "platform.h"
#include "observability.h"
#include "app.h"
#include "hw_io.h"
#include "bridge.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <setjmp.h>
#define DMA ((uintptr_t)0x40020000u)
#define DMAMUX ((uintptr_t)0x40020800u)
#define R ((uintptr_t)0x40021000u)
#define U ((uintptr_t)0x40013800u)
#define C ((uintptr_t)0x50000800u)
#define S ((uintptr_t)0xe000e010u)
#define F ((uintptr_t)0x40022000u)
#define P ((uintptr_t)0x40007000u)
#define TXE 0x80u
#define TC 0x40u
#define RXNE 0x20u
#define IDLE 0x10u
#define RXFT (1u<<26)
#define RXFTIE (1u<<28)
#define RXFTCFG (7u<<25)
#define RXFT_HALF (2u<<25)
#define RE 4u
#define RXIE 0x20u
#define TXIE 0x80u
#define TCIE 0x40u
#define FIFOEN (1u<<29)
#define TXFT (1u<<27)
#define TXFTIE (1u<<23)
#define TXFTCFG (7u<<29)
#define TXFT_HALF (2u<<29)
#define CHECK(c) do { if (!(c)) { fprintf(stderr,"FAIL %s:%d %s\n",__func__,__LINE__,#c); return false; } } while (0)
struct entry { uintptr_t address; uint32_t value; };
static struct entry map[96]; static unsigned map_used;
static uint64_t now_us, shift_end, first_wire, final_stop, peer_due, rx_restore_us;
static uint32_t irq_mask, disabled;
static bool inside_irq, shift_busy, pause_foreground, peer_pending, peer_received, peer_lost, freeze_tx;
static uint8_t shift_byte, wire[640], tx_fifo[8], rx_fifo[8];
static unsigned wire_n, expected_n, tx_head, tx_count, rx_fifo_head, rx_count;
static uint32_t reg(uintptr_t);
static void set(uintptr_t,uint32_t);
static unsigned tx_flushes, pre_save_delay;
static uint8_t rx_errors[8];
static bool idle_eligible,rx_threshold_reject;
static uint64_t rx_last;
static unsigned rx_rdr_reads,rx_flushes,idle_acks,rx_entries,rx_entry_rdr,max_entry_rdr;
static bool data_disable_order_error,error_enable_lost,rx_cfg_enabled;
/* Deterministic MMIO race injection:1 sampled IDLE,2 IDLECF-before,
 *3 IDLECF-after,4 sampled empty ISR,5 final observation read,6 RDR-after. */
static unsigned rx_hook,rx_hook_hits,rx_hook_left,rx_refill,rx_hook_batch;
static uint8_t rx_hook_value,rx_hook_error,rx_refill_value;
static bool rx_hook_returned,rx_hook_complete_idle;
static unsigned inject_reenable;
static bool error_at_tc;
static uint8_t reenable_error;
static bool check_rx_contract;
static void receive_marked(uint8_t byte,uint8_t error);
static void rx_inject(unsigned hook){if(rx_hook==hook&&rx_hook_left){--rx_hook_left;++rx_hook_hits;for(unsigned i=0;i<(rx_hook_batch?rx_hook_batch:1u);i++)receive_marked(rx_hook_value++,rx_hook_error);if(rx_hook_complete_idle){set(U+0x1c,reg(U+0x1c)|IDLE);idle_eligible=false;}rx_hook_returned=true;}}
static void rx_status(void){uint32_t x=reg(U+0x1c);x=(x&~(RXNE|RXFT))|(rx_count?RXNE:0u);unsigned cfg=(reg(U+8u)&RXFTCFG)>>25;static const unsigned levels[]={1,2,4,6,7,8};if(cfg<6u&&rx_count>=levels[cfg])x|=RXFT;set(U+0x1c,x);}

static unsigned tx_irq_entries, txe_irq_entries, tc_irq_entries, large_reply_irqs;
static unsigned irq_tdr_writes, last_tx_batch, max_tx_fill;
static const uint8_t *dma_test_source;static unsigned dma_test_size,dma_test_at,dma_irq_entries;
static bool dma_te_inject,dma_stuck,dma_at_lock,inside_dma,dma_before_wire,dma_te_on_disable,dma_bad_config,dma_te_at_tc,dma_raw_error_locked,dma_raw_error_on_clear,dma_raw_at_prearm,dma_raw_on_arm_read,dma_te_raw_after_flag,dma_raw_after_sample;
void platform_test_dma_source(const uint8_t *p,uint16_t n){dma_test_source=p;dma_test_size=n;dma_test_at=0;}
/* RX DMA channel2 model: CR3.DMAR + CCR2.EN move one FIFO byte per step into the
 * real ring at CMAR+(len-CNDTR); HT at len/2, TC at 0 (reload if CIRC). DDRE
 * blocks while the head byte (or ISR) carries PE/FE/NE/ORE. rx_dma_stall models a
 * channel that does not serve (used to force a real FIFO overrun). */
static volatile uint8_t *rx_dma_mem;static unsigned rx_dma_len,rx_dma_moves,rx_dma_te_after,rx_cndtr_inject;static bool rx_dma_stall,rx_ccr_reject;
void platform_test_rx_dma_target(volatile uint8_t *p,uint16_t n){rx_dma_mem=p;rx_dma_len=n;}
void DMA1_Channel1_IRQHandler(void);
static uint32_t inject_error_flags;
static bool inject_error_on_full, fifo_reject, fifo_enable_violation, threshold_reject, threshold_enable_violation;
static bool gate, premature_rx, tx_irq_observed, tc_irq_observed, abort_before_arm;
static uintptr_t last_read_address;
static bool bad_clock_order, pll_stall, vos_stall, regulator_stall, flash_reject, switch_stall;
static unsigned pll_polls;
static bool expect_loader, backup_reserved_write;
static bool prefetch_stuck,prefetch_pending,forbidden_flash_write;
static unsigned flash_writes,flash_reads,prefetch_reads,prefetch_delay;
static uint32_t prefetch_value,prefetch_corrupt,acr_before[8],acr_written[8];
static struct entry startup_writes[256];static unsigned startup_write_count;

static unsigned retained_write_order;
static uint64_t burst_due;
static unsigned burst_remaining;
static uint8_t burst_value;
static bool watch_irq_cleanup, inherited_irq_violation;
static bool retained_write_violation;
static uint32_t iwdg_keys[8];static unsigned iwdg_key_n;   /* fast12: IWDG_KR writes in order */
static jmp_buf loader_jump;
static uint32_t reg(uintptr_t a) { for(unsigned i=0;i<map_used;i++)if(map[i].address==a)return map[i].value;return 0; }
static void set(uintptr_t a,uint32_t v) {for(unsigned i=0;i<map_used;i++)if(map[i].address==a){map[i].value=v;return;} if(map_used==96)abort();map[map_used++]=(struct entry){a,v};}
static void service_irq(void);
static unsigned fifo_capacity(void) {return (reg(U)&FIFOEN)?8u:1u;}
static void tx_status(void) {
    uint32_t status=reg(U+0x1c);
    status=(status&~TXE)|(tx_count<fifo_capacity()?TXE:0u);
    static const unsigned empty_threshold[]={1u,2u,4u,6u,7u,8u};
    unsigned threshold=(reg(U+8u)&TXFTCFG)>>29;
    status&=~TXFT;
    if((reg(U)&FIFOEN) && threshold<6u && 8u-tx_count>=empty_threshold[threshold])status|=TXFT;
    if(!shift_busy && !tx_count)status|=TC;else status&=~TC;
    set(U+0x1c,status);
}
static void receive_marked(uint8_t byte,uint8_t error) {
    if(rx_count>=fifo_capacity())set(U+0x1c,reg(U+0x1c)|8u);
    else {unsigned i=(rx_fifo_head+rx_count)%8u;rx_fifo[i]=byte;rx_errors[i]=error;++rx_count;}
    idle_eligible=true;rx_last=now_us;rx_status();
}
static void receive_byte(uint8_t byte){receive_marked(byte,0);}
static void step(void) {
    ++now_us;
    if (!freeze_tx && shift_busy && now_us>=shift_end) {
        if(wire_n>=sizeof wire)abort();
        if(wire_n==0)first_wire=now_us;
        wire[wire_n++]=shift_byte;
        if(tx_count) {shift_byte=tx_fifo[tx_head];tx_head=(tx_head+1u)%8u;--tx_count;shift_end=now_us+10;}
        else shift_busy=false;
        tx_status();
        if(wire_n==expected_n){final_stop=now_us;peer_due=now_us+100;peer_pending=true;if(error_at_tc){error_at_tc=false;set(U+0x1c,reg(U+0x1c)|2u|IDLE);receive_byte(0xceu);}}
    }
    if(peer_pending && now_us>=peer_due) {
        peer_pending=false;
        if(!gate && (reg(U)&RE)) {receive_byte(0xffu);peer_received=true;}
        else peer_lost=true;
    }
    if(burst_remaining && now_us>=burst_due) {
        --burst_remaining;burst_due+=10u;
        if(!gate && (reg(U)&RE)) {
            receive_byte(burst_value++);
        }
    }
    if((reg(U+8u)&(1u<<7)) && (reg(DMA+8u)&1u) && tx_count<fifo_capacity() && dma_test_at<dma_test_size){
        uint8_t value=dma_test_source[dma_test_at++];
        if(shift_busy){tx_fifo[(tx_head+tx_count)%8u]=value;tx_count++;}else{shift_byte=value;shift_busy=true;shift_end=now_us+10;}
        if(tx_count>max_tx_fill)max_tx_fill=tx_count;set(DMA+12u,dma_test_size-dma_test_at);tx_status();
        if(inject_error_on_full && tx_count==8u){inject_error_on_full=false;set(U+0x1c,reg(U+0x1c)|inject_error_flags);}
        if(dma_te_inject && dma_test_at==3u){dma_te_inject=false;set(DMA,reg(DMA)|8u);}
        if(dma_test_at==dma_test_size)set(DMA,reg(DMA)|2u|(dma_te_at_tc?8u:0u));
    }
    if((reg(U+8u)&(1u<<6)) && (reg(DMA+0x1cu)&1u) && rx_count && rx_dma_mem && !rx_dma_stall &&
       !((reg(U+8u)&(1u<<13)) && ((reg(U+0x1c)|rx_errors[rx_fifo_head])&15u))){
        uint32_t n=reg(DMA+0x20u);if(!n||n>rx_dma_len||reg(DMA+0x24u)!=(uint32_t)(U+0x24u)||reg(DMAMUX+4u)!=50u||(reg(DMA+0x1cu)&0x7ffeu)!=0xa0u)abort();
        rx_dma_mem[rx_dma_len-n]=rx_fifo[rx_fifo_head];rx_fifo_head=(rx_fifo_head+1u)%8u;--rx_count;rx_dma_moves++;n--;
        if(rx_dma_te_after&&!--rx_dma_te_after){set(DMA,reg(DMA)|(1u<<7)|(1u<<4));set(DMA+0x1cu,reg(DMA+0x1cu)&~1u);}
        if(n==rx_dma_len/2u)set(DMA,reg(DMA)|(1u<<6)|(1u<<4));
        if(!n){set(DMA,reg(DMA)|(1u<<5)|(1u<<4));n=rx_dma_len;}
        set(DMA+0x20u,n);rx_status();
    }
    if(idle_eligible&&now_us-rx_last>=10u){set(U+0x1c,reg(U+0x1c)|IDLE);idle_eligible=false;}
    service_irq();
}
static void service_irq(void) {
    if(inside_irq || irq_mask)return;
    uint32_t status=reg(U+0x1c)|(rx_count?rx_errors[rx_fifo_head]:0u),cr=reg(U),cr3=reg(U+8u);
    if(((status&TXE)&&(cr&TXIE)) || ((status&TXFT)&&(cr3&TXFTIE)) || ((status&TC)&&(cr&TCIE)) || ((status&RXNE)&&(cr&RXIE)) || ((status&IDLE)&&(cr&IDLE)) || ((status&RXFT)&&(cr3&RXFTIE)) || ((status&14u)&&(cr3&1u)) || ((status&1u)&&(cr&(1u<<8)))) {
        inside_irq=true;
        irq_tdr_writes=0u;
        if((status&TXFT)&&(cr3&TXFTIE)){tx_irq_observed=true;++tx_irq_entries;}
        if((status&TXE)&&(cr&TXIE)){tx_irq_observed=true;++txe_irq_entries;}
        if((status&TC)&&(cr&TCIE)){tc_irq_observed=true;++tc_irq_entries;}
        uint32_t before_entries=observation_uart_entries,before_activity=observation_activity;
        rx_entry_rdr=0;rx_entries++;USART1_IRQHandler();if(rx_entry_rdr>max_entry_rdr)max_entry_rdr=rx_entry_rdr;
        if(observation_uart_entries-before_entries!=1u || observation_activity-before_activity!=1u)abort();
        if(irq_tdr_writes)last_tx_batch=irq_tdr_writes;
        inside_irq=false;
    }
    if(!inside_irq && !irq_mask && (reg(DMA)&(2u|8u)) && (reg(DMA+8u)&(2u|8u))){inside_irq=true;inside_dma=true;dma_irq_entries++;DMA1_Channel1_IRQHandler();if((reg(U)&TCIE) && wire_n<expected_n){if(!gate || (reg(U)&RE))abort();dma_before_wire=true;}inside_dma=false;inside_irq=false;}
}
uint32_t hw_read32(uintptr_t a) {
    if(a==F){flash_reads++;if(prefetch_pending&&++prefetch_reads>=prefetch_delay){set(F,prefetch_value);prefetch_pending=false;}}
    step();
    if(a==S+8){if(inside_irq&&rx_rdr_reads&&!rx_count)rx_inject(5);return (0x00ffffffu-(uint32_t)(now_us*64u))&0x00ffffffu;}
    if(a==R && (reg(R)&(1u<<24))) ++pll_polls;
    last_read_address=a;
    if(a==U+0x1c && irq_mask && !inside_dma && !gate && !(reg(U)&RE) && dma_raw_on_arm_read){dma_raw_on_arm_read=false;set(U+0x1c,reg(U+0x1c)|2u);}
    if(a==U+0x1c && inside_dma && irq_mask && dma_raw_error_locked){dma_raw_error_locked=false;set(U+0x1c,reg(U+0x1c)|2u);}
    uint32_t v=reg(a);
    if(a==U+0x1c){v|=rx_count?rx_errors[rx_fifo_head]:0u;if(v&IDLE)rx_inject(1);if(!(v&RXNE))rx_inject(4);}
    if(a==S+8u&&inside_irq&&rx_rdr_reads&&!rx_count)rx_inject(5);
    if(a==DMA && inside_dma && irq_mask && (v&8u) && dma_te_raw_after_flag){dma_te_raw_after_flag=false;set(U+0x1c,reg(U+0x1c)|2u);}
    if(a==DMA+0x20u && rx_cndtr_inject){unsigned k=rx_cndtr_inject;rx_cndtr_inject=0;for(unsigned i=0;i<k;i++){receive_byte((uint8_t)(0xc0u+i));step();}v=reg(a);}
    if(a==U+0x24) {
        if(!rx_count)abort();v=rx_fifo[rx_fifo_head];rx_fifo_head=(rx_fifo_head+1u)%8u;--rx_count;rx_rdr_reads++;rx_entry_rdr++;
        rx_status();if(rx_refill){rx_refill--;receive_byte(rx_refill_value++);}rx_inject(6);
    }
    return v;
}
void hw_write32(uintptr_t a,uint32_t v) {
    step();uint32_t old=reg(a);
    if(a==U+0x20u&&(v&IDLE)){rx_inject(2);idle_acks++;}
    if((a>=F+4u&&a<F+0x100u)||(a>=0x08000000u&&a<0x08010000u))forbidden_flash_write=true;
    if((a>=R&&a<R+0x100u)||(a>=P&&a<P+0x100u)||(a>=S&&a<S+12u)){if(startup_write_count==256)abort();startup_writes[startup_write_count++]=(struct entry){a,v};}
    if(a==F){if(flash_writes<8){acr_before[flash_writes]=old;acr_written[flash_writes]=v;}flash_writes++;if(v&0x100u){if(prefetch_stuck)v&=~0x100u;else if(prefetch_delay){prefetch_value=v;prefetch_pending=true;v&=~0x100u;}}if(flash_writes==2)v^=prefetch_corrupt;}

    if(a==DMA+8u && (v&1u)){
        if(v!=0x9bu || reg(DMA+12u)!=dma_test_size || reg(DMA+16u)!=(uint32_t)(U+0x28u) || reg(DMA+20u)!=(uint32_t)(uintptr_t)dma_test_source || reg(DMAMUX)!=51u)dma_bad_config=true;
    }
    if(a==DMA+0x1cu && rx_ccr_reject)v&=~1u;
    if(a==DMA+8u && (old&1u) && !(v&1u) && dma_te_on_disable){dma_te_on_disable=false;set(DMA,reg(DMA)|8u);}if(a==DMA+8u && dma_stuck && (old&1u) && !(v&1u))v|=1u;set(a,v);
    if(a>=0x4000b108u && a<=0x4000b13cu) backup_reserved_write=true;
    if(a==0x40003000u && iwdg_key_n<8u)iwdg_keys[iwdg_key_n++]=v;
    if(a>=0x200017c0u && a<=0x200017ccu) {   /* the 16 B trace; fast11 adds the reset record at 17D0..17DF */
        static const uintptr_t addresses[]={0x200017c0u,0x200017c4u,0x200017c8u,0x200017ccu,0x200017c0u};
        if(a!=addresses[retained_write_order%5u])retained_write_violation=true;
        if(retained_write_order%5u==0u && v!=0u)retained_write_violation=true;
        if(retained_write_order%5u==4u && v!=0xd1a6cafeu)retained_write_violation=true;
        ++retained_write_order;
    }
    if(a==R+0x60u && (v&(1u<<23))) set(a,v&~0xff800000u);   /* RCC_CSR.RMVF clears the reset flags (fast11) */
    if(a==R+0x5cu && (v&(1u<<16))) {
        for(uintptr_t b=0x4000b100u;b<=0x4000b110u;b+=4u)set(b,0u);
    }
    if(a==0xe000e100u)set(a,old|v);
    if(a==0xe000e180u)set(0xe000e100u,reg(0xe000e100u)&~v);
    if(a==0xe000e280u)set(0xe000e200u,reg(0xe000e200u)&~v);
    if(a==0xe000ed04u)set(a,old&~(((v&(1u<<25))?(1u<<26):0u)|((v&(1u<<27))?(1u<<28):0u)));
    if(a==R) {
        if(watch_irq_cleanup) {
            if(reg(0xe000e100u)!=0u || reg(0xe000e200u)!=0u || (reg(0xe000ed04u)&0x14000000u) || reg(S)!=0u)inherited_irq_violation=true;
            watch_irq_cleanup=false;
        }
        uint32_t ready=(1u<<10);
        if((v&(1u<<24)) && !pll_stall) ready|=1u<<25;
        set(R,(v&~(1u<<25))|ready);
    }
    if(a==P) set(P+0x14,(vos_stall?(1u<<10):0u)|(regulator_stall?(1u<<9):0u));
    if(a==F && flash_reject) set(F,v&~7u);
    if(a==R+8) {
        unsigned sw=v&7u;
        if(sw==2u && (((reg(F)&7u)!=2u) || ((reg(P)&0x600u)!=0x200u) || !(reg(R)&(1u<<25)))) bad_clock_order=true;
        if(!switch_stall || sw!=2u) set(R+8,(v&~0x38u)|(sw<<3));
    }
    if(a==U) {
        if(check_rx_contract && (old&1u)&&!(v&(1u<<8)))error_enable_lost=true;
        if(check_rx_contract && !(v&RE) && (v&(RXIE|IDLE)))data_disable_order_error=true;

        if((v&FIFOEN) && !(old&FIFOEN) && (v&1u))fifo_enable_violation=true;
        if(fifo_reject)set(U,reg(U)&~FIFOEN);
        uint32_t status=reg(U+0x1c);
        status=(status&~((1u<<21)|(1u<<22))) | ((v&8u)?1u<<21:0u) | ((v&RE)?1u<<22:0u);
        set(U+0x1c,status);
        tx_status();
        if((v&RE)&&!(old&RE)&&inject_reenable){receive_marked(0xadu,reenable_error);inject_reenable--;}
        if((v&RE) && !(old&RE) && expected_n!=0) {
            if(wire_n<expected_n)premature_rx=true;
            else rx_restore_us=now_us;
        }
    }
    if(a==U+8u) {
        if(check_rx_contract && !(v&1u))error_enable_lost=true;
        if(((old^v)&RXFTCFG)&&(reg(U)&1u))rx_cfg_enabled=true;
        if(rx_threshold_reject)set(a,reg(a)&~RXFTCFG);

        if(((old^v)&TXFTCFG) && (reg(U)&1u))threshold_enable_violation=true;
        if(threshold_reject)set(a,v&~TXFTCFG);
        tx_status();rx_status();
    }
    if(a==DMA+4u){if(v&((1u<<0)|(1u<<4)))abort();set(DMA,reg(DMA)&~v);if(dma_raw_error_on_clear){dma_raw_error_on_clear=false;set(U+0x1c,reg(U+0x1c)|2u);}}
    if(a==U+0x20 && inside_dma && dma_raw_after_sample){dma_raw_after_sample=false;set(U+0x1c,reg(U+0x1c)|4u);}
    if(a==U+0x20){set(U+0x1c,reg(U+0x1c)&~(v&(0x7fu|(1u<<11))));   /* ICR: PE..TC, and RTOCF (bit 11, FW231 fast start) */if(rx_count)rx_errors[rx_fifo_head]&=~v;if(v&IDLE)rx_inject(3);}
    if(a==U+0x18 && (v&8u)) {rx_flushes++;rx_fifo_head=rx_count=0;rx_status();}
    if(a==U+0x18 && (v&16u)) {tx_head=tx_count=0;++tx_flushes;tx_status();}
    if(a==C+0x18 && (v&0x40u))gate=true;
    if(a==C+0x28 && (v&0x40u))gate=false;
    if(a==U+0x28) {
        if(inside_irq)++irq_tdr_writes;
        if(shift_busy) {
            if(tx_count>=fifo_capacity())abort();
            tx_fifo[(tx_head+tx_count)%8u]=(uint8_t)v;++tx_count;
        } else {shift_byte=(uint8_t)v;shift_busy=true;shift_end=now_us+10;}
        tx_status();
        if(tx_count>max_tx_fill)max_tx_fill=tx_count;
        if(inject_error_on_full && tx_count==8u) {inject_error_on_full=false;set(U+0x1c,reg(U+0x1c)|inject_error_flags);}
    }
    service_irq();
}
void hw_relax(void) { unsigned duration=pause_foreground && !irq_mask?1000u:1u;for(unsigned i=0;i<duration;i++)step(); }
uint32_t platform_test_irq_save(void) {
    if(inside_dma && dma_at_lock && !irq_mask){dma_at_lock=false;set(U+0x1c,reg(U+0x1c)|2u);USART1_IRQHandler();}
    if(!irq_mask && pre_save_delay) {unsigned delay=pre_save_delay;pre_save_delay=0;for(unsigned i=0;i<delay;++i)step();}
    if(abort_before_arm && !irq_mask && !gate && !(reg(U)&RE) && last_read_address==U+0x1c) {
        abort_before_arm=false;set(U+0x1c,reg(U+0x1c)|2u);service_irq();
    }
    if(dma_raw_at_prearm && !irq_mask && !gate && !(reg(U)&RE) && last_read_address==U+0x1c){dma_raw_at_prearm=false;set(U+0x1c,reg(U+0x1c)|2u);}
    uint32_t old=irq_mask;irq_mask=1;return old;
}
void platform_test_irq_restore(uint32_t v) {if(check_rx_contract&&expected_n&&!(reg(U)&RE)&&((reg(U)&(RXIE|IDLE))||(reg(U+8u)&RXFTIE)))data_disable_order_error=true;irq_mask=v;service_irq();}
void platform_test_irq_enable(void) {irq_mask=0;service_irq();}
void platform_test_loader(void) {if(expect_loader)longjmp(loader_jump,1);abort();}
void bridge_disable(void) {++disabled;}
void bridge_early_off(void) {}
static void reset_fixture(void) {
    rx_dma_mem=NULL;rx_dma_len=rx_dma_moves=rx_dma_te_after=rx_cndtr_inject=0;rx_dma_stall=rx_ccr_reject=false;
    memset(rx_errors,0,sizeof rx_errors);idle_eligible=rx_threshold_reject=error_at_tc=false;rx_last=0;rx_rdr_reads=rx_flushes=idle_acks=rx_entries=rx_entry_rdr=max_entry_rdr=0;data_disable_order_error=error_enable_lost=rx_cfg_enabled=check_rx_contract=false;rx_hook=rx_hook_hits=rx_hook_left=rx_refill=inject_reenable=rx_hook_batch=0;rx_hook_value=rx_hook_error=rx_refill_value=reenable_error=0;rx_hook_returned=rx_hook_complete_idle=false;
    memset(map,0,sizeof map);map_used=0;now_us=0;shift_end=first_wire=final_stop=peer_due=rx_restore_us=0;
    dma_test_source=NULL;dma_test_size=dma_test_at=dma_irq_entries=0;dma_te_inject=dma_stuck=dma_at_lock=inside_dma=dma_before_wire=dma_te_on_disable=dma_bad_config=dma_te_at_tc=dma_raw_error_locked=dma_raw_error_on_clear=dma_raw_at_prearm=dma_raw_on_arm_read=dma_te_raw_after_flag=dma_raw_after_sample=false;irq_mask=1;disabled=0;inside_irq=shift_busy=pause_foreground=peer_pending=peer_received=peer_lost=freeze_tx=false;
    gate=premature_rx=tx_irq_observed=tc_irq_observed=abort_before_arm=false;
    bad_clock_order=pll_stall=vos_stall=regulator_stall=flash_reject=switch_stall=false;pll_polls=0;expect_loader=backup_reserved_write=retained_write_violation=false;retained_write_order=0;burst_due=0;burst_remaining=0;burst_value=0x40u;tx_head=tx_count=rx_fifo_head=rx_count=tx_flushes=pre_save_delay=0;tx_irq_entries=txe_irq_entries=tc_irq_entries=irq_tdr_writes=last_tx_batch=max_tx_fill=0;inject_error_flags=4u;inject_error_on_full=fifo_reject=fifo_enable_violation=threshold_reject=threshold_enable_violation=false;watch_irq_cleanup=inherited_irq_violation=false;last_read_address=0;wire_n=expected_n=0;
    set(R,1u<<10);set(R+0x60u,0xa4000000u);set(U+0x1c,TXE|TC);
    prefetch_stuck=prefetch_pending=forbidden_flash_write=false;flash_writes=flash_reads=prefetch_reads=prefetch_delay=0;prefetch_value=prefetch_corrupt=startup_write_count=0;
}
static bool setup(void){reset_fixture();CHECK(platform_init());check_rx_contract=true;platform_irq_enable();return true;}

static uint32_t word(const uint8_t*);
static bool test_prefetch_exact_preservation(void){
 const uint32_t seeds[]={0x40600u,0x40602u,0x40702u,0xa5a50602u,0x50602u};
 for(unsigned i=0;i<sizeof seeds/sizeof*seeds;i++){reset_fixture();set(F,seeds[i]);CHECK(platform_init());CHECK(reg(F)==((seeds[i]&~7u)|0x102u));CHECK(flash_writes==2);CHECK(acr_written[0]==((seeds[i]&~7u)|2u));CHECK(acr_before[1]==acr_written[0]);CHECK(acr_written[1]==(acr_before[1]|0x100u));CHECK((acr_written[1]^acr_before[1])==(seeds[i]&0x100u?0u:0x100u));CHECK(!(acr_written[1]&0x800u));CHECK(!forbidden_flash_write&&!bad_clock_order);uint8_t debug[56];platform_debug_read(debug);CHECK(word(debug+32)==reg(F));}
 puts("prefetch exact reset/latency/inherited/sentinel bit preservation and live ACR PASS");return true;
}
static bool test_prefetch_stuck_and_full_readback(void){
 for(unsigned mode=0;mode<3;mode++){reset_fixture();set(F,0x40602u);prefetch_stuck=mode==0;prefetch_corrupt=mode==1?0x40000u:mode==2?2u:0u;CHECK(!platform_init());CHECK((reg(R+8u)&7u)==0&&!(reg(R)&(1u<<24))&&!(reg(U)&1u)&&reg(S)==0);CHECK(flash_reads>=1000000u&&flash_reads<=1000010u);CHECK(reg(0x200017c4u)==0x10eu&&reg(0x200017c8u)==reg(F));CHECK(!forbidden_flash_write);if(mode==0)CHECK((reg(F)&7u)==2u&&!(reg(F)&0x100u));}
 puts("prefetch stuck0/non-bit preservation/latency mismatch bounded failure before PLL64/UART PASS");return true;
}
static bool test_prefetch_eventual_readback(void){reset_fixture();set(F,0x40600u);prefetch_delay=7;CHECK(platform_init());CHECK(prefetch_reads==7&&reg(F)==0x40702u&&!prefetch_pending&&!bad_clock_order&&!forbidden_flash_write);puts("prefetch eventual bounded readback PASS");return true;}
static bool test_irq_streams_while_foreground_is_preempted(void) {
    CHECK(setup());static const uint8_t data[]={0xff,0xff,0xfd,0,1,4,0,0x55,0,0xa1,0x0c};
    expected_n=sizeof data;pause_foreground=true;
    CHECK(platform_uart_transmit(data,sizeof data));
    CHECK(wire_n==sizeof data && memcmp(data,wire,sizeof data)==0);
    CHECK(dma_irq_entries==1u && tx_irq_entries==0u && tc_irq_observed);CHECK(tx_flushes==0u);
    CHECK(final_stop-first_wire <= (sizeof data-1u)*24u);
    CHECK(!premature_rx && !gate && (reg(U)&RE));
    return true;
}
static bool test_tc_restores_rx_before_foreground_and_preserves_peer_header(void) {
    CHECK(setup());static const uint8_t data[]={1,2,3,4,5};
    expected_n=sizeof data;pause_foreground=true;
    CHECK(platform_uart_transmit(data,sizeof data));
    while(peer_pending)step();
    CHECK(rx_restore_us>=final_stop && rx_restore_us-final_stop<50);
    CHECK(peer_received && !peer_lost);
    uint8_t header=0;CHECK(platform_uart_pop(&header) && header==0xff);
    CHECK(!platform_uart_pop(&header)); /* no cleanup flush after reception */
    CHECK((reg(U)&(TXIE|TCIE))==0 && !(reg(U+8u)&TXFTIE) && platform_uart_faults()==0);
    CHECK(dma_irq_entries==1u && tx_irq_entries==0u && txe_irq_entries==0u && tc_irq_entries==1u);
    return true;
}
/* FW231 fast start: the USART IRQ starts a reply (platform_uart_tx_irq) without waiting for it; the existing
 * DMA/TC interrupts complete it and give RX back. Nothing can preempt the priority-0 IRQ: modelled with PRIMASK. */
static unsigned rto_calls;
void platform_uart_rto(void){rto_calls++;}
static bool test_irq_started_reply_streams_and_restores_rx(void) {
    CHECK(setup());static const uint8_t data[]={0x00,0x07,0x1f,0x08,0x00,0x00,0x16,0xca};
    CHECK(reg(U)&(1u<<26));CHECK(reg(U+4u)&(1u<<23));                  /* RTOIE and RTOEN on after init */
    expected_n=sizeof data;
    uint32_t irq=platform_irq_save();bool started=platform_uart_tx_irq(data,sizeof data);
    CHECK(started && !(reg(U)&RE) && gate);                               /* receiver off, driver on, all in the IRQ */
    CHECK(!platform_uart_tx_irq(data,sizeof data));                        /* one owner at a time */
    platform_irq_restore(irq);
    for(unsigned i=0;i<200000u&&!platform_uart_tx_idle();i++)step();
    CHECK(platform_uart_tx_idle() && wire_n==sizeof data && memcmp(data,wire,sizeof data)==0);
    CHECK(!premature_rx && !gate && (reg(U)&RE) && tc_irq_observed && platform_uart_faults()==0);
    CHECK(platform_uart_transmit(data,sizeof data));                       /* the foreground path still works after it */
    return true;
}
static bool test_irq_start_refused_while_a_byte_arrives(void) {
    CHECK(setup());static const uint8_t data[]={1,2,3};
    set(U+0x1c,reg(U+0x1c)|(1u<<16));                                    /* ISR.BUSY: reception in progress */
    uint32_t irq=platform_irq_save();bool started=platform_uart_tx_irq(data,sizeof data);platform_irq_restore(irq);
    CHECK(!started && (reg(U)&RE) && !gate && wire_n==0u && platform_uart_tx_idle());
    set(U+0x1c,reg(U+0x1c)&~(1u<<16));
    return true;
}
static bool test_receiver_timeout_reaches_the_watcher_once(void) {
    CHECK(setup());rto_calls=0;
    set(U+0x1c,reg(U+0x1c)|(1u<<11));                                    /* RTOF */
    USART1_IRQHandler();
    CHECK(rto_calls==1u && !(reg(U+0x1c)&(1u<<11)));                      /* cleared through ICR.RTOCF */
    USART1_IRQHandler();CHECK(rto_calls==1u);
    platform_uart_set_rto(450u);CHECK(reg(U+0x14u)==450u);
    platform_uart_set_rto(0x12345678u);CHECK(reg(U+0x14u)==0x345678u);   /* 24-bit RTO, BLEN untouched */
    return true;
}
/* fast12 (owner 2026-11-20: "turn it on for watchdog"): RM0444 start order, 2 s timeout, refresh only without a window. */
static bool test_watchdog_started_at_two_seconds(void) {
    CHECK(setup());iwdg_key_n=0;
    platform_watchdog_start();
    CHECK(iwdg_key_n==3u && iwdg_keys[0]==0xccccu && iwdg_keys[1]==0x5555u && iwdg_keys[2]==0xaaaau);   /* start, unlock, refresh */
    CHECK(reg(0x40003004u)==4u && reg(0x40003008u)==999u);            /* /64, 1000 counts: 2.0 s at the 32 kHz LSI */
    CHECK(PLATFORM_IWDG_MS==(4u<<PLATFORM_IWDG_PR)*(PLATFORM_IWDG_RLR+1u)/32u);
    set(0x40003010u,0xfffu);iwdg_key_n=0;platform_watchdog_reload();CHECK(iwdg_key_n==1u && iwdg_keys[0]==0xaaaau);
    set(0x40003010u,0x200u);iwdg_key_n=0;platform_watchdog_reload();CHECK(iwdg_key_n==0u);   /* a window set by someone else: never refresh */
    puts("fast12: IWDG started (start, unlock, /64 x 1000 = 2.0 s, refresh), refresh only without a window PASS");
    return true;
}
/* fast15 (release review 2026-11-23 P2): if the IWDG never latches the new prescaler/reload (SR update bits stay set
 * past the wait), the start still completes (same key sequence, no behaviour change) and read 1112 word 0 bit 26 says
 * so: the watchdog then runs at its reset default, about 0.5 s. A normal start reports bit 26 clear. */
static bool test_watchdog_update_timeout_reported(void) {
    uint32_t info[3];
    CHECK(setup());set(0x4000300cu,1u);iwdg_key_n=0;
    platform_watchdog_start();
    CHECK(iwdg_key_n==3u && iwdg_keys[2]==0xaaaau);                    /* still started and refreshed */
    platform_reset_info(info);CHECK((info[0]&PLATFORM_RESET_IWDG_UPDATE_TIMEOUT)!=0u);
    CHECK(PLATFORM_RESET_IWDG_UPDATE_TIMEOUT==(1u<<26));
    CHECK(setup());set(R+0x60u,1u<<27);platform_boot_capture();      /* the next boot (power-on) rewrites word 0 */
    set(0x4000300cu,0u);iwdg_key_n=0;
    platform_watchdog_start();
    platform_reset_info(info);CHECK((info[0]&PLATFORM_RESET_IWDG_UPDATE_TIMEOUT)==0u);
    puts("fast15: IWDG update timeout -> read 1112 word 0 bit 26 (start unchanged), clear on a normal start PASS");
    return true;
}
/* fast11 (owner 2026-11-20: "a hung servo should reboot back into its firmware"): hang routing and the reset record. */
#define BOOT_ARMED() (reg(0x4000b100u)==0x544f4f42u && reg(0x4000b104u)==0x52544e45u)
#define RECORD(r,c) (reg(0x200017d0u)==0x31545352u && reg(0x200017d4u)==(r) && reg(0x200017d8u)==(c) && reg(0x200017dcu)==((r)^(c)^0xa5a5a5a5u))
static bool boot(uint32_t csr_flags, uint32_t info[3]) {          /* what firmware_main does at power-up / after a reset */
    uint32_t sram[4];for(unsigned i=0;i<4u;i++)sram[i]=reg(0x200017d0u+4u*i);   /* SRAM survives the reset; the model does not */
    CHECK(setup());for(unsigned i=0;i<4u;i++)set(0x200017d0u+4u*i,sram[i]);set(R+0x60u,csr_flags);platform_boot_capture();CHECK((reg(R+0x60u)&0xff000000u)==0u);
    CHECK(platform_arm_loader() && BOOT_ARMED());platform_reset_info(info);return true;
}
static bool hang(uint32_t reason, bool to_loader) {
    expect_loader=true;if(setjmp(loader_jump)==0)platform_hang_reset(reason);expect_loader=false;
    CHECK(BOOT_ARMED()==to_loader);return true;
}
static bool test_hang_reboots_the_application_once_healthy(void) {
    uint32_t info[3];
    CHECK(setup());set(0x200017d0u,0x31545352u);set(0x200017d4u,1u);set(0x200017d8u,7u);set(0x200017dcu,1u^7u^0xa5a5a5a5u);   /* power-up: even a valid-looking record is not ours */
    CHECK(boot(1u<<27,info) && (info[0]&0xffu)==PLATFORM_RESET_POWER && !(info[0]&(1u<<24)) && info[1]==0u && info[2]==0u);
    CHECK(RECORD(PLATFORM_RESET_NONE,0u));
    CHECK(hang(PLATFORM_RESET_STALL,true));                       /* not healthy yet: the loader, as before (start-up crash) */
    CHECK(RECORD(PLATFORM_RESET_STALL,1u));
    CHECK(boot(1u<<28,info) && (info[0]&0xffu)==PLATFORM_RESET_STALL && (info[0]&(1u<<24)) && info[1]==1u && info[2]==0u);
    platform_app_healthy();CHECK(!BOOT_ARMED());platform_reset_info(info);CHECK(info[0]&(1u<<25));
    CHECK(hang(PLATFORM_RESET_FAULT|(3u<<8),false));              /* healthy: back into the application */
    CHECK(RECORD(PLATFORM_RESET_FAULT|(3u<<8),2u|(1u<<16)));
    CHECK(boot(1u<<28,info) && info[0]==(PLATFORM_RESET_FAULT|(3u<<8)|(0x10u<<16)|(1u<<24)) && info[1]==2u && info[2]==1u);
    CHECK(RECORD(PLATFORM_RESET_NONE,2u|(1u<<16)));                /* counts carried over an unannounced reset */
    platform_app_healthy();
    CHECK(boot(1u<<29|1u<<26,info) && (info[0]&0xffu)==PLATFORM_RESET_WATCHDOG && info[1]==3u && info[2]==2u);   /* IWDG */
    platform_app_healthy();CHECK(hang(PLATFORM_RESET_DMA,false) && RECORD(PLATFORM_RESET_DMA,4u|(3u<<16)));
    CHECK(boot(1u<<28,info) && info[2]==3u);platform_app_healthy();
    CHECK(hang(PLATFORM_RESET_STALL,false) && RECORD(PLATFORM_RESET_STALL,5u|(4u<<16)));
    CHECK(boot(1u<<28,info) && info[2]==4u);platform_app_healthy();
    CHECK(hang(PLATFORM_RESET_STALL,true) && RECORD(PLATFORM_RESET_STALL,6u));   /* 5th quick hang in a row: the loader, streak 0 */
    CHECK(boot(1u<<28,info) && info[1]==6u && info[2]==0u);platform_app_healthy();
    platform_app_long_run();CHECK(RECORD(PLATFORM_RESET_NONE,6u|(1u<<24)));      /* 60 s up */
    CHECK(hang(PLATFORM_RESET_STALL,false) && RECORD(PLATFORM_RESET_STALL,7u));   /* after a long run: no streak */
    CHECK(boot(1u<<28,info) && info[2]==0u);platform_app_healthy();
    expect_loader=true;if(setjmp(loader_jump)==0)platform_reset_app();expect_loader=false;   /* 0x08: recorded as the host */
    CHECK(!BOOT_ARMED() && RECORD(PLATFORM_RESET_HOST,7u));
    CHECK(boot(1u<<28,info) && (info[0]&0xffu)==PLATFORM_RESET_HOST && info[1]==7u);
    CHECK(setup());set(R+0x60u,1u<<29);expect_loader=true;           /* 5th quick watchdog reset: the guard at boot */
    set(0x200017d0u,0x31545352u);set(0x200017d4u,0u);set(0x200017d8u,7u|(4u<<16));set(0x200017dcu,(7u|(4u<<16))^0xa5a5a5a5u);
    if(setjmp(loader_jump)==0)platform_boot_capture();
    expect_loader=false;CHECK(BOOT_ARMED() && RECORD(PLATFORM_RESET_WATCHDOG,8u));
    CHECK(!backup_reserved_write && !retained_write_violation);
    puts("fast11: hang -> loader before healthy, the application after (stall/fault/DMA/watchdog recorded, counts kept), 5 quick hangs -> loader, long run ends the streak, 0x08 recorded PASS");
    return true;
}
/* FW231 hardware finding (ID3, 2026-11-20): every boot arms BOOT/ENTR; a software reset keeps it,
 * so 0x08 must clear it or the servo stays in the loader. */
static bool test_reboot_clears_the_boot_time_loader_request(void) {
    CHECK(setup());
    CHECK(platform_arm_loader());                                  /* what firmware_main does at boot */
    CHECK(reg(0x4000b100u)==0x544f4f42u && reg(0x4000b104u)==0x52544e45u);
    unsigned before=disabled;expect_loader=true;
    if(setjmp(loader_jump)==0)platform_reset_app();
    expect_loader=false;
    CHECK(reg(0x4000b100u)==0u && reg(0x4000b104u)==0u);            /* request gone: loader boots the app */
    CHECK(disabled>before && !gate);                               /* output off, DE released */
    CHECK(platform_arm_loader() && platform_disarm_loader() && reg(0x4000b100u)==0u);
    puts("reboot clears the boot-time loader request (BDRST) before the reset; F9 path unchanged PASS");
    return true;
}
static bool test_error_preempting_before_arm_cannot_reassert_de_or_txe(void) {
    CHECK(setup()); static const uint8_t data[]={1,2,3};expected_n=sizeof data;
    abort_before_arm=true;
    CHECK(!platform_uart_transmit(data,sizeof data));
    CHECK(!gate && (reg(U)&RE) && !(reg(U)&(TXIE|TCIE)));
    CHECK(!(reg(U+8u)&TXFTIE));
    /* FW231: a line error aborts the reply (counted) but never cuts the bridge. */
    {uint32_t c[PLATFORM_UART_COUNTS];platform_uart_counts(c);CHECK(wire_n==0 && disabled==0 && c[1]>=1u && c[5]>=1u && !platform_uart_fatal());}
    return true;
}
static bool test_control_timer_uses_configured_period(void) {
    CHECK(setup()); (void)platform_control_timer_start();
    CHECK(reg((uintptr_t)0x40014428u)==63u);
    CHECK(reg((uintptr_t)0x4001442cu)==APP_PERIOD_US-1u);
    CHECK((reg((uintptr_t)0x4001440cu)&1u)!=0);
    CHECK((reg((uintptr_t)0xe000e414u)&0xff00u)==0x8000u);
    return true;
}
static bool test_timeout_aborts_and_releases_bus(void) {
    CHECK(setup());static const uint8_t data[]={1,2,3};expected_n=sizeof data;
    freeze_tx=true;pause_foreground=true;
    CHECK(!platform_uart_transmit(data,sizeof data));
    CHECK(!gate && (reg(U)&RE) && !(reg(U)&(TXIE|TCIE)));
    CHECK(!(reg(U+8u)&TXFTIE));
    CHECK(platform_uart_faults()&4u);CHECK(disabled==0); /* FW231: TX timeout counted, bridge untouched */
    {uint32_t c[PLATFORM_UART_COUNTS];platform_uart_counts(c);CHECK(c[5]>=1u && !platform_uart_fatal());}
    return true;
}
static bool test_clock_and_uart_run_at_requested_rates(void) {
    CHECK(setup());
    CHECK(!bad_clock_order);
    CHECK((reg(P)&0x600u)==0x200u);
    CHECK((reg(F)&7u)==2u);
    CHECK((reg(R+0x0c)&0xf0007f73u)==0x30000802u);
    CHECK((reg(R+8)&0x3fu)==0x12u);
    CHECK((reg(R+0x54)&3u)==2u && reg(U+12)==16u); /* boot 1 Mbaud; 3 Mbaud is a runtime switch */
    uint32_t before=hw_micros();for(unsigned i=0;i<100u;i++)step();
    uint32_t elapsed=hw_micros()-before;CHECK(elapsed>=100u && elapsed<=110u);
    return true;
}
static bool test_clock_failures_never_start_uart(void) {
    CHECK(setup());
    set(U,0u);set(U+0x1c,TXE|TC);set(R+8,0u);set(R,1u<<10);
    pll_stall=true;CHECK(!platform_init());CHECK(!(reg(U)&1u));CHECK(pll_polls<=1000002u);
    CHECK(setup());set(U,0u);set(U+0x1c,TXE|TC);vos_stall=true;
    CHECK(!platform_init());CHECK(!(reg(U)&1u));
    CHECK(setup());set(U,0u);set(U+0x1c,TXE|TC);regulator_stall=true;
    CHECK(!platform_init());CHECK(!(reg(U)&1u));
    CHECK(setup());set(U,0u);set(U+0x1c,TXE|TC);flash_reject=true;
    CHECK(!platform_init());CHECK(!(reg(U)&1u));
    CHECK(setup());set(U,0u);set(U+0x1c,TXE|TC);switch_stall=true;
    CHECK(!platform_init());CHECK(!(reg(U)&1u));
    return true;
}
static uint32_t word(const uint8_t *p) {return (uint32_t)p[0]|((uint32_t)p[1]<<8)|((uint32_t)p[2]<<16)|((uint32_t)p[3]<<24);}
static bool test_debug_before_clock_never_uses_uninitialized_time(void) {
    uint8_t b[56];platform_debug_mark(0x12u,0x34u);platform_debug_read(b);
    CHECK(word(b)==0x12u && word(b+4)==0x34u && word(b+52)==0u);
    return true;
}
static bool test_debug_snapshot_reports_live_registers_and_entry_reset_flags(void) {
    CHECK(setup());uint8_t b[56];
    set(R+0x60u,0x80000000u); /* Entry flags must remain the earlier A4000000. */
    set(R,0x11223344u);set(R+8,0x55667788u);set(R+0x0c,0xabcdef01u);
    set(P,0x12345678u);set(P+0x14,0x23456789u);set(F,0x3456789au);
    set(U+0x1c,0x12340000u);set(0x40005418u,0x456789abu);set(0x40012400u,0x56789abcu);
    platform_debug_mark(0xdeadbeefu,0xcafebabeu);platform_debug_read(b);
    CHECK(word(b)==0xdeadbeefu && word(b+4)==0xcafebabeu && word(b+8)==0xa4000000u);
    CHECK(word(b+12)==0x11223344u && word(b+16)==0x55667788u && word(b+20)==0xabcdef01u);
    CHECK(word(b+24)==0x12345678u && word(b+28)==0x23456789u && word(b+32)==0x3456789au);
    CHECK(word(b+36)==0x12340000u && word(b+40)==0u);
    CHECK(word(b+44)==0x456789abu && word(b+48)==0x56789abcu && word(b+52)==0u);
    return true;
}
static bool test_loader_retains_three_words_in_sram_with_magic_last(void) {
    CHECK(setup());platform_debug_mark(0xdeadu,0xc0deu);unsigned writes_before=retained_write_order;expect_loader=true;
    if(setjmp(loader_jump)==0)platform_enter_loader();
    expect_loader=false;
    CHECK(reg(0x4000b100u)==0x544f4f42u && reg(0x4000b104u)==0x52544e45u);
    CHECK(reg(0x200017c0u)==0xd1a6cafeu && reg(0x200017c4u)==0xdeadu);
    CHECK(reg(0x200017c8u)==0xc0deu && reg(0x200017ccu)==0xa4000000u);
    CHECK(retained_write_order==writes_before+5u && !retained_write_violation);
    CHECK(!backup_reserved_write && !gate);
    return true;
}
static bool test_failed_readiness_records_actual_register_before_recovery(void) {
    CHECK(setup());uint8_t b[56];set(U,0u);set(U+0x1c,TXE|TC);vos_stall=true;
    CHECK(!platform_init());platform_debug_read(b);
    CHECK(word(b)==0x104u && word(b+4)==0x400u && word(b+52)==0u);
    CHECK(setup());set(U,0u);set(U+0x1c,TXE|TC);pll_stall=true;
    CHECK(!platform_init());platform_debug_read(b);
    CHECK(word(b)==0x107u && word(b+4)==0x01000500u);
    return true;
}
static bool test_each_mark_commits_retained_record_without_orderly_reset(void) {
    CHECK(setup());unsigned writes_before=retained_write_order;
    platform_debug_mark(0x701u,0x12345678u);
    CHECK(reg(0x200017c0u)==0xd1a6cafeu && reg(0x200017c4u)==0x701u);
    CHECK(reg(0x200017c8u)==0x12345678u && reg(0x200017ccu)==0xa4000000u);
    CHECK(retained_write_order==writes_before+5u && !retained_write_violation);
    platform_debug_mark(0x702u,0xabcdef01u);
    CHECK(reg(0x200017c4u)==0x702u && reg(0x200017c8u)==0xabcdef01u);
    CHECK(retained_write_order==writes_before+10u && !retained_write_violation);
    return true;
}
static bool test_uart_overrun_is_latched_with_its_raw_status_bit(void) {
    CHECK(setup());burst_due=now_us+2u;burst_remaining=9u;rx_dma_stall=true;
    uint32_t saved=platform_irq_save();for(unsigned i=0;i<95u;++i)step();
    platform_irq_restore(saved);
    CHECK((platform_uart_faults()&0x802u)==0x802u);CHECK(disabled==0u); /* FW231: counted, not fatal */
    {uint32_t c[PLATFORM_UART_COUNTS];platform_uart_counts(c);CHECK(c[3]>=1u && !platform_uart_fatal());}
    CHECK(burst_remaining==0u);
    return true;
}
/* FW231 (fsr4 follow-up): the longest outer masked section's caller is recorded (read 1064 52 B). */
static __attribute__((noinline)) void masked_for(unsigned steps){uint32_t s=platform_irq_save();for(unsigned i=0;i<steps;++i)step();platform_irq_restore(s);}
static __attribute__((noinline)) void masked_nested(unsigned steps){uint32_t s=platform_irq_save();masked_for(steps);platform_irq_restore(s);}
static bool test_longest_masked_section_names_its_caller(void) {
    /* addresses are 32-bit on the M0+; on the 64-bit test host compare the low 32 bits */
    CHECK(setup());platform_irq_max_reset();CHECK(platform_irq_max_site()==0u);
    masked_for(10u);uint32_t a=platform_irq_max_site();
    CHECK(a-(uint32_t)(uintptr_t)masked_for>0u&&a-(uint32_t)(uintptr_t)masked_for<2048u);   /* inside the caller */
    masked_for(3u);CHECK(platform_irq_max_site()==a);                          /* shorter: kept */
    masked_nested(30u);uint32_t b=platform_irq_max_site();                     /* the OUTER caller, not the nested one */
    CHECK(b-(uint32_t)(uintptr_t)masked_nested>0u&&b-(uint32_t)(uintptr_t)masked_nested<2048u);
    platform_irq_max_reset();CHECK(platform_irq_max_site()==0u&&platform_irq_max_us()==0u);
    return true;
}
static bool test_outer_irq_mask_duration_survives_nested_saves(void) {
    CHECK(setup());CHECK(platform_irq_max_us()==0u);
    uint32_t outer=platform_irq_save();for(unsigned i=0;i<5u;++i)step();
    uint32_t inner=platform_irq_save();for(unsigned i=0;i<15u;++i)step();
    platform_irq_restore(inner);for(unsigned i=0;i<4u;++i)step();
    platform_irq_restore(outer);uint32_t measured=platform_irq_max_us();
    CHECK(measured>=24u && measured<=35u);
    outer=platform_irq_save();for(unsigned i=0;i<3u;++i)step();platform_irq_restore(outer);
    CHECK(platform_irq_max_us()==measured);
    uint8_t b[56];platform_debug_read(b);CHECK(word(b+52)==measured);
    return true;
}
static bool test_platform_discards_inherited_interrupt_ownership_before_clock_handoff(void) {
    CHECK(setup());irq_mask=1u;set(0xe000e100u,0xffffffffu);set(0xe000e200u,0xffffffffu);
    set(0xe000ed04u,0x14000000u);set(S,7u);watch_irq_cleanup=true;
    CHECK(platform_init());CHECK(!watch_irq_cleanup && !inherited_irq_violation);
    CHECK(reg(0xe000e100u)==((1u<<27)|(1u<<9)));CHECK(reg(0xe000e200u)==0u);
    CHECK((reg(0xe000ed04u)&0x14000000u)==0u);CHECK(reg(S)==5u);
    return true;
}
static bool test_fifo_buffers_masked_receive_burst_and_preserves_order(void) {
    CHECK(setup());CHECK((reg(U)&FIFOEN)!=0u && !fifo_enable_violation);
    burst_due=now_us+2u;burst_remaining=5u;
    uint32_t saved=platform_irq_save();for(unsigned i=0;i<55u;++i)step();
    platform_irq_restore(saved);CHECK(platform_uart_faults()==0u);
    for(unsigned i=0;i<5u;++i) {uint8_t byte=0;CHECK(platform_uart_pop(&byte));CHECK(byte==0x40u+i);}
    uint8_t byte=0;CHECK(!platform_uart_pop(&byte));CHECK(rx_count==0u&&rx_dma_moves==5u&&rx_rdr_reads==0u);
    return true;
}
static bool test_tx_abort_flushes_queued_fifo_bytes(void) {
    CHECK(setup());uint8_t data[32];for(unsigned i=0;i<32u;++i)data[i]=(uint8_t)i;
    expected_n=sizeof data;inject_error_on_full=true;
    CHECK(!platform_uart_transmit(data,sizeof data));CHECK(tx_flushes==1u && tx_count==0u);
    unsigned completed=wire_n;for(unsigned i=0;i<50u;++i)step();
    CHECK(wire_n<=completed+1u);CHECK(!shift_busy && tx_count==0u);
    CHECK(!gate && (reg(U)&RE) && (platform_uart_faults()&0x406u)==0x406u);
    CHECK(!(reg(U+8u)&TXFTIE) && !(reg(U)&(TXIE|TCIE)));
    return true;
}
static bool test_irq_timing_excludes_pre_disable_interrupt_delay(void) {
    CHECK(setup());pre_save_delay=350u;
    uint32_t saved=platform_irq_save();for(unsigned i=0;i<3u;++i)step();platform_irq_restore(saved);
    CHECK(platform_irq_max_us()>=3u && platform_irq_max_us()<=8u);
    return true;
}
static bool test_fifo_readback_failure_never_enables_uart(void) {
    CHECK(setup());set(U,0u);set(U+0x1c,TXE|TC);fifo_reject=true;
    CHECK(!platform_init());CHECK((reg(U)&1u)==0u);
    uint8_t b[56];platform_debug_read(b);CHECK(word(b)==0x10du && word(b+4)==0u);
    return true;
}
static bool test_full_reply_capacity_transmits640bytes_and_rejects641(void) {
    CHECK(setup());uint8_t data[640];for(unsigned i=0;i<sizeof data;++i)data[i]=(uint8_t)(i^0xa5u);
    expected_n=sizeof data;pause_foreground=true;
    CHECK(platform_uart_transmit(data,sizeof data));CHECK(wire_n==640u && memcmp(data,wire,640u)==0);
    CHECK(!premature_rx && !gate && (reg(U)&RE) && tx_flushes==0u);
    CHECK(dma_irq_entries==1u && tx_irq_entries==0u && txe_irq_entries==0u);
    CHECK(tc_irq_entries==1u && !(reg(U+8u)&TXFTIE));
    CHECK(max_tx_fill==8u);
    CHECK(final_stop-first_wire<=(sizeof data-1u)*11u);
    large_reply_irqs=dma_irq_entries;
    CHECK(setup());CHECK(!platform_uart_transmit(data,641u));
    CHECK(wire_n==0u && !gate && (reg(U)&RE));
    return true;
}
static bool test_half_empty_threshold_is_configured_with_uart_disabled(void) {
    CHECK(setup());CHECK((reg(U+8u)&TXFTCFG)==TXFT_HALF);
    CHECK(!(reg(U+8u)&TXFTIE) && !(reg(U)&TXIE));
    CHECK(!threshold_enable_violation && !fifo_enable_violation);
    return true;
}
static bool test_partial_final_threshold_batch_waits_for_physical_tc(void) {
    CHECK(setup());uint8_t data[14];for(unsigned i=0;i<sizeof data;++i)data[i]=(uint8_t)(i*17u);
    expected_n=sizeof data;pause_foreground=true;
    CHECK(platform_uart_transmit(data,sizeof data));CHECK(wire_n==sizeof data && memcmp(data,wire,sizeof data)==0);
    CHECK(dma_irq_entries==1u && tx_irq_entries==0u && txe_irq_entries==0u);
    CHECK(tc_irq_entries==1u && rx_restore_us>=final_stop && !premature_rx && !gate);
    CHECK(!(reg(U+8u)&TXFTIE) && !(reg(U)&(TXIE|TCIE)) && platform_uart_faults()==0u);
    return true;
}
static bool test_threshold_readback_failure_never_enables_uart(void) {
    CHECK(setup());set(U,0u);set(U+0x1c,TXE|TC);threshold_reject=true;
    CHECK(!platform_init());CHECK(!(reg(U)&1u));
    uint8_t b[56];platform_debug_read(b);CHECK(word(b)==0x10du && word(b+4)==(1u|(1u<<6)|(1u<<13)));
    return true;
}
static bool test_raw_rx_errors_abort_threshold_feeding_and_remain_sticky(void) {
    for(unsigned bit=0;bit<4u;++bit) {
        CHECK(setup());uint8_t data[32];memset(data,0xa5,sizeof data);expected_n=sizeof data;
        inject_error_on_full=true;inject_error_flags=1u<<bit;
        CHECK(!platform_uart_transmit(data,sizeof data));CHECK(disabled==0u); /* FW231 */
        {uint32_t c[PLATFORM_UART_COUNTS];platform_uart_counts(c);CHECK(c[bit]>=1u && c[5]>=1u && !platform_uart_fatal());}
        CHECK((platform_uart_faults()&(6u|(inject_error_flags<<8)))==(6u|(inject_error_flags<<8)));
        CHECK(tx_irq_entries==0u && txe_irq_entries==0u && tx_flushes==1u && tx_count==0u);
        CHECK(!(reg(U+8u)&TXFTIE) && !(reg(U)&(TXIE|TCIE)) && !gate && (reg(U)&RE));
        unsigned completed=wire_n;for(unsigned i=0;i<50u;++i)step();CHECK(wire_n<=completed+1u);
        receive_byte(0x5au);for(unsigned idle=0;idle<20;idle++)step();uint8_t byte=0u;CHECK(platform_uart_pop(&byte) && byte==0x5au);
        CHECK(platform_uart_faults()&(inject_error_flags<<8));
    }
    return true;
}
#ifdef PREFETCH_MAIN_TEST
static unsigned peripheral_start_calls;
bool bridge_init(void){peripheral_start_calls++;return true;}int16_t bridge_applied(void){return 0;}uint32_t bridge_faults(void){return 0;}bool bridge_set(int16_t v,uint16_t c){(void)v;(void)c;abort();}
bool sensing_init(void){peripheral_start_calls++;return true;}bool sensing_encoder_config(uint16_t*a,uint16_t*b){(void)a;(void)b;abort();}bool sensing_read_adc(sensing_adc*a){(void)a;abort();}bool sensing_read_adc_tick(sensing_adc*a){(void)a;abort();}bool sensing_read_encoder(uint16_t*a){(void)a;abort();}bool sensing_encoder_restore(void){abort();}uint32_t sensing_last_faults(void){return 0;}
static bool test_main_prefetch_failure_enters_loader(void){reset_fixture();set(F,0x40600);prefetch_stuck=true;expect_loader=true;peripheral_start_calls=0;if(!setjmp(loader_jump))firmware_main();expect_loader=false;CHECK(!peripheral_start_calls&&disabled&&!(reg(U)&1u)&&reg(S)==0);CHECK(reg(0x200017c4u)==0x10e&&reg(0x200017c8u)==0x40602);puts("actual firmware_main prefetch-failure retained-loader path before sensor/timer/UART PASS");return true;}
#endif
int main(int argc,char**argv) {
#ifdef PREFETCH_MAIN_TEST
    if(argc==2&&!strcmp(argv[1],"main-failure"))return !test_main_prefetch_failure_enters_loader();
#endif
    if(argc==2&&!strcmp(argv[1],"trace")){reset_fixture();set(F,0x40600);CHECK(platform_init());for(unsigned i=0;i<startup_write_count;i++)printf("%08lx %08x\n",(unsigned long)startup_writes[i].address,startup_writes[i].value);return 0;}
    if(argc==2&&!strcmp(argv[1],"exact"))return !test_prefetch_exact_preservation();
    if(argc==2&&!strcmp(argv[1],"failure"))return !test_prefetch_stuck_and_full_readback();
    if(argc==2&&!strcmp(argv[1],"eventual"))return !test_prefetch_eventual_readback();
    unsigned failed=0;
    if(!test_prefetch_exact_preservation())++failed;
    if(!test_prefetch_stuck_and_full_readback())++failed;
    if(!test_prefetch_eventual_readback())++failed;
    if(!test_debug_before_clock_never_uses_uninitialized_time())++failed;
    if(!test_debug_snapshot_reports_live_registers_and_entry_reset_flags())++failed;
    if(!test_loader_retains_three_words_in_sram_with_magic_last())++failed;
    if(!test_failed_readiness_records_actual_register_before_recovery())++failed;
    if(!test_each_mark_commits_retained_record_without_orderly_reset())++failed;
    if(!test_uart_overrun_is_latched_with_its_raw_status_bit())++failed;
    if(!test_outer_irq_mask_duration_survives_nested_saves())++failed;
    if(!test_platform_discards_inherited_interrupt_ownership_before_clock_handoff())++failed;
    if(!test_fifo_buffers_masked_receive_burst_and_preserves_order())++failed;
    if(!test_tx_abort_flushes_queued_fifo_bytes())++failed;
    if(!test_irq_timing_excludes_pre_disable_interrupt_delay())++failed;
    if(!test_fifo_readback_failure_never_enables_uart())++failed;
    if(!test_full_reply_capacity_transmits640bytes_and_rejects641())++failed;
    if(!test_half_empty_threshold_is_configured_with_uart_disabled())++failed;
    if(!test_partial_final_threshold_batch_waits_for_physical_tc())++failed;
    if(!test_threshold_readback_failure_never_enables_uart())++failed;
    if(!test_raw_rx_errors_abort_threshold_feeding_and_remain_sticky())++failed;
    if(!test_clock_and_uart_run_at_requested_rates())++failed;
    if(!test_clock_failures_never_start_uart())++failed;
    if(!test_irq_streams_while_foreground_is_preempted())++failed;
    if(!test_tc_restores_rx_before_foreground_and_preserves_peer_header())++failed;
    if(!test_timeout_aborts_and_releases_bus())++failed;
    if(!test_control_timer_uses_configured_period())++failed;
    if(!test_error_preempting_before_arm_cannot_reassert_de_or_txe())++failed;
    if(!test_reboot_clears_the_boot_time_loader_request())++failed;
    if(!test_hang_reboots_the_application_once_healthy())++failed;
    if(!test_watchdog_started_at_two_seconds())++failed;
    if(!test_watchdog_update_timeout_reported())++failed;
    if(!test_irq_started_reply_streams_and_restores_rx())++failed;
    if(!test_longest_masked_section_names_its_caller())++failed;
    if(!test_irq_start_refused_while_a_byte_arrives())++failed;
    if(!test_receiver_timeout_reaches_the_watcher_once())++failed;
    if(error_enable_lost||data_disable_order_error)++failed;
    printf("platform UART:32 cases,%u failures (MMIO/IRQ/TC-edge model;640 bytes/%u TX threshold IRQs)\n",failed,large_reply_irqs);
    return failed?1:0;
}
