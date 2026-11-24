#include "app.h"

#include "observability.h"
#include "bridge.h"
#include "hw_io.h"
#include "platform.h"
#include "protocol.h"
#include "sensing.h"
#include "deadline.h"
#include "bus.h"
#include "motion.h"
#ifndef APP_MOTION
#define APP_MOTION 1   /* read 1088 motion block; build.sh MOTION=0 leaves it out (LAB builds need that to fit) */
#endif
#define MAIN_STALL_US 250000u
#define FRAME_GAP_US 3000u
/* FW231 fast12 release ID, read 1120 (16 B; owner's library, design approved 2026-11-20):
 *   word 0 = release number (fast12 -> 12; bump with every pinned release) | variant << 16 (0 plain PD, 1 PD-comp,
 *            2 demo, 3 lab) | ABI << 24 (also read 1000 word 0 bits 24-31)
 *   word 1 = first 4 bytes of the release's source-tree SHA-256 (the release build passes FW_SOURCE_ID; 0 in a manual
 *            build); words 2, 3 = 0 (reserved).
 * ABI: 2 up to fast11; 3 from fast12 (reads 1112, 1120 and 1136 added). Bumped whenever the register map changes
 * (fast13 and fast14 change no register; fast15 only adds bit 26 to read 1112 word 0: still 3). 4 from fast16: the torque
 * commands, writes 264 and 268, and the torque-mode bits 2-3 of read 640 word 13.
 * fast17-size (v1.1 Stage 1): release 17, ABI still 4. It only removes registers no FW231 tool uses (reads 1536, 840, 600,
 * 336, 340, 860; writes 840, 336, 340), so openxl330 0.1.0 (MAX_ABI 4) can drive it unchanged; the fast17 feature release
 * (set-ID over the bus) moves to ABI 5. */
/* fast17-size: read 860 (AS5600 STATUS and transfer counters) is not used by the openxl330 host library (bench
 * tools read it: logging builds); the performance images leave it out (build.sh DIAG860=0); fast17: and its tick
 * counters and the first-trip words. Native default 1. */
#ifndef APP_DIAG860
#define APP_DIAG860 1
#endif
#ifndef APP_IDLOG
#define APP_IDLOG 1   /* fast17: set ID over the bus (write/read 1144); build.sh IDLOG=0 leaves it out */
#endif
#include "idlog.h"
/* read 1120 word 0 bits 16-23, the variant: 0 pd, 1 comp (M288 performance), 2 demo, 3 lab, 4 comp-m077 (M077 BETA
 * performance), 5 comp-log (M288 logging: DAQ=1 DIAG860=1), 6 comp-m077-log (M077 BETA logging). */
#define FW231_RELEASE 17u
#define FW231_ABI 5u   /* fast17: write/read 1144 (set ID over the bus); 856 may be read as 32 B (reacquire counters) */
#ifndef FW_SOURCE_ID
#define FW_SOURCE_ID 0u
#endif
#if   defined(APP_LAB) && APP_LAB
#define FW231_VARIANT 3u
#elif defined(APP_DEFAULT_COMP) && APP_DEFAULT_COMP && APP_GEAR == 77 && APP_DAQ && APP_DIAG860
#define FW231_VARIANT 6u   /* Stage 3: PD-comp for the XL330-M077 (BETA), logging build (DAQ ring + read 860) */
#elif defined(APP_DEFAULT_COMP) && APP_DEFAULT_COMP && APP_GEAR == 77
#define FW231_VARIANT 4u   /* Stage 3: PD-comp for the XL330-M077 (BETA), performance build */
#elif defined(APP_DEFAULT_COMP) && APP_DEFAULT_COMP && APP_DAQ && APP_DIAG860
#define FW231_VARIANT 5u   /* owner 2026-11-24: PD-comp, logging build (DAQ ring + read 860) */
#elif defined(APP_DEFAULT_COMP) && APP_DEFAULT_COMP
#define FW231_VARIANT 1u
#else
#define FW231_VARIANT 0u
#endif   /* FW231: Protocol2 inter-byte limit is 1.5 ms (was 100 ms; an earlier review) */
static app_state app;
/* Protocol parsing is single-foreground/nonreentrant; ISR never touches this plan. */
_Static_assert(sizeof(app_write_plan)<=DXL_MAX_READ,"foreground plan fits shared scratch");
static bool loader_requested;
static volatile uint32_t main_progress_us;
static uint32_t boot_us,next_tick_us,recorded_faults;   /* FW231: the unread per-phase tick maxima are gone (flash) */
/* FW231: one reply buffer (the old scheduled_reply copy is gone, -640 B BSS). Invariant: while a
 * reply is pending in work_reply, dxl_process can only write it after the pending reply was
 * cancelled (any CRC-valid instruction cancels first; status frames never reach dxl_process; a
 * CRC-invalid frame returns before any output write). Transmit is synchronous, so nothing writes
 * the buffer while the DMA reads it. */
static uint8_t frame[DXL_MAX_WIRE],work_reply[DXL_REPLY_CAP];
/* Command latency trace (2026-11-16). One record for the latest motion command
 * (260 duty, 256 target, 64 ARM): frame first/last byte pop time, commit time,
 * first tick whose applied PWM differs from the PWM at commit, and first tick
 * whose encoder moved >=2 counts from the position at that apply tick.
 * All times are hw_micros (same clock as DAQ time_us). Read at 976/64 B. */
/* FW231: the FW230 lab reads (976 command trace here; 896 observation snapshot and 864 DMA counters in
 * app.c) are off in every FW231 build to make room for Fast Sync Read + fast start. LAB=1 sh build.sh puts
 * them back. No FW231-era host tool reads them. */
#ifndef APP_LAB
#define APP_LAB 0
#endif
typedef struct {uint32_t seq,address,state,frame_start_us,frame_done_us,commit_us,commit_sample,apply_sample,apply_tick_us,apply_us;int32_t apply_pwm,base_counts;uint32_t move_sample,move_us;int32_t move_counts,pwm_before;} cmd_trace_t;
static cmd_trace_t cmd_trace;
/* Deadline budget (owner, 2026-11-16): a late/overrunning tick is a MISS, output keeps
 * running and the absolute schedule catches up. Fault only if more than 5% of the
 * ticks in the last 1 s (exact sliding window of 2000 ticks) missed. */
static deadline_budget deadline;static uint32_t deadline_skipped;
static void deadline_account(void){
 bool missed=app.tick_missed;app.tick_missed=false;
 for(;deadline_skipped;deadline_skipped--)if(deadline_tick(&deadline,true)){app_fault(&app,APP_FAULT_TIMING);bridge_disable();}
 if(deadline_tick(&deadline,missed)){app_fault(&app,APP_FAULT_TIMING);bridge_disable();}
}
_Static_assert(sizeof(cmd_trace_t)==64,"trace layout");
static uint32_t frame_start_us,frame_done_us,status_reads,status_fails,sensing_transfer_fails,encoder_high_bits;static bool adc_reseed;
/* FW231 bus (bus.h): chained group replies, bus-wide baud with confirm-or-revert (boot is
 * always 1 Mbaud; 844 <- 1M/2M/3M, also broadcast and while armed), effective ID from the
 * device block at 0x080040C0 or the stock NV journal. All foreground-owned. */
static bus_lease lease;static bool cmd_broadcast;
/* FW231 foreground diagnostics (read 1064 48 B): longest gap between main-loop iterations (ISR time
 * included) and what the iteration before it did; parser drops. Reset with write 1064 <- 0. */
static uint32_t parse_gap_resets,parse_loss_resets,status_crc_fails;   /* (FW231 fast5+: the main-loop gap diagnostic is gone; read 1064 words 8-9 read 0) */   /* FW231 opt-in command lease (bus.h item 5): off after boot */
/* FW231 Fast Sync Read 0x8A: a follower's running check of the shared status packet (header + earlier
 * segments) up to its own segment; fast_bad counts packets it stayed silent on (read 1080). */
static bus_fast fastw;static uint32_t fast_bad;
static uint32_t loader_crc;   /* fast13: the loader's CRC-32 (read 1136), computed once at boot */
/* FW231 fast9 (hardware bus3-05 2026-11-20: 4-byte rows lost, the middle servo never armed): read 1080 words 8-11,
 * since boot. fsr_seen: CRC-valid 0x8A requests parsed (any ID list); fsr_rows: of those, ones we armed a row for;
 * hdr_missed: packet starts (FF FF FD 00) that arrived outside a skip while the parser was not ready for them (a skip
 * ended inside one, or a frame overran it); parse_dropped: frames the parser rejected (an invalid length, or a 0x8A with
 * a bad CRC); frames dropped unfinished (frame gap, RX loss) are read 1064 word 11. */
static uint32_t fsr_seen,fsr_rows,hdr_missed,parse_dropped;
static bus_reply reply;static bus_baud baud;static uint8_t bus_id,devcfg_state;static bool reboot_requested;static uint32_t boot_bps=1000000u;
/* fast17 (v1.1 Stage 2): the ID set over the bus (write 1144), kept in the ID log page (firmware/idlog.h). Precedence:
 * the log's newest record (unless "no override") > the device block override > the stock NV journal ID. id_source:
 * 0 NV, 1 device block, 2 log. A write switches bus_id only once no reply is pending, so its reply leaves from the old
 * ID. frame_unicast: the frame being processed was addressed to this servo's ID (not 0xFE): 1144 needs it. */
#if APP_IDLOG
static void devcfg_copy(uint8_t out[BUS_DEVCFG_BYTES]);
static idlog id_log;static uint8_t id_source,id_switch_to;static bool id_switch_pending,frame_unicast;
/* the effective ID and its source, from the device block / NV journal and the log */
static uint8_t id_resolve(uint8_t *source){uint8_t block[BUS_DEVCFG_BYTES];devcfg_copy(block);uint8_t id=bus_devcfg_resolve(block,app.nv_config.id,&devcfg_state);
 *source=devcfg_state==BUS_DEVCFG_OVERRIDE?1u:0u;if(id_log.id!=IDLOG_NONE){id=id_log.id;*source=2u;}return id;}
static uint8_t id_write(const uint8_t *in,uint16_t n){
 if(n!=8u)return 5;
 if(in[0]!=0x53u||in[1]!=0x49u||in[2]!=0x44u||in[3]!=0x31u||in[5]||in[6]||in[7]||(in[4]>252u&&in[4]!=IDLOG_NONE))return 4;
 if(!frame_unicast)return 7;                                                     /* never from a broadcast / group frame */
 uint32_t irq=platform_irq_save();bool off=!app.enabled&&bridge_applied()==0&&joint_off(&app.control);platform_irq_restore(irq);
 if(!off)return 7;
 uint8_t r=idlog_write(&id_log,in[4]);if(r)return r;
 uint8_t source;id_switch_to=id_resolve(&source);id_source=source;id_switch_pending=true;return 0;}
static void id_read(uint8_t out[8]){uint16_t used=id_log.formatted&&id_log.next?(uint16_t)(id_log.next-1u):0u,free_=id_log.formatted?(uint16_t)(IDLOG_SLOTS-id_log.next):(uint16_t)IDLOG_RECORDS;
 out[0]=id_log.id;out[1]=id_source;out[2]=(uint8_t)used;out[3]=(uint8_t)(used>>8);out[4]=(uint8_t)free_;out[5]=(uint8_t)(free_>>8);out[6]=(uint8_t)((id_log.formatted?1u:0u)|(id_log.ecc_seen?2u:0u));out[7]=0;}
#endif
/* FW231 fast start (owner 2026-11-20 "Yes, with fast start"): a chained or Fast Sync Read reply may be started as
 * soon as the receiver timeout says the line has been idle for the chain gap and the watcher (bus_watch, on its own
 * RX-ring cursor) has seen the predecessor complete as the last thing on the wire, without waiting for the main
 * loop. The USART IRQ (priority 0, it also releases DE at TC) only notes the timeout and pends PendSV; the watcher
 * runs in PendSV, the LOWEST priority, below TIM16: bus work can never delay a control tick (hardware fsr2-02
 * 2026-11-20: 316 us of watcher work inside the priority-0 IRQ stretched ticks to 617 us and latched TIMING).
 * The main loop keeps its own path as the fallback. Claim: the watcher may start only while watch_on, under PRIMASK
 * with no byte received since it checked; the main loop clears watch_on (watch_stop, under PRIMASK) before it
 * transmits, cancels or abandons, and finalizes once at the top of the loop when watch_sent. At most WATCH_MAX
 * bytes per run, else the main loop takes over. RAM knob: write 1084 (1 B, 0 off / 1 on, default on); read 1080. */
#ifndef APP_FAST_START
#define APP_FAST_START 1   /* build.sh: 0 for the demo image (flash) */
#endif
enum{WATCH_MAX=160u};
static bus_watch watch;static volatile bool watch_on,watch_sent,rto_due;static bool fast_start=APP_FAST_START;static uint16_t watch_len;
static uint32_t watch_cursor,watch_tx_us,watch_irq_max_us,watch_started,fast_armed,fast_superseded;
#if APP_FAST_START
void platform_uart_rto(void){if(watch_on){rto_due=true;hw_write32((uintptr_t)0xe000ed04u,1u<<28);}}   /* USART IRQ: pend PendSV */
static void watch_run(void){   /* PendSV: TIM16, USART and DMA preempt it; the main loop cannot run meanwhile */
 rto_due=false;if(!watch_on)return;
 uint32_t t0=hw_micros(),irq=platform_irq_save(),w=platform_uart_rx_written();platform_irq_restore(irq);uint8_t st=BUS_WATCH_CAPTURING;
 if(w-watch_cursor>WATCH_MAX)st=BUS_WATCH_DEAD;   /* too far behind: the main loop decides */
 else st=bus_watch_run(&watch,platform_uart_rx_ring(),PLATFORM_RX_RING-1u,&watch_cursor,w);
 if(st!=BUS_WATCH_CAPTURING){   /* decided: start now (nothing received since), or leave it to the main loop */
  /* seal BEFORE the mask (hardware fsr3: sealing a 72-byte row inside it held PRIMASK 81 us); only our reply
   * buffer is written and the main loop cannot run in PendSV; if we do not send, it seals the same bytes itself */
  if(st==BUS_WATCH_READY&&watch.kind==BUS_REPLY_FAST)bus_fast_seal(&watch.fast,work_reply,watch_len);
  irq=platform_irq_save();
  if(st==BUS_WATCH_READY&&watch_on&&platform_uart_rx_written()==watch_cursor&&platform_uart_tx_irq(work_reply,watch_len)){watch_tx_us=hw_micros();watch_started++;watch_sent=true;}
  watch_on=false;platform_irq_restore(irq);}
 uint32_t d=hw_micros()-t0;if(d>watch_irq_max_us)watch_irq_max_us=d;   /* wall time, TIM16 preemption included */
}
#endif
static bool watch_stop(void){if(!APP_FAST_START)return false;uint32_t irq=platform_irq_save();watch_on=false;bool sent=watch_sent;platform_irq_restore(irq);return sent;}
/* Status LED (owner, 2026-11-20): blink after reset, then OFF; FW231: address 65 (stock LED) sets it. */
static bool led_done,led_on=true;
static volatile uint8_t lab_hang;   /* fast11 LAB images only (write 1116): a hang on purpose, after the reply went out */
static void trace_tick(uint32_t tick_us,uint32_t applied_us,uint32_t sample_us){
 if(!APP_LAB){(void)tick_us;(void)applied_us;(void)sample_us;return;}
 if(cmd_trace.state==1u&&app.samples!=cmd_trace.commit_sample&&app.actual_pwm!=cmd_trace.pwm_before){
  cmd_trace.state=2u;cmd_trace.apply_sample=app.samples;cmd_trace.apply_tick_us=tick_us;cmd_trace.apply_us=applied_us;cmd_trace.apply_pwm=app.actual_pwm;cmd_trace.base_counts=app.position_counts;
 }else if(cmd_trace.state==2u&&app.samples!=cmd_trace.apply_sample){
  int32_t d=app.position_counts-cmd_trace.base_counts;if(d<0)d=-d;
  if(d>=2){cmd_trace.state=3u;cmd_trace.move_sample=app.samples;cmd_trace.move_us=sample_us;cmd_trace.move_counts=app.position_counts;}
 }
}
/* OFF now, then (main loop) the loader (F9 / a stall) or, for 0x08 Reboot, a reset back into this application.
 * fast17: one function for both requests and one for both exits (flash). */
static void request_exit(bool *flag){uint32_t irq=platform_irq_save();bridge_disable();
 uint8_t off=0;(void)app_write(&app,64,1,&off);*flag=true;platform_irq_restore(irq);}
#define request_loader() request_exit(&loader_requested)
#define request_reboot() request_exit(&reboot_requested)
static void exit_orderly(bool loader) __attribute__((noreturn));
static void exit_orderly(bool loader){(void)platform_irq_save();bridge_disable();(void)sensing_encoder_restore();if(loader)platform_enter_loader();platform_reset_app();}
#define reboot_orderly() exit_orderly(false)
#define enter_loader_orderly() exit_orderly(true)
static void devcfg_copy(uint8_t out[BUS_DEVCFG_BYTES]){const volatile uint8_t *p=(const volatile uint8_t *)(uintptr_t)BUS_DEVCFG_ADDRESS;for(unsigned i=0;i<BUS_DEVCFG_BYTES;i++)out[i]=p[i];}
static bool frame_crc_ok(const uint8_t *f,uint16_t n){return n>=10u&&dxl_crc(f,(uint16_t)(n-2u))==(uint16_t)(f[n-2u]|((uint16_t)f[n-1u]<<8));}

static void enforce_disabled_expiry(uint32_t now){
 (void)now;
}
void TIM16_IRQHandler(void){
 uint32_t start=hw_micros();platform_control_timer_ack();enforce_disabled_expiry(start);
 /* GOBS tick observation: its only reader is the lab read 896 (fast9: LAB builds only, flash) */
 uint32_t obs_irq=0,obs_cycles=0,obs_entries=0;if(APP_LAB){obs_irq=platform_irq_save();observation_activity++;obs_cycles=observation_uart_cycles;obs_entries=observation_uart_entries;platform_irq_restore(obs_irq);}uint32_t origin=next_tick_us;
 /* fast10 (review F-6): a stalled main loop cannot act on loader_requested itself (and this tick keeps the IWDG fed): act now.
  * fast11 (owner: "a hung servo should reboot back into its firmware"): once healthy, reboot this application (recorded,
  * read 1112); before that, or after 5 quick hangs in a row, the loader as before (platform_hang_reset). */
 if((uint32_t)(start-main_progress_us)>MAIN_STALL_US){request_loader();app_fault(&app,APP_FAULT_TIMEOUT);platform_hang_reset(PLATFORM_RESET_STALL);}
 int32_t delta=(int32_t)(start-next_tick_us);uint32_t late=delta>0?(uint32_t)delta:0;
 /* Review 2026-11-20: the skipped period(s) are the misses; the period serviced now is judged by its own
  * deadline below (elapsed/pending checks). Marking it missed here too counted 51 late releases as 102. */
 if(delta>=(int32_t)APP_PERIOD_US){app_note_timing_trip(&app,1,late,0,0,0,0,0,app.samples+1u);
  /* v11: TIM16 free-runs, so a service later than one period merged whole periods into this
   * call. Resync to the period that is due now and count each merged one as a miss. */
  uint32_t skipped=(uint32_t)delta/APP_PERIOD_US;next_tick_us+=skipped*APP_PERIOD_US;deadline_skipped+=skipped;}
 next_tick_us+=APP_PERIOD_US;
 /* Angle every tick (proven 2-byte read); AS5600 STATUS every 8th tick in its own
  * 1-byte transaction (the 3-byte STATUS+RAW burst returned out-of-range words on the bench servo).
  * A failed STATUS read leaves the last status in force and is counted, not judged. */
 uint16_t raw=0;bool encoder_ok=sensing_read_encoder(&raw);uint32_t angle_us=hw_micros();   /* sample time: before the every-8th STATUS read (an earlier review) */if(APP_DIAG860&&encoder_ok&&(sensing_last_encoder_word()&0xf000u))encoder_high_bits++;
 if(encoder_ok&&(app.samples&7u)==0u){uint8_t st=0;if(sensing_read_encoder_status_byte(&st)){app.encoder_status=(uint16_t)(0x100u|st);if(APP_DIAG860)status_reads++;}else if(APP_DIAG860)status_fails++;}uint32_t encoder_end=hw_micros();   /* fast17: 860 counters, logging builds */
 /* v11: after a failed tick conversion the cache is invalid by design, so the next tick
  * reseeds with a full read instead of failing forever (app reuses the last good sample). */
 uint32_t hardware=sensing_last_faults();sensing_adc adc;bool adc_ok=adc_reseed?sensing_read_adc(&adc):sensing_read_adc_tick(&adc);adc_reseed=!adc_ok;uint32_t adc_end=hw_micros();
 hardware|=sensing_last_faults();
 /* Faults latched before this acquired endpoint describe the interval about
  * to settle. Mark forced OFF now, not after ordinary settlement has spent it.
  * Events after this poll are beyond encoder_end; retained postchecks mark
  * their newly pending interval. The extra ADC/poll delay is conservative. */
 /* v11: an I2C/ADC transfer failure is a BAD SAMPLE judged by app_tick's 5% budget, not an
  * immediate fault (independent review 2026-11-16). UART and bridge faults stay immediate. */
 if(APP_DIAG860&&hardware)sensing_transfer_fails++;
 if(platform_uart_fatal()||bridge_faults())app_fault(&app,APP_FAULT_SENSOR);
 bool has=app_tick(&app,encoder_ok,raw,&adc,adc_ok,angle_us);
 app.hardware_faults|=hardware;app.hardware_faults|=platform_uart_fatal()<<16;
 if(platform_uart_fatal()||bridge_faults())app_fault(&app,APP_FAULT_SENSOR);

 uint32_t elapsed=hw_micros()-start;
 if(elapsed>=APP_PERIOD_US)app_note_timing_trip(&app,2,late,elapsed,encoder_end-start,adc_end-encoder_end,elapsed-(adc_end-start),0,app.samples);
 app_record_timing(&app,elapsed);
 if(elapsed>=APP_PERIOD_US || platform_control_timer_pending()){if(elapsed<APP_PERIOD_US)app_note_timing_trip(&app,3,late,elapsed,encoder_end-start,adc_end-encoder_end,elapsed-(adc_end-start),0,app.samples);app.tick_missed=true;}
 uint32_t calc_end=hw_micros();
 uint32_t saved=platform_irq_save();bool applied=false;bool authorized_zero=has&&app.control.planned_pwm==0;
 bool permit=has && app.enabled && app.ready && app.motor_allowed && app.sensor_healthy && !app.faults && !platform_uart_fatal() && !bridge_faults();
 if((uint32_t)(hw_micros()-start)>=APP_PERIOD_US || platform_control_timer_pending())app.tick_missed=true; /* late output still applied */
 if(permit && (platform_uart_fatal()||bridge_faults())){app_fault(&app,APP_FAULT_SENSOR);permit=false;}
 if(permit){int16_t physical=(app.nv_config.drive_mode&1u)?(int16_t)-app.control.planned_pwm:app.control.planned_pwm;applied=bridge_set(physical,480u);if(!applied && bridge_faults())app_fault(&app,APP_FAULT_SENSOR);else if(!applied){bridge_disable();app_fault(&app,APP_FAULT_SENSOR);}}else bridge_disable();
 /* Recheck after the physical output transaction, immediately before commit. */
 if((uint32_t)(hw_micros()-start)>=APP_PERIOD_US || platform_control_timer_pending())app.tick_missed=true;
 if(platform_uart_fatal()||bridge_faults()){app_fault(&app,APP_FAULT_SENSOR);permit=false;bridge_disable();}
 int16_t physical=bridge_applied();int16_t logical=(app.nv_config.drive_mode&1u)?(int16_t)-physical:physical;
 if(has)(void)app_commit(&app,logical,(authorized_zero&&logical==0)|| (permit&&applied));
 trace_tick(start,hw_micros(),angle_us);
 platform_irq_restore(saved);
 if(APP_DAQ)app_record_applied(&app);if(app.faults)bridge_disable();
 elapsed=hw_micros()-start;uint32_t output_elapsed=start+elapsed-calc_end;
 if(elapsed>=APP_PERIOD_US)app_note_timing_trip(&app,4,late,elapsed,encoder_end-start,adc_end-encoder_end,calc_end-adc_end,output_elapsed,app.samples);
 app_record_timing(&app,elapsed);
 if(elapsed>=APP_PERIOD_US || platform_control_timer_pending()){if(elapsed<APP_PERIOD_US)app_note_timing_trip(&app,5,late,elapsed,encoder_end-start,adc_end-encoder_end,calc_end-adc_end,output_elapsed,app.samples);app.tick_missed=true;}
 if(app.faults && !recorded_faults){platform_debug_mark(0xf000u|app.faults,app.hardware_faults);recorded_faults=app.faults;}if(!app.faults)recorded_faults=0;
 /* fast12: no watchdog refresh here. The tick keeps running when the main loop hangs; refreshing from it would hide that.
  * The main loop refreshes; a stalled main loop is caught by the 250 ms check above (recorded), a frozen CPU by the IWDG. */
 /* Last pending check covers the ordinary guard/debug/watchdog tail. The
  * check itself and exception epilogue remain outside sampled elapsed. */
 if(platform_control_timer_pending()){app_note_timing_trip(&app,6,late,hw_micros()-start,encoder_end-start,adc_end-encoder_end,calc_end-adc_end,output_elapsed,app.samples);app.tick_missed=true;}
 deadline_account();
 /* GOBS tick observation (published here, consumed in PendSV): its one reader is lab read 896, so LAB builds only (FW231) */
 if(APP_LAB){obs_irq=platform_irq_save();uint32_t obs_now=hw_micros();obs_publish(&observation,app.samples,origin,obs_now-start,observation_uart_cycles-obs_cycles,observation_uart_entries-obs_entries);hw_write32((uintptr_t)0xe000ed04u,1u<<28);platform_irq_restore(obs_irq);}
}
void PendSV_Handler(void){
 if(APP_LAB){uint32_t irq=platform_irq_save();observation_activity++;uint32_t now=hw_micros();(void)obs_consume(&observation,now);platform_irq_restore(irq);}
#if APP_FAST_START
 if(rto_due)watch_run();   /* FW231 fast start, below TIM16 */
#endif
}
static void scan_stack_off(void){uint32_t irq=platform_irq_save();bool eligible=!app.enabled && bridge_applied()==0 && (observation_stack.flags&1u);uint32_t epoch=observation_activity;platform_irq_restore(irq);if(!eligible){irq=platform_irq_save();obs_watermark_commit(&observation_stack,observation_stack.free_bytes,false,false,false);platform_irq_restore(irq);return;}bool bottom=false;uint16_t free_b=obs_watermark_measure(&observation_stack,&bottom);irq=platform_irq_save();bool off=!app.enabled && bridge_applied()==0;obs_watermark_commit(&observation_stack,free_b,bottom,off,epoch==observation_activity);platform_irq_restore(irq);}
/* FW231 diagnostic reads: one snapshot of 32-bit words under PRIMASK, the length picks how many (flash). 0xff: not one.
 * 860 (12/20 B) encoder STATUS, status reads, fails, transfer fails, angle-word high bits.  852 (16 B) deadline budget.
 * 844 (4 B) baud word.  1000 (64 B) bus diagnostics, word 0 = ID | NV ID<<8 | devcfg state<<16 | start-up baud code<<18
 *   (3/4/5) | boot state<<21 (0 none,1 waiting,2 switched,3 rescued) | ABI 2<<24.
 * 1072 (16 B) lease: ms (0 off), action (0 OFF/1 hold), state (0 off,1 running,2 expired)|expiries<<8, ms since renewal.
 * 1064 (16/24/32/48 B) reply timing since the last reset (write 1064 <- 0): max request end -> TX start (direct, direct
 *   while armed), max predecessor status -> TX start (chained), longest PRIMASK section; 24 B adds max request end ->
 *   predecessor seen and predecessors finishing past our deadline; 32 B adds max slot-0 group latency and
 *   instruction|address<<16 of the slowest direct reply; 48 B adds the longest main-loop gap (us) and the step before it
 *   (1 reply sent, 2 request processed, 3 stack scan, 4 baud switch, 5 status parsed, 6 byte parsed, 7 reply deferred,
 *   8 idle), deepest RX backlog (bytes), parser drops (gap | loss<<8 | bad-CRC status<<16); 52 B adds the code address
 *   (caller's return address) of the longest masked section (0 = an inline one), to name it from the symbol table.
 * 1080 (16/24 B) Fast Sync Read / chain gap (RAM test knob): gap floor us (0 = default 40) | fast start on<<31, fast
 *   segments sent, abandoned (predecessor never completed by the deadline), silent on a bad prefix (header/CRC/
 *   predecessor ID); 24 B adds replies started by fast start and the watcher's longest run (us, wall time incl. TIM16
 *   preemption; write 1064 <- 0 resets); 32 B adds Fast Sync Read rows armed and rows still pending when a new request
 *   came, follower rows only (slot > 0; armed = sent + abandoned + bad prefix + superseded + pending); 48 B (fast9) adds
 *   0x8A requests seen (CRC-valid), 0x8A rows armed (any slot), packet starts missed, frames rejected by the parser. */
static uint8_t read_words(uint16_t a,uint16_t n,uint8_t *out){
 uint32_t v[16];bool ok;uint32_t irq=platform_irq_save();
 switch(a){
#if APP_DIAG860
 case 860u:v[0]=app.encoder_status;v[1]=status_reads;v[2]=status_fails;v[3]=sensing_transfer_fails;v[4]=encoder_high_bits;ok=n==12u||n==20u;break;
#endif
 case 1112u:platform_reset_info(v);ok=n==12u;break;   /* fast11: last reset cause, hang reboots, quick-hang streak */
 case 1120u:v[0]=FW231_RELEASE|((uint32_t)FW231_VARIANT<<16)|(FW231_ABI<<24);v[1]=FW_SOURCE_ID;v[2]=v[3]=0u;ok=n==16u;break;   /* fast12: release ID */
 case 852u:v[0]=deadline.total;v[1]=deadline.window;v[2]=deadline.window_max;v[3]=DEADLINE_MISS_LIMIT|(DEADLINE_WINDOW<<16);ok=n==16u;break;
 case 844u:v[0]=bus_baud_word(&baud);ok=n==4u;break;
 case 1000u:{uint32_t c[PLATFORM_UART_COUNTS];platform_uart_counts(c);
  v[0]=bus_id|((uint32_t)app.nv_config.id<<8)|((uint32_t)devcfg_state<<16)|((uint32_t)bus_baud_index(boot_bps)<<18)|((uint32_t)baud.boot_state<<21)|(FW231_ABI<<24);v[1]=bus_baud_word(&baud);
  for(unsigned i=0;i<6u;i++)v[2+i]=c[i];
  v[8]=reply.direct;v[9]=reply.chained;v[10]=reply.fallback;v[11]=reply.slotted;v[12]=reply.superseded;v[13]=reply.statuses;v[14]=reply.deferrals;v[15]=(baud.switches&0xffffu)|(baud.reverts<<16);ok=n==64u;break;}
 case 1072u:v[0]=lease.ms;v[1]=lease.action;v[2]=lease.state|(lease.expiries<<8);v[3]=lease.state==BUS_LEASE_RUNNING?(hw_micros()-lease.last_us)/1000u:0u;ok=n==16u;break;
 case 1064u:v[0]=reply.lat_direct_max;v[1]=0u;v[2]=reply.lat_chain_max;v[3]=platform_irq_max_us();v[4]=reply.pred_done_max;v[5]=reply.late_chains;
  v[6]=v[7]=v[8]=v[9]=0u;v[10]=platform_uart_rx_backlog_max();
  v[11]=(parse_gap_resets&0xffu)|((parse_loss_resets&0xffu)<<8)|(status_crc_fails<<16);v[12]=platform_irq_max_site();ok=n==16u||n==24u||n==32u||n==48u||n==52u;break;
 case 1080u:v[0]=reply.gap_min_us|((uint32_t)fast_start<<31);v[1]=reply.fast_sent;v[2]=reply.fast_dropped;v[3]=fast_bad;v[4]=watch_started;v[5]=watch_irq_max_us;
  v[6]=fast_armed;v[7]=fast_superseded;v[8]=fsr_seen;v[9]=fsr_rows;v[10]=hdr_missed;v[11]=parse_dropped;ok=n==16u||n==24u||n==32u||n==48u;break;
 default:platform_irq_restore(irq);return 0xffu;
 }
 platform_irq_restore(irq);if(!ok||!out)return 5;
 for(unsigned i=0;i<n;i++)out[i]=(uint8_t)(v[i>>2]>>(8u*(i&3u)));return 0;
}
static uint8_t read_atomic(void *ctx,uint16_t a,uint16_t n,uint8_t *out){
 uint8_t w=read_words(a,n,out);if(w!=0xffu)return w;
 /* FW231 motion block (motion.h): one tick's coherent inputs copied under PRIMASK, the estimate computed outside it */
 if(APP_MOTION&&a==MOTION_ADDRESS){if(n!=MOTION_BYTES||!out)return 5;uint32_t irq=platform_irq_save();
  motion_in m={app.position_counts,app.control.v_hat_q16,app_planned_accel(&app),app.actual_pwm,(uint16_t)(app.adc.voltage_decivolts*100u),app.adc.current_magnitude_ma,app.enabled};
  platform_irq_restore(irq);motion_block(&m,out);return 0;}
 if(a==1048u){if(n!=BUS_DEVCFG_BYTES||!out)return 5;devcfg_copy(out);return 0;}
#if APP_IDLOG
 if(a==1144u){if(n!=8u||!out)return 5;id_read(out);return 0;}   /* fast17: ID log state */
#endif
 /* read 1136 (8 B): CRC-32 of the retained loader and its size. A0 never reads that region (fast12). DEFINITION, stable across
  * releases: CRC-32/ISO-HDLC = zlib.crc32 (reflected polynomial 0xEDB88320, init 0xFFFFFFFF, final XOR 0xFFFFFFFF) over the
  * 16384 bytes 0x08000000..0x08003FFF in address order; word 1 = 16384. fast13 (owner: "we want that"): computed once at
  * boot (the loader cannot change while this firmware runs), so the read answers at once; fast12 computed it per read
  * (about 13 ms in the main loop on hardware). */
 if(a==1136u){if(n!=8u||!out)return 5;const uint32_t v[2]={loader_crc,APP_LOADER_BYTES};for(unsigned i=0;i<8u;i++)out[i]=(uint8_t)(v[i>>2]>>(8u*(i&3u)));return 0;}
 if(APP_LAB&&a==976u){if(n!=68u||!out)return 5;uint32_t irq=platform_irq_save();cmd_trace_t c=cmd_trace;uint32_t now=hw_micros();platform_irq_restore(irq);const uint32_t *w=(const uint32_t *)&c;
  for(unsigned i=0;i<17u;i++){uint32_t v=i<16u?w[i]:now;out[4*i]=(uint8_t)v;out[4*i+1]=(uint8_t)(v>>8);out[4*i+2]=(uint8_t)(v>>16);out[4*i+3]=(uint8_t)(v>>24);}return 0;}
 if(APP_DAQ&&a==1536u){if(!n||n>DXL_MAX_READ||n%APP_RECORD_BYTES)return 5;for(uint16_t i=0;i<n;i+=APP_RECORD_BYTES){uint32_t irq=platform_irq_save();uint8_t result=app_read(ctx,a,APP_RECORD_BYTES,out+i);platform_irq_restore(irq);if(result)return result;}return 0;}
 if((uint32_t)a+n<=256u){if(!ctx||!out||!n)return 5;uint32_t snap[APP_LOW_SNAPSHOT_WORDS];uint32_t irq=platform_irq_save();app_low_snapshot(ctx,snap);platform_irq_restore(irq);app_low_format(ctx,snap,a,n,out);return 0;}
 uint32_t irq=platform_irq_save();uint8_t result=app_read(ctx,a,n,out);platform_irq_restore(irq);return result;
}
static uint8_t write_atomic(void *ctx,uint16_t a,uint16_t n,const uint8_t *in);
/* Apply-on-receive for open-loop duty (2026-11-16). Called under PRIMASK right
 * after a successful 260 write. Same gating as the TIM16 output permit; if a tick
 * is already pending it simply applies the duty instead. Saves the 0-0.5 ms wait
 * for the next tick plus its ~0.22 ms acquisition. */
static void apply_openloop_now(void){
 joint_control *c=&app.control;
 if(!app.openloop||!app.enabled||!app.ready||!app.motor_allowed||!app.sensor_healthy||app.faults||platform_uart_fatal()||bridge_faults()||platform_control_timer_pending()||!c->configured||c->terminal||c->history<3)return;
 if(app.openloop_pwm==c->actual_pwm)return;
 if(!joint_prepare_pwm(c,app.openloop_pwm,(uint16_t)(app.adc.voltage_decivolts*100u),true))return;
 int16_t physical=(app.nv_config.drive_mode&1u)?(int16_t)-c->planned_pwm:c->planned_pwm;
 if(!bridge_set(physical,480u)){bridge_disable();app_fault(&app,APP_FAULT_SENSOR);return;}
 uint32_t now=hw_micros();
 (void)app_commit(&app,c->planned_pwm,true);app.commanded_pwm=c->planned_pwm;
 if(APP_LAB&&cmd_trace.state==1u){cmd_trace.state=2u;cmd_trace.apply_sample=app.samples;cmd_trace.apply_tick_us=now;cmd_trace.apply_us=now;cmd_trace.apply_pwm=app.actual_pwm;cmd_trace.base_counts=app.position_counts;}
}
static uint8_t write_untraced(void *ctx,uint16_t a,uint16_t n,const uint8_t *in);
/* Wraps motion commands: commit time and pre-commit PWM recorded under PRIMASK. */
static uint8_t write_traced(void *ctx,uint16_t a,uint16_t n,const uint8_t *in){
 bool motion=a==260u||a==256u||(a==64u&&n==1u&&in[0]==1u);if(!APP_LAB||!motion)return 0xffu;
 /* Arm BEFORE the write so a tick landing between commit and arming is never missed. */
 uint32_t irq=platform_irq_save();cmd_trace_t prior=cmd_trace;
 cmd_trace=(cmd_trace_t){.seq=prior.seq+1u,.address=a,.state=1u,.frame_start_us=frame_start_us,.frame_done_us=frame_done_us,.commit_us=hw_micros(),.commit_sample=app.samples,.pwm_before=app.actual_pwm};
 platform_irq_restore(irq);
 uint8_t r=write_untraced(ctx,a,n,in);
 irq=platform_irq_save();if(r)cmd_trace=prior;else cmd_trace.commit_us=hw_micros();platform_irq_restore(irq);
 return r;
}
static uint32_t get32(const uint8_t *p){return (uint32_t)p[0]|(uint32_t)p[1]<<8|(uint32_t)p[2]<<16|(uint32_t)p[3]<<24;}
static uint8_t write_untraced(void *ctx,uint16_t a,uint16_t n,const uint8_t *in){
 if(!ctx||!in||!n)return 5;
 if(a==256u || a==272u || (a==64u && n==1u && in[0]==1u)){
  /* FW231 (review): a commit that only lost a race with the control tick (tick pending, or state
   * changed between capture and commit) is retried; a Sync Write has no reply to report a silent 7.
   * A semantic busy from prepare (e.g. a profile still running) is returned at once. */
  uint8_t result=7;
  for(unsigned attempt=0;attempt<3u&&result==7u;attempt++){
   app_write_plan *plan=dxl_foreground_scratch();uint32_t irq=platform_irq_save();app_write_capture(ctx,plan);platform_irq_restore(irq);
   result=app_write_prepare(plan,a,n,in);if(result)return result;
   irq=platform_irq_save();
   if(platform_uart_fatal()){app_fault(&app,APP_FAULT_SENSOR);bridge_disable();platform_irq_restore(irq);return 7;}
   result=platform_control_timer_pending()?7u:app_write_commit(ctx,plan);
   if(app.faults||!app.enabled)bridge_disable();platform_irq_restore(irq);
  }
  return result;
 }
 if(a==320u){
  /* PD configuration: OFF/terminal only, parsed under PRIMASK (48 B, no
   * foreground/ISR split). The tick cannot observe a half-written config.
   * fast12: a control tick due at the check is retried here (the due tick runs between attempts), like 256/ARM; only a
   * servo that is not OFF answers 7 (hardware fsr11-01-id2: that race reached the host as a 7). */
  uint8_t result=7;
  for(unsigned attempt=0;attempt<3u&&result==7u;attempt++){
   uint32_t irq=platform_irq_save();
   if(app.enabled||bridge_applied()!=0||!joint_off(&app.control)||!app.control.terminal){platform_irq_restore(irq);return 7;}
   result=platform_control_timer_pending()?7u:joint_configure(&app.control,in,n)?0u:4u;
   platform_irq_restore(irq);
  }
  return result;
 }
 if(a==844u){if(n!=4u)return 5;return bus_baud_request(&baud,get32(in));}   /* FW231: 1M/2M/3M, also broadcast and while armed; confirm-or-revert */
 if(a==65u){if(n!=1u)return 5;if(in[0]>1u)return 4;led_done=true;led_on=in[0]!=0u;platform_led(led_on);return 0;}  /* FW231: stock LED */
 if(APP_DIAG860&&a==1080u){if(n!=4u)return 5;uint32_t v=get32(in);   /* fast17: test knob, logging builds */
  if(v>2000u)return 4;reply.gap_min_us=v;return 0;}   /* FW231: chain/segment gap floor in us, RAM only (0 = 40 us default; never below 1 byte) */
 if(APP_DIAG860&&APP_FAST_START&&a==1084u){   /* fast17: A/B test knob, logging builds */if(n!=1u)return 5;if(in[0]>1u)return 4;fast_start=in[0]!=0u;return 0;}   /* FW231: fast start on/off, RAM only (A/B tests) */
 if(APP_LAB&&a==1116u){if(n!=1u)return 5;if(in[0]<1u||in[0]>3u)return 4;lab_hang=in[0];return 0;}   /* fast11 LAB only: 1 stall, 2 fault, 3 watchdog */
 if(APP_DIAG860&&a==1064u){if(n!=4u)return 5;   /* fast17: reset of the timing maxima, a logging-build test action */if(in[0]|in[1]|in[2]|in[3])return 4;bus_reply_reset_maxima(&reply);platform_irq_max_reset();watch_irq_max_us=0u;
  parse_gap_resets=parse_loss_resets=status_crc_fails=0u;return 0;}  /* reset timing maxima */
 uint32_t irq=platform_irq_save();uint8_t result=app_write(ctx,a,n,in);
 if(!result&&a==260u)apply_openloop_now();
 if(app.faults||!app.enabled)bridge_disable();platform_irq_restore(irq);return result;
}
static uint8_t write_atomic(void *ctx,uint16_t a,uint16_t n,const uint8_t *in){
 /* FW231 command lease (opt-in deployment option, owner 2026-11-20). Renewed only by accepted motion
  * commands or the 1076 heartbeat addressed to this servo; never by reads or a plain broadcast write. */
 bool arm=a==64u&&n==1u&&in&&in[0]==1u;
 if(a==1072u)return bus_lease_config(&lease,in,n,hw_micros());
#if APP_IDLOG
 if(a==1144u)return id_write(in,n);   /* fast17: set ID (unicast, OFF only; in flash before the reply) */
#endif
 if(a==1076u){if(n!=1u)return 5;if(!cmd_broadcast)bus_lease_renew(&lease,hw_micros());return 0;}
 /* fast10 (review F-2): only while ARMED (holding). Disarmed (OFF written or a fault) a new target must be accepted, else the
  * next ARM plans from where the shaft is to the stale expiry target (a step when its duration was 1). */
 uint8_t gate=bus_lease_gate(&lease,a,arm,app.enabled);if(gate==7u)return 7;if(gate){bus_lease_rearm(&lease,hw_micros());return 0;}  /* expired in hold: re-arm first */
 uint8_t r=write_traced(ctx,a,n,in);if(r==0xffu)r=write_untraced(ctx,a,n,in);
 if(!r){if(arm)bus_lease_rearm(&lease,hw_micros());else if(bus_lease_motion(a)&&!cmd_broadcast)bus_lease_renew(&lease,hw_micros());}
 return r;}
static uint8_t flash_read(void *ctx,uint32_t a,uint16_t n,uint8_t *out){return app_flash_read(ctx,a,n,out);}

/* Status LED (owner, 2026-11-20): blink ~3.8 Hz (131 ms on / 131 ms off) for 3 s after every reset or
 * power-up, then OFF until a host writes address 65. platform_init latched it ON; only BSRR/BRR writes here (atomic). */
static void led_service(uint32_t now){
 if(led_done)return;uint32_t t=now-boot_us;
 if(t>=3000000u){platform_led(false);led_on=false;led_done=true;return;}
 bool on=((t>>17)&1u)==0u;if(on!=led_on){platform_led(on);led_on=on;}
}

void firmware_main(void){
 platform_boot_capture();   /* fast11: why the last reset happened (read 1112), before the loader request is armed again */
 if(!platform_arm_loader())platform_enter_loader();if(!platform_init())platform_enter_loader();
 bool bridge_ok=bridge_init(),sensor_ok=sensing_init();uint16_t before=0,after=0;if(sensor_ok)sensor_ok=sensing_encoder_config(&before,&after);sensing_adc seed;if(sensor_ok)sensor_ok=sensing_read_adc(&seed);
 loader_crc=app_loader_crc32();   /* fast13: once, before interrupts and the tick start (about 6-10 ms of boot) */
 (void)app_init(&app,(const uint8_t *)(uintptr_t)0x0800e000u,(const uint8_t *)(uintptr_t)0x0800f000u,APP_ALLOW_MOTOR!=0);app.sensor_conf_before=before;app.sensor_conf_after=after;
 /* FW231: effective ID (device block, else stock NV journal) and the live baud are what addresses 7/8 report. */
 {uint8_t block[BUS_DEVCFG_BYTES];devcfg_copy(block);bus_id=bus_devcfg_resolve(block,app.nv_config.id,&devcfg_state);boot_bps=bus_devcfg_boot_baud(block);}
#if APP_IDLOG
 idlog_scan(&id_log);bus_id=id_resolve(&id_source);   /* fast17: a logged ID wins */
#endif
 bus_reply_init(&reply);bus_baud_init(&baud);app.nv.bytes[7]=bus_id;app.nv.bytes[8]=bus_baud_index(baud.current);

 if(!bridge_ok||!sensor_ok)app_fault(&app,APP_FAULT_SENSOR);uint32_t shpr=hw_read32((uintptr_t)0xe000ed20u);hw_write32((uintptr_t)0xe000ed20u,(shpr&~(255u<<16))|(192u<<16));hw_write32((uintptr_t)0xe000ed04u,1u<<27);boot_us=hw_micros();main_progress_us=boot_us;bus_baud_boot(&baud,boot_bps,boot_us);next_tick_us=platform_control_timer_start()+APP_PERIOD_US;platform_irq_enable();platform_watchdog_start();   /* fast12: IWDG on (2 s) */
 uint16_t have=0,total=0,pending_size=0;uint32_t last_byte_us=boot_us,rx_loss_seen=platform_uart_rx_loss();bool pending_loader=false,pending_reboot=false,pending_fast=false;uint16_t skip_left=0,fast_after=0;uint32_t hdr=0;bool healthy_marked=false,long_run_marked=false;
 /* FW231 (hardware fsr-02 2026-11-20: the foreground ran 62..73 bytes behind the wire, one byte per loop
  * iteration): received bytes are taken in batches of up to 32 under one mask and parsed in an inner loop. */
 uint8_t rxb[32];uint32_t rxn=0,rxi=0;
 for(;;){
#if APP_IDLOG
  if(id_switch_pending&&!pending_size){id_switch_pending=false;bus_id=id_switch_to;app.nv.bytes[7]=bus_id;}   /* fast17: after the 1144 reply */
#endif
  if(loader_requested&&joint_off(&app.control))enter_loader_orderly();if(reboot_requested&&joint_off(&app.control))reboot_orderly();uint32_t now=hw_micros();enforce_disabled_expiry(now);main_progress_us=now;platform_watchdog_reload();led_service(now);
  if(APP_FAST_START&&watch_sent){watch_sent=false;   /* fast start: the USART IRQ sent our reply; account it like the foreground would */
   if(pending_size){if(!reply.triggered)bus_reply_trigger(&reply,watch_tx_us-reply.gap_us);bus_reply_sent(&reply,watch_tx_us);
    if(pending_fast){int32_t rem=(int32_t)(watch_cursor-platform_uart_rx_next())+(int32_t)(rxn-rxi);   /* prefix bytes not parsed yet */have=total=0;skip_left=(uint16_t)((rem>0?rem:0)+fast_after);}
    pending_size=0;pending_fast=false;}}
  /* fast10 (review F-4): our own TX (RE off, nothing received) is line activity for the 3 ms frame-gap timer: a row longer
   * than ~3 ms (>= ~290 B at 1M) otherwise cleared skip_left in the middle of the Fast Sync Read packet, and the parser hunted
   * for headers inside the unstuffed rows after ours. */
  if(!platform_uart_tx_idle())last_byte_us=now;
  /* fast11: healthy = 2 s up with the control tick running (>= 3000 ticks): hangs from now on reboot this application.
   * 60 s up: a later hang starts no quick-hang streak (the boot-loop guard counts only hangs within 60 s of boot). */
  if(!healthy_marked&&(uint32_t)(now-boot_us)>=2000000u&&app.samples>=3000u){platform_app_healthy();healthy_marked=true;}
  if(!long_run_marked&&(uint32_t)(now-boot_us)>=60000000u){platform_app_long_run();long_run_marked=true;}
  if(APP_LAB&&lab_hang&&!pending_size&&platform_uart_tx_idle()){uint8_t k=lab_hang;lab_hang=0;
   if(k==1u)for(;;){}                                        /* main loop stuck, ticks run: the stall check fires */
   if(k==2u)__asm__ volatile("udf #0");                      /* HardFault */
   (void)platform_irq_save();for(;;){}}                      /* 3: everything stuck: only the IWDG (if running) resets */
  if(platform_uart_fatal()){uint32_t irq=platform_irq_save();app_fault(&app,APP_FAULT_SENSOR);bridge_disable();platform_irq_restore(irq);}
  if(bus_lease_expire(&lease,now,app.enabled)){uint32_t irq=platform_irq_save();   /* opt-in lease ran out: configured response, once */
   if(lease.action==BUS_LEASE_ACT_OFF){uint8_t off=0;(void)app_write(&app,64,1,&off);bridge_disable();}
   else{static const uint8_t hold[4]={0,0,0,0x80};(void)app_write(&app,260,4,hold);}   /* INT32_MIN: back to PD at the current position */
   platform_irq_restore(irq);}
  /* FW231: take the next received byte BEFORE the reply due-check, so bytes that queued up while the tick ISR
   * held us count as bus activity (hardware 2026-11-20, bus2-03: 81 replies went out after "idle" checks
   * made with bytes of the predecessor still unread). It is parsed below, in order, as before. */
  if(rxi==rxn){rxi=0;rxn=platform_uart_read(rxb,sizeof rxb);
   if(rxn){if((have||skip_left)&&(uint32_t)(now-last_byte_us)>=FRAME_GAP_US){if(have)parse_gap_resets++;have=total=0;skip_left=0;}last_byte_us=now;
    uint32_t loss=platform_uart_rx_loss();if(loss!=rx_loss_seen){rx_loss_seen=loss;if(have)parse_loss_resets++;have=total=0;skip_left=0;}}}   /* bytes were lost: resync on the next header */
  bool rx_held=rxi<rxn;
  if(pending_size){uint32_t irq=platform_irq_save();if(!watch_sent&&bus_reply_abandon(&reply,now,last_byte_us,rx_held)){watch_on=false;pending_size=0;pending_fast=false;}platform_irq_restore(irq);}   /* Fast Sync Read: predecessor never completed */
  if(pending_size && bus_reply_due(&reply,now,last_byte_us,rx_held,baud.current)){if(reply.kind!=BUS_REPLY_DIRECT&&watch_stop())continue;   /* the IRQ sent it */
   uint32_t t0=hw_micros();bool deferred=false;
   /* group replies claim the line only if it is quiet right then (review): else parse what arrived, decide again */
   bool sent=reply.kind==BUS_REPLY_DIRECT?platform_uart_transmit(work_reply,pending_size):platform_uart_transmit_if_quiet(work_reply,pending_size,&deferred);
   if(deferred){last_byte_us=hw_micros();continue;}
   bus_reply_sent(&reply,t0);pending_size=0;have=total=0;last_byte_us=hw_micros();   /* fast10 (review F-4): the TX just ended */
   /* Fast Sync Read: RE was off during our part, and the rest of the packet is unstuffed: skip it, never hunt in it */
   if(pending_fast){have=total=0;skip_left=fast_after;pending_fast=false;}if(pending_loader&&sent)request_loader();if(pending_reboot&&sent)request_reboot();pending_loader=pending_reboot=false;main_progress_us=hw_micros();continue;}
  if(!pending_size&&platform_uart_tx_idle()){
   /* baud change: a confirmed 844 request, the end of the start-up rescue window, or an unconfirmed switch reverting */
   uint8_t why=bus_baud_switch_ready(&baud)?1u:bus_baud_boot_due(&baud,now)?2u:bus_baud_revert_due(&baud,now)?3u:0u;
   if(why){bool ok=platform_uart_set_baud(why==1u?baud.target:why==2u?baud.boot_target:baud.previous);platform_uart_rx_discard();uint32_t t=hw_micros();
    if(why==1u)bus_baud_switched(&baud,ok,t);else if(why==2u)bus_baud_boot_done(&baud,ok);else bus_baud_reverted(&baud,ok,t);
    app.nv.bytes[8]=bus_baud_index(baud.current);have=total=0;skip_left=0;rxn=rxi=0;continue;}
  }
  if(!rx_held){if((have||skip_left)&&(uint32_t)(now-last_byte_us)>=FRAME_GAP_US){if(have)parse_gap_resets++;have=total=0;skip_left=0;}
   /* stack-watermark scan (~200 us) only on a quiet bus: never between the bytes of a frame or while a reply waits.
    * FW231: LAB builds only (its one reader is the lab read 896). */
   if(APP_LAB&&!have&&!pending_size&&(uint32_t)(now-last_byte_us)>=2000u){scan_stack_off();}
   continue;}
  /* FW231 fast9 FIX (hardware bus3-05 2026-11-20, 4-byte Fast Sync Read rows lost ~0.07 %, always the middle servo):
   * never parse a byte while the watcher's send is unaccounted. The do/while below parses one byte before it checks
   * watch_sent; if the watcher sent just after the check at the top and a tick delayed this batch read until the next
   * servo's row had begun, that first byte was past watch_cursor, rem came out -1 (clamped to 0) and skip_left one
   * too long ate the 0xFF of the next request. Race-free: a batch read before the send holds only prefix bytes.
   * Model: tests/test_fsr_loop.c. */
  if(APP_FAST_START&&watch_sent)continue;
  do{uint8_t byte=rxb[rxi++];   /* inner loop: 'continue' takes the next received byte */
  hdr=(hdr<<8)|byte;if(hdr==0xfffffd00u&&!skip_left&&have!=3u)hdr_missed++;   /* a packet start the parser was not ready for */
  /* Fast Sync Read follower: check the shared packet byte by byte (from its first 0xFF) up to our segment */
  if(pending_size&&reply.kind==BUS_REPLY_FAST&&!reply.triggered){uint8_t st=bus_fast_feed(&fastw,byte);
   if(st==BUS_FAST_READY){bus_fast_seal(&fastw,work_reply,pending_size);bus_reply_trigger(&reply,last_byte_us);}
   else if(st==BUS_FAST_FAILED&&!watch_stop()){pending_size=0;pending_fast=false;bus_reply_done(&reply);fast_bad++;}}
  if(skip_left){skip_left--;continue;}
  if(have==0){if(byte==0xff){frame[have++]=byte;frame_start_us=now;}continue;}if(have==1){if(byte==0xff)frame[have++]=byte;else have=0;continue;}if(have==2){if(byte==0xfd)frame[have++]=byte;else have=byte==0xff?2:0;continue;}if(have==3&&byte!=0){have=byte==0xff?1:0;continue;}frame[have++]=byte;
  if(have==7){uint16_t len=(uint16_t)(frame[5]|((uint16_t)frame[6]<<8));if(len<3||len>DXL_MAX_WIRE-7){if(len>DXL_MAX_WIRE-7&&frame[4]==0xfeu)skip_left=len;else parse_dropped++;have=total=0;continue;}   /* long broadcast-ID frame (Fast Sync Read status is unstuffed): skip it whole */total=(uint16_t)(len+7);}
  if(!total||have!=total)continue;frame_done_us=last_byte_us;
  uint8_t instruction=frame[7];
  /* FW231: another servo's status packet. It confirms a new baud and can release our chained reply. */
  if(instruction==0x55u){if(frame_crc_ok(frame,have)){bus_baud_frame(&baud);bus_observe_status(&reply,frame[4],last_byte_us);}else status_crc_fails++;have=total=0;continue;}
  /* Any new CRC-valid instruction (for any ID) supersedes a pending group reply and confirms a new baud. */
  bool wants=pending_size||bus_baud_wants_frames(&baud),crc_ok=(wants||instruction==0x8au)&&frame_crc_ok(frame,have);
  if(instruction==0x8au){if(crc_ok)fsr_seen++;else parse_dropped++;}   /* fast9 diagnostics */
  if(wants&&crc_ok){bus_baud_frame(&baud);if(pending_size&&watch_stop())watch_sent=false;   /* fast start off before dxl_process reuses work_reply */
   if(pending_fast)fast_superseded++;   /* a Fast Sync Read row still pending when a new request came */
   if(bus_observe_instruction(&reply)){pending_size=0;pending_loader=pending_reboot=pending_fast=false;}}
 cmd_broadcast=frame[4]==0xfeu&&instruction==3u;   /* plain broadcast write: never renews a lease */
#if APP_IDLOG
  frame_unicast=frame[4]==bus_id;
#endif
 uint32_t irq=platform_irq_save();dxl_io io={.id=bus_id,.firmware=APP_FIRMWARE_VERSION,.model=app_model(&app),.context=&app,.read=read_atomic,.write=write_atomic,.flash_read=flash_read,.alert=app.faults?0x80:0};platform_irq_restore(irq);
  dxl_result result=dxl_process(&io,frame,have,work_reply,sizeof work_reply);have=total=0;cmd_broadcast=false;
#if APP_IDLOG
  frame_unicast=false;
#endif
  if(!result.valid_for_us)continue;bus_baud_frame(&baud);
  pending_size=result.reply_bytes;pending_loader=result.enter_loader;pending_reboot=result.reboot;pending_fast=pending_size&&result.fast;
  if(pending_fast){fsr_rows++;fast_armed+=result.response_slot!=0u;fast_after=(uint16_t)(result.fast_len+7u-result.prior_bytes-result.fast_seg);   /* followers' bytes after ours */
   if(result.response_slot)bus_fast_begin(&fastw,result.prior_bytes,result.fast_seg,result.fast_len,result.prev_id);}
  if(pending_size){bus_reply_arm(&reply,&result,instruction,frame_done_us,baud.current);}else bus_reply_done(&reply);
  if(APP_FAST_START&&fast_start&&(reply.kind==BUS_REPLY_CHAIN||reply.kind==BUS_REPLY_FAST)){uint32_t irq=platform_irq_save();
   bus_watch_begin(&watch,reply.kind,reply.prev_id,&fastw);watch_cursor=platform_uart_rx_next()-(rxn-rxi);watch_len=pending_size;watch_sent=false;   /* right after the request: batch bytes not parsed yet included */
   platform_uart_set_rto(reply.gap_us*(baud.current/1000000u));watch_on=true;platform_irq_restore(irq);}
  break;   /* a request was processed: back to the top for its reply */
  }while(rxi<rxn&&!(APP_FAST_START&&watch_sent));
 }
}
