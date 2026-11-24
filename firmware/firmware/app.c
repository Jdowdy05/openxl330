#include "app.h"

#include "observability.h"
#include "bam_native.h"
#include "platform.h"
#include "hw_io.h"
#include <limits.h>
#include <stddef.h>
#ifndef APP_LAB
#define APP_LAB 0   /* FW231: FW230 lab reads 864/896 only with LAB=1 */
#endif
_Static_assert(sizeof(app_speed_guard)==16,"speed guard fixed RAM");
_Static_assert(sizeof(torque_velocity_t)==32,"velocity fixed RAM");
_Static_assert(offsetof(torque_velocity_t,last_fraction_q16)==12,"fraction reuses padding");
_Static_assert(sizeof(app_record)==APP_RECORD_BYTES,"DAQ layout");
static uint16_t u16(const uint8_t *p){return (uint16_t)(p[0]|((uint16_t)p[1]<<8));}
static uint32_t u32(const uint8_t *p){return (uint32_t)u16(p)|((uint32_t)u16(p+2)<<16);}
static int32_t i32(const uint8_t *p){uint32_t x=u32(p);return x<=INT32_MAX?(int32_t)x:(int32_t)((int64_t)x-INT64_C(4294967296));}
__attribute__((unused)) static void p16(uint8_t *p,uint16_t v){p[0]=(uint8_t)v;p[1]=(uint8_t)(v>>8);}
__attribute__((unused)) static void p32(uint8_t *p,uint32_t v){p16(p,(uint16_t)v);p16(p+2,(uint16_t)(v>>16));}
/* PD-comp image (owner, 2026-11-20: "the PD but with the geartrain cancelation integrated into the controller").
 * Power-up compensation = the demo5/demo6 gear-train cancellation, loaded through the same validated setters as the
 * host writes 324/336/340: friction 85 % (c0 68451, c1 20054, veps 0.05 rad/s), b 0.021375 Nm*s/rad with duty drop
 * 0.006175, rotor-inertia feedback 60 % (J_fb 9040 Q24, fast accel, afilt 2), near-stop inertia gate 0.3 rad/s,
 * high-pass damping 0.030 Nm*s/rad @ shift 7. PD-comp v2 (2026-11-20, ID 3 trajectory tests traj-ablate-id3-pdcomp-01..03):
 * high-pass damping OFF (it resists absolute speed changes, so under PD it pushes past the target as a move stops:
 * 1-3 deg overshoot) and reference-acceleration feed-forward ja = 0.38e-3 kg*m^2 (residual horn-only inertia after
 * 60 % rotor cancellation; a heavier load only under-compensates). Boots OFF; the host still writes config (320), arms and sets gains as
 * before, and can change or zero any of these live. Only built in when APP_DEFAULT_COMP=1. */
#if APP_GEAR == 77
/* XL330-M077 BETA (Stage 3, DERIVED, not measured: docs/M077_BETA.md): PD-comp v2 scaled by
 * r = 77.5/288.4 = 0.268724: c0 x r (motor-side friction, the low bound: under-compensates if gearbox friction dominates),
 * c1 unchanged, b and b_drop x r^2 (back-EMF/viscous, motor side), J_fb x r^2 (60 % of the reflected rotor), ja = horn
 * 2e-6 + 40 % of the M077 rotor 6.48e-5, veps and the inertia gate x 1/r. Host config: K 0.013544 Nm/V (Q24 227225). */
static const uint32_t app_comp_default[10]={18394u,20054u,101u,469u,12194u,0u,0u,653u,0x102u,JC_COMP_MAGIC};
static const uint32_t app_comp2_default[4]={29u,73164u,1u,JC_COMP2_MAGIC};
#else
static const uint32_t app_comp_default[10]={68451u,20054u,1401u,6375u,3277u,0u,0u,9040u,0x102u,JC_COMP_MAGIC};   /* v2: ja 6375 Q24 = 0.38e-3 kg*m^2 */
static const uint32_t app_comp2_default[4]={405u,19661u,1u,JC_COMP2_MAGIC};
#endif
#if APP_TUNE2
static const uint32_t app_hpd_default[3]={0u,0u,JC_HPD_MAGIC};   /* v2: high-pass damping OFF (hand-feel thumb fix; under PD it pushes past the target at the end of a move) */
/* The setters parse little-endian bytes; on the (little-endian) M0+ and native hosts the uint32 tables already
 * are that layout (FW231: no copy loops, flash). */
_Static_assert(sizeof app_hpd_default==12u,"hpd default size");
#endif
_Static_assert(sizeof app_comp_default==40u&&sizeof app_comp2_default==16u,"comp default sizes");
_Static_assert(__BYTE_ORDER__==__ORDER_LITTLE_ENDIAN__,"comp default tables are passed as little-endian bytes");
bool app_load_default_comp(joint_control *c){
 return joint_set_comp(c,(const uint8_t *)app_comp_default,40)&&joint_set_comp2(c,(const uint8_t *)app_comp2_default,16)
#if APP_TUNE2
  &&joint_set_hpd(c,(const uint8_t *)app_hpd_default,12)
#endif
  ;}   /* !APP_TUNE2: high-pass damping stays off as joint_init left it (kd 0, shift 0, v_lp = v_hat = 0): the same state */
/* bitwise integer sqrt, 32 iterations max (runs under PRIMASK in app_read) */
static uint32_t isqrt64(uint64_t v){uint64_t r=0,b=UINT64_C(1)<<62;while(b>v)b>>=2;while(b){if(v>=r+b){v-=r+b;r=(r>>1)+b;}else r>>=1;b>>=2;}return (uint32_t)r;}
static int64_t divr(int64_t n,int64_t d){return n<0?-((-n+d/2)/d):(n+d/2)/d;}
static bool counts_q16(int32_t counts,int32_t *out){int64_t q=divr((int64_t)counts*411775,4096);if(q<INT32_MIN||q>INT32_MAX)return false;*out=(int32_t)q;return true;}
static void motor_off(app_state *s){s->speed_guard.ready=false;s->speed_guard.ticks=0;s->enabled=false;s->commanded_pwm=0;s->actual_pwm=0;s->reference.active=false;s->reference.v=0;s->reference.a=0;joint_stop(&s->control);s->epoch++;}
static void stop(app_state *s){motor_off(s);s->bam.running=false;s->openloop=false;s->openloop_pwm=0;s->control.tau_mode=JC_TAU_PD;}
static bool bam_equal(const app_bam *a,const app_bam *b){return a->center_counts==b->center_counts&&a->center_q16==b->center_q16&&a->tick==b->tick&&a->stride==b->stride&&a->kind==b->kind&&a->selected==b->selected&&a->running==b->running&&a->released==b->released&&a->learning==b->learning&&a->first==b->first;}
#if APP_BAM
/* Conservative position envelopes (FW226 bam_center, unchanged). */
static bool bam_center(uint8_t kind,int32_t counts,int32_t *q){static const int32_t lo[4]={-65534,-102944,0,-168480},hi[4]={65534,0,102944,168480};return counts_q16(counts,q)&&(int64_t)*q+lo[kind]>=INT32_MIN&&(int64_t)*q+hi[kind]<=INT32_MAX;}
/* Native BAM profile (FW226 semantics): tick advances every control tick while
 * running, even after lift_and_drop releases output; terminal at 6 s turns OFF. */
static bool bam_reference(app_state *s){
 if(s->bam.first)s->bam.first=false;else s->bam.tick++;
 bam_output b;if(!bam_eval(s->bam.kind,s->bam.tick,s->bam.center_q16,&b))return false;
 if(b.enable==2u&&!s->bam.released){motor_off(s);s->bam.released=true;}
 if(b.terminal){motor_off(s);s->bam.running=false;}
 s->reference.q=b.q;s->reference.v=b.v;s->reference.a=b.a;s->reference.elapsed=s->bam.tick;s->reference.duration=12000u;s->reference.active=b.active;return true;
}
#else
/* FW231 deployment/demo builds: native BAM profiles (272) are left out (flash; an earlier review). */
static bool bam_reference(app_state *s){(void)s;return false;}
#endif
void app_fault(app_state *s,uint32_t f){if(!s)return;
 s->faults|=f;stop(s);s->ready=false;s->good_ticks=0;}
bool app_init(app_state *s,const uint8_t *cal,const uint8_t *nv,bool allow){
 if(!s)return false;*s=(app_state){0};s->motor_allowed=allow && APP_ALLOW_MOTOR;joint_init(&s->control);
#if APP_DEFAULT_COMP
 if(!app_load_default_comp(&s->control)){app_fault(s,APP_FAULT_CONFIG);return false;}
#endif
 s->duration_ticks=2000;s->travel_limit=8192;s->speed_limit=15000;s->current_rms_limit_ma=1000u;s->temp_limit_c=65u;s->vmax_dv=80u;s->upper_volt_on=true;   /* fast17 (owner 2026-11-24): 8.0 V upper trip, 1152 = 1 */
 if(xl330_nv_open(&s->nv,nv,4096))s->nv_model=u16(s->nv.bytes);
 if(!s->nv_model || !xl330_nv_config_read(&s->nv,&s->nv_config) || s->nv_model!=APP_MODEL_NUMBER || !xl330_calibration_load(&s->calibration,cal,4096)){app_fault(s,APP_FAULT_CONFIG);return false;}
 s->startup_valid=true;return true;
}
void app_note_timing_trip(app_state *s,uint32_t site,uint32_t late,uint32_t elapsed,uint32_t enc,uint32_t adc,uint32_t calc,uint32_t out,uint32_t sample){if(!s||!site||s->first_trip[0])return;s->first_trip[0]=site;s->first_trip[1]=late;s->first_trip[2]=elapsed;s->first_trip[3]=enc;s->first_trip[4]=adc;s->first_trip[5]=calc;s->first_trip[6]=out;s->first_trip[7]=sample;}
/* A late tick is now a counted deadline miss, not a fault (main.c deadline budget). */
void app_record_timing(app_state *s,uint32_t elapsed){if(!s)return;if(elapsed>s->worst_tick_us)s->worst_tick_us=elapsed;if(elapsed>=APP_PERIOD_US)s->tick_missed=true;}
/* v11 (independent review 2026-11-16): integrity limits. */
#if APP_GEAR == 77
#define APP_JUMP_COUNTS 32   /* M077 BETA: 98 rad/s at the output, 2x the 6 V no-load speed (47.8 rad/s; M288: 12 = 36.8 rad/s, 2.9x) */
#else
#define APP_JUMP_COUNTS 12
#endif
#define APP_COAST_FREEZE_TICKS 20u
#define APP_REACQ_CONFIRM 4u   /* fast17: two-sample confirmation from this many bad samples in a row */
bool app_tick(app_state *s,bool enc,uint16_t raw,const sensing_adc *adc,bool adc_ok,uint32_t now){
 if(!s)return false;bool observation_ok=false;int32_t obs_counts=0,obs_fraction=0;s->last_flags=0;uint32_t dt=s->have_tick?now-s->last_tick_us:APP_PERIOD_US;s->have_tick=true;s->last_tick_us=now;s->now_us=now;s->samples++;
 /* Sample integrity (owner, 2026-11-16). Neither the AS5600 nor the ADC has a CRC, so a
  * sample is BAD when: the I2C/ADC transfer failed, the angle word is out of range, the
  * AS5600 reports no magnet (STATUS.MD clear, when main supplied STATUS), or the angle
  * jumped more than 12 counts per tick (v11; was 30) (x ticks since the last good one). A bad encoder
  * sample coasts on the estimator prediction; a bad ADC sample reuses the last good one.
  * Fault only if more than 5% of the last second's samples were bad. */
 s->hardware_faults=0;s->coast=false;
 bool adc_good=adc&&adc_ok&&!adc->faults;
 if(adc_good){s->adc=*adc;s->have_adc=true;}else s->hardware_faults|=(adc&&adc->faults)?adc->faults:0x40000000u;
 bool enc_good=enc&&raw<=4095u;
 if(enc_good&&(s->encoder_status&0x100u)){
  if(!(s->encoder_status&AS5600_MD)){enc_good=false;s->magnet_lost++;}
  else if(s->encoder_status&(AS5600_ML|AS5600_MH))s->magnet_warnings++;}
 /* fast17 sensor reacquire (design 2026-11-20): ONE speed bound for the jump gate and the tracker,
  * lim = APP_JUMP_COUNTS per elapsed tick (capped below a half turn); after a dropout of APP_REACQ_CONFIRM or more bad
  * samples, the first in-gate sample is only a candidate (counted bad) and the next one is accepted only if it agrees
  * with it within 2*APP_JUMP_COUNTS: a single glitch inside the wide gate is never latched as the position. */
 int32_t lim=APP_JUMP_COUNTS*(1+(int32_t)s->bad_streak);if(lim>2047)lim=2047;bool reacq=false;
 if(enc_good&&s->tracker.ready){int32_t d=((int32_t)raw-(int32_t)s->raw_angle)&4095;if(d>=2048)d-=4096;
  if(d>lim||d< -lim){enc_good=false;s->jump_rejects++;}
  else if(s->bad_streak>=APP_REACQ_CONFIRM){int32_t e=((int32_t)raw-(int32_t)s->reacq_cand)&4095;if(e>=2048)e-=4096;
   if(!s->have_cand||e>2*APP_JUMP_COUNTS||e< -2*APP_JUMP_COUNTS){if(s->have_cand)s->reacq_rejects++;s->reacq_cand=raw;s->have_cand=true;enc_good=false;}
   else{s->reacquisitions++;reacq=true;}}}
 if(!enc_good){s->hardware_faults|=0x80000000u;if(s->bad_streak<255u)s->bad_streak++;}   /* widen the gate on EVERY bad sample, also on early-exit ticks */
 if(!s->startup_valid){s->sensor_healthy=false;app_fault(s,APP_FAULT_SENSOR);goto finish;}
 if(bucket_tick(&s->sample_budget,!enc_good||!adc_good)){s->sensor_healthy=false;app_fault(s,APP_FAULT_SENSOR);goto finish;}
 if(!s->have_adc){s->sensor_healthy=false;goto finish;}
 /* PD baseline protections (owner, 2026-11-16): ONLY voltage, temperature and current
  * overload shut output off. No travel, speed or comms-timeout trips. */
 /* fast17 (owner 2026-11-24: "hard cap voltage limit to 8V and if its over 7V a warning ... can be toggled off", "the
  * opt-in voltage warning thing turns off the 8V voltage limit as well as the 7V one"): with 1152 = 1 (boot default) the
  * upper trip is the 848 vmax (default and cap 8.0 V) and status word 3 bit 0 warns above 7.0 V (clears at <= 6.8 V,
  * never latches, output stays on); 1152 = 0 turns both off. The 3.5 V lower trip always applies. */
 if(s->upper_volt_on){if(s->adc.voltage_decivolts>70u)s->volt_warn=1u;else if(s->adc.voltage_decivolts<=68u)s->volt_warn=0u;}else s->volt_warn=0u;
 if(s->adc.voltage_decivolts<35u||(s->upper_volt_on&&s->adc.voltage_decivolts>s->vmax_dv)){s->sensor_healthy=false;app_fault(s,APP_FAULT_VOLTAGE);goto finish;}
 if(s->adc.temperature_c>=s->temp_limit_c){s->sensor_healthy=false;app_fault(s,APP_FAULT_TEMPERATURE);goto finish;}
 /* Current overload = I^2t: first-order RMS filter, tau 4096 ticks (2.05 s). Short bursts
  * (accelerations, reversals) never trip; a sustained stall above the RMS limit does. 0 = off. */
 {uint64_t i=s->adc.current_magnitude_ma;s->i2_q12+=i*i-(s->i2_q12>>12);
  if(s->current_rms_limit_ma&&(s->i2_q12>>12)>(uint64_t)s->current_rms_limit_ma*s->current_rms_limit_ma){s->sensor_healthy=false;app_fault(s,APP_FAULT_CURRENT);goto finish;}}
 s->sensor_healthy=true;
 if(!dt||dt>TORQUE_MAX_DT_US){app_fault(s,APP_FAULT_TIMING);goto finish;}
 if(!enc_good){s->coast=s->tracker.ready;goto reference;}
 uint32_t gap=1u+s->bad_streak;int64_t y_old=(int64_t)s->position_counts*65536+s->velocity.last_fraction_q16;
 s->bad_streak=0;s->have_cand=false;
 int64_t pos=0;int32_t fraction=0;bool first=!s->tracker.ready;int64_t home=s->nv_config.homing_offset_raw;if(s->nv_config.drive_mode&1u)home=-home;
 uint16_t step=s->tracker.maximum_step;if(!first&&lim+64>(int32_t)step)s->tracker.maximum_step=(uint16_t)(lim+64>2047?2047:lim+64);   /* reacquire: + calibration slack */
 bool ok=first?(home>=INT32_MIN && home<=INT32_MAX && xl330_tracker_begin_fraction(&s->tracker,&s->calibration,raw,(s->nv_config.drive_mode&1u)!=0,(int32_t)home,128,&pos,&fraction)):xl330_tracker_update_fraction(&s->tracker,&s->calibration,raw,&pos,&fraction);
 if(!first)s->tracker.maximum_step=step;
 if(!ok||pos<INT32_MIN||pos>INT32_MAX){app_fault(s,APP_FAULT_SENSOR);goto finish;}
 s->position_counts=(int32_t)pos;s->raw_angle=raw;
 if(reacq)joint_reseed(&s->control,pos*65536+fraction,y_old,gap);   /* fast17 reacquire: see joint_reseed */
 if(!torque_velocity_update_fractional(&s->velocity,s->position_counts,fraction,now)){app_fault(s,APP_FAULT_SENSOR);goto finish;}s->velocity_mrad_s=s->velocity.velocity_mrad_s;
 if(first){int32_t q;if(!counts_q16(s->position_counts,&q)){app_fault(s,APP_FAULT_CONFIG);goto finish;}general_reference_init(&s->reference,q);if(!s->target_configured)s->target_counts=s->position_counts;}
 if(s->good_ticks<20u)s->good_ticks++;s->ready=s->good_ticks>=20u && !s->faults && s->sensor_healthy;


 observation_ok=true;obs_counts=s->position_counts;obs_fraction=fraction;
reference:
 if(s->bam.running){if(!bam_reference(s)){app_fault(s,APP_FAULT_CONFIG);observation_ok=false;}}
 else if(s->enabled&&!general_reference_tick(&s->reference)){app_fault(s,APP_FAULT_CONFIG);observation_ok=false;}
finish:
 if(s->control.configured&&!s->control.busy){
  if(s->coast&&joint_coast(&s->control,s->bad_streak>APP_COAST_FREEZE_TICKS)){}
  else if(!joint_observe(&s->control,obs_counts,obs_fraction,observation_ok)&&(observation_ok||s->enabled))app_fault(s,APP_FAULT_SENSOR);
  bool permit=s->enabled&&s->ready&&!s->faults;uint16_t mv=(uint16_t)(s->adc.voltage_decivolts*100u);
  bool has=s->openloop?joint_prepare_pwm(&s->control,s->openloop_pwm,mv,permit):joint_prepare(&s->control,(int64_t)s->reference.q*65536,s->reference.v,s->reference.a,mv,permit);
  if(s->control.terminal&&s->enabled){s->enabled=false;s->faults|=APP_FAULT_ENERGY;s->epoch++;}
  int64_t request=s->control.requested_q24;s->requested_q24=request>INT32_MAX?INT32_MAX:request<INT32_MIN?INT32_MIN:(int32_t)request;
  s->commanded_pwm=s->control.planned_pwm;s->last_flags=s->control.flags|(s->coast?0x4000u:0u);
  return has;
 }
 s->commanded_pwm=0;return false;
}
bool app_commit(app_state *s,int16_t actual,bool ok){
 if(!s)return false;s->actual_pwm=actual;
 if(!joint_applied(&s->control,actual,ok)){app_fault(s,APP_FAULT_SENSOR);return false;}return true;
}
#if APP_DAQ
static int16_t app_a_q4(int32_t a_q16){int32_t a=a_q16/4096;return (int16_t)(a>32767?32767:a< -32768?-32768:a);}
#endif
int32_t app_planned_accel(const app_state *s){return s->reference.active&&!s->openloop&&!s->control.tau_mode&&!s->bam.running?s->reference.a:0;}
void app_record_applied(app_state *s){if(!s)return;
#if APP_DAQ
 if(++s->daq_phase<(s->daq_decimation?s->daq_decimation:4u))return;s->daq_phase=0;if(s->ring_count==APP_RING_RECORDS){s->drops++;return;} /* undrained DAQ never disables output */unsigned at=s->ring_head+s->ring_count;if(at>=APP_RING_RECORDS)at-=APP_RING_RECORDS;s->ring[at]=(app_record){s->now_us,s->position_counts,s->reference.q,s->reference.v,s->control.v_hat_q16,s->requested_q24,s->commanded_pwm,s->actual_pwm,app_a_q4(s->control.a_hat_q16),(uint16_t)((s->last_flags&0x7fffu)|(s->faults?0x8000u:0u))};s->ring_count++;
#endif
}
static uint32_t status_word(const app_state *s,uint16_t a){switch(a){case 64:return s->enabled;case 68:return s->ready;case 72:return s->motor_allowed;case 76:return s->volt_warn|(s->upper_volt_on?0u:2u);   /* fast17: warnings (bit 0 > 7.0 V, bit 1 upper voltage protection off) */case 80:return s->faults;case 84:return s->hardware_faults;case 88:return s->samples;case 92:return s->now_us;case 96:return (uint32_t)s->position_counts;case 100:return (uint32_t)s->velocity_mrad_s;case 104:return s->worst_tick_us;case 108:return s->drops;case 112:return s->ring_count|(APP_PERIOD_US<<16);case 116:return s->calibration.origin;case 120:return s->raw_angle;case 124:return (uint32_t)s->adc.voltage_decivolts*100u|((uint32_t)s->adc.current_magnitude_ma<<16);case 128:return s->adc.temperature_c;default:return 0;}}
/* Low table (0..255): the only live inputs are 17 status words (64..131) and the
 * 8 first-trip words (224..255), both updated by TIM16. NV bytes (0..63) are
 * written only by app_init, so they may be formatted unmasked. Caller masks
 * around app_low_snapshot only; app_low_format is a pure function of the snapshot
 * plus immutable data, so the bytes equal a fully masked app_read at that instant. */
uint16_t app_model(const app_state *s){return s->nv_model?s->nv_model:(uint16_t)APP_MODEL_NUMBER;}
void app_low_snapshot(app_state *s,uint32_t *snap){
 for(unsigned i=0;i<17u;i++)snap[i]=status_word(s,(uint16_t)(64u+4u*i));
 if(APP_DIAG860)for(unsigned i=0;i<8u;i++)snap[17u+i]=s->first_trip[i];   /* fast17: first-trip words, logging builds only */
 s->last_heartbeat_us=s->now_us;
}
void app_low_format(const app_state *s,const uint32_t *snap,uint16_t address,uint16_t size,uint8_t *out){
 static const uint8_t marker[16]="FW231-PD-BUS";
 for(unsigned i=0;i<size;i++){uint16_t a=(uint16_t)(address+i);uint8_t v=0;
  if(a<64u)v=a<2u?(uint8_t)(app_model(s)>>(8u*a)):a==6?APP_FIRMWARE_VERSION:s->nv.bytes[a];
  else if(a<132u)v=(uint8_t)(snap[(a-64u)/4u]>>((a&3u)*8u));
  else if(a>=208u&&a<224u)v=marker[a-208u];
  else if(APP_DIAG860&&a>=224u&&a<256u)v=(uint8_t)(snap[17u+(a-224u)/4u]>>((a&3u)*8u));
  out[i]=v;}
}
/* fast17: one serializer for the word registers of app_read (flash): little-endian words, the read heartbeat, 0. */
static __attribute__((noinline)) uint8_t put_words(app_state *s,uint8_t *out,const uint32_t *v,unsigned n){for(unsigned i=0;i<4u*n;i++)out[i]=(uint8_t)(v[i>>2]>>(8u*(i&3u)));s->last_heartbeat_us=s->now_us;return 0;}
uint8_t app_read(void *ctx,uint16_t address,uint16_t size,uint8_t *out){
 app_state *s=ctx;if(!s||!out||!size||size>384u)return 5;
 if(0){}
#if APP_DAQ
 else if(address==1536u){if(size%APP_RECORD_BYTES)return 5;for(unsigned at=0;at<size;at+=APP_RECORD_BYTES){for(unsigned j=0;j<APP_RECORD_BYTES;j++)out[at+j]=0;if(!s->ring_count)continue;const app_record *r=&s->ring[s->ring_head];p32(out+at,r->t);p32(out+at+4,(uint32_t)r->counts);p32(out+at+8,(uint32_t)r->qd);p32(out+at+12,(uint32_t)r->vd);p32(out+at+16,(uint32_t)r->v_hat);p32(out+at+20,(uint32_t)r->torque);p16(out+at+24,(uint16_t)r->command);p16(out+at+26,(uint16_t)r->actual);p16(out+at+28,(uint16_t)r->a_hat_q4);p16(out+at+30,r->flags);if(++s->ring_head==APP_RING_RECORDS)s->ring_head=0;s->ring_count--;}}

 else if(address==600u&&size==32u){uint32_t v[8]={JC_CONFIG_MAGIC,1u,APP_PERIOD_US,APP_ALLOW_MOTOR,JC_CONFIG_BYTES,0x20261116u,1536u,APP_RECORD_BYTES|(APP_RING_RECORDS<<16)};return put_words(s,out,v,8u);}
 else if(address==840u&&size==4u){uint32_t v[1]={s->daq_decimation?s->daq_decimation:4u};return put_words(s,out,v,1u);}
#endif
 else if(address==640u&&size==64u){const joint_control *c=&s->control;uint32_t v[16]={s->samples,(uint32_t)s->reference.q,(uint32_t)s->reference.v,(uint32_t)s->reference.a,(uint32_t)s->requested_q24,(uint16_t)s->commanded_pwm|((uint32_t)(uint16_t)s->actual_pwm<<16),(uint32_t)c->v_hat_q16,(uint32_t)c->a_hat_q16,(uint32_t)c->v_raw_q16,c->flags|((uint32_t)c->history<<16)|((uint32_t)c->terminal<<24)|((uint32_t)c->configured<<25)|((uint32_t)s->bam.selected<<26)|((uint32_t)s->bam.running<<27)|((uint32_t)s->bam.released<<28)|((uint32_t)(s->bam.kind&3u)<<29),(uint32_t)s->target_counts,s->duration_ticks,s->reference.elapsed,s->reference.active|((uint32_t)s->openloop<<1)|((uint32_t)s->control.tau_mode<<2)|((uint32_t)(uint16_t)s->openloop_pwm<<16),c->observations,c->rejections|((uint32_t)c->est.resyncs<<16)};return put_words(s,out,v,16u);}
 else if(address==704u&&size==56u){const joint_control *c=&s->control;uint32_t v[14]={c->token,c->epoch,c->configured,(uint32_t)c->config.kp_q16,(uint32_t)c->config.kd_q16,c->gain_q24,c->config.torque_limit_q24,c->config.est.k0,c->config.est.k1,c->config.est.k2,(uint32_t)c->config.est.deadzone_q20,(uint32_t)c->config.est.resync_q20,c->config.est.lead_ticks,(uint32_t)c->nominal_q24};return put_words(s,out,v,14u);}
 else if(APP_LAB&&address==864u&&size==32u)platform_dma_read(out);   /* FW231: lab reads only with LAB=1 (main.c APP_LAB) */
 else if(APP_LAB&&address==896u&&size==64u)obs_serialize(&observation,platform_irq_max_us(),observation_stack.free_bytes,observation_stack.span_bytes,observation_stack.flags,observation_stack.sweeps,out);
 else if(APP_TUNE2&&address==340u&&size==12u){uint32_t v[3]={(uint32_t)s->control.hpd.kd_q16,s->control.hpd.shift,(uint32_t)s->control.v_lp_q16};return put_words(s,out,v,3u);}
 else if(APP_TUNE2&&address==336u&&size==16u){const jc_comp2 *g=&s->control.comp2;uint32_t v[4]={(uint32_t)g->b_drop_q16,(uint32_t)g->vgate_q16,g->flags,JC_COMP2_MAGIC};return put_words(s,out,v,4u);}
 else if(address==324u&&size==40u){const jc_comp *c=&s->control.comp;uint32_t v[10]={(uint32_t)c->c0_q24,(uint32_t)c->c1_q16,(uint32_t)c->b_q16,(uint32_t)c->ja_q24,(uint32_t)c->veps_q16,(uint32_t)c->boost_q24,c->boost_ticks,(uint32_t)c->jfb_q24,c->afilt_shift|(c->fast_accel?0x100u:0u),(uint32_t)s->control.comp_q24};return put_words(s,out,v,10u);}
 else if(APP_DIAG860&&address==856u&&(size==24u||size==32u)){   /* fast17: logging builds */uint32_t v[8]={s->sample_budget.total,s->sample_budget.window,s->sample_budget.window_max,s->jump_rejects,s->magnet_lost,s->magnet_warnings,s->reacquisitions,s->reacq_rejects};return put_words(s,out,v,size/4u);}   /* fast17: 32 B adds the reacquire counters */
 else if(address==848u&&size==16u){uint32_t v[4]={s->current_rms_limit_ma,s->temp_limit_c,s->vmax_dv,isqrt64(s->i2_q12>>12)};return put_words(s,out,v,4u);}
 else if(address==1152u&&size==1u){out[0]=s->upper_volt_on;}   /* fast17 */
 else if(address==256u&&size==16u){uint32_t v[4]={(uint32_t)s->target_counts,s->duration_ticks,0u,0u};return put_words(s,out,v,4u);}
 else if((uint32_t)address+size<=256u){uint32_t snap[APP_LOW_SNAPSHOT_WORDS];app_low_snapshot(s,snap);app_low_format(s,snap,address,size,out);}
 else return 4;s->last_heartbeat_us=s->now_us;return 0;
}
void app_write_capture(const app_state *s,app_write_plan *p){
 *p=(app_write_plan){0};p->epoch=s->epoch;p->faults=s->faults;p->duration=s->duration_ticks;p->profile=s->target_profile;p->position=s->position_counts;p->target=s->target_counts;
 p->ref_q=s->reference.q;p->enabled=s->enabled;p->ready=s->ready;p->motor_allowed=s->motor_allowed;p->startup_valid=s->startup_valid;
 p->sensor_healthy=s->sensor_healthy;p->configured=s->control.configured;p->ref_active=s->reference.active;p->reference=s->reference;if(APP_BAM)p->bam=s->bam;   /* fast17: !APP_BAM: bam is always zero */
}
uint8_t app_write_prepare(app_write_plan *p,uint16_t address,uint16_t size,const uint8_t *data){
 if(!p||!data||!size)return 5;p->kind=0;
#if !APP_BAM
 if(address==272u)return 7;
#else
 if(address==272u){if(size!=16u)return 5;if(p->enabled||p->bam.running)return 7;uint32_t kind=u32(data+4),mode=u32(data+8),stride=u32(data+12);int32_t center;
  if(!p->startup_valid||kind>3u||mode||stride!=1u||!bam_center((uint8_t)kind,i32(data),&center))return 4;
  p->requested_bam=(app_bam){.center_counts=i32(data),.center_q16=center,.kind=(uint8_t)kind,.stride=1u,.selected=true};p->kind=272;return 0;}
#endif
 if(address==256u){if(size!=16u)return 5;if(APP_BAM&&p->bam.running)return 7;int32_t target=i32(data);uint32_t duration=u32(data+4);
  uint32_t profile=u32(data+8);if(!p->startup_valid||!duration||profile>1u||u32(data+12))return 4;p->requested_profile=profile;
  p->changed=target!=p->target||(APP_BAM&&p->bam.selected)||profile;int32_t q;if(!counts_q16(target,&q))return 4;
  /* fast10 (review F-5): disarmed, plan nothing here (the reference is stale and the commit ignores it); ARM plans from the shaft.
   * fast12 (owner, 2026-11-20: "RETARGET without a doubt ... just act as if its a new goal command"): a new target while a
   * profiled move runs starts a new profile, from where the reference is now, exactly like a fresh goal (no blending). */
  if(p->changed&&p->enabled){if(p->ref_active)general_reference_init(&p->reference,p->reference.q);if(!(profile?general_reference_start_linear(&p->reference,q,duration):general_reference_start(&p->reference,q,duration)))return 4;}
  p->requested_target=target;p->requested_duration=duration;p->kind=256;return 0;}
 if(address==64u){if(size!=1u)return 5;if(data[0]!=1u)return 4;
  if(p->enabled||(APP_BAM&&p->bam.running)||!p->startup_valid||!p->motor_allowed||!p->ready||!p->sensor_healthy||p->faults||!p->configured)return 7;
#if APP_BAM
  if(p->bam.selected){bam_output b;if(!bam_eval(p->bam.kind,0,p->bam.center_q16,&b))return 4;general_reference_init(&p->reference,b.q);p->reference.v=b.v;p->reference.a=b.a;p->reference.active=true;p->reference.duration=12000u;p->kind=64;return 0;}
#endif
  int32_t initial,target;if(!counts_q16(p->position,&initial)||!counts_q16(p->target,&target))return 4;
  general_reference_init(&p->reference,initial);if(!(p->profile&&initial!=target?general_reference_start_linear(&p->reference,target,p->duration):general_reference_start(&p->reference,target,p->duration)))return 4;p->kind=64;return 0;}
 return 7;
}
uint8_t app_write_commit(app_state *s,const app_write_plan *p){
 if(!s||!p||!p->kind)return 5;if(s->epoch!=p->epoch||s->enabled!=p->enabled||(APP_BAM&&!bam_equal(&s->bam,&p->bam)))return 7;
 if(APP_BAM&&p->kind==272u){if(s->enabled||s->bam.running)return 7;s->bam=p->requested_bam;s->epoch++;s->last_heartbeat_us=s->now_us;return 0;}
 if(p->kind==256u){if(s->target_counts!=p->target||s->control.tau_mode)return 7;   /* fast16: no goal in torque mode (INT32_MIN first) */
  if(p->changed&&(s->reference.active!=p->ref_active||s->reference.q!=p->ref_q))return 7;
  if(p->changed&&s->enabled)s->reference=p->reference;
  if(APP_BAM)s->bam=(app_bam){0};s->target_profile=(uint8_t)p->requested_profile;s->target_counts=p->requested_target;s->duration_ticks=p->requested_duration;s->target_configured=true;s->epoch++;s->last_heartbeat_us=s->now_us;return 0;}
 if(p->kind==64u){if(!APP_ALLOW_MOTOR||s->enabled||!s->startup_valid||!s->motor_allowed||!s->ready||!s->sensor_healthy||s->faults||!s->control.configured||!joint_off(&s->control)||s->control.history<3||s->reference.q!=p->ref_q||s->reference.active!=p->ref_active||s->position_counts!=p->position||s->target_counts!=p->target||s->duration_ticks!=p->duration)return 7;
  if(APP_BAM&&s->bam.selected){s->bam.running=true;s->bam.released=false;s->bam.tick=0;s->bam.first=true;}
  s->reference=p->reference;s->arm_counts=s->position_counts;s->enabled=true;s->control.terminal=false;s->control.prev_tau_sign=0;s->control.boost_left=0;s->control.a_filt_q16=0;s->last_heartbeat_us=s->now_us;
#if APP_DAQ
  s->ring_head=s->ring_count=0;s->drops=0;s->daq_phase=0;
#endif
  s->epoch++;return 0;}
 return 7;
}
uint8_t app_write(void *ctx,uint16_t address,uint16_t size,const uint8_t *data){
 app_state *s=ctx;if(!s)return 5;
 /* Only a decoded endpoint call is a stage attempt. Protocol rejection
  * (including CRC/empty-range failure) never invokes this boundary. */
 if(!data||!size)return 5;
 if(address==256u||address==272u||(address==64u&&size==1u&&data[0]==1u)){app_write_plan p;app_write_capture(s,&p);uint8_t r=app_write_prepare(&p,address,size,data);return r?r:app_write_commit(s,&p);}
 if(address==64u){if(size!=1u)return 5;if(data[0])return 4;stop(s);return 0;}
 if(address==320u){if(s->enabled)return 7;return joint_configure(&s->control,data,size)?0:4;}
 #if APP_DAQ
 if(address==840u){if(size!=4u)return 5;uint32_t d=u32(data);if(d<1u||d>4u)return 4;if(s->enabled)return 7;s->daq_decimation=(uint8_t)d;s->daq_phase=0;s->ring_head=s->ring_count=0;return 0;}
 #endif
 /* 260 open-loop duty; fast16 264 output torque through the gear-train compensation, 268 raw motor torque (Nm Q24, any
  * value: the configured torque limit and the duty saturation apply). Each leaves the other mode; INT32_MIN to any of
  * them leaves both and holds where the shaft is now. */
 {uint32_t m=(uint32_t)address-260u;if(m<=8u&&!(m&3u)){m>>=2;   /* m: 0 duty (260), JC_TAU_COMP (264), JC_TAU_RAW (268) */
  if(size!=4u)return 5;int32_t v=i32(data);bool leave=v==INT32_MIN;
  if(!leave&&(!s->control.configured||(APP_BAM&&s->bam.running)||(!m&&(v< -480||v>480))))return 4;   /* rejected: no change */
  s->openloop=!leave&&!m;s->openloop_pwm=s->openloop?(int16_t)v:0;s->control.tau_mode=leave?JC_TAU_PD:(uint8_t)m;s->control.tau_q24=v;
  if(leave){int32_t q;
   /* back to PD holding where the shaft is now, never slewing back to an old reference */
   if(counts_q16(s->position_counts,&q)){general_reference_init(&s->reference,q);s->target_counts=s->position_counts;s->target_profile=0;}s->epoch++;}
  return 0;}}
 if(address==1152u){if(size!=1u)return 5;if(data[0]>1u)return 4;s->upper_volt_on=data[0]!=0u;if(!s->upper_volt_on)s->volt_warn=0u;return 0;}   /* fast17: upper voltage protection on/off (RAM, 1 at boot) */
 if(address==126u){if(size!=1u)return 5;if(data[0]!=0x5a)return 4;if(s->enabled||!joint_off(&s->control)||!s->startup_valid||!s->sensor_healthy)return 7;s->faults=0;s->good_ticks=0;s->ready=false;s->tracker.ready=false;torque_velocity_reset(&s->velocity);s->speed_guard.ready=false;s->speed_guard.ticks=0;s->control.history=0;est32_init(&s->control.est);for(unsigned i=0;i<8;i++)s->first_trip[i]=0;s->epoch++;return 0;}
 if(address==328u){return joint_set_gains(&s->control,data,size)?0:4;}  /* live gains */
 if(address==324u){return joint_set_comp(&s->control,data,size)?0:4;}   /* live compensation */
 #if APP_TUNE2 && !APP_LAB   /* fast17-size: only with APP_TUNE2 (off in the FW231 images); fast16: the lab (hang-test) image drops the demo-era live tunings 336/340 to fit; they write 7 there */
 if(address==336u){return joint_set_comp2(&s->control,data,size)?0:4;}
 if(address==340u){return joint_set_hpd(&s->control,data,size)?0:4;}
 #endif  /* demo5 candidate: high-pass damping */  /* demo4 candidate: duty-dependent b, near-stop inertia gate */
 if(address==848u){if(size!=12u)return 5;if(s->enabled)return 7;uint32_t cur=u32(data),temp=u32(data+4),vmax=u32(data+8);
  if(cur>100000u||temp<20u||temp>100u||vmax<50u||vmax>80u)return 4;s->current_rms_limit_ma=cur;s->temp_limit_c=(uint8_t)temp;s->vmax_dv=(uint8_t)vmax;return 0;}
 if(address==832u){return 4; /* travel/speed guards removed */
  if(size!=8u&&size!=12u)return 5;if(s->enabled||!joint_off(&s->control))return 7;uint32_t travel=u32(data),speed=u32(data+4),current=size==12u?u32(data+8):s->current_limit_ma;if(!travel||travel>INT32_MAX||!speed||speed>INT32_MAX||current>100000u)return 4;s->travel_limit=travel;s->speed_limit=speed;s->current_limit_ma=current;s->epoch++;return 0;}
 return 7;
}
/* fast12 (owner, 2026-11-20: "perform recommended"): A0 reads never return bytes of the retained ROBOTIS loader
 * (0x08000000..0x08003FFF), so the bootloader cannot be read out over the bus. Our program region and the servo's own
 * calibration/NV journal (0x0800E000..) stay readable (program readback, the owner's calibration backup). The flasher's
 * "loader unchanged" check uses app_loader_crc32 (read 1136) instead. */
uint8_t app_flash_read(void *ctx,uint32_t address,uint16_t size,uint8_t *out){if(!ctx||!out||!size||size>96u)return 5;if(address<APP_PROGRAM_BASE||address>=0x08010000u||size>0x08010000u-address)return 4;for(uint16_t i=0;i<size;i++){uintptr_t a=(uintptr_t)address+i;uint32_t w=hw_read32(a&~(uintptr_t)3u);out[i]=(uint8_t)(w>>((a&3u)*8u));}return 0;}
/* fast12: CRC-32 (IEEE 802.3, the same as zlib.crc32) of the whole loader region, 16 KiB. Only this one fixed-range digest
 * is offered: no address or length, so it says nothing about any part of the loader. ~4 ms in the main loop (nibble table). */
uint32_t app_loader_crc32(void){
 static const uint32_t t[16]={0x00000000u,0x1db71064u,0x3b6e20c8u,0x26d930acu,0x76dc4190u,0x6b6b51f4u,0x4db26158u,0x5005713cu,
                              0xedb88320u,0xf00f9344u,0xd6d6a3e8u,0xcb61b38cu,0x9b64c2b0u,0x86d3d2d4u,0xa00ae278u,0xbdbdf21cu};
 uint32_t c=0xffffffffu;
 for(uintptr_t a=APP_LOADER_BASE;a<APP_PROGRAM_BASE;a+=4u){uint32_t w=hw_read32(a);
  for(unsigned k=0;k<4u;k++){c^=(uint8_t)(w>>(8u*k));c=(c>>4)^t[c&15u];c=(c>>4)^t[c&15u];}}
 return ~c;
}
