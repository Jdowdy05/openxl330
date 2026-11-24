/* fast17-size golden traces of the FW231 control law (v1.1 plan, Stage 1: "golden trace parity of the control law,
 * bit-exact torque/duty over recorded scenarios").
 *
 * Scripted closed-loop scenarios drive the app layer exactly as TIM16 does (app_tick -> output permit -> app_commit),
 * through the real app.c, joint_control.c, est32.c, general_reference.c, torque.c, motion.c, deadline.c and the
 * calibration/NV code, against a deterministic integer plant (no floating point, so a native build and the Cortex-M0
 * build under qemu must agree bit for bit). Every tick is folded into a 64-bit FNV-1a hash: the controller outputs and
 * internal state (duty, torque request, compensation, estimator, reference, velocity, faults, flags), the read-1088
 * motion block, and every 50 ticks the reads 64/68, 0..255, 640, 704, 324 and 256. Host writes (config, gains,
 * compensation, targets, torque, duty, ARM/OFF, fault clear, protections) are folded in with their return codes.
 * Only registers that fast16 and fast17-size both implement are used, so the same driver runs on both.
 *
 * Output: one line per scenario "name ticks hash" and a final "golden <hash>" line. tests/check_golden.py compares
 * builds; tests/golden_qemu/ runs the image's own ARM objects on qemu -M microbit. GOLDEN_MARK brackets each tick's
 * control computation for the offline cycle count (tests/golden_cycles.py). */
#include "app.h"
#include "motion.h"
#include <stdbool.h>
#include <stdint.h>

void golden_out(const char *s);            /* native: stdout; qemu: semihosting */
#ifndef GOLDEN_MARK_WRITE
#define GOLDEN_MARK_WRITE 0
#endif
void golden_mark(unsigned what);           /* 1 = tick computation starts, 2 = it ends (cycle counting); no-op */

static uint64_t h=UINT64_C(0xcbf29ce484222325);
static void hb(uint8_t b){h^=b;h*=UINT64_C(0x100000001b3);}
static void hw32(uint32_t v){hb((uint8_t)v);hb((uint8_t)(v>>8));hb((uint8_t)(v>>16));hb((uint8_t)(v>>24));}
static void hi64(int64_t v){hw32((uint32_t)(uint64_t)v);hw32((uint32_t)((uint64_t)v>>32));}
static void hbytes(const uint8_t *p,unsigned n){for(unsigned i=0;i<n;i++)hb(p[i]);}
static void p32(uint8_t *p,uint32_t v){p[0]=(uint8_t)v;p[1]=(uint8_t)(v>>8);p[2]=(uint8_t)(v>>16);p[3]=(uint8_t)(v>>24);}
static char *put_hex(char *o,uint64_t v,unsigned digits){for(unsigned i=0;i<digits;i++)o[i]="0123456789abcdef"[(v>>(4u*(digits-1u-i)))&15u];return o+digits;}
static char *put_dec(char *o,uint32_t v){char t[10];unsigned n=0;do{t[n++]=(char)('0'+v%10u);v/=10u;}while(v);while(n)*o++=t[--n];return o;}
static char *put_str(char *o,const char *s){while(*s)*o++=*s++;return o;}

#define Q16I(num,den) ((int32_t)(((int64_t)(num)*65536+(den)/2)/(den)))
#define Q24I(num,den) ((uint32_t)(((int64_t)(num)*16777216+(den)/2)/(den)))
/* ---- fixtures: tests/test_pd.c fixture (calibration identity map, NV journal model 1200; GOLDEN_MODEL=1190 for an
 * M077 build smoke run, which has no fast16 reference) ---- */
#ifndef GOLDEN_MODEL
#if defined(APP_GEAR) && APP_GEAR == 77
#define GOLDEN_MODEL 1190u
#else
#define GOLDEN_MODEL 1200u
#endif
#endif
/* the plant and the host defaults of each gear (Stage 3): the M077 runs 1/r faster with r times the torque and r^2 the
 * inertia at the output, so its acceleration per duty and its friction (per inertia) are 1/r the M288's; its host
 * defaults are K 0.0504 r and the r^2-scaled gains (the host defaults); its jump gate is 32 counts per tick. */
#if defined(APP_GEAR) && APP_GEAR == 77
enum { PLANT_DRIVE = 7034, PLANT_FRICTION = 353, GATE = 32 };
#define DEF_KP Q16I(253,1000)
#define DEF_KD Q16I(72,10000)
#define DEF_K 227225u
#else
enum { PLANT_DRIVE = 1890, PLANT_FRICTION = 95, GATE = 12 };
#define DEF_KP Q16I(35,10)
#define DEF_KD Q16I(10,100)
#define DEF_K Q24I(504,10000)
#endif
static uint8_t cal[4096],nv[4096];
static void p16(uint8_t *p,uint16_t v){p[0]=(uint8_t)v;p[1]=(uint8_t)(v>>8);}
static void fixture(void){
 for(unsigned i=0;i<4096;i++){cal[i]=255;nv[i]=255;}
 p16(cal+0x800,2);p16(cal+0x810,1073);for(unsigned i=1;i<64;i++)p16(cal+0x810+8*(64-i),(uint16_t)(i*64));
 p32(nv,0xabab);p32(nv+4,0);p32(nv+8,0);p32(nv+12,0);
 static const uint8_t keys[]={0,1,7,8,10,11,20,21,22,23,36,37,38,39},v[]={GOLDEN_MODEL&255u,GOLDEN_MODEL>>8,1,3,0,3,0,0,0,0,224,1,0,0};
 for(unsigned i=0;i<sizeof keys;i++){p32(nv+16+8*i,keys[i]|((uint32_t)v[i]<<16));p32(nv+20+8*i,0);}
}

/* ---- plant: raw encoder counts, Q16 position/velocity per tick, integer only ---- */
typedef struct {int64_t th,om;int32_t ext,stall_extra;uint16_t mv;uint8_t temp;} plant;
static plant P;
static int64_t floordiv(int64_t a,int64_t b){int64_t q=a/b;if((a%b!=0)&&((a<0)!=(b<0)))q--;return q;}
static void plant_step(int16_t duty){
 /* ~11 rad/s no-load (3.6 counts/tick) and ~0.029 counts/tick^2 stall accel at full duty and 5 V (M288 horn). */
 int64_t drive=floordiv((int64_t)duty*P.mv*PLANT_DRIVE,480*5000);
 int64_t damp=floordiv(P.om*33,4096);
 int64_t net=drive-damp+P.ext;
 const int64_t fr=PLANT_FRICTION+P.stall_extra;
 if(P.om==0&&net<=fr&&net>=-fr)net=0;                /* stiction */
 else net-=P.om>0?fr:P.om<0?-fr:(net>0?fr:-fr);
 int64_t om=P.om+net;if((P.om>0&&om<0)||(P.om<0&&om>0))om=0;
 P.om=om;P.th+=om;
}
/* the test calibration maps raw counts in reverse (position falls as raw rises): positive duty drives raw down */
static uint16_t plant_raw(void){int64_t c=floordiv(-P.th,65536);int64_t r=c%4096;if(r<0)r+=4096;return (uint16_t)r;}

/* ---- the app, driven like TIM16 ---- */
static app_state s;
static uint32_t now,tick_no;
static bool enc_ok=true,adc_ok=true;static int32_t raw_override=-1;static uint32_t adc_faults;
static void rec_reads(void){
 static uint8_t b[256];static const uint16_t reads[][2]={{64,68},{0,256},{640,64},{704,56},{324,40},{256,16},{96,4}};
 for(unsigned i=0;i<sizeof reads/sizeof reads[0];i++){uint8_t r=app_read(&s,reads[i][0],reads[i][1],b);hb(r);if(!r)hbytes(b,reads[i][1]);}
}
static void rec_tick(bool has){
 const joint_control *c=&s.control;
 hw32(has);hw32(s.enabled);hw32(s.ready);hw32(s.sensor_healthy);hw32(s.faults);hw32(s.hardware_faults);hw32(s.samples);
 hw32((uint32_t)s.position_counts);hw32((uint32_t)s.velocity_mrad_s);hw32((uint32_t)s.commanded_pwm);hw32((uint32_t)s.actual_pwm);
 hw32((uint32_t)s.requested_q24);hw32(s.last_flags);hw32(s.coast);hw32(s.bad_streak);hw32(s.raw_angle);hw32(s.good_ticks);
 hw32(s.jump_rejects);hw32(s.magnet_lost);hi64((int64_t)s.i2_q12);hw32(s.target_counts);hw32(s.epoch);hw32(s.openloop);hw32((uint32_t)s.openloop_pwm);
 hw32((uint32_t)c->planned_pwm);hw32((uint32_t)c->actual_pwm);hi64(c->comp_q24);hw32((uint32_t)c->v_hat_q16);hw32((uint32_t)c->a_hat_q16);
 hw32((uint32_t)c->v_raw_q16);hw32((uint32_t)c->a_fast_q16);hw32((uint32_t)c->a_filt_q16);hw32((uint32_t)c->v_lp_q16);hw32((uint32_t)c->prev_tau_sign);
 hw32(c->boost_left);hi64(c->angle_q32);hi64(c->requested_q24);hw32((uint32_t)c->nominal_q24);hw32(c->flags);hw32(c->history);hw32(c->terminal);
 hw32(c->clock);hw32(c->observations);hw32(c->rejections);hw32(c->tau_mode);hw32((uint32_t)c->tau_q24);hw32(c->configured);
 const pd_config *k=&c->config;   /* configuration and compensation: the power-up loading path must leave the same state */
 hw32((uint32_t)k->kp_q16);hw32((uint32_t)k->kd_q16);hw32(k->torque_limit_q24);hw32(k->est.k0);hw32(k->est.k1);hw32(k->est.k2);
 hw32((uint32_t)k->est.deadzone_q20);hw32((uint32_t)k->est.resync_q20);hw32(k->est.lead_ticks);hw32(k->blank_reversal);hw32(c->gain_q24);hw32(c->token);hw32(c->epoch);
 const jc_comp *q=&c->comp;
 hw32((uint32_t)q->c0_q24);hw32((uint32_t)q->c1_q16);hw32((uint32_t)q->b_q16);hw32((uint32_t)q->ja_q24);hw32((uint32_t)q->veps_q16);hw32((uint32_t)q->boost_q24);
 hw32(q->boost_ticks);hw32((uint32_t)q->jfb_q24);hw32(q->afilt_shift);hw32(q->fast_accel);
 hw32((uint32_t)c->comp2.b_drop_q16);hw32((uint32_t)c->comp2.vgate_q16);hw32(c->comp2.flags);hw32((uint32_t)c->hpd.kd_q16);hw32(c->hpd.shift);
 hw32(s.current_rms_limit_ma);hw32(s.temp_limit_c);hw32(s.vmax_dv);
 const est32_state *e[2]={&c->est,&c->est_fast};
 for(unsigned i=0;i<2;i++){hi64(e[i]->y_last_q16);hw32((uint32_t)e[i]->e);hw32((uint32_t)e[i]->dq);hw32((uint32_t)e[i]->ddq);hw32(e[i]->primed);hw32(e[i]->resyncs);}
 const general_reference *r=&s.reference;
 hw32((uint32_t)r->q);hw32((uint32_t)r->v);hw32((uint32_t)r->a);hw32((uint32_t)r->start);hw32((uint32_t)r->delta);hw32((uint32_t)r->vscale);hw32((uint32_t)r->ascale);
 hw32(r->elapsed);hw32(r->duration);hw32(r->active);hw32(r->linear);
 hw32(s.velocity.last_us);hw32((uint32_t)s.velocity.last_position_counts);hw32((uint32_t)s.velocity.velocity_mrad_s);hw32((uint32_t)s.velocity.last_fraction_q16);hi64(s.velocity.velocity_q16);
 /* read 1088, built as main.c read_atomic builds it */
 motion_in m={s.position_counts,s.control.v_hat_q16,app_planned_accel(&s),s.actual_pwm,(uint16_t)(s.adc.voltage_decivolts*100u),s.adc.current_magnitude_ma,s.enabled};
 uint8_t mb[MOTION_BYTES];motion_block(&m,mb);hbytes(mb,MOTION_BYTES);
 if(tick_no%50u==0u)rec_reads();
}
static void tick(void){
 now+=500u;tick_no++;
 sensing_adc adc={0};
 uint16_t mv=P.mv;int32_t cur=s.actual_pwm<0?-s.actual_pwm:s.actual_pwm;
 adc.voltage_decivolts=(uint16_t)(mv/100u);adc.current_magnitude_ma=(uint16_t)(cur*2+((tick_no*37u)&7u)+(uint32_t)P.stall_extra);adc.temperature_c=P.temp;adc.timestamp_us=now;adc.faults=adc_faults;
 uint16_t raw=raw_override>=0?(uint16_t)raw_override:plant_raw();
 if(!GOLDEN_MARK_WRITE)golden_mark(1u);
 bool has=app_tick(&s,enc_ok,raw,&adc,adc_ok,now);
 bool permit=has&&s.enabled&&s.ready&&s.motor_allowed&&s.sensor_healthy&&!s.faults;
 bool authorized_zero=has&&s.control.planned_pwm==0;
 int16_t applied=permit?s.control.planned_pwm:0;   /* bridge_set accepts every planned duty (|duty| <= 480) */
 if(has)(void)app_commit(&s,applied,(authorized_zero&&applied==0)||permit);
 if(!GOLDEN_MARK_WRITE)golden_mark(2u);
#if defined(APP_DAQ) && APP_DAQ
 app_record_applied(&s);   /* the logging builds' DAQ ring fill, as TIM16 calls it (status words 108/112 show it) */
#endif
 rec_tick(has);
 plant_step(applied);
}
static void run(unsigned n){for(unsigned i=0;i<n;i++)tick();}
/* GOLDEN_MARK_WRITE=address: the cycle markers bracket app_write calls to that address instead of the ticks (the PRIMASK
 * section main.c's write_untraced holds around it); tests/golden_qemu/run.sh passes MARK_WRITE through. */
static uint8_t wr(uint16_t a,uint16_t n,const uint8_t *d){if(GOLDEN_MARK_WRITE&&a==GOLDEN_MARK_WRITE)golden_mark(1u);uint8_t r=app_write(&s,a,n,d);if(GOLDEN_MARK_WRITE&&a==GOLDEN_MARK_WRITE)golden_mark(2u);hw32(0x57000000u|((uint32_t)a<<8)|r);return r;}
static void wr_u8(uint16_t a,uint8_t v){(void)wr(a,1,&v);}
static void wr_i32(uint16_t a,int32_t v){uint8_t b[4];p32(b,(uint32_t)v);(void)wr(a,4,b);}
static uint32_t token;
enum {K0=737857423u,K1=69489982u,K2=3272216u};

static void config(int32_t kp_q16,int32_t kd_q16,uint32_t gain_q24,uint32_t limit_q24,uint32_t k0,uint32_t k1,uint32_t k2,int32_t dz,int32_t resync,uint32_t flags){
 uint8_t b[48]={0};p32(b,++token);p32(b+4,(uint32_t)kp_q16);p32(b+8,(uint32_t)kd_q16);p32(b+12,gain_q24);p32(b+16,limit_q24);
 p32(b+20,k0);p32(b+24,k1);p32(b+28,k2);p32(b+32,(uint32_t)dz);p32(b+36,(uint32_t)resync);p32(b+40,flags);p32(b+44,JC_CONFIG_MAGIC);(void)wr(320,48,b);}
static void config_default(void){config(DEF_KP,DEF_KD,DEF_K,2u<<24,K0,K1,K2,0,16<<20,1u);}  /* openxl330 defaults of the gear (M288: Kp 3.5, Kd 0.10, K 0.0504), limit 2 */
static void gains(int32_t kp,int32_t kd){uint8_t b[8];p32(b,(uint32_t)kp);p32(b+4,(uint32_t)kd);(void)wr(328,8,b);}
static void comp(const int32_t v[9]){uint8_t b[40];for(unsigned i=0;i<9;i++)p32(b+4*i,(uint32_t)v[i]);p32(b+36,JC_COMP_MAGIC);(void)wr(324,40,b);}
static void target(int32_t counts,uint32_t ticks,uint32_t profile){uint8_t b[16];p32(b,(uint32_t)counts);p32(b+4,ticks);p32(b+8,profile);p32(b+12,0);(void)wr(256,16,b);}
static void arm(void){wr_i32(260,INT32_MIN);target(s.position_counts,1,0);wr_u8(64,1);}
static void off(void){wr_u8(64,0);}
static void clear(void){wr_u8(126,0x5a);}
static void protect(uint32_t ma,uint32_t c,uint32_t dv){uint8_t b[12];p32(b,ma);p32(b+4,c);p32(b+8,dv);(void)wr(848,12,b);}
static int32_t deg(int32_t d){return (int32_t)floordiv((int64_t)d*4096,360);}

static void boot(int64_t start_counts){
 fixture();
 (void)app_init(&s,cal,nv,true);
 P=(plant){.th=start_counts*65536+32768,.om=0,.ext=0,.stall_extra=0,.mv=5000,.temp=30};
 now=1000000u;tick_no=0;enc_ok=adc_ok=true;raw_override=-1;adc_faults=0;token=0;
 protect(1000u,65u,70u);                   /* fast16's defaults (fast17 boots with vmax 8.0 V): traces compare across that change */
 run(30);                                  /* tracker, velocity, estimator history, ready */
}

/* ---- scenarios ---- */
static void sc_hold(void){                 /* deployment defaults: ARM, hold, pushes from outside, sustained load */
 boot(2048);config_default();run(5);arm();run(400);
 P.ext=600;run(60);P.ext=0;run(300);P.ext=-1500;run(30);P.ext=0;run(400);
 P.ext=200;run(800);P.ext=0;run(200);off();run(50);
}
static void sc_moves(void){                /* quintic and linear moves, steps, retarget mid-move, multi-turn */
 boot(1000);config_default();run(5);arm();run(100);int32_t h0=s.position_counts;
 target(h0+deg(10),400,0);run(500);target(h0-deg(30),300,0);run(450);target(h0+deg(90),200,0);run(350);
 target(h0+deg(45),1,0);run(400);target(h0+deg(400),800,1);run(300);target(h0+deg(200),400,0);run(700);
 target(h0+deg(200)-deg(5),1,0);run(300);target(h0-deg(720),1200,0);run(1400);
 target(h0,60,0);run(250);target(h0+deg(3),2,1);run(100);off();run(20);
}
static void sc_torque(void){               /* 264 / 268 torque, 260 duty, leave with INT32_MIN, goal refused in torque mode */
 boot(3000);config_default();run(5);arm();run(100);
 wr_i32(264,(int32_t)Q24I(2,100));run(80);wr_i32(264,-(int32_t)Q24I(3,100));run(120);target(s.position_counts+50,100,0);
 wr_i32(268,(int32_t)Q24I(5,100));run(60);wr_i32(268,-(int32_t)Q24I(25,10));run(40);wr_i32(264,INT32_MIN);run(200);
 wr_i32(260,120);run(50);wr_i32(260,-480);run(40);wr_i32(260,481);run(5);wr_i32(260,0);run(30);wr_i32(260,INT32_MIN);run(150);
 wr_i32(264,(int32_t)Q24I(1,100));run(30);off();run(10);arm();run(100);wr_i32(268,INT32_MIN);run(50);
 wr_i32(268,(int32_t)Q24I(1,1000));run(200);wr_i32(264,(int32_t)Q24I(1,10));run(60);wr_i32(260,INT32_MIN);run(100);off();run(10);
}
static void sc_tuning(void){               /* live gains and compensation, zeroed compensation, torque limit saturation */
 boot(500);config_default();run(5);arm();run(100);int32_t h0=s.position_counts;
 gains(Q16I(171,100),Q16I(43,1000));target(h0+deg(20),300,0);run(400);
 gains(Q16I(50,1),Q16I(2,1));target(h0-deg(20),100,0);run(300);gains(0,0);run(200);
 static const int32_t zero[9]={0};comp(zero);gains(Q16I(35,10),Q16I(10,100));target(h0+deg(30),300,0);run(500);
 static const int32_t c2[9]={80531,23593,1401,6375,3277,250000,20,9040,0x103};comp(c2);target(h0,40,0);run(300);
 static const int32_t c3[9]={68451,20054,1401,6375,3277,0,0,15099,0x002};comp(c3);target(h0+deg(60),150,0);run(400);
 static const int32_t bad[9]={68451,20054,9999,6375,3277,0,0,9040,0x102};comp(bad);
 off();run(5);config(Q16I(35,10),Q16I(10,100),Q24I(504,10000),Q24I(5,100),K0,K1,K2,0,16<<20,0x101u);run(5);arm();
 target(h0+deg(90),100,0);run(400);wr_i32(264,(int32_t)Q24I(1,1));run(50);wr_i32(264,INT32_MIN);run(100);
 /* hpd and comp2 through the setters the power-up path uses (336/340 are not host writes in fast17-size) */
 {uint8_t b[12];p32(b,1966);p32(b+4,7);p32(b+8,JC_HPD_MAGIC);hw32(joint_set_hpd(&s.control,b,12));}
 target(h0,300,0);run(400);
 {uint8_t b[16];p32(b,0);p32(b+4,0);p32(b+8,0);p32(b+12,JC_COMP2_MAGIC);hw32(joint_set_comp2(&s.control,b,16));}
 target(h0+deg(15),200,0);run(300);off();run(10);
}
static void sc_estimators(void){           /* T15/T20 trackers, deadzone, resync, blank-reversal bit, plain PD gains */
 boot(2500);
 config(Q16I(5,1),Q16I(6,100),Q24I(504,10000),2u<<24,737857423u,46330471u,970024u,1<<14,8<<20,0x100u);run(5);arm();int32_t h0=s.position_counts;
 target(h0+deg(25),200,0);run(300);P.ext=900;run(40);P.ext=0;run(300);off();run(5);
 config(Q16I(171,100),Q16I(43,1000),Q24I(588,10000),Q24I(25,100),K0,K1,K2,0,1<<20,0x001u);run(5);arm();
 target(h0-deg(25),250,0);run(400);P.ext=-3000;run(10);P.ext=0;run(300);off();run(5);
 config(Q16I(2,1),Q16I(5,100),Q24I(504,10000),2u<<24,K0,K1,K2,0,16<<20,0u);run(5);arm();target(h0,1,0);run(300);off();run(5);
}
static void sc_sensors(void){              /* bad samples (coast), jumps, missing encoder, ADC failures, recovery */
 boot(2048);config_default();run(5);arm();run(100);int32_t h0=s.position_counts;target(h0+deg(30),400,0);run(100);
 enc_ok=false;run(3);enc_ok=true;run(20);raw_override=4096+5;run(2);raw_override=-1;run(20);
 raw_override=(plant_raw()+200)%4096;run(1);raw_override=-1;run(30);
 raw_override=(s.raw_angle+4096-(GATE+1))%4096;run(1);raw_override=-1;run(30);   /* exactly one count past the gate */
 raw_override=(s.raw_angle+GATE)%4096;run(1);raw_override=-1;run(30);        /* exactly at the gate: accepted */
 enc_ok=false;run(25);enc_ok=true;run(60);adc_ok=false;run(4);adc_ok=true;run(20);adc_faults=0x2u;run(3);adc_faults=0;run(50);
 for(unsigned i=0;i<120;i++){enc_ok=(i%9u)!=0u;tick();}enc_ok=true;run(100);
 for(unsigned i=0;i<200;i++){enc_ok=(i%3u)!=0u;tick();}enc_ok=true;run(50);   /* > 5 % bad: SENSOR fault */
 off();run(2100);clear();run(40);arm();run(200);off();run(10);
}
static void sc_protect(void){              /* voltage, temperature, I2t, clear and re-arm, protection config */
 boot(2048);config_default();run(5);protect(1000,65,70);arm();run(100);
 P.mv=7100;run(3);P.mv=5000;run(10);off();run(5);clear();run(40);arm();run(50);
 P.mv=3400;run(3);P.mv=5000;off();run(5);clear();run(40);arm();run(50);
 P.temp=66;run(3);P.temp=30;off();run(5);clear();run(40);
 protect(0,65,70);arm();run(100);P.mv=6200;run(50);P.mv=5000;off();run(10);
 protect(100001,65,70);protect(1000,19,70);protect(300,60,60);arm();wr_u8(64,2);
 P.stall_extra=900;P.ext=-400;wr_i32(264,(int32_t)Q24I(2,10));run(6000);P.stall_extra=0;P.ext=0;off();run(30);
}
static void sc_fast(void){                 /* near the jump gate: fast free spin, open-loop duty reversals, wrap-around */
 boot(4000);config_default();run(5);arm();run(50);
 wr_i32(260,480);run(600);wr_i32(260,-480);run(400);wr_i32(260,300);run(200);wr_i32(260,INT32_MIN);run(400);
 int32_t h0=s.position_counts;target(h0-deg(1080),900,1);run(1100);target(h0,500,0);run(800);off();run(20);
}
static const struct {const char *name;void (*fn)(void);} scenarios[]={
 {"hold",sc_hold},{"moves",sc_moves},{"torque",sc_torque},{"tuning",sc_tuning},{"estimators",sc_estimators},
 {"sensors",sc_sensors},{"protect",sc_protect},{"fast",sc_fast}};
int golden_main(void){
 uint64_t all=UINT64_C(0xcbf29ce484222325);char line[96];
 for(unsigned i=0;i<sizeof scenarios/sizeof scenarios[0];i++){
  h=UINT64_C(0xcbf29ce484222325);scenarios[i].fn();
  char *o=put_str(line,scenarios[i].name);*o++=' ';o=put_dec(o,tick_no);*o++=' ';o=put_hex(o,h,16);*o++='\n';*o=0;golden_out(line);
  for(unsigned k=0;k<8;k++){all^=(uint8_t)(h>>(8*k));all*=UINT64_C(0x100000001b3);}
 }
 char *o=put_str(line,"golden ");o=put_hex(o,all,16);*o++='\n';*o=0;golden_out(line);
 return 0;
}
#ifndef GOLDEN_EMBEDDED
#include <stdio.h>
void golden_out(const char *s){fputs(s,stdout);}
void golden_mark(unsigned what){(void)what;}
int main(void){return golden_main();}
#endif
