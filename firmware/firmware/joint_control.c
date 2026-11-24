#include "joint_control.h"
#include <limits.h>
static uint32_t r32(const uint8_t *p){return (uint32_t)p[0]|(uint32_t)p[1]<<8|(uint32_t)p[2]<<16|(uint32_t)p[3]<<24;}
static int32_t ri32(const uint8_t *p){uint32_t n=r32(p);return n<=INT32_MAX?(int32_t)n:(int32_t)((int64_t)n-INT64_C(4294967296));}
static inline __attribute__((always_inline)) int64_t round_shift(int64_t x,unsigned n){
 uint64_t m=x<0?UINT64_C(0)-(uint64_t)x:(uint64_t)x;
 int64_t q=(int64_t)((m+(UINT64_C(1)<<(n-1u)))>>n);
 return x<0?-q:q;
}
/* fast17-size: joint_control.c is built -Os (build.sh). One out-of-line copy per shift: small, and each shifts by a
 * constant (an out-of-line round_shift(x,n) shifts by a variable n, slow on the M0+: the tick cost +700 cycles). */
static __attribute__((noinline)) int64_t round_shift8(int64_t x){return round_shift(x,8);}
static __attribute__((noinline)) int64_t round_shift16(int64_t x){return round_shift(x,16);}
static __attribute__((noinline)) int64_t round_shift24(int64_t x){return round_shift(x,24);}
void joint_init(joint_control *s){*s=(joint_control){0};s->terminal=true;est32_init(&s->est);est32_init(&s->est_fast);}
/* v12: fixed fast tracker for the inertia-feedback acceleration (KF30 gains, lead 1). */
static est32_config fast_config(const joint_control *s){est32_config f=s->config.est;f.k0=UINT32_C(737857423);f.k1=UINT32_C(69489982);f.k2=UINT32_C(3272216);f.deadzone_q20=0;f.lead_ticks=1;return f;}
static void fast_update(joint_control *s,int64_t y){est32_config f=fast_config(s);est32_output o={0};
 if(!est32_update(&s->est_fast,&f,y,&o)){est32_init(&s->est_fast);s->a_fast_q16=0;}else s->a_fast_q16=o.resynced?0:o.a_q16;}
bool joint_off(const joint_control *s){return s&&!s->actual_pwm;}
void joint_stop(joint_control *s){s->terminal=true;s->actual_pwm=0;s->planned_pwm=0;s->requested_q24=0;s->nominal_q24=0;}
bool joint_plan_pwm(int64_t u,uint32_t gain,uint16_t mv,int16_t old,bool blank,joint_plan *p){
 /* fast17 (review P1-V1): the upper supply bound is a sanity range (20 V); the 8.0 V upper protection is app_tick's,
  * switchable with 1152 (was 7.0 V here, which refused every plan above it). The bound also keeps the uint32 product
  * gain*vd below 2^32 (gain <= 2^24, vd <= 200: 3.4e9): check that product before raising it. */
 if(!p||!gain||gain>(1u<<24)||mv<3500||mv>20000||old< -480||old>480)return false;
 /* gain is Q24 Nm/V. Unchanged from the qualified image. */
 uint32_t vd=mv%100u==0u?mv/100u:0u;
 uint64_t rail=vd?(gain*vd)/10u:(uint64_t)gain*mv/1000u;
 if(!rail||rail>INT32_MAX)return false;
 uint64_t mag=u<0?UINT64_C(0)-(uint64_t)u:(uint64_t)u;
 uint32_t pwm,flags=0;
 if(mag>=rail){pwm=480;if(mag>rail)flags|=JC_SATURATED;}
 else pwm=(uint32_t)((mag*480u+rail/2u)/rail);
 int16_t signed_pwm=u<0?-(int16_t)pwm:(int16_t)pwm;
 if(blank&&old&&signed_pwm&&((old<0)!=(signed_pwm<0))){signed_pwm=0;flags|=JC_BLANKED;}
 uint32_t magnitude=(uint32_t)(signed_pwm<0?-signed_pwm:signed_pwm);
 uint64_t nominal;
 if(vd){
  uint32_t scaled=vd*magnitude,q=gain/4800u,r=gain-q*4800u;
  nominal=q*scaled+(r*scaled+2400u)/4800u;
 }else nominal=((uint64_t)gain*mv*magnitude+240000u)/480000u;
 int32_t torque=signed_pwm<0?-(int32_t)nominal:(int32_t)nominal;
 if((int64_t)torque!=u)flags|=JC_QUANTIZED;
 *p=(joint_plan){signed_pwm,torque,flags};return true;
}
bool joint_configure(joint_control *s,const uint8_t *p,uint16_t n){
 if(!s||!p||n!=JC_CONFIG_BYTES||!joint_off(s)||!s->terminal)return false;
 uint32_t token=r32(p);
 if(!token||token<=s->token||s->epoch==UINT32_MAX||r32(p+44)!=JC_CONFIG_MAGIC)return false;
 pd_config c={.kp_q16=ri32(p+4),.kd_q16=ri32(p+8),.torque_limit_q24=r32(p+16),
  .est={r32(p+20),r32(p+24),r32(p+28),ri32(p+32),ri32(p+36),(uint8_t)(r32(p+40)&1u)},.blank_reversal=(r32(p+40)&0x100u)!=0};
 uint32_t gain=r32(p+12);
 if(c.kp_q16<0||c.kp_q16>JC_KP_MAX_Q16||c.kd_q16<0||c.kd_q16>JC_KD_MAX_Q16
    ||!c.torque_limit_q24||c.torque_limit_q24>JC_TORQUE_LIMIT_MAX_Q24
    ||!gain||gain>(1u<<24)||(r32(p+40)&~0x101u)||!est32_config_valid(&c.est))return false;
 s->config=c;s->gain_q24=gain;s->token=token;s->epoch++;
 est32_init(&s->est);est32_init(&s->est_fast);s->a_fast_q16=0;s->v_lp_q16=0;s->history=0;s->flags=0;s->clock=0;s->configured=true;return true;
}
bool jc_angle_from_fractional_counts(int32_t whole,int32_t residue,int64_t *q){
 if(!q||residue< -65535||residue>65535)return false;
 /* q/count=411775/2^28 rad, identical to the qualified image's adapter. */
 *q=(int64_t)whole*INT64_C(6588400)+round_shift16((int64_t)residue*INT64_C(6588400));
 return true;
}
static void reject(joint_control *s){s->history=0;est32_init(&s->est);est32_init(&s->est_fast);s->a_fast_q16=0;s->rejections++;s->flags|=JC_OBSERVATION_REJECT;joint_stop(s);}
bool joint_observe(joint_control *s,int32_t counts,int32_t fraction,bool valid){
 if(!s||!s->configured||s->busy)return false;
 s->clock++;
 int64_t angle=0;est32_output o={0};int32_t dq_before=s->est.dq;
 if(!valid||!jc_angle_from_fractional_counts(counts,fraction,&angle)
    ||!est32_update(&s->est,&s->config.est,(int64_t)counts*65536+fraction,&o)){reject(s);return false;}
 /* v11 (review finding 4): est32 seeds velocity from the single raw step on resync, so one
  * corrupted sample gave +-full torque on consecutive ticks. Keep the previous velocity and
  * zero acceleration; the tracker re-converges from the new position. */
 if(o.resynced&&s->history){s->est.dq=dq_before;s->est.ddq=0;o.v_q16=s->v_hat_q16;o.a_q16=0;}
 if(s->history){
  int64_t v=round_shift16((angle-s->angle_q32)*2000); /* nominal-H raw difference, logged only */
  s->v_raw_q16=v>INT32_MAX?INT32_MAX:v<INT32_MIN?INT32_MIN:(int32_t)v;
 }else s->v_raw_q16=0;
 s->angle_q32=angle;s->v_hat_q16=o.v_q16;s->a_hat_q16=o.a_q16;s->observations++;
 fast_update(s,(int64_t)counts*65536+fraction);
 s->a_filt_q16+=(int32_t)(((int64_t)(s->comp.fast_accel?s->a_fast_q16:o.a_q16)-s->a_filt_q16)>>s->comp.afilt_shift);
 s->v_lp_q16+=(int32_t)(((int64_t)s->v_hat_q16-s->v_lp_q16)>>s->hpd.shift);   /* demo5: high-pass damping state */
 if(o.resynced)s->flags|=JC_RESYNC;
 if(s->history<3)s->history++;
 return true;
}
static int64_t clamp64(int64_t x,int64_t lim){return x>lim?lim:x< -lim?-lim:x;}
bool joint_set_gains(joint_control *s,const uint8_t *p,uint16_t n){
 if(!s||!p||n!=8u||!s->configured)return false;
 int32_t kp=ri32(p),kd=ri32(p+4);
 if(kp<0||kp>JC_KP_MAX_Q16||kd<0||kd>JC_KD_MAX_Q16)return false;
 s->config.kp_q16=kp;s->config.kd_q16=kd;return true;
}
bool joint_set_comp(joint_control *s,const uint8_t *p,uint16_t n){
 if(!s||!p||n!=JC_COMP_BYTES||r32(p+36)!=JC_COMP_MAGIC)return false;
 jc_comp c={ri32(p),ri32(p+4),ri32(p+8),ri32(p+12),ri32(p+16),ri32(p+20),r32(p+24),ri32(p+28),r32(p+32)&0xffu,((r32(p+32)>>8)&1u)!=0};
 if(c.jfb_q24<0||c.jfb_q24>83886||c.afilt_shift>7u||(r32(p+32)>>9))return false;   /* jfb <= 5e-3 kg*m^2 */
 /* bounds: c0<=0.1 Nm, c1<=1.0, b<=0.05 Nm*s/rad, ja<=0.01 kg*m^2, 0.01<=veps<=5 rad/s, boost<=0.3 Nm, <=40 ticks */
 if(c.c0_q24<0||c.c0_q24>1677722||c.c1_q16<0||c.c1_q16>65536||c.b_q16<0||c.b_q16>3277||c.ja_q24<0||c.ja_q24>167772
    ||((c.c0_q24||c.c1_q16)&&c.veps_q16<655)||c.veps_q16>327680||c.boost_q24<0||c.boost_q24>5033165||c.boost_ticks>40u)return false;
 s->comp=c;s->boost_left=0;return true;
}
bool joint_set_comp2(joint_control *s,const uint8_t *p,uint16_t n){
 if(!s||!p||n!=JC_COMP2_BYTES||r32(p+12)!=JC_COMP2_MAGIC)return false;
 jc_comp2 g={ri32(p),ri32(p+4),r32(p+8)};
 if(g.b_drop_q16<0||g.b_drop_q16>3277||g.vgate_q16<0||g.vgate_q16>327680||(g.flags&~1u))return false;
 s->comp2=g;return true;
}
bool joint_set_hpd(joint_control *s,const uint8_t *p,uint16_t n){
 if(!s||!p||n!=JC_HPD_BYTES||r32(p+8)!=JC_HPD_MAGIC)return false;
 jc_hpd h={ri32(p),r32(p+4)};
 if(h.kd_q16<0||h.kd_q16>65536||(h.kd_q16&&(h.shift<4u||h.shift>12u))||(!h.kd_q16&&h.shift>12u))return false;
 s->hpd=h;s->v_lp_q16=s->v_hat_q16;return true;   /* start with no high-pass transient */
}
bool joint_prepare(joint_control *s,int64_t target,int32_t velocity,int32_t accel,uint16_t mv,bool permit){
 if(!s||!s->configured||s->busy)return false;
 joint_plan plan={0};
 s->planned_pwm=0;s->nominal_q24=0;
 if(permit&&!s->terminal&&s->history>=3){
  int64_t e=target-s->angle_q32;
  const int64_t lim=INT64_C(1)<<38; /* 64 rad: keeps Kp*e inside int64 */
  if(e>lim)e=lim;else if(e< -lim)e=-lim;
  int64_t u=round_shift24((int64_t)s->config.kp_q16*e)
           +round_shift8((int64_t)s->config.kd_q16*((int64_t)velocity-s->v_hat_q16));
  const jc_comp *c=&s->comp;
  u+=round_shift16((int64_t)c->ja_q24*accel);                     /* inertia feedforward (reference accel) */
  if(s->tau_mode)u=s->tau_q24;                                      /* fast16: a torque command replaces the PD law */
  u=clamp64(u,INT64_C(1)<<31);
  int64_t add=0;
  if(s->tau_mode!=JC_TAU_RAW){                                      /* fast16: raw motor torque skips the compensation */
  int64_t inertia=c->jfb_q24?round_shift16((int64_t)c->jfb_q24*s->a_filt_q16):0;  /* rotor inertia feedback */
  {const jc_comp2 *g=&s->comp2;int32_t av=s->v_hat_q16<0?-s->v_hat_q16:s->v_hat_q16;
   if((g->flags&1u)&&inertia&&av<g->vgate_q16&&!((inertia>0&&s->v_hat_q16>0)||(inertia<0&&s->v_hat_q16<0)))inertia=0;}  /* demo4: no braking near stop */
  add+=inertia;
  if(c->c0_q24||c->c1_q16){                                         /* load-dependent Coulomb, smooth sign */
   /* smooth sign, Q16: exactly +-1 outside the knee (no divide); defined multiply inside it
    * (was a left shift of a negative velocity = UB, found in review 2026-11-20). Same values as before. */
   int64_t sgn=0;
   if(c->veps_q16){int64_t v=s->v_hat_q16,av=v<0?-v:v;sgn=av>=c->veps_q16?(v<0?-65536:65536):(v<0?-(int64_t)((uint64_t)av*65536u/(uint32_t)c->veps_q16):(int64_t)((uint64_t)av*65536u/(uint32_t)c->veps_q16));}
   int64_t load=u+inertia;                                          /* torque the gears transmit */
   int64_t mag=c->c0_q24+round_shift16((int64_t)c->c1_q16*(load<0?-load:load));
   add+=round_shift16(mag*sgn);
  }
  {int32_t ap=s->actual_pwm<0?-s->actual_pwm:s->actual_pwm;int64_t be=(int64_t)c->b_q16-(int64_t)((uint32_t)s->comp2.b_drop_q16*(uint32_t)ap/480u);if(be<0)be=0;
   add+=round_shift8(be*s->v_hat_q16);}                            /* viscous + back-EMF, duty-dependent (demo4) */
  if(s->hpd.kd_q16)add+=round_shift8(-(int64_t)s->hpd.kd_q16*((int64_t)s->v_hat_q16-s->v_lp_q16));   /* demo5: high-pass damping */
  int32_t tsign=u>0?1:u<0?-1:0;
  if(c->boost_q24&&c->boost_ticks&&tsign&&s->prev_tau_sign&&tsign!=s->prev_tau_sign)s->boost_left=c->boost_ticks;
  if(tsign)s->prev_tau_sign=tsign;
  if(s->boost_left){add+=tsign*(int64_t)c->boost_q24;s->boost_left--;}  /* close the backlash gap faster */
  }
  s->comp_q24=add;u+=add;
  uint32_t f=0;int64_t L=s->config.torque_limit_q24;
  if(u>L){u=L;f=JC_LIMITED;}else if(u< -L){u=-L;f=JC_LIMITED;}
  s->requested_q24=u-add;   /* output-torque command after the limit (motor total = requested + comp) */
  if(!joint_plan_pwm(u,s->gain_q24,mv,s->actual_pwm,s->config.blank_reversal,&plan)){reject(s);plan=(joint_plan){0};}
  else s->flags=(uint16_t)((s->flags&~(uint16_t)(JC_QUANTIZED|JC_SATURATED|JC_BLANKED|JC_LIMITED))|plan.flags|f);
 }else s->requested_q24=0;
 if(s->terminal||!permit)plan=(joint_plan){0};
 if(!plan.pwm&&joint_off(s))return false;
 s->planned_pwm=plan.pwm;s->nominal_q24=plan.nominal_q24;return true;
}
bool joint_prepare_pwm(joint_control *s,int16_t pwm,uint16_t mv,bool permit){
 if(!s||!s->configured||s->busy)return false;
 s->planned_pwm=0;s->nominal_q24=0;
 int16_t p=(permit&&!s->terminal&&s->history>=3&&pwm>=-480&&pwm<=480)?pwm:0;
 if(!p&&joint_off(s)){s->requested_q24=0;return false;}
 /* nominal torque = gain*V*pwm/480 (gain Q24 Nm/V, mv millivolts) */
 uint64_t nm=((uint64_t)s->gain_q24*mv*(uint32_t)(p<0?-p:p))/480000u;int64_t nominal=p<0?-(int64_t)nm:(int64_t)nm;
 s->planned_pwm=p;s->nominal_q24=(int32_t)nominal;s->requested_q24=nominal;
 s->flags=(uint16_t)(s->flags&~(uint16_t)(JC_QUANTIZED|JC_SATURATED|JC_BLANKED|JC_LIMITED));return true;
}
/* fast17 reacquire (review P1-R1): after a dropout the coasted (possibly frozen) prediction can be far from the confirmed
 * sample, and est32 drops a step of >= 64 counts (un-primed: SENSOR). Re-seed both trackers at the measured position y with
 * one tick of the motion dy across the gap, so the next est32_update has no innovation; filtered accelerations restart at 0. */
void joint_reseed(joint_control *s,int64_t y_q16,int64_t y_old_q16,uint32_t gap){
 int64_t d=y_q16-y_old_q16;                                            /* the motion across the gap; one tick of it: */
 if(d<=-(INT64_C(1)<<31)||d>=(INT64_C(1)<<31))return;
 uint32_t m=(uint32_t)(d<0?-d:d)/gap;int64_t dy_q16=d<0?-(int64_t)m:(int64_t)m;   /* unsigned 32-bit divide (no 64-bit signed one) */
 if(dy_q16>(INT64_C(1)<<22)||dy_q16< -(INT64_C(1)<<22))return;       /* beyond 64 counts/tick: let est32 reject it */
 est32_state *e[2]={&s->est,&s->est_fast};
 for(unsigned i=0;i<2u;i++){e[i]->y_last_q16=y_q16-dy_q16;e[i]->e=0;e[i]->dq=(int32_t)dy_q16*16;e[i]->ddq=0;e[i]->primed=1;}
 s->a_fast_q16=0;s->a_filt_q16=0;
}
bool joint_coast(joint_control *s,bool freeze){
 if(!s||!s->configured||s->busy||s->history<3)return false;
 s->est.ddq=0;if(freeze)s->est.dq=0;   /* v11 (review finding 5): no constant-acceleration runaway */
 s->est_fast.ddq=0;if(freeze)s->est_fast.dq=0;{int64_t yf;if(s->est_fast.primed&&est32_predict(&s->est_fast,&yf))fast_update(s,yf);else{est32_init(&s->est_fast);s->a_fast_q16=0;}}
 int64_t y;if(!est32_predict(&s->est,&y))return false;
 int64_t whole=y>=0?y/65536:-((-y+65535)/65536);int32_t frac=(int32_t)(y-whole*65536);
 if(whole<INT32_MIN||whole>INT32_MAX)return false;
 s->clock++;int64_t angle=0;est32_output o={0};
 if(!jc_angle_from_fractional_counts((int32_t)whole,frac,&angle)||!est32_update(&s->est,&s->config.est,y,&o))return false;
 s->v_raw_q16=0;s->angle_q32=angle;s->v_hat_q16=o.v_q16;s->a_hat_q16=o.a_q16;
 s->a_filt_q16+=(int32_t)(((int64_t)(s->comp.fast_accel?s->a_fast_q16:o.a_q16)-s->a_filt_q16)>>s->comp.afilt_shift);
 s->v_lp_q16+=(int32_t)(((int64_t)s->v_hat_q16-s->v_lp_q16)>>s->hpd.shift);return true;
}
bool joint_applied(joint_control *s,int16_t actual,bool ok){
 if(!s||!s->configured)return false;
 if(!ok||actual!=s->planned_pwm){s->flags|=JC_DRIVER_REJECT;joint_stop(s);return false;}
 s->actual_pwm=actual;return true;
}
