/* PD baseline native tests: control law, configuration, gating, app wiring. */
#include "app.h"
#include "hw_io.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
static void p16(uint8_t*p,uint16_t v){p[0]=(uint8_t)v;p[1]=(uint8_t)(v>>8);}
static void p32(uint8_t*p,uint32_t v){p16(p,(uint16_t)v);p16(p+2,(uint16_t)(v>>16));}
static uint32_t g32(const uint8_t*p){return (uint32_t)p[0]|(uint32_t)p[1]<<8|(uint32_t)p[2]<<16|(uint32_t)p[3]<<24;}
static void fixture(uint8_t *cal,uint8_t *nv){memset(cal,255,4096);memset(nv,255,4096);p16(cal+0x800,2);p16(cal+0x810,1073);for(unsigned i=1;i<64;i++)p16(cal+0x810+8*(64-i),i*64);p32(nv,0xabab);p32(nv+4,0);p32(nv+8,0);p32(nv+12,0);const uint8_t keys[]={0,1,7,8,10,11,20,21,22,23,36,37,38,39},v[]={176,4,1,3,0,3,0,0,0,0,224,1,0,0};for(unsigned i=0;i<sizeof keys;i++){p32(nv+16+8*i,keys[i]|((unsigned)v[i]<<16));p32(nv+20+8*i,0);}}
#define Q16(x) ((int32_t)lround((x)*65536.0))
#define Q24(x) ((uint32_t)llround((x)*16777216.0))
/* KF30 lead-1 gains from the offline estimator study (2026-11-15) (CAKalman(30, lead=H)). */
enum {K0=737857423u,K1=69489982u,K2=3272216u};
static void config(uint8_t *b,uint32_t token,double kp,double kd,double gain,double limit){
 memset(b,0,JC_CONFIG_BYTES);p32(b,token);p32(b+4,(uint32_t)Q16(kp));p32(b+8,(uint32_t)Q16(kd));p32(b+12,Q24(gain));p32(b+16,Q24(limit));
 p32(b+20,K0);p32(b+24,K1);p32(b+28,K2);p32(b+32,0);p32(b+36,16u<<20);p32(b+40,1);p32(b+44,JC_CONFIG_MAGIC);}
static const double RAD_PER_COUNT=411775.0/268435456.0;

static void law_tests(void){
 joint_control c;joint_init(&c);uint8_t b[JC_CONFIG_BYTES];
 /* validation */
 config(b,1,2,0.05,0.0588,0.3);assert(!joint_configure(&c,b,47));
 config(b,0,2,0.05,0.0588,0.3);assert(!joint_configure(&c,b,48));
 config(b,1,-1,0.05,0.0588,0.3);assert(!joint_configure(&c,b,48)); /* Kp=0 is now legal (transparent mode); negative is not */
 config(b,1,51,0.05,0.0588,0.3);assert(!joint_configure(&c,b,48));
 config(b,1,2,-0.01,0.0588,0.3);assert(!joint_configure(&c,b,48));
 config(b,1,2,2.5,0.0588,0.3);assert(!joint_configure(&c,b,48));
 config(b,1,2,0.05,0.0588,0);assert(!joint_configure(&c,b,48));
 config(b,1,2,0.05,0.0588,2.5);assert(!joint_configure(&c,b,48));
 config(b,1,2,0.05,0,0.3);assert(!joint_configure(&c,b,48));
 config(b,1,2,0.05,0.0588,0.3);p32(b+40,2);assert(!joint_configure(&c,b,48));p32(b+40,0x201);assert(!joint_configure(&c,b,48));
 config(b,1,2,0.05,0.0588,0.3);p32(b+44,0);assert(!joint_configure(&c,b,48));
 config(b,1,2,0.05,0.0588,0.3);assert(joint_configure(&c,b,48)&&c.configured&&c.terminal);
 assert(!joint_configure(&c,b,48)); /* token must increase */
 /* history gate: no output before 3 observations */
 c.terminal=false;
 assert(joint_observe(&c,1000,0,true));assert(!joint_prepare(&c,(int64_t)2000*6588400,0,0,5000,true));
 assert(joint_observe(&c,1000,0,true));assert(joint_observe(&c,1000,0,true));assert(c.history==3);
 /* P term: error 100 counts, v_hat = 0 at rest */
 int64_t target=(int64_t)1100*6588400;
 assert(joint_prepare(&c,target,0,0,5000,true));
 double e=100*RAD_PER_COUNT,u=2.0*e; /* 0.307 Nm > 0.3 limit -> limited */
 assert(c.flags&JC_LIMITED);assert(c.requested_q24==(int64_t)Q24(0.3));
 /* rail at 5.0 V: 0.0588*5 = 0.294 Nm < 0.3 -> saturated full PWM */
 assert(c.planned_pwm==480&&(c.flags&JC_SATURATED));
 assert(joint_applied(&c,c.planned_pwm,true)&&c.actual_pwm==480);
 /* small error: exact law, unclamped */
 target=(int64_t)1010*6588400;assert(joint_observe(&c,1000,0,true));assert(joint_prepare(&c,target,0,0,5000,true));
 e=10*RAD_PER_COUNT;u=2.0*e-(Q16(0.05)/65536.0)*c.v_hat_q16/65536.0;
 assert(fabs(c.requested_q24/16777216.0-u)<1e-6);
 int pwm=(int)lround(u/(0.0588*5.0)*480);assert(abs(c.planned_pwm-pwm)<=1&&!(c.flags&JC_LIMITED));
 assert(joint_applied(&c,c.planned_pwm,true));
 /* reversal (default fast): direct sign change, no blanked tick */
 target=(int64_t)990*6588400;assert(joint_observe(&c,1000,0,true));assert(joint_prepare(&c,target,0,0,5000,true));
 assert(c.planned_pwm<0&&!(c.flags&JC_BLANKED));assert(joint_applied(&c,c.planned_pwm,true));
 { /* legacy blank option (config word 40 bit 8) still available */
  joint_control d=c;d.config.blank_reversal=true;target=(int64_t)1010*6588400;assert(joint_observe(&d,1000,0,true));assert(joint_prepare(&d,target,0,0,5000,true));
  assert(d.planned_pwm==0&&(d.flags&JC_BLANKED)); }
 { joint_plan pp;assert(joint_plan_pwm(Q24(0.1),Q24(0.0588),6900,0,false,&pp)&&pp.pwm>0);assert(joint_plan_pwm(Q24(0.1),Q24(0.0588),7100,0,false,&pp)&&!joint_plan_pwm(Q24(0.1),Q24(0.0588),20001,0,false,&pp))   /* fast17 P1-V1: 7.1 V plans; > 20 V refused */; }
 /* D term against the reference velocity: v_ref=+1 rad/s at rest pushes positive */
 assert(joint_observe(&c,1000,0,true));assert(joint_prepare(&c,(int64_t)1000*6588400,Q16(1.0),0,5000,true));
 assert(c.requested_q24>0&&fabs(c.requested_q24/16777216.0-(Q16(0.05)/65536.0)*(1.0-c.v_hat_q16/65536.0))<1e-6);
 /* driver mismatch stops; invalid observation stops and clears history */
 assert(!joint_applied(&c,7,true)&&c.terminal&&c.actual_pwm==0);
 c.terminal=false;assert(!joint_observe(&c,1000,0,false)&&c.history==0&&c.terminal);
 /* config refused while output is applied */
 c.actual_pwm=5;config(b,9,2,0.05,0.0588,0.3);assert(!joint_configure(&c,b,48));c.actual_pwm=0;
 puts("PD law: validation, history gate, P/D exactness, limit, rail, reversal blank, stop paths PASS");
}

static void estimator_tests(void){
 joint_control c;joint_init(&c);uint8_t b[JC_CONFIG_BYTES];config(b,1,2,0.05,0.0588,0.3);assert(joint_configure(&c,b,48));
 /* constant 2 rad/s ramp (sub-count fractional input) converges */
 double w=2.0,cpt=w*0.0005/RAD_PER_COUNT; /* counts per tick */
 for(int k=0;k<2000;k++){double y=1000+cpt*k;int32_t whole=(int32_t)floor(y);int32_t frac=(int32_t)lround((y-whole)*65536);if(frac==65536){whole++;frac=0;}assert(joint_observe(&c,whole,frac,true));}
 assert(fabs(c.v_hat_q16/65536.0-w)<0.01);assert(fabs(c.a_hat_q16/65536.0)<1.0);
 /* quantized ramp (whole counts only): bounded velocity ripple */
 joint_init(&c);assert(joint_configure(&c,b,48));double vmin=1e9,vmax=-1e9;
 for(int k=0;k<4000;k++){int32_t y=1000+(int32_t)floor(cpt*k);assert(joint_observe(&c,y,0,true));if(k>2000){double v=c.v_hat_q16/65536.0;if(v<vmin)vmin=v;if(v>vmax)vmax=v;}}
 printf("estimator: quantized 2 rad/s ramp v_hat in [%.3f, %.3f] rad/s\n",vmin,vmax);
 assert(vmin>1.5&&vmax<2.5);
 puts("estimator wiring: fractional ramp convergence, quantized ripple bound PASS");
}

static void app_tests(void){
 static uint8_t cal[4096],nv[4096];fixture(cal,nv);static app_state s;assert(app_init(&s,cal,nv,true));
 uint8_t b[128],one=1,off=0;
 assert(app_read(&s,208,16,b)==0&&!memcmp(b,"FW231-PD-BUS",13));assert(app_read(&s,0,64,b)==0&&b[6]==231);
 assert(app_read(&s,600,32,b)==0&&g32(b)==JC_CONFIG_MAGIC&&g32(b+16)==JC_CONFIG_BYTES);
 /* retired learner/energy addresses are gone */
 assert(app_write(&s,384,64,b)!=0&&app_write(&s,448,96,b)!=0&&app_write(&s,544,8,b)!=0&&app_write(&s,1056,8,b)!=0&&app_write(&s,1000,52,b)!=0);
 assert(app_read(&s,1104,96,b)!=0&&app_read(&s,1200,48,b)!=0&&app_read(&s,1280,64,b)!=0);
 assert(app_write(&s,64,1,&one)!=0); /* not configured */
 uint8_t c[JC_CONFIG_BYTES];config(c,1,1.5,0.04,0.0588,0.25);assert(app_write(&s,320,48,c)==0);
 assert(app_read(&s,704,56,b)==0&&g32(b)==1&&g32(b+12)==(uint32_t)Q16(1.5)&&g32(b+16)==(uint32_t)Q16(0.04)&&g32(b+24)==Q24(0.25));
 sensing_adc adc={.voltage_decivolts=50,.temperature_c=25};
 for(unsigned i=1;i<=20;i++)(void)app_tick(&s,true,1234,&adc,true,500*i);
 assert(s.ready&&s.control.history==3);
 int32_t here=s.position_counts;
 /* target +40 counts as a true step (duration 1 tick) */
 p32(b,(uint32_t)(here+40));p32(b+4,1);p32(b+8,0);p32(b+12,0);assert(app_write(&s,256,16,b)==0);
 assert(app_write(&s,64,1,&one)==0&&s.enabled);
 assert(app_write(&s,320,48,c)==7); /* no reconfigure while enabled */
 bool has=app_tick(&s,true,1234,&adc,true,10500);assert(has);
 double e=40*RAD_PER_COUNT,u=1.5*e;assert(s.commanded_pwm>0);
 assert(fabs(s.requested_q24/16777216.0-u)<0.002);
 assert(app_commit(&s,s.commanded_pwm,true));
 for(int k=0;k<4;k++){s.daq_phase=3;app_record_applied(&s);}
 assert(app_read(&s,1536,32,b)==0&&(int32_t)g32(b+4)==here&&(int32_t)g32(b+20)==s.requested_q24&&(int16_t)(b[24]|b[25]<<8)==s.commanded_pwm);
 /* Stop, fault clear resets the estimator history */
 assert(app_write(&s,64,1,&off)==0&&!s.enabled);
 (void)app_tick(&s,true,1234,&adc,true,11000);assert(app_commit(&s,0,true));
 assert(joint_off(&s.control));uint8_t clr=0x5a;assert(app_write(&s,126,1,&clr)==0&&s.control.history==0);
 /* travel, speed and comms-timeout trips are gone: large excursion + 1 s silence keep running */
 for(unsigned i=1;i<=25;i++)(void)app_tick(&s,true,1234,&adc,true,11000+500*i);
 s.travel_limit=5;p32(b,(uint32_t)(s.position_counts+3));p32(b+4,1);p32(b+8,0);p32(b+12,0);assert(app_write(&s,256,16,b)==0);assert(app_write(&s,64,1,&one)==0);
 for(unsigned i=0;i<2000;i++)(void)app_tick(&s,true,(uint16_t)((1224+i*7)%4096),&adc,true,30000+500*i);
 assert(!(s.faults&(APP_FAULT_TRAVEL|APP_FAULT_SPEED|APP_FAULT_TIMEOUT))&&s.enabled);
 assert(app_write(&s,832,8,b)==4);
 puts("app wiring: identity, retired addresses, configure/readback, step ARM, PWM sign, DAQ record, Stop/clear, no travel/speed/timeout trips PASS");
}

static void bam_tests(void){
 static uint8_t cal[4096],nv[4096];fixture(cal,nv);static app_state s;assert(app_init(&s,cal,nv,true));
 uint8_t b[64],one=1,c[JC_CONFIG_BYTES];config(c,1,2,0.05,0.0588,2.0);assert(app_write(&s,320,48,c)==0);
 sensing_adc adc={.voltage_decivolts=50,.temperature_c=25};uint32_t now=0;
 for(unsigned i=1;i<=20;i++){now+=500;(void)app_tick(&s,true,1234,&adc,true,now);}
 int32_t here=s.position_counts;
 /* invalid selections */
 p32(b,(uint32_t)here);p32(b+4,4);p32(b+8,0);p32(b+12,1);assert(app_write(&s,272,16,b)==4);
 p32(b+4,1);p32(b+8,1);assert(app_write(&s,272,16,b)==4);p32(b+8,0);p32(b+12,50);assert(app_write(&s,272,16,b)==4);
 /* lift_and_drop: select, ARM, release at 2 s, terminal OFF at 6 s */
 p32(b+12,1);assert(app_write(&s,272,16,b)==0&&s.bam.selected&&!s.bam.running);
 s.travel_limit=100000;s.speed_limit=1000000;
 assert(app_write(&s,64,1,&one)==0&&s.enabled&&s.bam.running);
 assert(app_write(&s,272,16,b)==7);p32(b+4,0);assert(app_write(&s,256,16,b)==7);p32(b+4,1);
 unsigned released_at=0,terminal_at=0;
 for(unsigned k=0;k<=12001;k++){now+=500;bool has=app_tick(&s,true,1234,&adc,true,now);if(has)(void)app_commit(&s,s.commanded_pwm,true);
  if(!released_at&&s.bam.released)released_at=k;if(!terminal_at&&!s.bam.running)terminal_at=k;
  s.ring_count=0;s.last_heartbeat_us=now; /* host drain + heartbeat stand-in */
  if(k<3990&&!(s.enabled&&!s.faults)){printf("k=%u faults=%#x\n",k,s.faults);assert(0);}}
 assert(released_at==4000&&terminal_at==12000&&!s.enabled&&!s.faults&&!s.bam.running);
 assert(s.reference.q==s.bam.center_q16-102944);
 /* a 256 target clears the selection */
 p32(b,(uint32_t)here);p32(b+4,1);p32(b+8,0);p32(b+12,0);assert(app_write(&s,256,16,b)==0&&!s.bam.selected);
 printf("BAM app: select validation, ARM runs native profile, lift_and_drop release tick %u, terminal tick %u, retarget clears PASS\n",released_at,terminal_at);
}

static void guard_tests(void){
 static uint8_t cal[4096],nv[4096];fixture(cal,nv);static app_state s;assert(app_init(&s,cal,nv,true));
 uint8_t b[64],one=1,c[JC_CONFIG_BYTES];config(c,1,2,0.05,0.0588,2.0);assert(app_write(&s,320,48,c)==0);
 /* I2t current overload: 2.5 A bursts of 100 ms never trip; sustained 2.5 A trips after
  * ~0.36 s (tau 2.05 s, 1.0 A RMS default); 0.9 A forever never trips; 0 disables. */
 sensing_adc adc={.voltage_decivolts=50,.temperature_c=25,.current_magnitude_ma=0};uint32_t now=0;
 for(unsigned i=1;i<=20;i++){now+=500;(void)app_tick(&s,true,1234,&adc,true,now);}
 assert(app_read(&s,848,16,b)==0&&g32(b)==1000&&g32(b+4)==65&&g32(b+8)==80);   /* fast17: 8.0 V default (owner 2026-11-24) */
 for(unsigned burst=0;burst<5;burst++){adc.current_magnitude_ma=2500;for(unsigned i=0;i<200;i++){now+=500;(void)app_tick(&s,true,1234,&adc,true,now);}
  adc.current_magnitude_ma=0;for(unsigned i=0;i<8000;i++){now+=500;(void)app_tick(&s,true,1234,&adc,true,now);}}
 assert(!s.faults&&s.sensor_healthy);
 adc.current_magnitude_ma=900;for(unsigned i=0;i<40000;i++){now+=500;(void)app_tick(&s,true,1234,&adc,true,now);}
 assert(!s.faults);assert(app_read(&s,848,16,b)==0&&g32(b+12)>=890&&g32(b+12)<=900);
 adc.current_magnitude_ma=2500;unsigned trip=0;for(unsigned i=0;i<4000&&!trip;i++){now+=500;(void)app_tick(&s,true,1234,&adc,true,now);if(s.faults&APP_FAULT_CURRENT)trip=i;}
 printf("I2t: 2.5 A after 0.9 A steady trips at tick %u (%.3f s)\n",trip,trip*0.0005);assert(trip>100&&trip<1000);
 adc.current_magnitude_ma=0;for(unsigned i=0;i<20000;i++){now+=500;(void)app_tick(&s,true,1234,&adc,true,now);}
 uint8_t clr=0x5a;assert(app_write(&s,126,1,&clr)==0);
 p32(b,0);p32(b+4,65);p32(b+8,70);assert(app_write(&s,848,12,b)==0&&s.current_rms_limit_ma==0);
 adc.current_magnitude_ma=2500;for(unsigned i=0;i<4000;i++){now+=500;(void)app_tick(&s,true,1234,&adc,true,now);}
 assert(!(s.faults&APP_FAULT_CURRENT));adc.current_magnitude_ma=0;
 for(unsigned i=0;i<30000;i++){now+=500;(void)app_tick(&s,true,1234,&adc,true,now);} /* cool down before re-enabling */
 p32(b,1000);assert(app_write(&s,848,12,b)==0);now+=500;(void)app_tick(&s,true,1234,&adc,true,now);assert(!s.faults);
 p32(b+4,101);assert(app_write(&s,848,12,b)==4);p32(b+4,65);p32(b+8,81);assert(app_write(&s,848,12,b)==4);p32(b+8,70);
 adc.voltage_decivolts=68;now+=500;(void)app_tick(&s,true,1234,&adc,true,now);assert(!s.faults); /* regen up to 7.0 V tolerated */
 adc.voltage_decivolts=71;now+=500;(void)app_tick(&s,true,1234,&adc,true,now);assert(s.faults&APP_FAULT_VOLTAGE);
 adc.voltage_decivolts=50;now+=500;(void)app_tick(&s,true,1234,&adc,true,now);assert(app_write(&s,126,1,&clr)==0);
 adc.temperature_c=65;now+=500;(void)app_tick(&s,true,1234,&adc,true,now);assert(s.faults&APP_FAULT_TEMPERATURE);
 adc.temperature_c=25;now+=500;(void)app_tick(&s,true,1234,&adc,true,now);assert(app_write(&s,126,1,&clr)==0);
 for(unsigned i=0;i<25;i++){now+=500;(void)app_tick(&s,true,1234,&adc,true,now);}
 assert(s.ready&&!s.faults);
 /* constant-velocity profile: 100 counts over 400 ticks, DAQ never drained -> no fault */
 int32_t here=s.position_counts;p32(b,(uint32_t)(here+100));p32(b+4,400);p32(b+8,1);p32(b+12,0);assert(app_write(&s,256,16,b)==0);
 assert(app_write(&s,64,1,&one)==0&&s.enabled&&s.reference.linear);
 int32_t v0=s.reference.v;
 for(unsigned k=0;k<300;k++){now+=500;s.last_heartbeat_us=now;bool has=app_tick(&s,true,1234,&adc,true,now);if(has)(void)app_commit(&s,s.commanded_pwm,true);app_record_applied(&s);}
 if(!(s.enabled&&!s.faults&&s.drops>0)){printf("en=%d faults=%#x drops=%u\n",s.enabled,s.faults,s.drops);assert(0);}
 double vexp=100*RAD_PER_COUNT/0.2;assert(fabs(v0/65536.0-vexp)<0.01&&s.reference.v==v0);
 p32(b+8,2);uint8_t off=0;assert(app_write(&s,64,1,&off)==0);assert(app_write(&s,256,16,b)==4);
 puts("protections: I2t current overload (bursts pass, sustained trips, 0 off), 7.0 V / 3.5 V voltage, temperature; DAQ overflow never faults; constant-velocity profile PASS");
}

/* FW231 motion block: the planned acceleration fed to est_tau_out is the running quintic's, else 0. */
static void planned_accel_tests(void){
 static uint8_t cal[4096],nv[4096];fixture(cal,nv);static app_state s;assert(app_init(&s,cal,nv,true));
 uint8_t b[64],one=1,off=0,c[JC_CONFIG_BYTES];config(c,1,2,0.05,0.0588,2.0);assert(app_write(&s,320,48,c)==0);
 sensing_adc adc={.voltage_decivolts=50,.temperature_c=25};uint32_t now=0;
 for(unsigned i=1;i<=20;i++){now+=500;(void)app_tick(&s,true,1234,&adc,true,now);}
 assert(app_planned_accel(&s)==0);                                                          /* OFF */
 int32_t here=s.position_counts;p32(b,(uint32_t)(here+200));p32(b+4,400);p32(b+8,0);p32(b+12,0);assert(app_write(&s,256,16,b)==0);
 assert(app_write(&s,64,1,&one)==0&&s.enabled&&s.reference.active&&!s.reference.linear);
 int32_t first=0,last=0;unsigned nonzero=0;
 for(unsigned k=0;k<450;k++){now+=500;s.last_heartbeat_us=now;bool has=app_tick(&s,true,1234,&adc,true,now);if(has)(void)app_commit(&s,s.commanded_pwm,true);
  int32_t a=app_planned_accel(&s);assert(a==(s.reference.active?s.reference.a:0));
  if(a){nonzero++;if(!first)first=a;last=a;}}
 assert(nonzero>300&&first>0&&last<0&&!s.reference.active&&app_planned_accel(&s)==0);         /* accel, decel, then hold */
 p32(b,(uint32_t)(here+400));p32(b+4,1);assert(app_write(&s,256,16,b)==0&&app_planned_accel(&s)==0);   /* pure step */
 p32(b,(uint32_t)(here+600));p32(b+4,400);p32(b+8,1);assert(app_write(&s,256,16,b)==0&&s.reference.linear);
 now+=500;(void)app_tick(&s,true,1234,&adc,true,now);assert(s.reference.active&&app_planned_accel(&s)==0);  /* linear */
 now+=500;(void)app_tick(&s,true,1234,&adc,true,now);
 p32(b,(uint32_t)(here+800));p32(b+4,400);p32(b+8,0);
 for(unsigned k=0;k<500&&s.reference.active;k++){now+=500;(void)app_tick(&s,true,1234,&adc,true,now);}
 assert(app_write(&s,256,16,b)==0);for(unsigned k=0;k<50;k++){now+=500;(void)app_tick(&s,true,1234,&adc,true,now);}
 assert(s.reference.active&&s.reference.a&&app_planned_accel(&s)!=0);                      /* quintic running... */
 p32(b,120);assert(app_write(&s,260,4,b)==0&&s.openloop&&app_planned_accel(&s)==0);          /* ...open-loop duty: 0 */
 assert(app_write(&s,64,1,&off)==0&&app_planned_accel(&s)==0);                              /* OFF */
 puts("motion block planned acceleration: quintic only (accel then decel), 0 for hold, step, linear, open-loop, OFF PASS");
}

static void openloop_tests(void){
 static uint8_t cal[4096],nv[4096];fixture(cal,nv);static app_state s;assert(app_init(&s,cal,nv,true));
 uint8_t b[64],one=1,off=0,c[JC_CONFIG_BYTES];config(c,1,2,0.05,0.0588,2.0);
 assert(app_write(&s,260,4,(uint8_t[]){240,0,0,0})==4); /* not configured */
 assert(app_write(&s,320,48,c)==0);
 sensing_adc adc={.voltage_decivolts=50,.temperature_c=25};uint32_t now=0;
 for(unsigned i=1;i<=20;i++){now+=500;(void)app_tick(&s,true,1234,&adc,true,now);}
 int32_t here=s.position_counts;p32(b,(uint32_t)here);p32(b+4,1);p32(b+8,0);p32(b+12,0);assert(app_write(&s,256,16,b)==0);
 p32(b,481);assert(app_write(&s,260,4,b)==4);p32(b,(uint32_t)-481);assert(app_write(&s,260,4,b)==4);
 p32(b,(uint32_t)-300);assert(app_write(&s,260,4,b)==0&&s.openloop);
 /* OFF: open-loop selected but nothing applied */
 now+=500;(void)app_tick(&s,true,1234,&adc,true,now);assert(s.commanded_pwm==0);
 assert(app_write(&s,64,1,&one)==0&&s.enabled&&s.openloop);
 for(unsigned k=0;k<10;k++){now+=500;s.last_heartbeat_us=now;bool has=app_tick(&s,true,1234,&adc,true,now);assert(has);assert(app_commit(&s,s.commanded_pwm,true));}
 assert(s.commanded_pwm==-300&&s.actual_pwm==-300);
 double nominal=0.0588*5.0*-300/480;assert(fabs(s.requested_q24/16777216.0-nominal)<1e-4);
 p32(b,480);assert(app_write(&s,260,4,b)==0);now+=500;(void)app_tick(&s,true,1234,&adc,true,now);assert(s.commanded_pwm==480); /* fast reversal, no blank */
 assert(app_commit(&s,s.commanded_pwm,true));
 p32(b,0x80000000u);assert(app_write(&s,260,4,b)==0&&!s.openloop&&s.target_counts==s.position_counts&&!s.reference.active);
 now+=500;(void)app_tick(&s,true,1234,&adc,true,now);assert(abs(s.commanded_pwm)<480); /* PD holds here */
 p32(b,100);assert(app_write(&s,260,4,b)==0);assert(app_write(&s,64,1,&off)==0&&!s.openloop&&!s.enabled);
 {uint8_t d[4]={1,0,0,0};assert(app_write(&s,840,4,d)==0&&s.daq_decimation==1);uint8_t r4[4];assert(app_read(&s,840,4,r4)==0&&r4[0]==1);d[0]=5;assert(app_write(&s,840,4,d)==4);
  unsigned before=s.ring_count;for(unsigned k=0;k<3;k++)app_record_applied(&s);assert(s.ring_count==before+3);}
 puts("open-loop duty: bounds, OFF gating, direct duty incl. fast reversal, nominal torque, exit re-anchors PD, Stop clears PASS");
}

/* fast16 (owner 2026-11-23: "yes please add a real torque command"). 264: output torque (Nm Q24) through the gear-train
 * compensation instead of the PD law; 268: raw motor torque, no compensation; INT32_MIN to 260/264/268 leaves the mode and
 * holds here. Interactions: duty and torque exclude each other, a goal (256) is refused in torque mode, OFF/faults clear it,
 * only the configured torque limit and the duty saturation apply. */
#define S24(x) ((int32_t)Q24(x))   /* signed Nm Q24 */
static void torque_tests(void){
 /* joint level: a moving shaft (2 rad/s) with comp, PD gains large and the target far away: the PD law is ignored */
 joint_control c;joint_init(&c);uint8_t cb[JC_CONFIG_BYTES];config(cb,1,20,1.0,0.0588,2.0);assert(joint_configure(&c,cb,48));
 c.terminal=false;uint8_t p[40];memset(p,0,40);p32(p,S24(0.0055));p32(p+4,(uint32_t)Q16(0.36));p32(p+8,(uint32_t)Q16(0.024));p32(p+16,(uint32_t)Q16(0.1));p32(p+36,JC_COMP_MAGIC);
 assert(joint_set_comp(&c,p,40));
 double cpt=2.0*0.0005/RAD_PER_COUNT;for(int k=0;k<2000;k++){double y=1000+cpt*k;int32_t w=(int32_t)floor(y);int32_t f=(int32_t)lround((y-w)*65536);if(f==65536){w++;f=0;}assert(joint_observe(&c,w,f,true));}
 int64_t far=c.angle_q32-((int64_t)3<<32);   /* 3 rad behind: Kp 20 would want -60 Nm */
 c.tau_mode=JC_TAU_COMP;c.tau_q24=S24(0.05);assert(joint_prepare(&c,far,0,0,5000,true));
 double v=c.v_hat_q16/65536.0,want=0.0055*fmin(1,v/0.1)+0.36*0.05+0.024*v;
 printf("torque mode 0.05 Nm at %.3f rad/s: requested %.5f, comp %.5f (want %.5f)\n",v,c.requested_q24/16777216.0,c.comp_q24/16777216.0,want);
 assert(c.requested_q24==S24(0.05)&&fabs(c.comp_q24/16777216.0-want)<2e-4&&c.planned_pwm>0&&!(c.flags&JC_LIMITED));
 int16_t with_comp=c.planned_pwm;
 c.tau_mode=JC_TAU_RAW;assert(joint_prepare(&c,far,0,0,5000,true));
 assert(c.requested_q24==S24(0.05)&&c.comp_q24==0&&c.planned_pwm>0&&c.planned_pwm<with_comp);
 {joint_plan pl;assert(joint_plan_pwm(S24(0.05),c.gain_q24,5000,c.actual_pwm,c.config.blank_reversal,&pl));assert(c.planned_pwm==pl.pwm);}   /* duty = tau/(K*V) */
 c.tau_q24=S24(-3.0);assert(joint_prepare(&c,far,0,0,5000,true));
 assert(c.requested_q24==-S24(2.0)&&(c.flags&JC_LIMITED)&&c.planned_pwm==-480);   /* only the configured limit, then duty saturation */
 c.tau_mode=JC_TAU_PD;assert(joint_prepare(&c,far,0,0,5000,true)&&c.requested_q24<-S24(1.9));   /* back to the PD law */
 /* model check (owner: "good debug and review checks"): commanded N*m -> duty through the torque constant K (config 320
  * gain, Nm/V at the output) and the supply, duty = round(480 * tau_motor / (K * V)), saturating at 480; raw: tau_motor =
  * tau; comp (this shaft turns at +2 rad/s): tau_motor = tau + c0*sat(v/veps) + c1*|tau| + b*v. Both signs, three supplies. */
 {static const double taus[]={0.001,0.005,0.02,0.05,0.1,0.15,0.2,0.3,1.0};static const uint16_t mvs[]={3700,5000,6500};unsigned checked=0;
  double vv=c.v_hat_q16/65536.0;
  for(unsigned m=0;m<3u;m++)for(unsigned k=0;k<sizeof taus/sizeof*taus;k++)for(int sg=-1;sg<=1;sg+=2)for(unsigned mode=JC_TAU_COMP;mode<=JC_TAU_RAW;mode++){
   double tau=sg*taus[k],motor=mode==JC_TAU_RAW?tau:tau+0.0055*fmin(1,vv/0.1)+0.36*fabs(tau)+0.024*vv;
   c.tau_mode=(uint8_t)mode;c.tau_q24=S24(tau);assert(joint_prepare(&c,far,0,0,mvs[m],true));
   double want=480.0*motor/(0.0588*mvs[m]/1000.0);if(want>480)want=480;if(want< -480)want=-480;
   if(fabs(c.planned_pwm-want)>1.0){printf("duty model: tau %.4f mode %u %u mV: pwm %d want %.2f\n",tau,mode,mvs[m],c.planned_pwm,want);assert(0);}
   assert(c.requested_q24==S24(tau));checked++;}
  printf("torque -> duty model: %u cases (9 torques x 2 signs x 3 supplies x comp/raw) within 1 count\n",checked);}

 /* app level: writes, modes and interactions */
 static uint8_t cal[4096],nv[4096];fixture(cal,nv);static app_state s;assert(app_init(&s,cal,nv,true));
 uint8_t b[64],one=1,off=0,c2[JC_CONFIG_BYTES];config(c2,1,2,0.05,0.0588,2.0);
 p32(b,S24(0.1));assert(app_write(&s,264,4,b)==4&&s.control.tau_mode==JC_TAU_PD);   /* not configured */
 assert(app_write(&s,320,48,c2)==0);
 assert(app_write(&s,264,2,b)==5&&app_write(&s,268,8,b)==5);
 sensing_adc adc={.voltage_decivolts=50,.temperature_c=25};uint32_t now=0;
 for(unsigned i=1;i<=20;i++){now+=500;(void)app_tick(&s,true,1234,&adc,true,now);}
 int32_t here=s.position_counts;p32(b,(uint32_t)here);p32(b+4,1);p32(b+8,0);p32(b+12,0);assert(app_write(&s,256,16,b)==0);
 p32(b,S24(0.1));assert(app_write(&s,264,4,b)==0&&s.control.tau_mode==JC_TAU_COMP&&s.control.tau_q24==S24(0.1)&&!s.openloop);
 now+=500;(void)app_tick(&s,true,1234,&adc,true,now);assert(s.commanded_pwm==0);          /* OFF: selected, nothing applied */
 assert(app_write(&s,64,1,&one)==0&&s.enabled&&s.control.tau_mode==JC_TAU_COMP);       /* ARM keeps the mode */
 for(unsigned k=0;k<5;k++){now+=500;s.last_heartbeat_us=now;assert(app_tick(&s,true,1234,&adc,true,now));assert(app_commit(&s,s.commanded_pwm,true));}
 assert(s.requested_q24==S24(0.1)&&s.commanded_pwm>0);
 {double duty=0.1/(0.0588*5.0)*480;assert(fabs(s.commanded_pwm-duty)<=1.0);}             /* static, comp 0 in this build: tau/(K*V) */
 assert(app_planned_accel(&s)==0);
 {uint8_t r[64];assert(app_read(&s,640,64,r)==0);assert(((g32(r+52)>>2)&3u)==JC_TAU_COMP&&(int32_t)g32(r+16)==S24(0.1));}   /* status: mode, command */
 /* a goal is refused in torque mode; the stored target is unchanged */
 int32_t t0=s.target_counts;p32(b,(uint32_t)(here+100));p32(b+4,400);p32(b+8,0);p32(b+12,0);assert(app_write(&s,256,16,b)==7&&s.target_counts==t0);
 /* a rejected duty write changes nothing; a valid one leaves torque mode, and torque leaves duty */
 p32(b,600);assert(app_write(&s,260,4,b)==4&&s.control.tau_mode==JC_TAU_COMP&&!s.openloop);
 p32(b,200);assert(app_write(&s,260,4,b)==0&&s.openloop&&s.control.tau_mode==JC_TAU_PD);
 p32(b,S24(-0.05));assert(app_write(&s,268,4,b)==0&&!s.openloop&&s.openloop_pwm==0&&s.control.tau_mode==JC_TAU_RAW);
 now+=500;s.last_heartbeat_us=now;assert(app_tick(&s,true,1234,&adc,true,now));assert(app_commit(&s,s.commanded_pwm,true));assert(s.commanded_pwm<0&&s.control.comp_q24==0);
 {uint8_t r[64];assert(app_read(&s,640,64,r)==0&&((g32(r+52)>>2)&3u)==JC_TAU_RAW);}
 /* INT32_MIN to 264 leaves torque mode and holds here; a goal is accepted again */
 uint32_t e0=s.epoch;p32(b,0x80000000u);assert(app_write(&s,264,4,b)==0&&s.control.tau_mode==JC_TAU_PD&&!s.openloop&&s.target_counts==s.position_counts&&s.epoch!=e0);
 now+=500;(void)app_tick(&s,true,1234,&adc,true,now);assert(abs(s.commanded_pwm)<480);
 p32(b,(uint32_t)here);p32(b+4,1);p32(b+8,0);p32(b+12,0);assert(app_write(&s,256,16,b)==0);
 /* a planned move's acceleration (est_tau_out, read 1088) is not used once torque mode takes over */
 p32(b,(uint32_t)(here+200));p32(b+4,400);assert(app_write(&s,256,16,b)==0);
 for(unsigned k=0;k<20;k++){now+=500;s.last_heartbeat_us=now;if(app_tick(&s,true,1234,&adc,true,now))(void)app_commit(&s,s.commanded_pwm,true);}
 assert(s.reference.active&&app_planned_accel(&s)!=0);
 p32(b,S24(0.02));assert(app_write(&s,264,4,b)==0&&s.reference.active&&app_planned_accel(&s)==0);
 /* the lease-expiry HOLD action (main.c writes INT32_MIN to 260) leaves torque mode too */
 assert(app_write(&s,260,4,(uint8_t[]){0,0,0,0x80})==0&&s.control.tau_mode==JC_TAU_PD&&s.enabled);
 p32(b,S24(0.02));assert(app_write(&s,264,4,b)==0&&s.control.tau_mode==JC_TAU_COMP);
 /* OFF clears torque mode; so does a fault */
 p32(b,S24(0.1));assert(app_write(&s,264,4,b)==0);assert(app_write(&s,64,1,&off)==0&&s.control.tau_mode==JC_TAU_PD&&!s.enabled);
 assert(app_write(&s,268,4,b)==0);app_fault(&s,APP_FAULT_TEMPERATURE);assert(s.control.tau_mode==JC_TAU_PD);
 puts("torque mode (264 comp / 268 raw): PD law replaced, comp or raw, limit+saturation only, ARM keeps it, goal refused, duty/torque exclusive, rejected write no change, INT32_MIN holds here, OFF/fault clear PASS");
}

static void integrity_tests(void){
 static uint8_t cal[4096],nv[4096];fixture(cal,nv);static app_state s;assert(app_init(&s,cal,nv,true));
 uint8_t b[64],one=1,c[JC_CONFIG_BYTES];config(c,1,2,0.05,0.0588,2.0);assert(app_write(&s,320,48,c)==0);
 sensing_adc adc={.voltage_decivolts=50,.temperature_c=25};uint32_t now=0;
 s.encoder_status=0x120u; /* status supplied, MD set */
 for(unsigned i=1;i<=30;i++){now+=500;(void)app_tick(&s,true,1234,&adc,true,now);}
 assert(s.ready&&!s.faults&&s.control.history==3);
 int32_t here=s.position_counts;p32(b,(uint32_t)here);p32(b+4,1);p32(b+8,0);p32(b+12,0);assert(app_write(&s,256,16,b)==0);
 assert(app_write(&s,64,1,&one)==0&&s.enabled);
 /* single garbage samples while enabled: jump, magnet lost, I2C fail, out-of-range, ADC fail -> coast, keep running */
 uint16_t raws[5]={(uint16_t)(1234+500),1234,1234,0x1234,1234};
 for(unsigned k=0;k<5;k++){
  now+=500;s.last_heartbeat_us=now;uint16_t st=k==1?0x100u:0x120u;s.encoder_status=st;
  bool enc=k!=2;bool aok=k!=4;
  bool has=app_tick(&s,enc,raws[k],&adc,aok,now);if(has)(void)app_commit(&s,s.commanded_pwm,true);
  assert(s.enabled&&!s.faults);if(k<4)assert(s.coast&&(s.last_flags&0x4000u));
  s.encoder_status=0x120u;for(unsigned j=0;j<5;j++){now+=500;bool h=app_tick(&s,true,1234,&adc,true,now);if(h)(void)app_commit(&s,s.commanded_pwm,true);assert(!s.coast);}
 }
 assert(s.jump_rejects==1&&s.magnet_lost==1&&s.position_counts==here);
 uint8_t r[24];assert(app_read(&s,856,24,r)==0&&g32(r)==5&&g32(r+12)==1&&g32(r+16)==1);
 /* magnet warnings (ML/MH) are counted but not bad */
 s.encoder_status=0x130u;now+=500;(void)app_tick(&s,true,1234,&adc,true,now);assert(!s.coast&&s.magnet_warnings==1);s.encoder_status=0x120u;
 /* a real fast move after several bad ticks is accepted (threshold widens with the streak) */
 for(unsigned k=0;k<3;k++){now+=500;(void)app_tick(&s,false,0,&adc,true,now);}
 now+=500;(void)app_tick(&s,true,1234+20,&adc,true,now);assert(!s.coast&&s.bad_streak==0); /* 20 counts over 4 ticks: real */
 for(unsigned k=0;k<4;k++){now+=500;(void)app_tick(&s,true,(uint16_t)(1254-5*(k+1)),&adc,true,now);assert(!s.coast);} /* back at 5 counts/tick */
 /* persistent garbage: >5% bad in a second faults */
 unsigned trip=0;for(unsigned k=0;k<2000&&!trip;k++){now+=500;s.last_heartbeat_us=now;(void)app_tick(&s,k%4u!=0u,1234,&adc,true,now);if(s.faults&APP_FAULT_SENSOR)trip=k;}
 printf("integrity: 25%% bad samples trip SENSOR after %u ticks (bad in window %u)\n",trip,s.sample_budget.window);assert(trip>300&&s.sample_budget.window>100&&!s.enabled);
 puts("integrity: jump/magnet/I2C/range/ADC single faults coast without stopping, warnings counted, fast genuine move accepted, persistent >5% bad faults PASS");
}

/* Review 2026-11-20 (P1): Coulomb sign used a left shift of a negative velocity (UB). Mirror of the
 * +2 rad/s case below, run under fail-fast UBSan: the result must be the exact negative. */
static void comp_negative_direction_tests(void){
 for(int dir=1;dir>=-1;dir-=2){
  joint_control c;joint_init(&c);uint8_t b[JC_CONFIG_BYTES];config(b,1,0,0,0.0588,2.0);assert(joint_configure(&c,b,48));
  c.terminal=false;for(int k=0;k<3;k++)assert(joint_observe(&c,100000,0,true));
  uint8_t p[40];memset(p,0,40);p32(p,Q24(0.0055));p32(p+4,(uint32_t)Q16(0.36));p32(p+8,(uint32_t)Q16(0.024));p32(p+16,(uint32_t)Q16(0.1));p32(p+36,JC_COMP_MAGIC);
  assert(joint_set_comp(&c,p,40));
  double cpt=dir*3.08*0.0005/RAD_PER_COUNT;for(int k=0;k<2000;k++){double y=100000+cpt*k;int32_t w=(int32_t)floor(y);int32_t f=(int32_t)lround((y-w)*65536);if(f==65536){w++;f=0;}assert(joint_observe(&c,w,f,true));}
  assert(joint_prepare(&c,c.angle_q32,0,0,5000,true));
  double v=c.v_hat_q16/65536.0,want=0.0055*(v>0?fmin(1,v/0.1):-fmin(1,-v/0.1))+0.024*v;
  assert(fabs(c.comp_q24/16777216.0-want)<2e-4);
  printf("comp at %+.3f rad/s: %+.5f Nm (want %+.5f)\n",v,c.comp_q24/16777216.0,want);
 }
}
static void comp_tests(void){comp_negative_direction_tests();
 joint_control c;joint_init(&c);uint8_t b[JC_CONFIG_BYTES];config(b,1,0,0,0.0588,2.0);assert(joint_configure(&c,b,48)); /* Kp=Kd=0 */
 c.terminal=false;for(int k=0;k<3;k++)assert(joint_observe(&c,1000,0,true));
 /* no compensation: zero torque */
 assert(!joint_prepare(&c,(int64_t)1000*6588400,0,0,5000,true)||c.planned_pwm==0);
 uint8_t p[40];memset(p,0,40);p32(p,Q24(0.0055));p32(p+4,(uint32_t)Q16(0.36));p32(p+8,(uint32_t)Q16(0.024));p32(p+12,Q24(1.2e-3));p32(p+16,(uint32_t)Q16(0.1));p32(p+36,JC_COMP_MAGIC);
 assert(joint_set_comp(&c,p,40));
 /* moving +2 rad/s with zero output torque wanted: comp = sat*(c0) + b*v */
 double cpt=2.0*0.0005/RAD_PER_COUNT;for(int k=0;k<2000;k++){double y=1000+cpt*k;int32_t w=(int32_t)floor(y);int32_t f=(int32_t)lround((y-w)*65536);if(f==65536){w++;f=0;}assert(joint_observe(&c,w,f,true));}
 assert(joint_prepare(&c,c.angle_q32,0,0,5000,true));
 double v=c.v_hat_q16/65536.0,want=0.0055*fmin(1,v/0.1)+0.024*v;
 printf("comp at %.3f rad/s: %.5f Nm (want %.5f)\n",v,c.comp_q24/16777216.0,want);assert(fabs(c.comp_q24/16777216.0-want)<2e-4&&c.planned_pwm>0);
 /* load term scales with |tau|: Kp live = 1 Nm/rad, 0.1 rad error -> tau 0.1, comp adds 0.36*0.1 */
 uint8_t g[8];p32(g,(uint32_t)Q16(1.0));p32(g+4,0);assert(joint_set_gains(&c,g,8));
 assert(joint_prepare(&c,c.angle_q32+(int64_t)(0.1*4294967296.0),0,0,5000,true));
 want=0.1+(0.0055+0.36*0.1)*fmin(1,v/0.1)+0.024*v;double got=(c.requested_q24+c.comp_q24)/16777216.0;
 assert(fabs(got-want)<5e-4);
 /* inertia feedforward: 1.2e-3 * 100 rad/s^2 = 0.12 Nm */
 p32(g,0);assert(joint_set_gains(&c,g,8));memset(p,0,40);p32(p+12,Q24(1.2e-3));p32(p+16,(uint32_t)Q16(0.1));p32(p+36,JC_COMP_MAGIC);assert(joint_set_comp(&c,p,40));
 assert(joint_prepare(&c,c.angle_q32,0,Q16(100.0),5000,true));assert(fabs(c.requested_q24/16777216.0-0.12)<1e-4);
 /* bounds + live gains validation */
 p32(p+4,(uint32_t)Q16(1.5));assert(!joint_set_comp(&c,p,40));p32(p+4,0);p32(p+16,0);p32(p,Q24(0.005));assert(!joint_set_comp(&c,p,40));
 p32(g,(uint32_t)-1);assert(!joint_set_gains(&c,g,8));
 /* backlash boost: on a torque sign flip add boost for N ticks */
 memset(p,0,40);p32(p+16,(uint32_t)Q16(0.1));p32(p+20,Q24(0.05));p32(p+24,3);p32(p+36,JC_COMP_MAGIC);assert(joint_set_comp(&c,p,40));
 p32(g,(uint32_t)Q16(1.0));assert(joint_set_gains(&c,g,8));
 assert(joint_prepare(&c,c.angle_q32+(int64_t)(0.02*4294967296.0),0,0,5000,true));assert(c.comp_q24==0);
 int boosts=0;for(int k=0;k<5;k++){assert(joint_prepare(&c,c.angle_q32-(int64_t)(0.02*4294967296.0),0,0,5000,true));if(c.comp_q24==-(int64_t)Q24(0.05))boosts++;}
 assert(boosts==3);
 /* rotor-inertia feedback: jfb * a_filt, and it enters the c1 load */
 memset(p,0,40);p32(p+16,(uint32_t)Q16(0.1));p32(p+28,Q24(1.0e-3));p32(p+32,0);p32(p+36,JC_COMP_MAGIC);assert(joint_set_comp(&c,p,40));
 p32(g,0);p32(g+4,0);assert(joint_set_gains(&c,g,8));c.a_filt_q16=Q16(50.0);
 assert(joint_prepare(&c,c.angle_q32,0,0,5000,true));assert(fabs(c.comp_q24/16777216.0-0.05)<1e-4);
 p32(p+28,Q24(6e-3));assert(!joint_set_comp(&c,p,40));p32(p+28,0);p32(p+32,8);assert(!joint_set_comp(&c,p,40));
 puts("compensation: Coulomb+load+viscous exact, inertia FF, live gains incl Kp=0, bounds, backlash boost on sign flip, rotor-inertia feedback PASS");
}
/* v11 regressions for the independent review (2026-11-16). */
static void v11_tests(void){
 joint_control c;joint_init(&c);uint8_t b[JC_CONFIG_BYTES];config(b,1,2,0.05,0.0588,2.0);assert(joint_configure(&c,b,48));c.terminal=false;
 for(int k=0;k<20;k++){assert(joint_observe(&c,1000,0,true));(void)joint_prepare(&c,c.angle_q32,0,0,5000,true);}
 int64_t hold=c.angle_q32;
 /* finding 4: one +20-count glitch (> 16-count resync) and the return sample: no velocity spike, no +-full torque */
 for(int k=0;k<2;k++){assert(joint_observe(&c,k==0?1020:1000,0,true));assert(c.flags&JC_RESYNC);
  assert(abs(c.v_hat_q16)<Q16(0.5));(void)joint_prepare(&c,hold,0,0,5000,true);assert(abs(c.planned_pwm)<200);}
 puts("v11: glitch + resync keeps velocity, |pwm| < 200 on both ticks PASS");
 /* finding 5: coast at constant velocity, freeze to zero velocity on request */
 joint_init(&c);config(b,2,0,0,0.0588,2.0);assert(joint_configure(&c,b,48));c.terminal=false;
 for(int k=0;k<100;k++){double t=k*0.0005,y=1000+0.5*330.0*t*t/RAD_PER_COUNT;int32_t w=(int32_t)floor(y);int32_t f=(int32_t)lround((y-w)*65536);if(f==65536){w++;f=0;}assert(joint_observe(&c,w,f,true));}
 int32_t v0=c.v_hat_q16;for(int k=0;k<20;k++)assert(joint_coast(&c,false));
 printf("v11: coast 20 ticks from %.2f rad/s -> %.2f rad/s (old law would add ~3.3)\n",v0/65536.0,c.v_hat_q16/65536.0);
 assert(abs(c.v_hat_q16-v0)<Q16(0.5));assert(joint_coast(&c,true)&&c.v_hat_q16==0);
 /* finding 10: all-zero compensation block = off, accepted */
 uint8_t p[40];memset(p,0,40);p32(p+36,JC_COMP_MAGIC);assert(joint_set_comp(&c,p,40));
 p32(p,Q24(0.005));assert(!joint_set_comp(&c,p,40)); /* Coulomb on still needs veps */
 /* app: jump filter 12 counts, vmax bound 8.0 V (fast17; was 7.0 V), ARM clears boost/sign/filter state */
 static uint8_t cal[4096],nv[4096];fixture(cal,nv);static app_state s;assert(app_init(&s,cal,nv,true));
 uint8_t cb[JC_CONFIG_BYTES],q[16],one=1;config(cb,1,2,0.05,0.0588,2.0);assert(app_write(&s,320,48,cb)==0);
 p32(q,1000);p32(q+4,65);p32(q+8,81);assert(app_write(&s,848,12,q)==4);p32(q+8,75);assert(app_write(&s,848,12,q)==0);p32(q+8,70);assert(app_write(&s,848,12,q)==0);
 sensing_adc adc={.voltage_decivolts=50,.temperature_c=25};uint32_t now=0;s.encoder_status=0x120u;
 for(unsigned i=1;i<=30;i++){now+=500;(void)app_tick(&s,true,1234,&adc,true,now);}
 now+=500;(void)app_tick(&s,true,1234+13,&adc,true,now);assert(s.coast&&s.jump_rejects==1);
 now+=500;(void)app_tick(&s,true,1234,&adc,true,now);assert(!s.coast);
 now+=500;(void)app_tick(&s,true,1234+12,&adc,true,now);assert(!s.coast&&s.jump_rejects==1);
 now+=500;(void)app_tick(&s,true,1234+12,&adc,true,now);
 int32_t here=s.position_counts;uint8_t r[16];p32(r,(uint32_t)here);p32(r+4,1);p32(r+8,0);p32(r+12,0);assert(app_write(&s,256,16,r)==0);
 s.control.prev_tau_sign=1;s.control.boost_left=7;s.control.a_filt_q16=12345;
 assert(app_write(&s,64,1,&one)==0&&s.enabled&&s.control.prev_tau_sign==0&&s.control.boost_left==0&&s.control.a_filt_q16==0);
 puts("v11: jump filter 12 counts, vmax <= 8.0 V (fast17), ARM clears boost/sign/filter state PASS");
}
/* v12: fast acceleration source for the inertia feedback. */
static void v12_tests(void){
 enum {T0=566220224u,T1=26672629u,T2=418895u}; /* slow T15 tracker for the D term */
 joint_control slow,fast;uint8_t b[JC_CONFIG_BYTES];
 for(int i=0;i<2;i++){joint_control *c=i?&fast:&slow;joint_init(c);config(b,1,0,0,0.0588,2.0);p32(b+20,T0);p32(b+24,T1);p32(b+28,T2);assert(joint_configure(c,b,48));c->terminal=false;
  uint8_t p[40];memset(p,0,40);p32(p+28,Q24(1.0e-3));p32(p+32,i?0x100u:0u);p32(p+36,JC_COMP_MAGIC);assert(joint_set_comp(c,p,40));assert(c->comp.fast_accel==(i==1));}
 /* hold, then a constant 200 rad/s^2 acceleration starts: after 10 ms the fast source is much closer */
 double a=200.0;int32_t f10s=0,f10f=0;
 for(int k=0;k<200;k++){double t=k<100?0:(k-100)*0.0005,y=1000+0.5*a*t*t/RAD_PER_COUNT;int32_t w=(int32_t)floor(y);int32_t f=(int32_t)lround((y-w)*65536);if(f==65536){w++;f=0;}
  assert(joint_observe(&slow,w,f,true));assert(joint_observe(&fast,w,f,true));if(k==120){f10s=slow.a_filt_q16;f10f=fast.a_filt_q16;}}
 printf("v12: a_filt 10 ms after a 200 rad/s^2 onset: shared T15 %.1f, fast %.1f rad/s^2\n",f10s/65536.0,f10f/65536.0);
 assert(f10f>f10s+Q16(40.0)&&fast.a_fast_q16>Q16(150.0));
 /* readback carries the flag; bit 9 is rejected */
 static uint8_t cal[4096],nv[4096];fixture(cal,nv);static app_state s;assert(app_init(&s,cal,nv,true));
 uint8_t cb[JC_CONFIG_BYTES];config(cb,1,2,0.05,0.0588,2.0);assert(app_write(&s,320,48,cb)==0);
 uint8_t p[40],r[40];memset(p,0,40);p32(p+32,0x102u);p32(p+36,JC_COMP_MAGIC);assert(app_write(&s,324,40,p)==0);
 assert(app_read(&s,324,40,r)==0&&g32(r+32)==0x102u);p32(p+32,0x402u);assert(app_write(&s,324,40,p)==4); /* v13: bit 9 is model_accel, bit 10 rejected */
 /* coast keeps the fast tracker sane and freeze zeroes its velocity */
 assert(joint_coast(&fast,false)&&joint_coast(&fast,true));
 puts("v12: fast accel source lower lag than shared T15, flag readback, bit-10 reject, coast PASS");
}
/* demo4 candidate: duty-dependent back-EMF compensation and near-stop inertia gate. */
static void d4_setup(joint_control *c,double v_rad_s,double a_rad_s2,int16_t last_pwm){
 uint8_t b[JC_CONFIG_BYTES];joint_init(c);config(b,1,0,0,0.0588,2.0);assert(joint_configure(c,b,48));c->terminal=false;
 for(int k=0;k<3;k++)assert(joint_observe(c,1000,0,true));
 c->v_hat_q16=Q16(v_rad_s);c->a_filt_q16=Q16(a_rad_s2);c->actual_pwm=last_pwm;}
static double d4_comp(joint_control *c){(void)joint_prepare(c,c->angle_q32,0,0,5000,true);return c->comp_q24/16777216.0;}
static void demo4_tests(void){
 joint_control c;uint8_t p[40],g[16];
 /* comp: b 0.0225 only, then + jfb 0.5e-3 (fast flag off; a_filt injected directly) */
 memset(p,0,40);p32(p+8,(uint32_t)Q16(0.0225));p32(p+16,(uint32_t)Q16(0.1));p32(p+36,JC_COMP_MAGIC);
 d4_setup(&c,2.0,0,480);assert(joint_set_comp(&c,p,40));double off=d4_comp(&c);
 assert(fabs(off-0.045)<2e-4);                                   /* defaults: b*v exactly as demo3 */
 memset(g,0,16);p32(g+12,JC_COMP2_MAGIC);assert(joint_set_comp2(&c,g,16));assert(fabs(d4_comp(&c)-off)<1e-9);  /* all-zero = off */
 p32(g,(uint32_t)Q16(0.0065));assert(joint_set_comp2(&c,g,16));
 double full=d4_comp(&c);c.actual_pwm=0;double zero=d4_comp(&c);c.actual_pwm=-240;double half=d4_comp(&c);
 printf("demo4: b_eff*v at |duty| 100/0/50 %%: %.4f %.4f %.4f Nm (want 0.0320 0.0450 0.0385)\n",full,zero,half);
 assert(fabs(full-0.032)<2e-4&&fabs(zero-0.045)<2e-4&&fabs(half-0.0385)<2e-4);
 /* gate: jfb 0.5e-3, a = -40 rad/s^2 (braking), v = +0.1 rad/s (< vgate 0.3) */
 memset(p,0,40);p32(p+16,(uint32_t)Q16(0.1));p32(p+28,Q24(0.5e-3));p32(p+36,JC_COMP_MAGIC);
 d4_setup(&c,0.1,-40,0);assert(joint_set_comp(&c,p,40));double brake=d4_comp(&c);assert(fabs(brake+0.02)<2e-4);
 memset(g,0,16);p32(g+4,(uint32_t)Q16(0.3));p32(g+8,1);p32(g+12,JC_COMP2_MAGIC);assert(joint_set_comp2(&c,g,16));
 assert(fabs(d4_comp(&c))<1e-6);                                 /* braking near stop: dropped */
 c.v_hat_q16=Q16(0.5);assert(fabs(d4_comp(&c)+0.02)<2e-4);       /* above vgate: kept */
 c.v_hat_q16=Q16(0.1);c.a_filt_q16=Q16(40);assert(fabs(d4_comp(&c)-0.02)<2e-4);   /* accelerating with motion: kept */
 c.v_hat_q16=0;assert(fabs(d4_comp(&c))<1e-6);                   /* at rest: no push from a lagged estimate */
 /* validation and app wiring */
 p32(g,4000);assert(!joint_set_comp2(&c,g,16));p32(g,0);p32(g+8,2);assert(!joint_set_comp2(&c,g,16));p32(g+8,1);p32(g+12,0);assert(!joint_set_comp2(&c,g,16));
 static uint8_t cal[4096],nv[4096];fixture(cal,nv);static app_state s;assert(app_init(&s,cal,nv,true));
 uint8_t cb[JC_CONFIG_BYTES],r[16];config(cb,1,2,0.05,0.0588,2.0);assert(app_write(&s,320,48,cb)==0);
 p32(g,1000);p32(g+4,(uint32_t)Q16(0.3));p32(g+8,1);p32(g+12,JC_COMP2_MAGIC);assert(app_write(&s,336,16,g)==0);
 assert(app_read(&s,336,16,r)==0&&g32(r)==1000u&&g32(r+4)==(uint32_t)Q16(0.3)&&g32(r+8)==1u&&g32(r+12)==JC_COMP2_MAGIC);
 puts("demo4: zero = demo3, duty-dependent b, near-stop inertia gate, validation, 336 wiring PASS");
}
/* demo5 candidate: high-pass damping (write 340). */
static void demo5_tests(void){
 joint_control c;uint8_t h[12];
 /* off by default and with kd 0: identical compensation */
 d4_setup(&c,2.0,0,0);double base=d4_comp(&c);memset(h,0,12);p32(h+8,JC_HPD_MAGIC);assert(joint_set_hpd(&c,h,12));assert(fabs(d4_comp(&c)-base)<1e-9);
 /* steady 2 rad/s for 2 s through the real observer: high-pass part decays to ~0 */
 uint8_t b[JC_CONFIG_BYTES];joint_init(&c);config(b,1,0,0,0.0588,2.0);assert(joint_configure(&c,b,48));c.terminal=false;
 p32(h,(uint32_t)Q16(0.02));p32(h+4,8);assert(joint_set_hpd(&c,h,12));
 double cpt=2.0*0.0005/RAD_PER_COUNT;double peak=0;
 for(int k=0;k<4000;k++){double y=1000+cpt*k;int32_t w=(int32_t)floor(y);int32_t f=(int32_t)lround((y-w)*65536);if(f==65536){w++;f=0;}
  assert(joint_observe(&c,w,f,true));if(k>=3){(void)joint_prepare(&c,c.angle_q32,0,0,5000,true);double q=fabs(c.comp_q24/16777216.0);if(k<400&&q>peak)peak=q;}}
 double late=c.comp_q24/16777216.0;
 printf("demo5: HP damping kd 0.02 shift 8 at steady 2 rad/s: early peak %.4f Nm, after 2 s %.5f Nm\n",peak,late);
 assert(peak>0.02&&fabs(late)<0.001);
 /* a sudden velocity change is damped at full kd */
 c.v_lp_q16=0;c.v_hat_q16=Q16(2.0);(void)joint_prepare(&c,c.angle_q32,0,0,5000,true);assert(fabs(c.comp_q24/16777216.0+0.04)<2e-4);
 /* validation */
 p32(h,70000);assert(!joint_set_hpd(&c,h,12));p32(h,(uint32_t)Q16(0.02));p32(h+4,3);assert(!joint_set_hpd(&c,h,12));p32(h+4,13);assert(!joint_set_hpd(&c,h,12));
 p32(h+4,8);p32(h+8,0);assert(!joint_set_hpd(&c,h,12));
 /* app wiring */
 static uint8_t cal[4096],nv[4096];fixture(cal,nv);static app_state s;assert(app_init(&s,cal,nv,true));
 uint8_t cb[JC_CONFIG_BYTES],r[12];config(cb,1,2,0.05,0.0588,2.0);assert(app_write(&s,320,48,cb)==0);
 p32(h,(uint32_t)Q16(0.02));p32(h+4,9);p32(h+8,JC_HPD_MAGIC);assert(app_write(&s,340,12,h)==0);
 assert(app_read(&s,340,12,r)==0&&g32(r)==(uint32_t)Q16(0.02)&&g32(r+4)==9u);
 puts("demo5: HP damping off by default, steady motion undamped, sudden change damped, validation, 340 wiring PASS");
}
/* PD-comp image (owner, 2026-11-20): gear-train cancellation loaded at power-up when APP_DEFAULT_COMP=1. */
static void pdcomp_tests(void){
 static const uint32_t ec[9]={68451u,20054u,1401u,6375u,3277u,0u,0u,9040u,0x102u},e2[4]={405u,19661u,1u,JC_COMP2_MAGIC};
 joint_control c;joint_init(&c);assert(app_load_default_comp(&c));
 assert(c.comp.c0_q24==68451&&c.comp.c1_q16==20054&&c.comp.b_q16==1401&&c.comp.ja_q24==6375&&c.comp.veps_q16==3277&&c.comp.boost_q24==0&&c.comp.boost_ticks==0u);
 assert(c.comp.jfb_q24==9040&&c.comp.afilt_shift==2u&&c.comp.fast_accel);
 assert(c.comp2.b_drop_q16==405&&c.comp2.vgate_q16==19661&&c.comp2.flags==1u&&c.hpd.kd_q16==0&&c.hpd.shift==0u);   /* v2: HP off */
 static uint8_t cal[4096],nv[4096];fixture(cal,nv);static app_state s;assert(app_init(&s,cal,nv,true));
 uint8_t r[40];assert(app_read(&s,324,40,r)==0);
 for(unsigned i=0;i<9u;i++)assert(g32(r+4*i)==(APP_DEFAULT_COMP?ec[i]:0u));
 assert(app_read(&s,336,16,r)==0);for(unsigned i=0;i<3u;i++)assert(g32(r+4*i)==(APP_DEFAULT_COMP?e2[i]:0u));
 assert(app_read(&s,340,12,r)==0&&g32(r)==0u&&g32(r+4)==0u);   /* v2: HP damping off in every build */
 assert(!s.enabled&&!s.faults);                                       /* boots OFF, no fault */
 /* the host's config write keeps the compensation */
 uint8_t cb[JC_CONFIG_BYTES];config(cb,1,1.71,0.043,0.0504,2.0);assert(app_write(&s,320,48,cb)==0);
 assert(app_read(&s,324,40,r)==0);for(unsigned i=0;i<9u;i++)assert(g32(r+4*i)==(APP_DEFAULT_COMP?ec[i]:0u));
 assert(app_read(&s,340,12,r)==0&&g32(r)==0u);
 /* and the host can still zero it live */
 uint8_t z[40]={0};p32(z+36,JC_COMP_MAGIC);assert(app_write(&s,324,40,z)==0);assert(app_read(&s,324,40,r)==0);for(unsigned i=0;i<9u;i++)assert(g32(r+4*i)==0u);
 printf("pdcomp: APP_DEFAULT_COMP=%d power-up comp %s, survives config 320, host can zero it PASS\n",APP_DEFAULT_COMP,APP_DEFAULT_COMP?"= v2 gear-train values (demo5/6 minus HP damping, plus accel FF)":"= zero");
}
/* fast10 review F-1 (reproduced natively on fast9, repro/stuck_*.c): limp and OFF, a sample-budget SENSOR fault (101 bad
 * ADC conversions) or a 100 ms supply sag (VOLTAGE exit), then > ~1 deg of shaft motion, used to leave SENSOR stuck until a
 * reboot: the early exits skipped bad_streak++, so every later sample failed the jump gate. Now every bad tick widens it. */
static void stuck_sensor_tests(void){
 static uint8_t cal[4096],nv[4096];
 for(int scenario=0;scenario<2;scenario++){
  fixture(cal,nv);static app_state s;memset(&s,0,sizeof s);assert(app_init(&s,cal,nv,true));
  sensing_adc ok={.voltage_decivolts=50,.temperature_c=25},low={.voltage_decivolts=33,.temperature_c=25};uint32_t now=0;s.encoder_status=0x120u;
  for(unsigned i=0;i<40;i++){now+=500;(void)app_tick(&s,true,1234,&ok,true,now);}
  assert(s.ready&&!s.faults);
  uint16_t raw=1234;
  if(scenario==0){for(unsigned k=0;k<200&&!(s.faults&APP_FAULT_SENSOR);k++){now+=500;(void)app_tick(&s,true,1234,&ok,false,now);}
   assert(s.faults&APP_FAULT_SENSOR);raw=1234+13;}                                  /* then the joint settles 13 counts away */
  else for(unsigned i=0;i<200;i++){now+=500;if((i&1)&&raw<1284)raw++;(void)app_tick(&s,true,raw,&low,true,now);}   /* sags, drifts 50 */
  for(unsigned i=0;i<6000;i++){now+=500;(void)app_tick(&s,true,raw,&ok,true,now);}
  uint8_t clr=0x5a,r=app_write(&s,126,1,&clr);
  printf("stuck-sensor (%s): after 3 s good: healthy %d, bad in window %u, jump rejects %u, clear -> %u, faults 0x%x\n",scenario?"100 ms sag to 3.3 V, 50 counts":"SENSOR budget, 13 counts",
         s.sensor_healthy,s.sample_budget.window,(unsigned)s.jump_rejects,r,(unsigned)s.faults);
  assert(s.sensor_healthy&&r==0u&&!s.faults);
 }
 puts("fast10 F-1: SENSOR budget fault / supply sag followed by shaft motion while limp clears after good samples PASS");
}
/* fast10 review F-5: a target written while OFF was planned against the stale reference (left mid-move by OFF) and refused
 * (7 or 4); disarmed only the target is checked, and ARM plans from the shaft. */
static void stale_reference_tests(void){
 static uint8_t cal[4096],nv[4096];fixture(cal,nv);static app_state s;assert(app_init(&s,cal,nv,true));
 uint8_t b[16],one=1,off=0,c[JC_CONFIG_BYTES];config(c,1,1.5,0.04,0.0588,2.0);assert(app_write(&s,320,48,c)==0);
 sensing_adc adc={.voltage_decivolts=50,.temperature_c=25};uint32_t now=0;s.encoder_status=0x120u;
 for(unsigned i=0;i<30;i++){now+=500;(void)app_tick(&s,true,1234,&adc,true,now);}
 int32_t here=s.position_counts;
 p32(b,(uint32_t)here);p32(b+4,1);p32(b+8,0);p32(b+12,0);assert(app_write(&s,256,16,b)==0);assert(app_write(&s,64,1,&one)==0&&s.enabled);
 p32(b,(uint32_t)(here+3000));p32(b+4,4000);assert(app_write(&s,256,16,b)==0);   /* a 2 s move the shaft never follows */
 for(unsigned i=0;i<1000;i++){now+=500;s.last_heartbeat_us=now;if(app_tick(&s,true,1234,&adc,true,now))(void)app_commit(&s,s.commanded_pwm,true);}
 assert(app_write(&s,64,1,&off)==0&&!s.enabled);
 for(unsigned i=0;i<4;i++){now+=500;if(app_tick(&s,true,1234,&adc,true,now))(void)app_commit(&s,0,true);}
 p32(b,(uint32_t)here);p32(b+4,10);p32(b+8,0);p32(b+12,0);                           /* back to where the shaft is, 5 ms quintic */
 uint8_t r=app_write(&s,256,16,b);
 printf("stale reference: OFF mid-move (reference %s), target = shaft written OFF -> %u\n",s.reference.active?"still active":"stale",r);
 assert(r==0u);
 assert(app_write(&s,64,1,&one)==0&&s.enabled&&s.target_counts==here);
 puts("fast10 F-5: a target written OFF is range-checked only; ARM plans from the shaft PASS");
}

/* fast12 (owner, 2026-11-20: "perform recommended"): A0 never returns loader bytes; the loader is proven unchanged by one
 * CRC-32 of the whole region (read 1136), the same as zlib.crc32. */
static uint32_t ref_crc32(uintptr_t from,uintptr_t to){uint32_t c=0xffffffffu;
 for(uintptr_t a=from;a<to;a++){c^=(uint8_t)(hw_read32(a&~(uintptr_t)3u)>>(8u*(a&3u)));for(int k=0;k<8;k++)c=(c>>1)^(0xedb88320u&-(c&1u));}return ~c;}
static void loader_privacy_tests(void){
 static uint8_t cal[4096],nv[4096];fixture(cal,nv);static app_state s;assert(app_init(&s,cal,nv,true));uint8_t b[96];
 assert(app_flash_read(&s,0x08000000u,16,b)==4&&app_flash_read(&s,0x08003ff0u,16,b)==4&&app_flash_read(&s,0x08003ffcu,8,b)==4);   /* loader, and a read straddling into it */
 assert(app_flash_read(&s,0x08004000u,96,b)==0&&app_flash_read(&s,0x0800e000u,96,b)==0&&app_flash_read(&s,0x0800ffa0u,96,b)==0); /* program, own NVM */
 assert(b[0]==(uint8_t)hw_read32(0x0800ffa0u));
 uint32_t want=ref_crc32(0x08000000u,0x08004000u),got=app_loader_crc32();
 printf("loader privacy: A0 refuses 0x08000000..0x08003FFF, program/NVM readable; CRC-32 %08x (reference %08x)\n",got,want);
 assert(got==want);
 puts("fast12: loader bytes never readable, whole-region CRC-32 equals zlib's PASS");
}
/* fast12 (owner, 2026-11-20: "RETARGET without a doubt ... act as if its a new goal command"): a new 256 target while a
 * profiled move runs is accepted and starts a new profile from where the reference is, like a fresh goal. */
static void retarget_tests(void){
 static uint8_t cal[4096],nv[4096];fixture(cal,nv);static app_state s;assert(app_init(&s,cal,nv,true));
 uint8_t b[16],one=1,c[JC_CONFIG_BYTES];config(c,1,1.5,0.04,0.0588,2.0);assert(app_write(&s,320,48,c)==0);
 sensing_adc adc={.voltage_decivolts=50,.temperature_c=25};uint32_t now=0;s.encoder_status=0x120u;
 for(unsigned i=0;i<30;i++){now+=500;(void)app_tick(&s,true,1234,&adc,true,now);}
 int32_t here=s.position_counts;
 p32(b,(uint32_t)here);p32(b+4,1);p32(b+8,0);p32(b+12,0);assert(app_write(&s,256,16,b)==0);assert(app_write(&s,64,1,&one)==0);
 p32(b,(uint32_t)(here+3000));p32(b+4,4000);assert(app_write(&s,256,16,b)==0);           /* a 2 s move */
 for(unsigned i=0;i<1000;i++){now+=500;s.last_heartbeat_us=now;if(app_tick(&s,true,1234,&adc,true,now))(void)app_commit(&s,s.commanded_pwm,true);}
 assert(s.reference.active);int32_t mid=s.reference.q;
 p32(b,(uint32_t)(here+1000));p32(b+4,400);uint8_t r=app_write(&s,256,16,b);           /* new goal mid-move: 0.2 s */
 printf("retarget: new target during a profiled move -> %u (fast11: 7); profile restarts at q %d -> %d\n",r,mid,s.reference.start);
 assert(r==0u&&s.reference.active&&s.reference.start==mid&&s.target_counts==here+1000&&s.duration_ticks==400u);
 for(unsigned i=0;i<401;i++){now+=500;s.last_heartbeat_us=now;if(app_tick(&s,true,1234,&adc,true,now))(void)app_commit(&s,s.commanded_pwm,true);}
 int32_t q;assert(!s.reference.active);q=s.reference.q;
 int32_t want=(int32_t)(((int64_t)(here+1000)*411775+2048)/4096);
 assert(q==want&&!s.faults&&s.enabled);
 p32(b,(uint32_t)(here+500));p32(b+4,200);p32(b+8,1);assert(app_write(&s,256,16,b)==0);  /* linear profile ... */
 for(unsigned i=0;i<50;i++){now+=500;s.last_heartbeat_us=now;if(app_tick(&s,true,1234,&adc,true,now))(void)app_commit(&s,s.commanded_pwm,true);}
 p32(b,(uint32_t)(here+2000));p32(b+4,300);p32(b+8,1);assert(app_write(&s,256,16,b)==0); /* ... retargeted too */
 assert(s.reference.active&&s.reference.linear);
 puts("fast12: retarget mid-move (quintic and linear) starts a new profile from the current reference, reaches the new goal PASS");
}

int main(void){
#if APP_DEFAULT_COMP
 pdcomp_tests();torque_tests();return 0;   /* other suites assume zero power-up compensation; fast16: the torque path also
                                             * runs with the deployed power-up compensation loaded */
#else
 law_tests();estimator_tests();app_tests();bam_tests();guard_tests();planned_accel_tests();openloop_tests();torque_tests();integrity_tests();comp_tests();v11_tests();v12_tests();demo4_tests();demo5_tests();pdcomp_tests();stuck_sensor_tests();stale_reference_tests();loader_privacy_tests();retarget_tests();return 0;
#endif
}
