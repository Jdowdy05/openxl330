/* v1.1 Stage 3: XL330-M077 BETA build (APP_GEAR=77, APP_DEFAULT_COMP=1). Native, ASan/UBSan. Checks: the image arms only
 * on stock model 1190 (an M288's 1200 faults CONFIG), addresses 0/1 report the servo's NV model, the power-up compensation
 * is the M077 table, the jump gate is 32 counts, the motion block uses the M077 constants and its 64 rad/s clamp. */
#include "app.h"
#include "motion.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
#if APP_GEAR != 77 || !APP_DEFAULT_COMP
#error build with -DAPP_GEAR=77 -DAPP_DEFAULT_COMP=1
#endif
static void p16(uint8_t*p,uint16_t v){p[0]=(uint8_t)v;p[1]=(uint8_t)(v>>8);}
static void p32(uint8_t*p,uint32_t v){p16(p,(uint16_t)v);p16(p+2,(uint16_t)(v>>16));}
static uint32_t g32(const uint8_t*p){return (uint32_t)p[0]|(uint32_t)p[1]<<8|(uint32_t)p[2]<<16|(uint32_t)p[3]<<24;}
static void fixture(uint8_t *cal,uint8_t *nv,uint16_t model){memset(cal,255,4096);memset(nv,255,4096);p16(cal+0x800,2);p16(cal+0x810,1073);for(unsigned i=1;i<64;i++)p16(cal+0x810+8*(64-i),(uint16_t)(i*64));p32(nv,0xabab);p32(nv+4,0);p32(nv+8,0);p32(nv+12,0);
 const uint8_t keys[]={0,1,7,8,10,11,20,21,22,23,36,37,38,39};uint8_t v[]={176,4,1,3,0,3,0,0,0,0,224,1,0,0};v[0]=(uint8_t)model;v[1]=(uint8_t)(model>>8);
 for(unsigned i=0;i<sizeof keys;i++){p32(nv+16+8*i,keys[i]|((unsigned)v[i]<<16));p32(nv+20+8*i,0);}}
static uint8_t cal[4096],nv[4096];static app_state s;
int main(void){
 uint8_t b[64];
 /* identity: an M288's NV journal (1200) never arms the M077 image; it still reports the servo's real model */
 fixture(cal,nv,1200);assert(!app_init(&s,cal,nv,true)&&(s.faults&APP_FAULT_CONFIG)&&!s.startup_valid);
 assert(app_model(&s)==1200&&app_read(&s,0,2,b)==0&&b[0]==0xb0&&b[1]==4);
 fixture(cal,nv,1190);assert(app_init(&s,cal,nv,true)&&s.startup_valid&&app_model(&s)==1190);
 assert(app_read(&s,0,8,b)==0&&b[0]==0xa6&&b[1]==4&&b[6]==APP_FIRMWARE_VERSION);
 /* unreadable NV: no model from it, the image's own model is reported */
 memset(nv,0,sizeof nv);assert(!app_init(&s,cal,nv,true)&&app_model(&s)==1190);
 /* power-up compensation: the M077 table (read 324), comp2 and hpd as derived in docs/M077_BETA.md */
 fixture(cal,nv,1190);assert(app_init(&s,cal,nv,true));
 assert(app_read(&s,324,40,b)==0);static const uint32_t want[9]={18394u,20054u,101u,469u,12194u,0u,0u,653u,0x102u};
 for(unsigned i=0;i<9;i++)assert(g32(b+4*i)==want[i]);
 assert(s.control.comp2.b_drop_q16==29&&s.control.comp2.vgate_q16==73164&&s.control.comp2.flags==1u&&s.control.hpd.kd_q16==0);
 /* jump gate 32 counts per tick (M288 12): 32 accepted, 33 rejected, from rest */
 sensing_adc adc={.voltage_decivolts=50,.temperature_c=25};uint32_t t=1000000u;uint16_t raw=1000;
 for(unsigned i=0;i<30;i++)(void)app_tick(&s,true,raw,&adc,true,t+=500u);
 assert(s.ready&&s.jump_rejects==0);
 raw=(uint16_t)(raw+32);(void)app_tick(&s,true,raw,&adc,true,t+=500u);assert(s.jump_rejects==0&&s.raw_angle==raw);
 (void)app_tick(&s,true,(uint16_t)(raw+33),&adc,true,t+=500u);assert(s.jump_rejects==1&&s.raw_angle==raw);
 /* motion block: M077 constants; speed clamp 64 rad/s (M288 32): est torque keeps changing between 40 and 60 rad/s */
 motion_in m={0,40<<16,0,480,5000,500,true};int32_t tm40,to40,tm60,to60;motion_torque(&m,&tm40,&to40);m.v_hat_q16=60<<16;motion_torque(&m,&tm60,&to60);
 assert(tm60<tm40&&to60<to40);
 m.v_hat_q16=0;m.pwm=480;motion_torque(&m,&tm40,&to40);   /* stall: K*V*d, motion-block K 0.0620 x r */
 double kv=tm40/16777216.0;assert(kv>0.080&&kv<0.087);   /* 0.0620*r = 0.01665 Nm/V x 5 V */
 m.v_hat_q16=70<<16;motion_torque(&m,&tm60,&to60);m.v_hat_q16=80<<16;motion_torque(&m,&tm40,&to40);assert(tm60==tm40);   /* clamped */
 puts("M077 beta: arms only on model 1190, reports the NV model, M077 comp tables, 32-count gate, motion constants + 64 rad/s clamp PASS");
 return 0;
}
