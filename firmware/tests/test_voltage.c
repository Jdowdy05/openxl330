/* fast17 (owner 2026-11-24): "hard cap voltage limit to 8V and if its over 7V a warning is sent from the motors but this
 * warning can be toggled off in the control table by users"; "the opt-in voltage warning thing turns off the 8V voltage
 * limit as well as the 7V one". Boot: vmax 8.0 V, 1152 = 1. 848 accepts vmax 5.0..8.0 V. Status word 3 (address 76): bit 0
 * above 7.0 V (clears at <= 6.8 V, never latches), bit 1 protection off. 1152 = 0: no upper trip, no warning; 3.5 V stays. */
#include "app.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
static void p16(uint8_t*p,uint16_t v){p[0]=(uint8_t)v;p[1]=(uint8_t)(v>>8);}
static void p32(uint8_t*p,uint32_t v){p16(p,(uint16_t)v);p16(p+2,(uint16_t)(v>>16));}
static uint32_t g32(const uint8_t*p){return (uint32_t)p[0]|(uint32_t)p[1]<<8|(uint32_t)p[2]<<16|(uint32_t)p[3]<<24;}
static void fixture(uint8_t *cal,uint8_t *nv){memset(cal,255,4096);memset(nv,255,4096);p16(cal+0x800,2);p16(cal+0x810,1073);for(unsigned i=1;i<64;i++)p16(cal+0x810+8*(64-i),(uint16_t)(i*64));p32(nv,0xabab);p32(nv+4,0);p32(nv+8,0);p32(nv+12,0);
 const uint8_t keys[]={0,1,7,8,10,11,20,21,22,23,36,37,38,39},v[]={(uint8_t)APP_MODEL_NUMBER,(uint8_t)(APP_MODEL_NUMBER>>8),1,3,0,3,0,0,0,0,224,1,0,0};
 for(unsigned i=0;i<sizeof keys;i++){p32(nv+16+8*i,keys[i]|((unsigned)v[i]<<16));p32(nv+20+8*i,0);}}
static uint8_t cal[4096],nv[4096];static app_state s;static uint32_t t=1000000u;
static void tick(uint16_t dv){sensing_adc adc={.voltage_decivolts=dv,.temperature_c=25};(void)app_tick(&s,true,1000,&adc,true,t+=500u);}
static uint32_t warn(void){uint8_t b[4];assert(app_read(&s,76,4,b)==0);return g32(b);}
static uint8_t prot(void){uint8_t b[1];assert(app_read(&s,1152,1,b)==0);return b[0];}
int main(void){
 fixture(cal,nv);assert(app_init(&s,cal,nv,true));
 uint8_t b[16];assert(app_read(&s,848,16,b)==0&&g32(b+8)==80u&&prot()==1u);           /* boot: 8.0 V, protection on */
 for(int i=0;i<30;i++)tick(50);assert(!s.faults&&warn()==0u);
 tick(71);assert(warn()==1u&&!s.faults);tick(78);assert(warn()==1u&&!s.faults);       /* 7.1..7.8 V: warning, no fault */
 tick(69);assert(warn()==1u);tick(68);assert(warn()==0u);                              /* hysteresis: clears at <= 6.8 V */
 tick(80);assert(!s.faults);tick(81);assert(s.faults&APP_FAULT_VOLTAGE);               /* trip above vmax 8.0 V */
 /* 848: vmax 5.0..8.0 V; 8.1 V refused */
 uint8_t w[12];p32(w,1000);p32(w+4,65);p32(w+8,81);assert(app_write(&s,848,12,w)==4);p32(w+8,80);assert(app_write(&s,848,12,w)==0);
 p32(w+8,49);assert(app_write(&s,848,12,w)==4);
 /* 1152 = 0: no upper trip and no warning (also at 9 V), status bit 1 shows it; 3.5 V still trips; 1152 = 1 restores */
 fixture(cal,nv);assert(app_init(&s,cal,nv,true));for(int i=0;i<30;i++)tick(50);
 uint8_t off=0,on=1,bad=2;assert(app_write(&s,1152,1,&bad)==4&&app_write(&s,1152,2,b)==5&&app_write(&s,1152,1,&off)==0);
 assert(prot()==0u);tick(90);assert(!s.faults&&warn()==2u);tick(75);assert(warn()==2u);
 tick(34);assert(s.faults&APP_FAULT_VOLTAGE);
 assert(app_write(&s,1152,1,&on)==0&&prot()==1u);
 fixture(cal,nv);assert(app_init(&s,cal,nv,true));assert(prot()==1u);                  /* every boot: on */
 /* review P1-V1: ARMED, configured: a move and open-loop duty keep running at 7.1 / 7.5 / 8.0 V (1152 = 1) and at
  * 8.5 V with 1152 = 0 (no upper trip); the planner used to refuse every plan above 7.0 V (fault 0x400). */
 static const uint16_t dv[4]={71,75,80,85};
 for(unsigned k=0;k<4u;k++)for(unsigned mode=0;mode<2u;mode++){
  fixture(cal,nv);assert(app_init(&s,cal,nv,true));for(int i=0;i<30;i++)tick(50);
  if(dv[k]>80u)assert(app_write(&s,1152,1,&off)==0);
  uint8_t c[48]={0};p32(c,1);p32(c+4,229376u);p32(c+8,6554u);p32(c+12,845572u);p32(c+16,2u<<24);p32(c+20,737857423u);p32(c+24,69489982u);p32(c+28,3272216u);p32(c+36,16u<<20);p32(c+40,1);p32(c+44,JC_CONFIG_MAGIC);
  assert(app_write(&s,320,48,c)==0);for(int i=0;i<5;i++)tick(50);
  uint8_t g[16];p32(g,(uint32_t)(s.position_counts+200));p32(g+4,400);p32(g+8,0);p32(g+12,0);assert(app_write(&s,256,16,g)==0);
  assert(app_write(&s,64,1,&on)==0&&s.enabled);
  if(mode){uint8_t d[4];p32(d,240);assert(app_write(&s,260,4,d)==0);}
  unsigned duty=0;
  for(int i=0;i<200;i++){tick(dv[k]);if(s.commanded_pwm)duty++;(void)app_commit(&s,s.control.planned_pwm,true);}
  assert(s.enabled&&!s.faults&&s.control.rejections==0&&duty>=190u);
 }
 puts("voltage: boot 8.0 V + 1152 on, 7 V warning with hysteresis (no fault), trip > vmax, 848 range 5.0..8.0 V, 1152 off disables both, 3.5 V kept, on at boot; armed move and duty at 7.1/7.5/8.0 V and 8.5 V (1152 off) PASS");
 return 0;
}
