/* fast17 sensor reacquire (design 2026-11-20): one bound for gate and tracker, two-sample confirmation
 * after >= 4 bad samples. Native, ASan/UBSan, the app layer with the identity-map fixture of test_pd.c. */
#include "app.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
#define APP_REACQ_CONFIRM_TEST 4   /* app.c APP_REACQ_CONFIRM */
#if APP_GEAR == 77
#define GATE 32   /* app.c APP_JUMP_COUNTS */
#else
#define GATE 12
#endif
static void p16(uint8_t*p,uint16_t v){p[0]=(uint8_t)v;p[1]=(uint8_t)(v>>8);}
static void p32(uint8_t*p,uint32_t v){p16(p,(uint16_t)v);p16(p+2,(uint16_t)(v>>16));}
static void fixture(uint8_t *cal,uint8_t *nv){memset(cal,255,4096);memset(nv,255,4096);p16(cal+0x800,2);p16(cal+0x810,1073);for(unsigned i=1;i<64;i++)p16(cal+0x810+8*(64-i),(uint16_t)(i*64));p32(nv,0xabab);p32(nv+4,0);p32(nv+8,0);p32(nv+12,0);
 const uint8_t keys[]={0,1,7,8,10,11,20,21,22,23,36,37,38,39},v[]={(uint8_t)APP_MODEL_NUMBER,(uint8_t)(APP_MODEL_NUMBER>>8),1,3,0,3,0,0,0,0,224,1,0,0};
 for(unsigned i=0;i<sizeof keys;i++){p32(nv+16+8*i,keys[i]|((unsigned)v[i]<<16));p32(nv+20+8*i,0);}}
static uint8_t cal[4096],nv[4096];static app_state s;static uint32_t t;static int32_t raw_q;   /* raw angle x 16 */
static sensing_adc adc={.voltage_decivolts=50,.temperature_c=25};
static void tick(bool ok,int32_t speed_q){raw_q+=speed_q;uint16_t raw=(uint16_t)((raw_q>>4)&4095);(void)app_tick(&s,ok,raw,&adc,true,t+=500u);}
static void glitch(uint16_t raw){(void)app_tick(&s,true,raw,&adc,true,t+=500u);}
static void boot(void){fixture(cal,nv);assert(app_init(&s,cal,nv,true));t=1000000u;raw_q=1000<<4;for(int i=0;i<40;i++)tick(true,0);assert(s.ready);}
int main(void){
 const int32_t v=(int32_t)(GATE*16/4);          /* a quarter of the gate per tick (M288: 3 counts/tick = 9.2 rad/s) */
 /* 42 dropped samples at speed, then good ones: no SENSOR fault (fast16 faulted: 43 x 3 = 129 > 128), one reacquisition */
 boot();for(int i=0;i<50;i++)tick(true,v);int32_t p0=s.position_counts;
 for(int i=0;i<42;i++)tick(false,v);
 tick(true,v);assert(!s.faults&&s.have_cand&&s.reacquisitions==0);          /* candidate: counted bad */
 tick(true,v);assert(!s.faults&&!s.have_cand&&s.reacquisitions==1&&s.bad_streak==0);
 int32_t moved=s.position_counts-p0;if(moved<0)moved=-moved;assert(moved>=(44*v)/16-2&&moved<=(44*v)/16+2);
 for(int i=0;i<50;i++)tick(true,v);assert(!s.faults);
 /* a glitch right after a dropout lands inside the wide gate: it becomes the candidate, the next true sample disagrees
  * with it -> rejected again (new candidate), the one after confirms: never latched as the position */
 for(int i=0;i<20;i++)tick(false,v);
 uint16_t truth=(uint16_t)(((raw_q+v)>>4)&4095);glitch((uint16_t)((truth+150)&4095));raw_q+=v;assert(s.have_cand);
 tick(true,v);assert(s.have_cand&&s.reacq_rejects==1&&s.reacquisitions==1);
 tick(true,v);assert(!s.have_cand&&s.reacquisitions==2&&!s.faults);
 /* short dropouts (< 4) never use a candidate */
 for(int i=0;i<3;i++)tick(false,v);tick(true,v);assert(!s.have_cand&&s.reacquisitions==2&&!s.faults);
 /* 101 dropped samples still latch SENSOR (5 % of 1 s) */
 boot();for(int i=0;i<10;i++)tick(true,v);for(int i=0;i<101;i++)tick(false,v);tick(true,v);tick(true,v);assert(s.faults&APP_FAULT_SENSOR);
 /* review P1-R1: a CONFIGURED servo (estimator running, coasting through the dropout, frozen after 20 ticks), OFF and
  * armed, at the gear's no-load speed (M288 ~3.6, M077 ~13.5 counts/tick) and the longest dropout the 5 % budget allows:
  * no estimator rejection, no SENSOR fault, and v_hat right after the reacquisition is the true speed. */
 for(int armed=0;armed<2;armed++){
  const int32_t vnl=GATE==32?216:58;                         /* raw counts x 16 per tick: 13.5 / 3.625 */
  boot();uint8_t c[48]={0};p32(c,1);p32(c+4,(uint32_t)(GATE==32?16581:229376));p32(c+8,(uint32_t)(GATE==32?472:6554));
  p32(c+12,GATE==32?227225u:845572u);p32(c+16,2u<<24);p32(c+20,737857423u);p32(c+24,69489982u);p32(c+28,3272216u);p32(c+32,0);p32(c+36,16u<<20);p32(c+40,1);p32(c+44,JC_CONFIG_MAGIC);
  assert(app_write(&s,320,48,c)==0);for(int i=0;i<10;i++)tick(true,0);
  if(armed){uint8_t g[16];p32(g,(uint32_t)s.position_counts);p32(g+4,1);p32(g+8,0);p32(g+12,0);uint8_t one=1;assert(app_write(&s,256,16,g)==0&&app_write(&s,64,1,&one)==0&&s.enabled);}
  for(int i=0;i<200;i++)tick(true,vnl);
  for(int drop=10;drop<=95;drop+=17){
   uint32_t rej=s.control.rejections,acq=s.reacquisitions;
   for(int i=0;i<drop;i++)tick(false,vnl);
   tick(true,vnl);tick(true,vnl);
   assert(!(s.faults&APP_FAULT_SENSOR)&&s.control.rejections==rej);
   if(drop>=(int)APP_REACQ_CONFIRM_TEST)assert(s.reacquisitions==acq+1u);
   double v=s.control.v_hat_q16/65536.0,want=-(vnl/16.0)*2000.0*(411775.0/268435456.0);   /* the fixture map counts down */
   assert(v>want*1.15-0.5&&v<want*0.85+0.5);
   for(int i=0;i<2200;i++)tick(true,vnl);                    /* let the 1 s budget window clear */
  }
  assert(!s.faults);
 }
 /* read 856: 24 B as before, 32 B adds reacquisitions and rejected candidates */
 uint8_t b[32];assert(app_read(&s,856,24,b)==0&&app_read(&s,856,32,b)==0&&app_read(&s,856,28,b)!=0);
 printf("reacquire: 42-sample dropout at speed recovers, glitch after a dropout never latched, short dropouts unchanged, 101 still faults, read 856/32 PASS\n");
 return 0;
}
