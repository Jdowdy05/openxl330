/* Offline mathematical adaptation of BAM trajectory definitions.
 * Copyright 2025 Marc Duclusaud & Grégoire Passault; Apache-2.0.
 * https://github.com/Rhoban/bam/blob/b9e11118e17fe8ec803d430202f6fcf31d57901e/bam/trajectory.py
 * No hardware/arming operations. */
#include "bam_native.h"
#include <limits.h>
#include <stddef.h>
/* Nearest Q15 sine nodes, saturated to 32767 at unity. */
static const uint16_t sine[65]={
 0,804,1608,2411,3212,4011,4808,5602,
 6393,7180,7962,8740,9512,10279,11039,11793,
 12540,13279,14010,14733,15447,16151,16846,17531,
 18205,18868,19520,20160,20788,21403,22006,22595,
 23170,23732,24279,24812,25330,25833,26320,26791,
 27246,27684,28106,28511,28899,29269,29622,29957,
 30274,30572,30853,31114,31357,31581,31786,31972,
 32138,32286,32413,32522,32610,32679,32729,32758,
 32767};
/* Inputs are bounded well away from INT64_MIN. Signed nearest, ties away. */
static int32_t rs(int64_t x,unsigned n){uint64_t m=x<0?(uint64_t)-x:(uint64_t)x;int32_t y=(int32_t)((m+(UINT64_C(1)<<(n-1)))>>n);return x<0?-y:y;}
static int32_t trig(uint32_t turns){
 uint32_t phase=turns&0xffffffu,quadrant=phase>>22,offset=phase&0x3fffffu;
 if(quadrant&1u)offset=0x400000u-offset;
 uint32_t index=offset>>16,frac=offset&65535u;int32_t y;
 if(index==64)y=sine[64];else {int32_t d=(int32_t)sine[index+1]-(int32_t)sine[index];y=sine[index]+(int32_t)(((uint32_t)d*frac+32768u)>>16);}
 return (quadrant&2u)?-y:y;
}
#ifdef BAM_TESTING
int32_t bam_test_trig(uint32_t phase){return trig(phase);}
int32_t bam_test_round(int64_t value,unsigned shift){return rs(value,shift);}
#endif
static uint32_t phase_linear(uint32_t n,uint32_t mult){return (uint32_t)(((uint64_t)n*mult*UINT64_C(5734161139223)+UINT64_C(2147483648))>>32);}
static int32_t qmul(int32_t a,int32_t b){return rs((int64_t)a*b,16);}
bool bam_eval(uint8_t kind,uint32_t tick,int32_t center,bam_output *out){
 if(!out||kind>3||tick>12000)return false;
 int32_t q=0,v=0,a=0;bam_output o={0};o.active=tick<12000;o.terminal=tick==12000;o.enable=tick==12000?0:1;
 if(kind==0){
  uint32_t ph=(uint32_t)(((uint64_t)tick*tick*UINT64_C(2867080570)+UINT64_C(2147483648))>>32);
  int32_t s=trig(ph),c=trig(ph+0x400000u),t=(int32_t)((tick*8192u+125u)/250u),t2=qmul(t,t);
  q=2*s;v=rs((int64_t)2*t*c,15);a=4*c-rs((int64_t)4*t2*s,15);
 } else if(kind==1 || kind==2){
  if(kind==1 && tick>=4000){q=-102944;v=0;a=0;if(tick<12000)o.enable=2;}
  else {
   uint32_t n=kind==1?tick:(tick<6000?tick:tick-6000),duration=kind==1?4000u:6000u;
   int32_t x=(int32_t)((n*65536u+duration/2u)/duration),x2=qmul(x,x),x3=qmul(x2,x);
   int32_t h=3*x2-2*x3,hp=6*x-6*x2,hpp=6*65536-12*x;
   if(kind==1){q=-qmul(102944,h);v=-qmul(51472,hp);a=-qmul(25736,hpp);}
   else if(tick<6000){q=qmul(102944,h);v=qmul(34315,hp);a=qmul(11438,hpp);}
   else {q=102944-qmul(20589,h);v=-qmul(6863,hp);a=-qmul(2288,hpp);o.junction=tick==6000;}
  }
 }else{
  uint32_t p1=phase_linear(tick,1),p3=phase_linear(tick,3),p7=phase_linear(tick,7);
  int32_t s1=trig(p1),c1=trig(p1+0x400000u),s3=trig(p3),c3=trig(p3+0x400000u),s7=trig(p7),c7=trig(p7+0x400000u);
  q=rs((int64_t)102944*s1,15)+rs((int64_t)c3-c7,1);
  v=rs((int64_t)102944*c1,15)-rs((int64_t)3*s3,1)+rs((int64_t)7*s7,1);
  a=-rs((int64_t)102944*s1,15)-rs((int64_t)9*c3,1)+rs((int64_t)49*c7,1);
 }
 int64_t absolute=(int64_t)center+q;if(absolute<INT32_MIN||absolute>INT32_MAX)return false;
 o.q=(int32_t)absolute;o.v=v;o.a=a;*out=o;return true;
}
