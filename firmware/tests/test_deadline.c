#include "deadline.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
int main(void){
 static deadline_budget d;memset(&d,0,sizeof d);
 /* 5% exactly (every 20th tick) for 10 s: never trips, window stays at 100 */
 for(unsigned i=0;i<20000;i++)assert(!deadline_tick(&d,i%20u==0u));
 assert(d.window==100&&d.window_max==100&&d.total==1000);
 /* a burst of 100 consecutive misses on top of a clean window: 100 -> no trip, 101st -> trip */
 memset(&d,0,sizeof d);for(unsigned i=0;i<2000;i++)assert(!deadline_tick(&d,false));
 for(unsigned i=0;i<100;i++)assert(!deadline_tick(&d,true));
 assert(deadline_tick(&d,true));
 /* misses age out after exactly 2000 ticks */
 memset(&d,0,sizeof d);for(unsigned i=0;i<100;i++)deadline_tick(&d,true);
 for(unsigned i=0;i<1900;i++)deadline_tick(&d,false);assert(d.window==100);
 deadline_tick(&d,false);assert(d.window==99);
 for(unsigned i=0;i<99;i++)deadline_tick(&d,false);assert(d.window==0);
 /* sliding (not fixed) window: 80 misses at the end of one second + 30 at the start of the next trips */
 memset(&d,0,sizeof d);for(unsigned i=0;i<1920;i++)deadline_tick(&d,false);
 bool trip=false;for(unsigned i=0;i<80;i++)trip|=deadline_tick(&d,true);
 for(unsigned i=0;i<30;i++)trip|=deadline_tick(&d,true);
 assert(trip);
 {static bucket_budget b;memset(&b,0,sizeof b);
  for(unsigned i=0;i<20000;i++)assert(!bucket_tick(&b,i%20u==0u));      /* steady 5% never trips */
  assert(b.window<=105);
  memset(&b,0,sizeof b);for(unsigned i=0;i<2000;i++)bucket_tick(&b,false);
  bool t=false;for(unsigned i=0;i<101;i++)t|=bucket_tick(&b,true);assert(t);   /* 101 in a row trips */
  memset(&b,0,sizeof b);for(unsigned i=0;i<100;i++)bucket_tick(&b,true);for(unsigned i=0;i<2000;i++)bucket_tick(&b,false);assert(b.window==0); /* ages out */
  assert(sizeof b<=32);}
 puts("deadline budget: 5% steady never trips, 101st miss in 1 s trips, exact 2000-tick ageing, sliding across second boundaries; bucketed variant PASS");
 return 0;
}
