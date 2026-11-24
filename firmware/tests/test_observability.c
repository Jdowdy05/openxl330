#include "observability.h"
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
int main(void){
 obs_state s={0};uint8_t out[64];obs_publish(&s,7,0xfffffff0u,31,100,2);assert((s.flags&17u)==17u);assert(obs_consume(&s,0x00000020u));assert(s.post_sample==7 && s.post_latest==48 && !(s.flags&16u));
 obs_publish(&s,8,10,40,0,0);obs_publish(&s,9,20,41,0,0);assert(s.overwritten==1 && (s.flags&4u));assert(obs_consume(&s,600));assert(s.post_sample==9 && s.late==1);assert(!obs_consume(&s,601));
 s.overwritten=UINT32_MAX;obs_publish(&s,10,0,10,0,0);obs_publish(&s,11,0,10,0,0);assert(s.overwritten==UINT32_MAX);
 obs_serialize(&s,123,0,0,0,0,out);assert(!memcmp(out,"GOBS",4) && out[4]==1 && out[52]==123);
 uint32_t words[4]={0xa5a5a5a5u,0xa5a5a5a5u,1,0xa5a5a5a5u};obs_watermark w;obs_watermark_init(&w,words,4);assert(obs_watermark_scan(&w,1,1,true,true)==8 && (w.flags&8));words[0]=0;assert(obs_watermark_scan(&w,1,2,true,true)==8 && (w.flags&2));assert(w.sweeps==1 && (w.flags&16));
 for(unsigned depth=0;depth<=512;depth++){uint32_t memory[512];for(unsigned i=0;i<512;i++)memory[i]=i<depth?0xa5a5a5a5u:0;obs_watermark x;obs_watermark_init(&x,memory,512);assert(obs_watermark_scan(&x,0,0,true,true)==depth*4u);assert(x.sweeps==1 && !!(x.flags&2)==(depth==0));uint16_t saved=x.free_bytes;assert(obs_watermark_scan(&x,1,2,true,true)==saved && x.sweeps==1);}
 for(unsigned split=0;split<=512;split++){uint32_t memory[512];for(unsigned i=0;i<512;i++)memory[i]=0xa5a5a5a5u;obs_watermark x;obs_watermark_init(&x,memory,512);bool bottom=false;uint16_t candidate=obs_watermark_measure(&x,&bottom);memory[split==512?511:split]=0;obs_watermark_commit(&x,candidate,bottom,true,false);assert(x.free_bytes==2048 && x.sweeps==0 && (x.flags&16));assert(obs_watermark_scan(&x,0,0,true,true)==4u*(split==512?511:split));}
 obs_state z={0};z.late=UINT32_MAX;obs_publish(&z,5,100,1,0,0);assert(obs_consume(&z,600) && z.late==UINT32_MAX);obs_publish(&z,6,700,1,0,0);assert(z.flags&16);assert(obs_consume(&z,701) && z.post_sample==6);
 obs_watermark bad;obs_watermark_init(&bad,words,0);assert(bad.flags==4 && !bad.span_bytes);obs_state q={0};obs_publish(&q,1,0,0,0,UINT32_MAX);assert(q.uart_entries==UINT16_MAX && (q.flags&8));assert(!obs_consume(&q,0x80000000u) && !(q.flags&2) && q.overwritten==1 && (q.flags&4));obs_publish(&q,2,100,1,0,0);assert(obs_consume(&q,101) && q.overwritten==1 && (q.flags&4));q.overwritten=UINT32_MAX;obs_publish(&q,3,0,1,0,0);assert(!obs_consume(&q,0x80000000u) && q.overwritten==UINT32_MAX);
 puts("observability mailbox wrap/coalescing, serialized ABI and watermark discard PASS");
}
