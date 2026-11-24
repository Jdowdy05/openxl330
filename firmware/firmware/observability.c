#include "observability.h"
#include <limits.h>
obs_state observation;
obs_watermark observation_stack;
volatile uint32_t observation_activity,observation_uart_cycles,observation_uart_entries;
static void p16(uint8_t *p,uint16_t v){p[0]=(uint8_t)v;p[1]=(uint8_t)(v>>8);}
static void p32(uint8_t *p,uint32_t v){p16(p,(uint16_t)v);p16(p+2,(uint16_t)(v>>16));}
void obs_publish(obs_state *s,uint32_t sample,uint32_t origin,uint32_t elapsed,uint32_t cycles,uint32_t entries){
 if(s->flags&16u){if(s->overwritten!=UINT32_MAX)s->overwritten++;s->flags|=4u;}
 s->end_sample=sample;s->origin=origin;s->end_latest=elapsed;if(elapsed>s->end_max)s->end_max=elapsed;s->uart_latest=cycles;if(cycles>s->uart_max)s->uart_max=cycles;
 if(entries>UINT16_MAX){entries=UINT16_MAX;s->flags|=8u;}s->uart_entries=(uint16_t)entries;if(entries>s->uart_entries_max)s->uart_entries_max=(uint16_t)entries;s->flags|=17u;
}
bool obs_consume(obs_state *s,uint32_t now){if(!(s->flags&16u))return false;s->flags&=(uint16_t)~16u;uint32_t d=now-s->origin;if(d>=0x80000000u){s->flags&=(uint16_t)~2u;if(s->overwritten!=UINT32_MAX)s->overwritten++;s->flags|=4u;return false;}s->post_sample=s->end_sample;s->post_latest=d;if(d>s->post_max)s->post_max=d;if(d>=500 && s->late!=UINT32_MAX)s->late++;s->flags|=2u;return true;}
void obs_serialize(const obs_state*s,uint32_t mask,uint16_t free_b,uint16_t span,uint16_t stackflags,uint16_t scans,uint8_t *o){p32(o,0x53424f47u);p16(o+4,1);p16(o+6,s->flags);p32(o+8,s->end_sample);p32(o+12,s->end_latest);p32(o+16,s->end_max);p32(o+20,s->post_sample);p32(o+24,s->post_latest);p32(o+28,s->post_max);p32(o+32,s->overwritten);p32(o+36,s->late);p32(o+40,s->uart_latest);p32(o+44,s->uart_max);p16(o+48,s->uart_entries);p16(o+50,s->uart_entries_max);p32(o+52,mask);p16(o+56,free_b);p16(o+58,span);p16(o+60,stackflags);p16(o+62,scans);}
void obs_watermark_init(obs_watermark*w,volatile uint32_t*base,uint16_t words){w->base=base;w->sweeps=0;if(!base||!words||words>512){w->free_bytes=0;w->span_bytes=0;w->flags=4;return;}w->span_bytes=(uint16_t)(words*4u);w->free_bytes=w->span_bytes;w->flags=1;}
uint16_t obs_watermark_measure(const obs_watermark*w,bool*bottom){uint16_t n=w->span_bytes/4u,first=n;*bottom=false;for(uint16_t i=0;i<n;i++){if(w->base[i]!=0xa5a5a5a5u){if(i==0)*bottom=true;if(first==n)first=i;}}return (uint16_t)(first*4u);}
void obs_watermark_commit(obs_watermark*w,uint16_t free_b,bool bottom,bool eligible,bool same){if(bottom)w->flags|=2;if(!eligible){w->flags=(w->flags&~16u)|32u;return;}if(!same){w->flags=(w->flags&~32u)|16u;return;}if(free_b<w->free_bytes)w->free_bytes=free_b;w->flags=(w->flags&~48u)|8u;w->sweeps++;}
uint16_t obs_watermark_scan(obs_watermark*w,uint32_t a,uint32_t b,bool eligible,bool after){if(!(w->flags&1u)||!eligible){obs_watermark_commit(w,w->free_bytes,false,false,false);return w->free_bytes;}bool bottom=false;uint16_t f=obs_watermark_measure(w,&bottom);obs_watermark_commit(w,f,bottom,after,a==b);return w->free_bytes;}
