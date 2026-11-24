#include "general_reference.h"
#include <limits.h>
static int64_t divr(int64_t n,int64_t d){uint64_t u=(uint64_t)d;return n<0?-(int64_t)(((uint64_t)-n+u/2u)/u):(int64_t)(((uint64_t)n+u/2u)/u);}
static int64_t mulq30(int64_t a,int64_t b){return divr(a*b,INT64_C(1073741824));}
void general_reference_init(general_reference *r,int32_t q){*r=(general_reference){.q=q,.start=q};}
bool general_reference_start(general_reference *r,int32_t target,uint32_t ticks){
 if(!r || r->active || !ticks || ticks>3037000499u)return false;
 int64_t delta=(int64_t)target-r->q;if(delta<INT32_MIN || delta>INT32_MAX)return false;
 /* PD baseline: duration 1 tick is a true step (no profile, no accel cap). */
 if(ticks==1u){r->start=r->q=target;r->delta=0;r->vscale=r->ascale=0;r->elapsed=r->duration=1;r->v=r->a=0;r->active=false;return true;}
 int64_t vs=divr(delta*2000,ticks),as=divr(delta*4000000,(int64_t)ticks*ticks);
 int64_t av=vs<0?-vs:vs,aa=as<0?-as:as;
 /* PD baseline: only representation bounds (Q16 v/a peaks 1.875x / 5.78x scale fit int32). */
 if(av*2>INT32_MAX || aa*6>INT32_MAX)return false;
 r->start=r->q;r->delta=(int32_t)delta;r->vscale=(int32_t)vs;r->ascale=(int32_t)as;
 r->elapsed=0;r->duration=ticks;r->v=0;r->a=0;r->linear=false;r->active=delta!=0;return true;
}
bool general_reference_start_linear(general_reference *r,int32_t target,uint32_t ticks){
 if(!r || r->active || ticks<2u || ticks>3037000499u)return false;
 int64_t delta=(int64_t)target-r->q;if(delta<INT32_MIN || delta>INT32_MAX)return false;
 int64_t vs=divr(delta*2000,ticks);if(vs<INT32_MIN || vs>INT32_MAX)return false;
 r->start=r->q;r->delta=(int32_t)delta;r->vscale=(int32_t)vs;r->ascale=0;r->elapsed=0;r->duration=ticks;
 r->v=delta?(int32_t)vs:0;r->a=0;r->linear=true;r->active=delta!=0;return true;
}
bool general_reference_tick(general_reference *r){
 if(!r)return false;if(!r->active)return true;
 if(++r->elapsed>=r->duration){r->q=r->start+r->delta;r->v=0;r->a=0;r->active=false;return true;}
 if(r->linear){int64_t q=(int64_t)r->start+divr((int64_t)r->delta*r->elapsed,r->duration);if(q<INT32_MIN||q>INT32_MAX)return false;r->q=(int32_t)q;r->v=r->vscale;r->a=0;return true;}
 int64_t s=(int64_t)(((uint64_t)r->elapsed*UINT64_C(1073741824))/r->duration);
 int64_t s2=mulq30(s,s),s3=mulq30(s2,s),s4=mulq30(s3,s),s5=mulq30(s4,s);
 int64_t f=10*s3-15*s4+6*s5,fd=30*s2-60*s3+30*s4,fdd=60*s-180*s2+120*s3;
 int64_t q=(int64_t)r->start+divr((int64_t)r->delta*f,INT64_C(1073741824));
 int64_t v=divr((int64_t)r->vscale*fd,INT64_C(1073741824)),a=divr((int64_t)r->ascale*fdd,INT64_C(1073741824));
 if(q<INT32_MIN || q>INT32_MAX || v<INT32_MIN || v>INT32_MAX || a<INT32_MIN || a>INT32_MAX)return false;
 r->q=(int32_t)q;r->v=(int32_t)v;r->a=(int32_t)a;return true;
}
