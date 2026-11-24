#include "general_reference.h"
#include <assert.h>
#include <stdio.h>
int main(void){general_reference r;general_reference_init(&r,0);assert(general_reference_start(&r,65536,2000));
 assert(r.q==0 && r.v==0 && r.a==0);for(int i=0;i<1000;i++){int32_t before=r.q;assert(general_reference_tick(&r));assert(r.q>=before);}
 assert(r.q==32768 && r.v==122880 && r.a==0);for(int i=0;i<1000;i++){int32_t before=r.q;assert(general_reference_tick(&r));assert(r.q>=before);}
 assert(r.q==65536 && r.v==0 && r.a==0 && !r.active);
 assert(!general_reference_start(&r,0,0));puts("stationary quintic endpoints, midpoint and bounds PASS");}
