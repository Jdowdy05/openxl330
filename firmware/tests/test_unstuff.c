#include "protocol.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
static int calls;
static uint8_t wr(void*c,uint16_t a,uint16_t n,const uint8_t*p){(void)c;assert(a==320&&n==3&&p[0]==255&&p[1]==255&&p[2]==253);calls++;return 0;}
int main(void){uint8_t w[17]={255,255,253,0,1,10,0,3,64,1,255,255,253,253,0,0,0},out[20];/* body is 7 bytes including one stuffed byte */w[5]=9;unsigned n=16;uint16_t crc=dxl_crc(w,n-2);w[n-2]=(uint8_t)crc;w[n-1]=crc>>8;dxl_io io={.id=1,.write=wr};dxl_result r=dxl_process(&io,w,n,out,sizeof out);assert(r.valid_for_us&&calls==1);assert(w[7]==3&&w[10]==255&&w[12]==253);/* test second suffix shifts left */
 uint8_t x[18]={255,255,253,0,1,11,0,3,64,1,255,255,253,253,42,43,0,0};crc=dxl_crc(x,16);x[16]=crc;x[17]=crc>>8;io.write=0;(void)dxl_process(&io,x,18,out,sizeof out);assert(x[13]==42&&x[14]==43);
 puts("CRC-first leftward in-place decode PASS");}
