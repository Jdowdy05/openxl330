#include "protocol.h"
/* The production decoder now owns mutable frame storage. Preserve each test's
 * immutable/reusable wire fixture by modeling main's fresh receive buffer. */
static dxl_result fresh_process(const dxl_io *io,const uint8_t *wire,uint16_t n,uint8_t *out,uint16_t cap){uint8_t copy[DXL_MAX_WIRE];if(!wire)return dxl_process(io,0,n,out,cap);for(unsigned i=0;i<n&&i<sizeof copy;i++)copy[i]=wire[i];return dxl_process(io,copy,n,out,cap);}
#define dxl_process fresh_process
#include <assert.h>
#include <stdio.h>
static unsigned reads;
static uint8_t flash(void *ctx,uint32_t a,uint16_t n,uint8_t *out){(void)ctx;assert(a==0x0800e000u && n==4);for(int i=0;i<4;i++)out[i]=(uint8_t)(i+11);reads++;return 0;}
static unsigned frame(uint8_t *p,uint8_t id,uint8_t op,const uint8_t *arg,unsigned n){p[0]=p[1]=255;p[2]=253;p[3]=0;p[4]=id;p[5]=(uint8_t)(n+3);p[6]=0;p[7]=op;for(unsigned i=0;i<n;i++)p[8+i]=arg[i];uint16_t c=dxl_crc(p,8+n);p[8+n]=(uint8_t)c;p[9+n]=(uint8_t)(c>>8);return 10+n;}
int main(void){dxl_io io={.id=2,.firmware=233,.model=1200,.flash_read=flash};uint8_t in[32],out[100];unsigned n=frame(in,2,0xf9,0,0);dxl_result r=dxl_process(&io,in,n,out,sizeof out);assert(r.valid_for_us && r.enter_loader && r.reply_bytes==11 && out[4]==2);r=dxl_process(&io,in,n,out,0);assert(!r.enter_loader);n=frame(in,1,0xf9,0,0);r=dxl_process(&io,in,n,out,sizeof out);assert(!r.valid_for_us && !r.enter_loader);io.id=1;r=dxl_process(&io,in,n,out,sizeof out);assert(r.valid_for_us && r.enter_loader && out[4]==1);io.id=2;uint8_t arg[]={0x00,0xe0,0x00,0x08,4,0};n=frame(in,2,0xa0,arg,6);r=dxl_process(&io,in,n,out,sizeof out);assert(r.valid_for_us && r.reply_bytes==15 && reads==1 && out[9]==11 && out[12]==14);io.id=1;n=frame(in,1,0xa0,arg,6);r=dxl_process(&io,in,n,out,sizeof out);assert(r.valid_for_us && r.reply_bytes==15 && reads==2 && out[4]==1 && out[9]==11 && out[12]==14);puts("ID1/ID2 exact A0 reads and acknowledged F9 dispatch PASS");}
