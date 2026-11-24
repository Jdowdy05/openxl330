#include "hw_io.h"
#include <stdint.h>
/* flash 0x08000000..0x0800FFFF reads a fixed pattern (fast12 loader CRC and A0 range tests); everything else 0 */
uint32_t hw_read32(uintptr_t a){return a>=0x08000000u&&a<0x08010000u?(uint32_t)((a-0x08000000u)*2654435761u+0x9e3779b9u):0u;}
void hw_write32(uintptr_t a,uint32_t v){(void)a;(void)v;}
uint32_t platform_irq_max_us(void){return 0;}

void platform_dma_read(uint8_t out[32]){for(unsigned i=0;i<32;i++)out[i]=0;}
