/* tests/golden_qemu runtime for tests/golden_control.c on qemu -M microbit: vectors, reset, semihosting output and exit,
 * and the two cycle-count markers the m0cycles plugin watches. */
#include <stdint.h>
extern int golden_main(void);
extern uint32_t _sidata,_sdata,_edata,_sbss,_ebss,_estack;
static int sh(int op,void *arg){register int r0 __asm__("r0")=op;register void *r1 __asm__("r1")=arg;__asm__ volatile("bkpt 0xab":"+r"(r0):"r"(r1):"memory");return r0;}
void golden_out(const char *s){sh(4,(void*)s);}
__attribute__((noinline)) void golden_mark_begin(void){__asm__ volatile("":::"memory");}
__attribute__((noinline)) void golden_mark_end(void){__asm__ volatile("":::"memory");}
void golden_mark(unsigned what){if(what==1u)golden_mark_begin();else golden_mark_end();}
static void rt_exit(int code){uint32_t a[2]={0x20026u,(uint32_t)code};sh(0x20,a);for(;;){}}
void Reset_Handler(void){uint32_t *s=&_sidata,*d=&_sdata;while(d<&_edata)*d++=*s++;for(d=&_sbss;d<&_ebss;)*d++=0;rt_exit(golden_main());}
void Fault_Handler(void){golden_out("FAULT\n");rt_exit(99);}
__attribute__((section(".vectors"),used)) const void *vectors[16]={&_estack,Reset_Handler,Fault_Handler,Fault_Handler};
