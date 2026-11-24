/* fast17: firmware/nmi.c (the NMI vector) on qemu -M microbit with a stub platform_nmi_idlog_ecc(). 1) The stub says
 * "an ECC error in the log page": the NMI returns, the interrupted code goes on with its registers intact. 2) The stub
 * says "not ours": Default_Handler runs with MSP = the exception frame and LR = EXC_RETURN, as the fast16 vector did. */
#include <stdbool.h>
#include <stdint.h>
extern uint32_t _sidata,_sdata,_edata,_sbss,_ebss,_estack;
void NMI_Handler(void);
static int sh(int op,void *arg){register int r0 __asm__("r0")=op;register void *r1 __asm__("r1")=arg;__asm__ volatile("bkpt 0xab":"+r"(r0):"r"(r1):"memory");return r0;}
static void out(const char *s){sh(4,(void*)s);}
static void quit(int code){uint32_t a[2]={0x20026u,(uint32_t)code};sh(0x20,a);for(;;){}}
static volatile int answer, calls;
bool platform_nmi_idlog_ecc(void){calls++;return answer!=0;}
static volatile uint32_t frame_sp, got_sp, got_lr, got_pc;
void Default_Handler_c(uint32_t sp,uint32_t lr){got_sp=sp;got_lr=lr;got_pc=((volatile uint32_t*)sp)[6];
 if(calls==2&&got_lr==0xfffffff9u&&got_sp==frame_sp&&got_pc>=0x100u){out("PASS default path: MSP = exception frame, LR = EXC_RETURN\n");quit(0);}
 out("FAIL default path\n");quit(3);}
__attribute__((naked)) void Default_Handler(void){__asm__ volatile("mrs r0, msp\n\tmov r1, lr\n\tb Default_Handler_c");}
static void pend_nmi(void){*(volatile uint32_t*)0xe000ed04u=1u<<31;__asm__ volatile("dsb\n\tisb":::"memory");}
int main(void){
 register uint32_t a __asm__("r4")=0x44444444u,b __asm__("r5")=0x55555555u,c __asm__("r6")=0x66666666u,d __asm__("r7")=0x77777777u;
 answer=1;pend_nmi();
 __asm__ volatile("":"+r"(a),"+r"(b),"+r"(c),"+r"(d));
 if(calls!=1||a!=0x44444444u||b!=0x55555555u||c!=0x66666666u||d!=0x77777777u){out("FAIL handled path\n");quit(2);}
 out("PASS handled path: NMI returned, r4-r7 intact\n");
 answer=0;uint32_t sp;__asm__ volatile("mov %0, sp":"=r"(sp));frame_sp=sp-32u;   /* 8-byte aligned here: no pad word */
 pend_nmi();
 out("FAIL: returned from an unhandled NMI\n");quit(4);return 0;
}
void Reset_Handler(void){uint32_t *s=&_sidata,*d=&_sdata;while(d<&_edata)*d++=*s++;for(d=&_sbss;d<&_ebss;)*d++=0;quit(main());}
__attribute__((section(".vectors"),used)) const void *vectors[16]={&_estack,Reset_Handler,NMI_Handler,Default_Handler};
