/* qemu TCG plugin: Cortex-M0+ cycle model between two marker addresses (begin=, end=).
 * Core cycles per ARM DDI 0484 (M0+ TRM table 3-1, single-cycle multiplier); plus a flash model: +ws cycles for every
 * taken control transfer and every literal-pool load (STM32G0 at 64 MHz: 2 wait states). Reports per-region counts. */
#include <qemu-plugin.h>
#include <glib.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
QEMU_PLUGIN_EXPORT int qemu_plugin_version = QEMU_PLUGIN_VERSION;
typedef struct {uint64_t pc;uint8_t size,base,cls;} rec;   /* cls: 0 plain, 1 cond branch, 2 always-taken transfer, 3 literal load */
static uint64_t begin_pc,end_pc;static unsigned ws=2;
static bool active;static const rec *prev;
static uint64_t insns,core,flash;            /* current region */
static GArray *r_insn,*r_core,*r_flash;
static unsigned pop8(unsigned v){unsigned n=0;while(v){n+=v&1u;v>>=1;}return n;}
static void classify(rec *r,const uint8_t *b,size_t n){
 uint16_t h=(uint16_t)(b[0]|b[1]<<8);r->base=1;r->cls=0;
 if(n==4){uint16_t l=(uint16_t)(b[2]|b[3]<<8);
  if((h&0xf800)==0xf000&&(l&0xd000)==0xd000){r->base=3;r->cls=2;}      /* BL */
  else r->base=3;                                                          /* MSR/MRS/DSB/DMB/ISB */
  return;}
 if((h&0xff00)==0x4700){r->base=2;r->cls=2;}                               /* BX/BLX */
 else if((h&0xfe00)==0xb400)r->base=(uint8_t)(1+pop8(h&0x1ffu));            /* PUSH */
 else if((h&0xfe00)==0xbc00){unsigned k=pop8(h&0xffu);if(h&0x100u){r->base=(uint8_t)(3+k+1);r->cls=2;}else r->base=(uint8_t)(1+k);}  /* POP (pc: 3+N) */
 else if((h&0xf000)==0xc000)r->base=(uint8_t)(1+pop8(h&0xffu));            /* LDM/STM */
 else if((h&0xf000)==0xd000&&((h>>8)&0xf)<0xe){r->base=1;r->cls=1;}         /* B<cond>: +1 when taken */
 else if((h&0xf800)==0xe000){r->base=2;r->cls=2;}                          /* B */
 else if((h&0xf800)==0x4800){r->base=2;r->cls=3;}                          /* LDR literal */
 else if((h&0xf000)==0x5000||(h&0xe000)==0x6000||(h&0xf000)==0x8000||(h&0xf000)==0x9000)r->base=2;   /* LDR/STR */
 else if((h&0xfc00)==0x4400){unsigned rd=((h>>4)&8u)|(h&7u);unsigned op=(h>>8)&3u;if(rd==15u&&op!=1u){r->base=2;r->cls=2;}}  /* ADD/MOV pc */
}
static void exec_cb(unsigned int vcpu,void *ud){(void)vcpu;const rec *r=ud;
 if(r->pc==begin_pc){active=true;insns=core=flash=0;prev=NULL;}
 if(active){
  if(prev&&prev->cls==1&&r->pc!=prev->pc+prev->size){core+=1;flash+=ws;}   /* conditional branch was taken */
  insns++;core+=r->base;if(r->cls==2||r->cls==3)flash+=ws;prev=r;
  if(r->pc==end_pc){active=false;g_array_append_val(r_insn,insns);g_array_append_val(r_core,core);g_array_append_val(r_flash,flash);}
 }
}
static void tb_cb(struct qemu_plugin_tb *tb,void *ud0){(void)ud0;size_t n=qemu_plugin_tb_n_insns(tb);
 for(size_t i=0;i<n;i++){struct qemu_plugin_insn *in=qemu_plugin_tb_get_insn(tb,i);rec *r=g_new0(rec,1);uint8_t b[4]={0};
  size_t sz=qemu_plugin_insn_size(in);qemu_plugin_insn_data(in,b,sz>4?4:sz);r->pc=qemu_plugin_insn_vaddr(in);r->size=(uint8_t)sz;classify(r,b,sz);
  qemu_plugin_register_vcpu_insn_exec_cb(in,exec_cb,QEMU_PLUGIN_CB_NO_REGS,r);}}
static int cmp64(const void *a,const void *b){uint64_t x=*(const uint64_t*)a,y=*(const uint64_t*)b;return x<y?-1:x>y;}
static void stat(GString *o,const char *name,GArray *a){guint n=a->len;if(!n){g_string_append_printf(o,"%s n=0\n",name);return;}
 uint64_t *v=g_memdup2(a->data,n*sizeof(uint64_t));qsort(v,n,sizeof *v,cmp64);uint64_t sum=0;for(guint i=0;i<n;i++)sum+=v[i];
 g_string_append_printf(o,"%s n=%u mean=%.1f p50=%llu p99=%llu p999=%llu max=%llu\n",name,n,(double)sum/n,(unsigned long long)v[n/2],(unsigned long long)v[(n*99)/100],(unsigned long long)v[(n*999)/1000],(unsigned long long)v[n-1]);g_free(v);}
static void atexit_cb(void *p){(void)p;GString *o=g_string_new("");
 stat(o,"insns",r_insn);stat(o,"core_cycles",r_core);
 GArray *tot=g_array_new(false,false,sizeof(uint64_t));for(guint i=0;i<r_core->len;i++){uint64_t t=g_array_index(r_core,uint64_t,i)+g_array_index(r_flash,uint64_t,i);g_array_append_val(tot,t);}
 stat(o,"cycles_ws",tot);qemu_plugin_outs(o->str);g_string_free(o,true);}
QEMU_PLUGIN_EXPORT int qemu_plugin_install(qemu_plugin_id_t id,const qemu_info_t *info,int argc,char **argv){(void)info;
 for(int i=0;i<argc;i++){if(!strncmp(argv[i],"begin=",6))begin_pc=strtoull(argv[i]+6,0,0);else if(!strncmp(argv[i],"end=",4))end_pc=strtoull(argv[i]+4,0,0);else if(!strncmp(argv[i],"ws=",3))ws=(unsigned)strtoul(argv[i]+3,0,0);}
 r_insn=g_array_new(false,false,sizeof(uint64_t));r_core=g_array_new(false,false,sizeof(uint64_t));r_flash=g_array_new(false,false,sizeof(uint64_t));
 qemu_plugin_register_vcpu_tb_trans_cb(id,tb_cb,NULL);qemu_plugin_register_atexit_cb(id,atexit_cb,NULL);return 0;}
