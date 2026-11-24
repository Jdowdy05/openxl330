/* Real platform RX DMA (channel2, circular, polled HT/TC/TE) under the shared MMIO
 * model. No RX data interrupt exists; these cases check configuration, ordering
 * across laps, exact fail-closed overflow/lost-lap detection, the CNDTR/flag race,
 * DDRE error discard, TE, TX ownership and loader quiesce. Model results only. */
#define main inherited_uart_main
#include "test_platform_uart.c"
#undef main
static void pump(unsigned n){for(unsigned i=0;i<n;i++)step();}
static bool start(void){reset_fixture();CHECK(platform_init());check_rx_contract=true;platform_irq_enable();return true;}
static void arrive(uint8_t v){receive_byte(v);pump(10);}
static unsigned pop_all(uint8_t *out,unsigned cap){unsigned n=0;while(n<cap&&platform_uart_pop(out+n))n++;return n;}
static bool config(void){
 CHECK(start());
 CHECK((reg(U)&(1u|RE|8u|(1u<<8)|FIFOEN))==(1u|RE|8u|(1u<<8)|FIFOEN));CHECK(!(reg(U)&(IDLE|RXIE|TXIE|TCIE)));
 CHECK((reg(U+8u)&((1u<<6)|(1u<<13)|1u|RXFTIE|TXFTCFG))==((1u<<6)|(1u<<13)|1u|TXFT_HALF));
 CHECK(reg(DMA+0x1cu)==0xa1u&&reg(DMA+0x20u)==1024u&&reg(DMA+0x24u)==(uint32_t)(U+0x24u)&&reg(DMAMUX+4u)==50u&&rx_dma_mem&&rx_dma_len==1024u);
 CHECK(reg(0xe000e100u)==((1u<<27)|(1u<<9)));CHECK(!(reg(0xe000e100u)&(1u<<10)));
 reset_fixture();rx_ccr_reject=true;CHECK(!platform_init());
 puts("RX DMA config: DMAR+DDRE, no RX data IRQ (IDLE/RXFNE/RXFT off), CCR2 MINC|CIRC|EN no IE, CNDTR1024, DMAMUX50, IRQ10 disabled, CCR2 readback fail-closed PASS");return true;}
static bool batch(void){
 CHECK(start());burst_remaining=256;burst_value=0;burst_due=now_us+1;pump(3000);
 static uint8_t out[1100];unsigned n=pop_all(out,sizeof out);CHECK(n==256);for(unsigned i=0;i<n;i++)CHECK(out[i]==(uint8_t)i);
 CHECK(rx_entries==0u&&rx_rdr_reads==0u&&rx_dma_moves==256u&&!platform_uart_faults()&&!disabled);
 printf("continuous256: %u USART IRQ entries, %u CPU RDR reads, %u DMA moves, exact order PASS\n",rx_entries,rx_rdr_reads,rx_dma_moves);return true;}
/* FW231: the main loop reads the backlog in batches (platform_uart_read, one mask per batch): same bytes, same
 * order, never more than asked, across ring laps; pop is the 1-byte case of it. */
static bool reads(void){
 CHECK(start());burst_remaining=300;burst_value=0;burst_due=now_us+1;pump(3500);
 static uint8_t out[64];unsigned got=0,calls=0,n;
 while((n=platform_uart_read(out,32u))!=0u){CHECK(n<=32u);for(unsigned i=0;i<n;i++)CHECK(out[i]==(uint8_t)(got+i));got+=n;calls++;}
 CHECK(got==300u&&calls==10u&&platform_uart_rx_backlog_max()==300u&&!platform_uart_faults());
 CHECK(start());unsigned sent=0;got=0;
 while(sent<5000u){unsigned k=(sent*37u)%61u+1u;for(unsigned i=0;i<k&&sent<5000u;i++)arrive((uint8_t)(sent++*11u));
  while((n=platform_uart_read(out,(sent%7u)*9u+1u))!=0u)for(unsigned i=0;i<n;i++){CHECK(out[i]==(uint8_t)(got*11u));got++;}}
 CHECK(got==5000u&&!platform_uart_faults()&&!disabled);
 puts("batched RX read: 300-byte backlog in 32-byte batches, 5000 bytes across laps in mixed batch sizes, exact order PASS");return true;}
static bool laps(void){
 static const unsigned chunks[]={1,7,64,255,256,300,511,512};
 for(unsigned c=0;c<sizeof chunks/sizeof*chunks;c++){
  CHECK(start());unsigned sent=0,got=0;uint8_t b;
  while(sent<5000u){for(unsigned i=0;i<chunks[c]&&sent<5000u;i++)arrive((uint8_t)(sent++*7u));while(platform_uart_pop(&b)){CHECK(b==(uint8_t)(got*7u));got++;}}
  CHECK(got==5000u&&!platform_uart_faults()&&!disabled&&rx_entries==0u);}
 puts("5000-byte streams, chunks 1..512 between pops (~5 laps): exact order, no fault, zero USART IRQ PASS");return true;}
static bool starved(void){
 static uint8_t out[1100];
 CHECK(start());for(unsigned i=0;i<1023u;i++)arrive((uint8_t)i);CHECK(pop_all(out,sizeof out)==1023u);for(unsigned i=0;i<1023u;i++)CHECK(out[i]==(uint8_t)i);CHECK(!platform_uart_faults());
 CHECK(start());for(unsigned i=0;i<600u;i++)arrive((uint8_t)i);CHECK(pop_all(out,10)==10u);for(unsigned i=600;i<1100u;i++)arrive((uint8_t)i);
 uint8_t b;CHECK(!platform_uart_pop(&b));CHECK((platform_uart_faults()&1u)&&!disabled&&!platform_uart_fatal());{uint32_t c[PLATFORM_UART_COUNTS];platform_uart_counts(c);CHECK(c[4]==1u);}
 for(unsigned i=0;i<20u;i++)arrive((uint8_t)(0x30u+i));for(unsigned i=0;i<20u;i++){CHECK(platform_uart_pop(&b)&&b==0x30u+i);}CHECK(!platform_uart_pop(&b));
 CHECK(start());for(unsigned i=0;i<1024u;i++)arrive((uint8_t)i);CHECK(!platform_uart_pop(&b));CHECK((platform_uart_faults()&1u)&&!disabled&&!platform_uart_fatal());
 for(unsigned i=0;i<700u;i++)arrive((uint8_t)(i^0x55u));for(unsigned i=0;i<700u;i++){CHECK(platform_uart_pop(&b)&&b==(uint8_t)(i^0x55u));}
 puts("starved1023 exact; true overwrite (600,pop10,+500) and lost lap (1024) recorded+counted (FW231: bridge untouched) with backlog discarded, then exact resync PASS");return true;}
static uint64_t seed;static unsigned rnd(unsigned n){seed^=seed<<13;seed^=seed>>7;seed^=seed<<17;return (unsigned)(seed%n);}
static bool fuzz(void){
 /* Reference: a pop must never return a byte other than the next sent byte unless a
  * fault was latched before that pop returned; a true overwrite of an unread byte
  * must latch a fault no later than the first pop after it. */
 unsigned runs=0,faulted=0,overwrites=0;
 for(unsigned r=0;r<300u;r++){seed=0x9e3779b97f4a7c15ull+r;CHECK(start());unsigned sent=0,got=0;bool ow=false;
  while(sent<4000u){unsigned burst=rnd(r%3?700u:300u);for(unsigned i=0;i<burst&&sent<4000u;i++){if(sent-got>=1024u)ow=true;arrive((uint8_t)(sent*13u+r));sent++;}
   unsigned pops=rnd(900u);for(unsigned i=0;i<pops;i++){uint8_t b;bool have=platform_uart_pop(&b);if(ow){CHECK(platform_uart_faults()&1u);}if(platform_uart_faults())break;if(!have)break;CHECK(b==(uint8_t)(got*13u+r));got++;}
   if(platform_uart_faults())break;}
  if(ow)overwrites++;if(platform_uart_faults()){faulted++;CHECK(!disabled&&!platform_uart_fatal());}runs++;}
 printf("fuzz %u schedules: %u with a true overwrite, %u fault-latched; no corrupted byte returned before a fault PASS\n",runs,overwrites,faulted);CHECK(faulted>=overwrites);return true;}
static bool race(void){
 static uint8_t out[1100];uint8_t b;
 CHECK(start());for(unsigned i=0;i<511u;i++)arrive((uint8_t)i);CHECK(pop_all(out,sizeof out)==511u);
 rx_cndtr_inject=1;CHECK(!platform_uart_pop(&b));CHECK(platform_uart_pop(&b)&&b==0xc0u);CHECK(!platform_uart_pop(&b));CHECK(!platform_uart_faults());
 CHECK(start());for(unsigned i=0;i<1023u;i++)arrive((uint8_t)i);CHECK(pop_all(out,1023u)==1023u);rx_cndtr_inject=2;
 CHECK(!platform_uart_pop(&b));CHECK(platform_uart_pop(&b)&&b==0xc0u);CHECK(platform_uart_pop(&b)&&b==0xc1u);CHECK(!platform_uart_faults());
 puts("boundary crossed between flag poll and CNDTR read (HT and TC): underestimate only, next pop exact, no fault PASS");return true;}
static bool errors(void){
 static uint8_t out[1100];unsigned cases=0;
 for(unsigned bit=0;bit<4u;bit++)for(unsigned pos=0;pos<8u;pos++){
  CHECK(start());
  const uint32_t loss_before=platform_uart_rx_loss();
  irq_mask=1;rx_dma_stall=true;for(unsigned i=0;i<8u;i++)receive_marked((uint8_t)(0x40u+i),i==pos?(uint8_t)(1u<<bit):0);rx_dma_stall=false;
  for(unsigned i=0;i<pos;i++)step();irq_mask=0;service_irq();pump(30);
  CHECK((platform_uart_faults()&(2u|((1u<<bit)<<8)))==(2u|((1u<<bit)<<8)));CHECK(!disabled&&!platform_uart_fatal()&&rx_flushes==1u);
  CHECK(platform_uart_rx_loss()==loss_before+1u);
  {uint32_t c[PLATFORM_UART_COUNTS];platform_uart_counts(c);CHECK(c[bit]==1u);}
  unsigned n=pop_all(out,sizeof out);CHECK(n==pos);for(unsigned i=0;i<n;i++)CHECK(out[i]==0x40u+i);
  arrive(0x99);uint8_t b;CHECK(platform_uart_pop(&b)&&b==0x99);
  CHECK(platform_uart_rx_loss()==loss_before+1u);cases++;}
 printf("per-position PE/FE/NE/ORE with DDRE: %u cases, prefix delivered, erroneous byte never delivered (flush before clear), later bytes resume PASS\n",cases);return true;}
static bool transfer_error(void){
 static uint8_t out[16];CHECK(start());rx_dma_te_after=5;for(unsigned i=0;i<8u;i++)arrive((uint8_t)i);
 CHECK(pop_all(out,sizeof out)==0u);CHECK((platform_uart_faults()&0x10u)&&platform_uart_fatal()&&disabled&&!(reg(DMA+0x1cu)&1u));
 puts("RX DMA transfer error: fault 0x10, channel disabled, backlog not delivered PASS");return true;}
static bool tx(void){
 static const uint8_t data[]={1,2,3,4};static uint8_t out[16];
 for(unsigned error=0;error<5;error++){CHECK(start());expected_n=sizeof data;inject_reenable=1;reenable_error=error?(uint8_t)(1u<<(error-1)):0;bool ok=platform_uart_transmit(data,sizeof data);pump(40);
  CHECK(!data_disable_order_error&&!error_enable_lost&&!rx_cfg_enabled);CHECK((reg(U)&(RE|IDLE|RXIE|(1u<<8)))==(RE|(1u<<8)));CHECK(!(reg(U+8u)&RXFTIE)&&(reg(U+8u)&(1u<<6)));
  if(error){CHECK(!ok&&platform_uart_faults()&&!disabled&&!platform_uart_fatal());CHECK(pop_all(out,sizeof out)==0u);}else{CHECK(ok);CHECK(pop_all(out,sizeof out)==1u&&out[0]==0xad);CHECK(!rx_count);}}
 CHECK(start());static const uint8_t hdr[]={0xff,0xff,0xfd};expected_n=sizeof hdr;CHECK(platform_uart_transmit(hdr,sizeof hdr));pump(200);uint8_t b;CHECK(peer_received&&platform_uart_pop(&b)&&b==0xff);CHECK(rx_entries==tc_irq_entries);
 for(unsigned mode=0;mode<4;mode++){CHECK(start());expected_n=sizeof data;if(mode==0)abort_before_arm=true;if(mode==1)dma_te_inject=true;if(mode==2)freeze_tx=true;if(mode==3)dma_raw_at_prearm=true;CHECK(!platform_uart_transmit(data,sizeof data));CHECK(!data_disable_order_error&&!error_enable_lost);CHECK((reg(U)&(RE|IDLE|RXIE|(1u<<8)))==(RE|(1u<<8)));CHECK(reg(DMA+0x1cu)&1u);}
 puts("TX: RE off during reply, re-enabled at TC with no RX IRQ, peer header via DMA, errors/abort keep RX DMA armed PASS");return true;}
static bool quiesce(void){
 CHECK(start());expect_loader=true;if(setjmp(loader_jump)==0)platform_enter_loader();expect_loader=false;
 CHECK(!(reg(U+8u)&(1u<<6))&&!(reg(R+0x38u)&1u)&&(reg(R+0x28u)&1u));CHECK(!(reg(0xe000e100u)&(1u<<10)));
 puts("loader quiesce: DMAR cleared, DMA1/DMAMUX held in reset with clock gated PASS");return true;}
/* FW231 (review): a group reply claims the line only when nothing is unread and no byte is arriving. */
static bool quiet(void){
 static const uint8_t data[]={1,2,3};uint8_t b;bool deferred=false;
 CHECK(start());arrive(0x5a);
 CHECK(!platform_uart_transmit_if_quiet(data,sizeof data,&deferred)&&deferred);
 CHECK(!gate&&(reg(U)&RE)&&!platform_uart_faults()&&!disabled);                 /* untouched: RE on, no DE, no fault */
 {uint32_t c[PLATFORM_UART_COUNTS];platform_uart_counts(c);CHECK(c[5]==0u);}
 CHECK(platform_uart_pop(&b)&&b==0x5a);                                          /* the byte is still delivered */
 set(U+0x1cu,reg(U+0x1cu)|(1u<<16));deferred=false;                              /* reception in progress (BUSY) */
 CHECK(!platform_uart_transmit_if_quiet(data,sizeof data,&deferred)&&deferred&&(reg(U)&RE));
 set(U+0x1cu,reg(U+0x1cu)&~(1u<<16));
 expected_n=sizeof data;deferred=true;
 CHECK(platform_uart_transmit_if_quiet(data,sizeof data,&deferred)&&!deferred);pump(40);
 CHECK(wire_n==sizeof data&&!memcmp(wire,data,sizeof data)&&(reg(U)&RE));
 puts("quiet transmit: unread RX byte or ISR.BUSY defers without touching RE/DE; quiet line transmits PASS");return true;}
int main(int argc,char**argv){if(argc!=2)return 2;
 static const struct{const char*n;bool(*f)(void);}t[]={{"config",config},{"quiet",quiet},{"batch",batch},{"reads",reads},{"laps",laps},{"starved",starved},{"fuzz",fuzz},{"race",race},{"errors",errors},{"te",transfer_error},{"tx",tx},{"quiesce",quiesce}};
 for(unsigned i=0;i<sizeof t/sizeof*t;i++)if(!strcmp(argv[1],t[i].n))return !t[i].f();return 2;}
