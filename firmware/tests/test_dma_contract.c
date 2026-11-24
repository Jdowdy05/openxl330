#define main legacy_uart_test_main
#include "test_platform_uart.c"
#undef main
static bool test_dma_init(void){
 CHECK(setup());
 CHECK((reg(R+0x38u)&1u)!=0u);
 CHECK(reg(0x40020800u)==51u);
 CHECK((reg(0xe000e100u)&(1u<<9))!=0u);CHECK((reg(0xe000e408u)&0xff00u)==0x4000u);
 CHECK((reg(U+8u)&1u)!=0u && (reg(U)&(1u<<8)));
 puts("DMA clock, DMAMUX, IRQ9 and UART EIE initialized");return true;
}
static bool test_dma_wire(void){CHECK(setup());uint8_t before[32],after[32];platform_dma_read(before);static const uint8_t data[]={1,2,3,4,5,6,7,8,9};expected_n=sizeof data;CHECK(platform_uart_transmit(data,sizeof data));CHECK(wire_n==sizeof data && memcmp(wire,data,sizeof data)==0);CHECK(!dma_bad_config && dma_irq_entries==1 && dma_before_wire && tc_irq_entries==1 && rx_restore_us>=final_stop && !premature_rx && !gate);platform_dma_read(after);CHECK(word(after)==0x414d4447u && word(after+4)==1u && word(after+8)==word(before+8)+1u && word(after+12)==word(before+12)+1u && word(after+20)==word(before+20)+1u);return true;}
static bool test_sizes_and_stale(void){
 static uint8_t data[640];for(unsigned i=0;i<sizeof data;i++)data[i]=(uint8_t)(i^0xfdu);unsigned sizes[]={1,8,9,640};
 for(unsigned j=0;j<4;j++){CHECK(setup());expected_n=sizes[j];CHECK(platform_uart_transmit(data,(uint16_t)sizes[j]));CHECK(wire_n==sizes[j] && !memcmp(data,wire,sizes[j]));CHECK(!dma_bad_config && dma_irq_entries==1 && (sizes[j]<9 || dma_before_wire) && tc_irq_entries==1 && !gate && (reg(U)&RE));set(DMA,reg(DMA)|2u);DMA1_Channel1_IRQHandler();CHECK(!(reg(U)&TCIE) && !gate);}
 CHECK(setup());CHECK(!platform_uart_transmit(data,0));CHECK(!gate);return true;
}
static bool test_error_and_race(void){
 static uint8_t data[32];CHECK(setup());expected_n=sizeof data;dma_te_inject=true;dma_te_raw_after_flag=true;dma_raw_after_sample=true;CHECK(!platform_uart_transmit(data,sizeof data));CHECK((platform_uart_faults()&0x606u)==0x606u && !gate && !(reg(U)&TCIE));
 CHECK(setup());expected_n=sizeof data;dma_te_inject=true;CHECK(!platform_uart_transmit(data,sizeof data));CHECK(!gate && disabled && !(reg(U)&TCIE) && !(reg(U+8u)&(1u<<7)) && !(reg(DMA+8u)&1u));set(DMA,2u);DMA1_Channel1_IRQHandler();CHECK(!(reg(U)&TCIE) && !gate);
 CHECK(setup());expected_n=sizeof data;dma_te_at_tc=true;CHECK(!platform_uart_transmit(data,sizeof data));CHECK(!gate && disabled && !(reg(U)&TCIE));
 CHECK(setup());expected_n=sizeof data;dma_te_on_disable=true;CHECK(!platform_uart_transmit(data,sizeof data));CHECK(!gate && disabled && !(reg(U)&TCIE));
 CHECK(setup());set(DMA,8u);expected_n=sizeof data;CHECK(!platform_uart_transmit(data,sizeof data));CHECK(!gate && disabled && !(reg(U)&TCIE));
 CHECK(setup());expected_n=sizeof data;dma_raw_on_arm_read=true;CHECK(!platform_uart_transmit(data,sizeof data));CHECK((platform_uart_faults()&0x202u)==0x202u && !gate && !(reg(U)&TCIE));
 CHECK(setup());expected_n=sizeof data;dma_raw_at_prearm=true;CHECK(!platform_uart_transmit(data,sizeof data));CHECK((platform_uart_faults()&0x202u)==0x202u && !gate && !(reg(U)&TCIE));
 CHECK(setup());expected_n=sizeof data;dma_raw_error_on_clear=true;CHECK(!platform_uart_transmit(data,sizeof data));CHECK((platform_uart_faults()&0x202u)==0x202u && !gate && !(reg(U)&TCIE));
 CHECK(setup());expected_n=sizeof data;dma_raw_error_locked=true;CHECK(!platform_uart_transmit(data,sizeof data));CHECK((platform_uart_faults()&0x202u)==0x202u && !gate && !(reg(U)&TCIE));
 CHECK(setup());expected_n=sizeof data;dma_at_lock=true;CHECK(!platform_uart_transmit(data,sizeof data));CHECK(!gate && !disabled && !platform_uart_fatal() && !(reg(U)&TCIE) && !(reg(U+8u)&(1u<<7))); /* FW231: FE during TX = lost reply, no bridge cut */
 return true;
}
static bool test_stuck_noreturn(void){
 static uint8_t data[32];CHECK(setup());expected_n=sizeof data;dma_te_inject=true;dma_stuck=true;expect_loader=true;
 if(setjmp(loader_jump)==0){(void)platform_uart_transmit(data,sizeof data);return false;}
 expect_loader=false;CHECK(!gate && disabled && !(reg(R+0x38u)&1u) && (reg(R+0x28u)&1u));return true;
}
static bool test_clean_recovery(void){static const uint8_t data[]={9,8,7};CHECK(setup());expected_n=32;static uint8_t bad[32];dma_te_inject=true;CHECK(!platform_uart_transmit(bad,sizeof bad));for(unsigned i=0;i<40;i++)step();unsigned before=wire_n;expected_n=before+sizeof data;CHECK(platform_uart_faults()!=0);CHECK(platform_uart_transmit(data,sizeof data));CHECK(wire_n==expected_n && !memcmp(wire+before,data,sizeof data) && !gate && (reg(U)&RE));return true;}
static bool test_inherited_stuck(void){CHECK(setup());irq_mask=1;set(DMA+8u,1u);dma_stuck=true;CHECK(!platform_init());CHECK(!(reg(R+0x38u)&1u) && (reg(R+0x28u)&1u));return true;}
static bool test_loader_quiesce(void){CHECK(setup());set(DMA+8u,1u);set(U+8u,reg(U+8u)|(1u<<7));set(DMAMUX,51u);set(0xe000e200u,reg(0xe000e200u)|(1u<<9));expect_loader=true;
 if(setjmp(loader_jump)==0)platform_enter_loader();expect_loader=false;CHECK(!(reg(R+0x38u)&1u) && (reg(R+0x28u)&1u));CHECK(!(reg(U+8u)&(1u<<7)) && !(reg(0xe000e100u)&(1u<<9)) && !(reg(0xe000e200u)&(1u<<9)) && !gate);return true;}
int main(void){return test_clean_recovery() && test_inherited_stuck() && test_loader_quiesce() && test_dma_init() && test_dma_wire() && test_sizes_and_stale() && test_error_and_race() && test_stuck_noreturn()?0:1;}
