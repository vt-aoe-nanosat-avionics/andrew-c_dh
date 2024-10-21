// flash_write_demo.c
// Tests Tartan Artibeus EXPT board flash write demonstration
//
// Written by Bradley Denby
// Other contributors: None
//
// See the top-level LICENSE file for the license.

// Standard library
#include <stddef.h>
#include <stdint.h>

// libopencm3 library
#include <libopencm3/stm32/rcc.h>    // reset and clock control functions
#include <libopencm3/stm32/gpio.h>   // GPIO functions
#include <libopencm3/stm32/usart.h> // usart_send, USART1
//#include <libopencm3/stm32/quadspi.h> // QUADSPI functions

#include <IS25LP128F.h>              // IS25LP128F flash memory macros
#include <quadspi_common_v1.h>       // QUADSPI functions

// Main
int main(void) {
  // clock setup
  rcc_osc_on(RCC_HSI16);   // high-speed (16 MHz) internal RC oscillator
  //flash_prefetch_enable(); // buffer used for instruction fetches
  //flash_set_ws(4);         // number of wait states should be matched to clock
  //flash_dcache_enable();   // enable data cache
  //flash_icache_enable();   // enable instruction cache
    //// 16MHz/4 = 4MHz; 4*40=160MHz VCO; 80MHz main PLL
  rcc_set_main_pll(RCC_PLLCFGR_PLLSRC_HSI16,4,40,0,0,RCC_PLLCFGR_PLLR_DIV2);
  rcc_osc_on(RCC_PLL);     // internal phase-locked loop
    //// At this point, either rcc_wait_for_osc_ready() or do other things
	rcc_periph_clock_enable(RCC_GPIOA);  // enable clock for port with USART pins
	rcc_periph_clock_enable(RCC_GPIOC);  // enable clock for port with LED pins
  rcc_periph_clock_enable(RCC_USART1); // enable clock for USART1 peripheral
  // USART setup
    //// Setup GPIO pins for TX and RX
  gpio_mode_setup(GPIOA,GPIO_MODE_AF,GPIO_PUPD_NONE,GPIO9|GPIO10);
  gpio_set_af(GPIOA,GPIO_AF7,GPIO9);  // USART1_TX and alternate function 7
  gpio_set_af(GPIOA,GPIO_AF7,GPIO10); // USART1_RX and alternate function 7
	usart_set_baudrate(USART1,38400);
	usart_set_databits(USART1,8);
	usart_set_stopbits(USART1,USART_STOPBITS_1);
	usart_set_mode(USART1,USART_MODE_TX);
	usart_set_parity(USART1,USART_PARITY_NONE);
	usart_set_flow_control(USART1,USART_FLOWCONTROL_NONE);
	usart_enable(USART1);

  // QUADSPI setup
  rcc_periph_clock_enable(RCC_QSPI);
  quadspi_disable();
  quadspi_set_flash_size(23); // 128 Mbit = 16 Mbyte = 2^(n+1) // n = 23
  quadspi_set_high_time(6);   // 1/2 clock cycle
  quadspi_set_prescaler(0);   // 1:1 prescaler
  quadspi_enable_sample_shift();
	quadspi_clear_flag(QUADSPI_FCR_CTOF | QUADSPI_FCR_CSMF | QUADSPI_FCR_CTCF | QUADSPI_FCR_CTEF);
  quadspi_enable();

  quadspi_send_instruction(IS25LP128F_CMD_RESET_ENABLE, QUADSPI_CCR_MODE_1LINE);
  quadspi_send_instruction(IS25LP128F_CMD_RESET, QUADSPI_CCR_MODE_1LINE);

  quadspi_send_instruction(IS25LP128F_CMD_WRITE_ENABLE, QUADSPI_CCR_MODE_1LINE);
  quadspi_write_register(IS25LP128F_CMD_WRITE_READ_PARAMETERS, QUADSPI_CCR_MODE_1LINE, 0x09 << 4);

  quadspi_send_instruction(IS25LP128F_CMD_WRITE_ENABLE, QUADSPI_CCR_MODE_1LINE);
  quadspi_send_instruction(IS25LP128F_CMD_ENTER_QPI_MODE, QUADSPI_CCR_MODE_1LINE);


  uint8_t memoryContent[8];
	quadspi_read(0x000000, 8U, memoryContent);

  usart_send_blocking(USART1,memoryContent[0]);
  usart_send_blocking(USART1,memoryContent[1]);
  usart_send_blocking(USART1,memoryContent[2]);
  usart_send_blocking(USART1,memoryContent[3]);
  usart_send_blocking(USART1,memoryContent[4]);
  usart_send_blocking(USART1,memoryContent[5]);
  usart_send_blocking(USART1,memoryContent[6]);
  usart_send_blocking(USART1,memoryContent[7]);

  memoryContent[0] = 0x68;
  memoryContent[1] = 0x65;
  memoryContent[2] = 0x6c;
  memoryContent[3] = 0x6c;
  memoryContent[4] = 0x6f;
  memoryContent[5] = 0x20;
  memoryContent[6] = 0x77;
  memoryContent[7] = 0x6f;

  quadspi_write(0x000000, 8U, memoryContent);
  

  return 0;
}
