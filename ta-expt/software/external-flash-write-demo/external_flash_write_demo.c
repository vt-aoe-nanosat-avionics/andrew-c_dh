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
#include <libopencm3/stm32/quadspi.h> // QUADSPI functions

#include <IS25LP128F.h>              // IS25LP128F flash memory macros


//void quadspi_write(uint32_t address, uint32_t length, uint8_t data[]);
//void quadspi_read(uint32_t address, uint32_t length, uint8_t* data);


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
	rcc_periph_clock_enable(RCC_GPIOC);  // enable clock for port with QUADSPI pins
  rcc_periph_clock_enable(RCC_USART1); // enable clock for USART1 peripheral
  rcc_periph_clock_enable(RCC_QSPI);   // enable clock for QUADSPI peripheral
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
  
  usart_send_blocking(USART1,'\r');
  usart_send_blocking(USART1,'\n');
  usart_send_blocking(USART1,'s');
  usart_send_blocking(USART1,'t');
  usart_send_blocking(USART1,'a');
  usart_send_blocking(USART1,'r');
  usart_send_blocking(USART1,'t');
  usart_send_blocking(USART1,'i');
  usart_send_blocking(USART1,'n');
  usart_send_blocking(USART1,'g');


  // QUADSPI setup
  rcc_periph_clock_enable(RCC_QSPI);

  gpio_mode_setup(GPIOC, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO1 | GPIO2 | GPIO3 | GPIO4);
  gpio_mode_setup(GPIOC, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO11);
  gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO3);
  gpio_set_output_options(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_VERYHIGH, GPIO1 | GPIO2 | GPIO3 | GPIO4);
  gpio_set_output_options(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_VERYHIGH, GPIO3);
  gpio_set_output_options(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_VERYHIGH, GPIO11);
  gpio_set_af(GPIOA, GPIO_AF10, GPIO3);
  gpio_set_af(GPIOC, GPIO_AF10, GPIO1 | GPIO2 | GPIO3 | GPIO4);
  gpio_set_af(GPIOC, GPIO_AF5, GPIO11);

  quadspi_disable();
  quadspi_set_flash_size(23); // 128 Mbit = 16 Mbyte = 2^(n+1) // n = 23
  quadspi_set_cs_high_time(6);   // 1/2 clock cycle
  quadspi_set_prescaler(0);   // 1:2 prescaler
	quadspi_clear_flag(QUADSPI_FCR_CTOF | QUADSPI_FCR_CSMF | QUADSPI_FCR_CTCF | QUADSPI_FCR_CTEF);
  quadspi_select_flash(QUADSPI_FLASH_SEL_2);
  quadspi_enable();

  uint8_t memoryContent[8];
  struct quadspi_command enableWrite;
  enableWrite.instruction.mode = QUADSPI_CCR_MODE_1LINE;
  enableWrite.instruction.instruction = IS25LP128F_CMD_WRITE_ENABLE;

  struct quadspi_command enableQPI;
  enableQPI.instruction.mode = QUADSPI_CCR_MODE_1LINE;  
  enableQPI.instruction.instruction = IS25LP128F_CMD_ENTER_QPI_MODE;

  struct quadspi_command write_readParameters;
  write_readParameters.instruction.mode = QUADSPI_CCR_MODE_4LINE;
  write_readParameters.instruction.instruction = IS25LP128F_CMD_WRITE_READ_PARAMETERS;

  uint32_t readParameters = 0x09 << 4;


  struct quadspi_command command;
  command.instruction.mode = QUADSPI_CCR_MODE_4LINE;
  command.instruction.instruction = IS25LP128F_CMD_QUAD_OUTPUT_FAST_READ;
  command.address.mode = QUADSPI_CCR_MODE_1LINE;
  command.address.size = QUADSPI_CCR_SIZE_24BIT;
  command.address.address = 0x000000;
  command.dummy_cycles = 6;
  command.data_mode = QUADSPI_CCR_MODE_4LINE;

//  quadspi_send_instruction(IS25LP128F_CMD_RESET_ENABLE, QUADSPI_CCR_MODE_1LINE);
//  usart_send_blocking(USART1,'\r');
//  usart_send_blocking(USART1,'\n');
//  usart_send_blocking(USART1,'s');
//  quadspi_send_instruction(IS25LP128F_CMD_RESET, QUADSPI_CCR_MODE_1LINE);
//
//  quadspi_send_instruction(IS25LP128F_CMD_WRITE_ENABLE, QUADSPI_CCR_MODE_1LINE);
//  quadspi_write_register(IS25LP128F_CMD_WRITE_READ_PARAMETERS, QUADSPI_CCR_MODE_1LINE, 0x09 << 4);
//
//  quadspi_send_instruction(IS25LP128F_CMD_WRITE_ENABLE, QUADSPI_CCR_MODE_1LINE);
//  quadspi_send_instruction(IS25LP128F_CMD_ENTER_QPI_MODE, QUADSPI_CCR_MODE_1LINE);

  quadspi_write(&enableWrite, memoryContent, 0);
  quadspi_write(&enableQPI, memoryContent, 0);

  //quadspi_write(&enableWrite, memoryContent, 0);
  //quadspi_write(&write_readParameters, &readParameters, 4);


  quadspi_read(&command, memoryContent, 8U);
	//quadspi_read(0x000000, 8U, memoryContent);

  usart_send_blocking(USART1,'\r');
  usart_send_blocking(USART1,'\n');
  usart_send_blocking(USART1,'r');
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


  quadspi_write(&enableWrite, memoryContent, 0);

  command.instruction.instruction = IS25LP128F_CMD_QUAD_PAGE_PROGRAM;
  command.address.mode = QUADSPI_CCR_MODE_1LINE;
  command.address.size = QUADSPI_CCR_SIZE_24BIT;
  command.address.address = 0x000000;
  quadspi_write(&command, memoryContent, 8U);
  //quadspi_write(0x000000, 8U, memoryContent);

  quadspi_read(&command, memoryContent, 8U);
  //quadspi_read(0x000000, 8U, memoryContent);

  usart_send_blocking(USART1,'\r');
  usart_send_blocking(USART1,'\n');
  usart_send_blocking(USART1,'w');
  usart_send_blocking(USART1,memoryContent[0]);
  usart_send_blocking(USART1,memoryContent[1]);
  usart_send_blocking(USART1,memoryContent[2]);
  usart_send_blocking(USART1,memoryContent[3]);
  usart_send_blocking(USART1,memoryContent[4]);
  usart_send_blocking(USART1,memoryContent[5]);
  usart_send_blocking(USART1,memoryContent[6]);
  usart_send_blocking(USART1,memoryContent[7]);

  //usart_send_blocking(USART1,'\r');
  //usart_send_blocking(USART1,'\n');
  //usart_send_blocking(USART1,'f');
  //usart_send_blocking(USART1,'i');
  //usart_send_blocking(USART1,'n');
  //usart_send_blocking(USART1,'i');
  //usart_send_blocking(USART1,'s');
  //usart_send_blocking(USART1,'h');
  //usart_send_blocking(USART1,'\r');
  //usart_send_blocking(USART1,'\n');
  

  return 0;
}


//void quadspi_write(uint32_t address, uint32_t length, uint8_t data[])
//{
//	quadspi_send_instruction(IS25LP128F_CMD_WRITE_ENABLE, QUADSPI_CCR_MODE_4LINE);
//	while(quadspi_get_busy()) {
//		;
//	}
//	quadspi_clear_flag(QUADSPI_FCR_CTOF | QUADSPI_FCR_CSMF | QUADSPI_FCR_CTCF | QUADSPI_FCR_CTEF);
//	quadspi_set_data_length(length-1);
//	quadspi_set_fifo_threshold(1);
//
//	quadspi_set_fmode(QUADSPI_CCR_FMODE_IWRITE);
//	quadspi_set_data_mode(QUADSPI_CCR_MODE_4LINE);
//	quadspi_set_instruction_mode(QUADSPI_CCR_MODE_4LINE);
//	quadspi_set_address_mode(QUADSPI_CCR_MODE_4LINE);
//	quadspi_set_address_size(QUADSPI_CCR_ADSIZE_24BIT);
//	quadspi_set_instruction(IS25LP128F_CMD_QUAD_PAGE_PROGRAM);
//	quadspi_set_address(address);
//
//	uint32_t data_index = 0;
//	do {
//		quadspi_set_data(*(uint32_t*)data);
//		data_index++;
//		while ((QUADSPI_SR & (QUADSPI_SR_FLEVEL_MASK << QUADSPI_SR_FLEVEL_SHIFT)) != 0x00); //wait for the data to be shifted out
//	} while (quadspi_get_busy());
//
//
//	quadspi_clear_flag(QUADSPI_FCR_CTOF | QUADSPI_FCR_CSMF | QUADSPI_FCR_CTCF | QUADSPI_FCR_CTEF);
//}
//
//void quadspi_read(uint32_t address, uint32_t length, uint8_t* data)
//{
//	while(quadspi_get_busy()) {
//		;
//	}
//	quadspi_clear_flag(QUADSPI_FCR_CTOF | QUADSPI_FCR_CSMF | QUADSPI_FCR_CTCF | QUADSPI_FCR_CTEF);
//	quadspi_set_data_length(length-1);
//	quadspi_set_fifo_threshold(1);
//
//	quadspi_set_fmode(QUADSPI_CCR_FMODE_IREAD);
//	quadspi_set_data_mode(QUADSPI_CCR_MODE_4LINE);
//	quadspi_set_instruction_mode(QUADSPI_CCR_MODE_4LINE);
//	quadspi_set_address_mode(QUADSPI_CCR_MODE_4LINE);
//	quadspi_set_address_size(QUADSPI_CCR_ADSIZE_24BIT);
//	quadspi_set_instruction(IS25LP128F_CMD_QUAD_OUTPUT_FAST_READ);
//	quadspi_set_address(address);
//
//	int index = 0;
//
//	while(quadspi_get_busy()) {
//		if(quadspi_get_flag(QUADSPI_SR_FTF) > 0) {
//			data[index++] = quadspi_get_data();
//		}
//	}
//
//	quadspi_clear_flag(QUADSPI_FCR_CTOF | QUADSPI_FCR_CSMF | QUADSPI_FCR_CTCF | QUADSPI_FCR_CTEF);
//}