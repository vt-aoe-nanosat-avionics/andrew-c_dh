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

#define ADDRESS 0x003FA5A5


//void quadspi_write(uint32_t address, uint32_t length, uint8_t data[]);
//void quadspi_read(uint32_t address, uint32_t length, uint8_t* data);


// Main
int main(void) {
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
  quadspi_set_prescaler(0);   // 1:1 prescaler
	quadspi_clear_flag(QUADSPI_FCR_CTOF | QUADSPI_FCR_CSMF | QUADSPI_FCR_CTCF | QUADSPI_FCR_CTEF);
  quadspi_select_flash(QUADSPI_FLASH_SEL_2);
  quadspi_set_threshold_level(7);
  quadspi_enable();


  rcc_periph_clock_enable(RCC_QSPI);

  uint8_t memoryContent[256];

  struct quadspi_command enableWrite;
  enableWrite.instruction.mode = QUADSPI_CCR_MODE_1LINE;
  enableWrite.instruction.instruction = IS25LP128F_CMD_WRITE_ENABLE;
  enableWrite.alternative_bytes.mode = QUADSPI_CCR_MODE_NONE;
  enableWrite.address.mode = QUADSPI_CCR_MODE_NONE;
  enableWrite.dummy_cycles = 0;
  enableWrite.data_mode = QUADSPI_CCR_MODE_NONE;

  struct quadspi_command enableQPI;
  enableQPI.instruction.mode = QUADSPI_CCR_MODE_1LINE;  
  enableQPI.instruction.instruction = IS25LP128F_CMD_ENTER_QPI_MODE;
  enableQPI.alternative_bytes.mode = QUADSPI_CCR_MODE_NONE;
  enableQPI.address.mode = QUADSPI_CCR_MODE_NONE;
  enableQPI.dummy_cycles = 0;
  enableQPI.data_mode = QUADSPI_CCR_MODE_NONE;

  // Read data from the flash
  struct quadspi_command read;
  read.instruction.mode = QUADSPI_CCR_MODE_4LINE;
  read.instruction.instruction = IS25LP128F_CMD_FAST_READ;
  read.address.mode = QUADSPI_CCR_MODE_4LINE;
  read.address.address = ADDRESS;
  read.address.size = QUADSPI_CCR_SIZE_32BIT;
  read.alternative_bytes.mode = QUADSPI_CCR_MODE_NONE;
  read.dummy_cycles = 6;
  read.data_mode = QUADSPI_CCR_MODE_4LINE;

  // Set sector of the flash to 1s
  struct quadspi_command erase;
  erase.instruction.mode = QUADSPI_CCR_MODE_4LINE;
  erase.instruction.instruction = IS25LP128F_CMD_ERASE_SECTOR;
  erase.address.mode = QUADSPI_CCR_MODE_4LINE;
  erase.address.address = ADDRESS;
  erase.address.size = QUADSPI_CCR_SIZE_32BIT;
  erase.alternative_bytes.mode = QUADSPI_CCR_MODE_NONE;
  erase.dummy_cycles = 0;
  erase.data_mode = QUADSPI_CCR_MODE_NONE;


  // Program data to the flash
  struct quadspi_command write;
  write.instruction.mode = QUADSPI_CCR_MODE_4LINE;
  write.instruction.instruction = IS25LP128F_CMD_PAGE_PROGRAM;
  write.address.mode = QUADSPI_CCR_MODE_4LINE;
  write.address.address = ADDRESS;
  write.address.size = QUADSPI_CCR_SIZE_32BIT;
  write.alternative_bytes.mode = QUADSPI_CCR_MODE_NONE;
  write.dummy_cycles = 0;
  write.data_mode = QUADSPI_CCR_MODE_4LINE;



  quadspi_wait_while_busy();
  quadspi_write(&enableWrite, memoryContent, 0);
  quadspi_wait_while_busy();
  quadspi_write(&enableQPI, memoryContent, 0);

  quadspi_wait_while_busy();
  quadspi_read(&read, memoryContent, 8);

  usart_send_blocking(USART1,'\r');
  usart_send_blocking(USART1,'\n');
  usart_send_blocking(USART1,'r');
  usart_send_blocking(USART1,'\r');
  usart_send_blocking(USART1,'\n');
  usart_send_blocking(USART1,memoryContent[0]);
  usart_send_blocking(USART1,memoryContent[1]);
  usart_send_blocking(USART1,memoryContent[2]);
  usart_send_blocking(USART1,memoryContent[3]);
  usart_send_blocking(USART1,memoryContent[4]);
  usart_send_blocking(USART1,memoryContent[5]);
  usart_send_blocking(USART1,memoryContent[6]);
  usart_send_blocking(USART1,memoryContent[7]);


  //while(1);

  enableWrite.instruction.mode = QUADSPI_CCR_MODE_4LINE;
  quadspi_wait_while_busy();
  quadspi_write(&enableWrite, memoryContent, 0);

  quadspi_wait_while_busy();
  quadspi_write(&erase, memoryContent, 0);
  for(int i = 0; i < 400000; i++) {__asm__("nop");}

  quadspi_wait_while_busy();
  quadspi_read(&read, memoryContent, 8);
  for(int i = 0; i < 400000; i++) {__asm__("nop");}


  uint32_t test = 0x0000000000000012;
  memoryContent[0] = 0x68;
  memoryContent[1] = 0x65;
  memoryContent[2] = 0x6c;
  memoryContent[3] = 0x6c;
  memoryContent[4] = 0x6f;
  memoryContent[5] = 0x20;
  memoryContent[6] = 0x77;
  memoryContent[7] = 0x6f;

  quadspi_wait_while_busy();
  quadspi_write(&enableWrite, memoryContent, 0);

  quadspi_wait_while_busy();
  quadspi_write(&write, memoryContent, 8);
  //for(int i = 0; i < 40000; i++) {__asm__("nop");}

  uint8_t flashContent[8];

  quadspi_wait_while_busy();
  quadspi_read(&read, flashContent, 8);


  usart_send_blocking(USART1,'\r');
  usart_send_blocking(USART1,'\n');
  usart_send_blocking(USART1,'w');
  usart_send_blocking(USART1,'\r');
  usart_send_blocking(USART1,'\n');
  usart_send_blocking(USART1,flashContent[0]);
  usart_send_blocking(USART1,flashContent[1]);
  usart_send_blocking(USART1,flashContent[2]);
  usart_send_blocking(USART1,flashContent[3]);
  usart_send_blocking(USART1,flashContent[4]);
  usart_send_blocking(USART1,flashContent[5]);
  usart_send_blocking(USART1,flashContent[6]);
  usart_send_blocking(USART1,flashContent[7]);
  

  return 0;
}
