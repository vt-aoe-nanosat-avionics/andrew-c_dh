// blink.c
// Makes the Tartan Artibeus EXPT board LEDs blink
//
// Written by Bradley Denby
// Other contributors: Andrew McGrellis
//
// See the top-level LICENSE file for the license.

// libopencm3
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/rtc.h>
#include <libopencm3/stm32/pwr.h>
#include <libopencm3/stm32/gpio.h>

int main(void) {
  rcc_osc_on(RCC_LSE);
  rcc_periph_clock_enable(RCC_GPIOC);
  gpio_mode_setup(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO10);
  gpio_mode_setup(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO12);
  gpio_set(GPIOC, GPIO10);
  gpio_set(GPIOC, GPIO12);

 
  while(1) {
    rtc_set_wakeup_time(10, 0b10);
    //MMIO32(0xE000E010) &= ~(1UL << 1U);
    //pwr_enable_standby_mode();

    gpio_toggle(GPIOC, GPIO10);
    for(int i=0; i<400000; i++) {
      __asm__("nop");
    }
    gpio_toggle(GPIOC, GPIO12);
    pwr_enable_standby_mode();
  }

}
