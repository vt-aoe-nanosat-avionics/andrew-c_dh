// blink.c
// Makes the Tartan Artibeus EXPT board LEDs blink
//
// Written by Andrew McGrellis
// Other contributors: None
//
// See the top-level LICENSE file for the license.

// libopencm3
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/pwr.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/scb.h>

int main(void) {
  rcc_periph_clock_enable(RCC_GPIOC);
  rcc_periph_clock_enable(RCC_TIM2);
  gpio_mode_setup(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO10);
  gpio_mode_setup(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO12);
  gpio_set(GPIOC, GPIO10);
  gpio_clear(GPIOC, GPIO12);

  scb_clear_sleepdeep();
  scb_set_sleeponexit();

  timer_set_mode(TIM2, TIM_CR1_CKD_CK_INT_MUL_2, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
  timer_set_period(TIM2, 1250000);
  timer_enable_irq(TIM2, TIM_DIER_UIE);
  nvic_enable_irq(NVIC_TIM2_IRQ);
  timer_enable_counter(TIM2);

  while(1) {
    __asm__("nop");
  }
}

void tim2_isr(void) {
  timer_clear_flag(TIM2, TIM_SR_UIF);
  gpio_toggle(GPIOC, GPIO10);
  gpio_toggle(GPIOC, GPIO12);
}
