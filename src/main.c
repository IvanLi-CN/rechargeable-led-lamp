/* Template app on which you can build your own. */

#include <stdio.h>

#include "ch32v003fun.h"

#define SYSTICK_SR_CNTIF (1 << 0)
#define SYSTICK_CTLR_STE (1 << 0)
#define SYSTICK_CTLR_STIE (1 << 1)
#define SYSTICK_CTLR_STCLK (1 << 2)
#define SYSTICK_CTLR_STRE (1 << 3)
#define SYSTICK_CTLR_SWIE (1 << 31)

enum {
  BTN_WARM = 0b00,
  BTN_COLD = 0b01,
  BTN_LIGHT = 0b10,
  BTN_DARK = 0b11,
  BTN_MODE = 0b100,
  BTN_EN = 0b1000,
} typedef btn_t;

uint32_t count;

u8 color_temperature = 127;
int8_t color_temperature_step = 0;
u32 color_temperaature_btn_downed_at = 0;

u8 luminance = 127;
int8_t luminance_step = 0;
u32 luminance_btn_downed_at = 0;

void EXTI7_0_IRQHandler(void) __attribute__((interrupt));
void SysTick_Handler(void) __attribute__((interrupt));

void EXTI7_0_IRQHandler(void) {
  if (EXTI->INTFR & EXTI_FTENR_TR0) {
    color_temperature_step = -1;
  } else if (EXTI->INTFR & EXTI_FTENR_TR1) {
    color_temperature_step = 1;
  } else {
    color_temperature_step = 0;
  }

  if (EXTI->INTFR & EXTI_FTENR_TR2) {
    luminance_step = -1;
  } else if (EXTI->INTFR & EXTI_FTENR_TR3) {
    luminance_step = 1;
  } else {
    luminance_step = 0;
  }

  color_temperature += color_temperature_step;
  luminance += luminance_step;

  EXTI->INTFR =
      EXTI_INTENR_MR0 | EXTI_INTENR_MR1 | EXTI_INTENR_MR2 | EXTI_INTENR_MR3;
}

void SysTick_Handler(void) {
  if (EXTI->INTFR & EXTI_FTENR_TR0) {
    color_temperature_step = -1;
  } else if (EXTI->INTFR & EXTI_FTENR_TR1) {
    color_temperature_step = 1;
  } else {
    color_temperature_step = 0;
  }

  if (EXTI->INTFR & EXTI_FTENR_TR2) {
    luminance_step = -1;
  } else if (EXTI->INTFR & EXTI_FTENR_TR3) {
    luminance_step = 1;
  } else {
    luminance_step = 0;
  }

  color_temperature += color_temperature_step;
  luminance += luminance_step;

  EXTI->INTFR =
      EXTI_INTENR_MR0 | EXTI_INTENR_MR1 | EXTI_INTENR_MR2 | EXTI_INTENR_MR3;
}

void set_btn_long_press_timer(btn_t btn) {}

void init_systick() {
  // Enable SysTick
  SysTick->CTLR = SYSTICK_CTLR_SWIE | SYSTICK_CTLR_STRE | SYSTICK_CTLR_STIE |
                  SYSTICK_CTLR_STE;
}

void init_tim1() {
  RCC->APB2PCENR |= RCC_APB2Periph_GPIOD | RCC_APB2Periph_TIM1;

  // PD0 is T1CH1N, 10MHz Output alt func, push-pull
  GPIOD->CFGLR |= (GPIO_Speed_10MHz | GPIO_CNF_OUT_PP_AF) << (4 * 0);
  // PD1 is T1CH1N, 10MHz Output alt func, push-pull
  GPIOD->CFGLR |= (GPIO_Speed_10MHz | GPIO_CNF_OUT_PP_AF) << (4 * 1);

  // Reset TIM1 to init all regs
  RCC->APB2PRSTR |= RCC_APB2Periph_TIM1;
  RCC->APB2PRSTR &= ~RCC_APB2Periph_TIM1;

  // CTLR1: default is up, events generated, edge align
  // SMCFGR: default clk input is CK_INT

  // Prescaler
  TIM1->PSC = 0x0f00;

  // Auto Reload - sets period
  TIM1->ATRLR = 255;

  // Reload immediately
  TIM1->SWEVGR |= TIM_UG;

  // Enable CH1N output, positive pol
  TIM1->CCER |= TIM_CC1NE | TIM_CC1NP;

  // CH1 Mode is output, PWM1 (CC1S = 00, OC1M = 110)
  TIM1->CHCTLR1 |= TIM_OC1M_2 | TIM_OC1M_1;

  // Set the Capture Compare Register value to 50% initially
  TIM1->CH1CVR = 128;

  // Enable TIM1 outputs
  TIM1->BDTR |= TIM_MOE;

  // Enable TIM1
  TIM1->CTLR1 |= TIM_CEN;
}

void init_btns() {
  // Enable GPIOC, GPIOD, TIM1 and AFIO
  RCC->APB2PCENR |= RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO;

  // GPIO C0. C1 Push-Pull
  GPIOC->CFGLR |= (GPIO_Speed_In | GPIO_CNF_IN_PUPD) << (4 * 0);
  GPIOC->CFGLR |= (GPIO_Speed_In | GPIO_CNF_IN_PUPD) << (4 * 1);

  // Enable interruptions for PC0, PC1
  AFIO->EXTICR |= 2 << 0;
  AFIO->EXTICR |= 2 << 2;

  // Enable interruptions
  EXTI->INTENR |= EXTI_INTENR_MR0 | EXTI_INTENR_MR1;
  EXTI->FTENR |= EXTI_FTENR_TR0 | EXTI_FTENR_TR1;

  // Enable interrupt handler for EXTI7_0
  NVIC_EnableIRQ(EXTI7_0_IRQn);
}

int main() {
  SystemInit();

  init_systick();

  init_tim1();

  init_btns();

  // Enable interrupt handler for SysTick
  NVIC_EnableIRQ(SysTicK_IRQn);

  while (1) {
    // u8 gpio_c0 = GPIOC->INDR & GPIO_INDR_IDR0;
    // u8 gpio_c1 = GPIOC->INDR & GPIO_INDR_IDR1 >> 1;

    // output bits
    printf("GPIOC: %s%s. irq count: %ld \n",
           GPIOC->INDR & GPIO_INDR_IDR0 ? "1" : "0",
           GPIOC->INDR & GPIO_INDR_IDR1 ? "1" : "0", count);

    Delay_Ms(250);
  }
}
