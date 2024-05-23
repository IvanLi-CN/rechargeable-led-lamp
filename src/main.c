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
  BTN_COOL = 0b01,
  BTN_LIGHT = 0b10,
  BTN_DARK = 0b11,
  BTN_MODE = 0b100,
  BTN_EN = 0b1000,
} typedef btn_t;

enum {
  BTN_MARK_COLOR_TEMPERATURE = 0b01,
  BTN_MARK_LUMINANCE = 0b10,
} typedef btn_mark_t;

uint32_t count;

u16 color_temperature = 127;
u32 color_temperature_btn_downed_at = 0;

u16 luminance = 127;
u32 luminance_btn_downed_at = 0;

u16 warm_value = 20;
u16 cool_value = 20;

btn_mark_t tick_btn_mark = BTN_MARK_COLOR_TEMPERATURE;

#define GPIO_Pin_Warm GPIO_Pin_5
#define GPIO_Pin_Cool GPIO_Pin_3
#define GPIO_Pin_Light GPIO_Pin_6
#define GPIO_Pin_Dark GPIO_Pin_2

void EXTI7_0_IRQHandler(void) __attribute__((interrupt));
void SysTick_Handler(void) __attribute__((interrupt));
void btns_down(btn_mark_t btn_mark);

void set_btn_long_press_irq_tick() {
  u32 ct_w = !(GPIOD->INDR & GPIO_Pin_Warm);
  u32 ct_c = !(GPIOD->INDR & GPIO_Pin_Cool);
  u32 l_l = !(GPIOD->INDR & GPIO_Pin_Light);
  u32 l_d = !(GPIOA->INDR & GPIO_Pin_Dark);

  if ((ct_w || ct_c) && (l_l || l_d)) {
    if (color_temperature_btn_downed_at < luminance_btn_downed_at) {
      tick_btn_mark = BTN_MARK_COLOR_TEMPERATURE;
      SysTick->CMP = color_temperature_btn_downed_at + 200000;
    } else {
      tick_btn_mark = BTN_MARK_LUMINANCE;
      SysTick->CMP = luminance_btn_downed_at + 200000;
    }

    if (SysTick->CMP < SysTick->CNT) {
      btns_down(tick_btn_mark);
    }
    return;
  }

  if (ct_w || ct_c) {
    tick_btn_mark = BTN_MARK_COLOR_TEMPERATURE;
    SysTick->CMP = color_temperature_btn_downed_at + 200000;
  } else if (l_l || l_d) {
    tick_btn_mark = BTN_MARK_LUMINANCE;
    SysTick->CMP = luminance_btn_downed_at + 200000;
  }
}

void btns_down(btn_mark_t btn_mark) {
  if (btn_mark == BTN_MARK_COLOR_TEMPERATURE) {
    if ((GPIOD->INDR & GPIO_Pin_Warm) == 0) {
      if (color_temperature > 0) {
        color_temperature--;
      }
      color_temperature_btn_downed_at = SysTick->CNT;
      set_btn_long_press_irq_tick();
    }

    if ((GPIOD->INDR & GPIO_Pin_Cool) == 0) {
      if (color_temperature < 255) {
        color_temperature++;
      }
      color_temperature_btn_downed_at = SysTick->CNT;
      set_btn_long_press_irq_tick();
    }
  } else if (btn_mark == BTN_MARK_LUMINANCE) {
    if ((GPIOA->INDR & GPIO_Pin_Dark) == 0) {
      if (luminance > 0) {
        luminance--;
      }
      luminance_btn_downed_at = SysTick->CNT;
      set_btn_long_press_irq_tick();
    }

    if ((GPIOD->INDR & GPIO_Pin_Light) == 0) {
      if (luminance < 255) {
        luminance++;
      }
      luminance_btn_downed_at = SysTick->CNT;
      set_btn_long_press_irq_tick();
    }
  }
  warm_value = (u32)color_temperature * (u32)luminance / 255;
  cool_value = luminance - warm_value;

  TIM1->CH1CVR = cool_value;
  TIM1->CH3CVR = warm_value;
}

void EXTI7_0_IRQHandler(void) {
  printf("EXTI7_0_IRQHandler, %d, %d, %d, %d, %d, %d \n",
         EXTI->INTFR & EXTI_INTENR_MR6 ? 1 : 0,
         EXTI->INTFR & EXTI_INTENR_MR5 ? 1 : 0,
         EXTI->INTFR & EXTI_INTENR_MR4 ? 1 : 0,
         EXTI->INTFR & EXTI_INTENR_MR3 ? 1 : 0,
         EXTI->INTFR & EXTI_INTENR_MR2 ? 1 : 0,
         EXTI->INTFR & EXTI_INTENR_MR1 ? 1 : 0);
  if ((EXTI->INTFR & EXTI_INTENR_MR5) || (EXTI->INTFR & EXTI_INTENR_MR3)) {
    btns_down(BTN_MARK_COLOR_TEMPERATURE);
  }

  if ((EXTI->INTFR & EXTI_INTENR_MR6) || (EXTI->INTFR & EXTI_INTENR_MR2)) {
    btns_down(BTN_MARK_LUMINANCE);
  }

  EXTI->INTFR = EXTI_INTENR_MR6 | EXTI_INTENR_MR5 | EXTI_INTENR_MR4 |
                EXTI_INTENR_MR3 | EXTI_INTENR_MR2 | EXTI_INTENR_MR1;
}

void SysTick_Handler(void) {
  SysTick->SR = 0;
  btns_down(tick_btn_mark);
}

void init_systick() {
  // Enable SysTick
  SysTick->CTLR = SYSTICK_CTLR_SWIE | SYSTICK_CTLR_STIE | SYSTICK_CTLR_STE;
}

void init_tim1() {
  RCC->APB2PCENR |= RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOC |
                    RCC_APB2Periph_TIM1 | RCC_APB2Periph_AFIO;

  // PD2 is T1CH1, 10MHz Output alt func, push-pull
  GPIOD->CFGLR &= ~(0xf << (4 * 2));
  GPIOD->CFGLR |= (GPIO_Speed_10MHz | GPIO_CNF_OUT_PP_AF) << (4 * 2);

  // PC3 is T1CH3, 10MHz Output alt func, push-pull
  GPIOC->CFGLR &= ~(0xf << (4 * 3));
  GPIOC->CFGLR |= (GPIO_Speed_10MHz | GPIO_CNF_OUT_PP_AF) << (4 * 3);

  // Reset TIM1 to init all regs
  RCC->APB2PRSTR |= RCC_APB2Periph_TIM1;
  RCC->APB2PRSTR &= ~RCC_APB2Periph_TIM1;

  // CTLR1: default is up, events generated, edge align
  // SMCFGR: default clk input is CK_INT

  // Prescaler
  TIM1->PSC = 0x00ff;

  // Auto Reload - sets period
  TIM1->ATRLR = 255;

  // Reload immediately
  TIM1->SWEVGR |= TIM_UG;

  // Enable CH1 output, positive pol
  TIM1->CCER |= TIM_CC1E;

  // Enable CH3 output, positive pol
  TIM1->CCER |= TIM_CC3E;

  // CH1 Mode is output, PWM1 (CC1S = 00, OC1M = 110)
  TIM1->CHCTLR1 |= TIM_OC1M_2 | TIM_OC1M_1;

  // CH3 Mode is output, PWM1 (CC3S = 00, OC3M = 110)
  TIM1->CHCTLR2 |= TIM_OC3M_2 | TIM_OC3M_1;

  // Set the Capture Compare Register value
  TIM1->CH1CVR = warm_value;

  // Set the Capture Compare Register value
  TIM1->CH3CVR = cool_value;

  // Enable TIM1 outputs
  TIM1->BDTR |= TIM_MOE;

  // Enable TIM1
  TIM1->CTLR1 |= TIM_CEN;
}

void init_btns() {
  // Enable GPIOA, GPIOD
  RCC->APB2PCENR |=
      RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOD | RCC_APB2Periph_AFIO;

  // PD6: LIG
  // PD5: WARN
  // PD4: MODE
  // PD3: COOL
  // PA2: DIM
  // PA1: PWR

  // GPIO PD6, PD5, PD4, PD3, PA2, PA1 Push-Pull
  GPIOD->CFGLR &= ~(0xf << (4 * 6));
  GPIOD->CFGLR |= (GPIO_Speed_In | GPIO_CNF_IN_PUPD) << (4 * 6);
  GPIOD->CFGLR &= ~(0xf << (4 * 5));
  GPIOD->CFGLR |= (GPIO_Speed_In | GPIO_CNF_IN_PUPD) << (4 * 5);
  GPIOD->CFGLR &= ~(0xf << (4 * 4));
  GPIOD->CFGLR |= (GPIO_Speed_In | GPIO_CNF_IN_PUPD) << (4 * 4);
  GPIOD->CFGLR &= ~(0xf << (4 * 3));
  GPIOD->CFGLR |= (GPIO_Speed_In | GPIO_CNF_IN_PUPD) << (4 * 3);
  GPIOA->CFGLR &= ~(0xf << (4 * 2));
  GPIOA->CFGLR |= (GPIO_Speed_In | GPIO_CNF_IN_PUPD) << (4 * 2);
  GPIOA->CFGLR &= ~(0xf << (4 * 1));
  GPIOA->CFGLR |= (GPIO_Speed_In | GPIO_CNF_IN_PUPD) << (4 * 1);

  // GPIO PD6, PD5, PD4, PD3, PA2, PA1 Pull-Up
  GPIOD->OUTDR |= GPIO_Pin_6 | GPIO_Pin_5 | GPIO_Pin_4 | GPIO_Pin_3;
  GPIOA->OUTDR |= GPIO_Pin_2 | GPIO_Pin_1;

  // Enable interruptions for PD6, PD5, PD4, PD3, PA2, PA1
  AFIO->EXTICR |= AFIO_EXTICR_EXTI6_PD | AFIO_EXTICR_EXTI5_PD |
                  AFIO_EXTICR_EXTI4_PD | AFIO_EXTICR_EXTI3_PD |
                  AFIO_EXTICR_EXTI2_PA | AFIO_EXTICR_EXTI1_PA;

  // Enable interruptions
  EXTI->INTENR |= EXTI_INTENR_MR6 | EXTI_INTENR_MR5 | EXTI_INTENR_MR4 |
                  EXTI_INTENR_MR3 | EXTI_INTENR_MR2 | EXTI_INTENR_MR1;
  EXTI->FTENR |= EXTI_FTENR_TR6 | EXTI_FTENR_TR5 | EXTI_FTENR_TR4 |
                 EXTI_FTENR_TR3 | EXTI_FTENR_TR2 | EXTI_FTENR_TR1;

  // Enable interrupt handler for EXTI7_0
  NVIC_EnableIRQ(EXTI7_0_IRQn);

  EXTI->INTFR = EXTI_INTENR_MR6 | EXTI_INTENR_MR5 | EXTI_INTENR_MR4 |
                EXTI_INTENR_MR3 | EXTI_INTENR_MR2 | EXTI_INTENR_MR1;
}

int main() {
  SystemInit();

  init_systick();

  init_btns();
  init_tim1();

  // Enable interrupt handler for SysTick
  NVIC_EnableIRQ(SysTicK_IRQn);

  while (1) {
    // u8 gpio_c0 = GPIOC->INDR & GPIO_INDR_IDR0;
    // u8 gpio_c1 = GPIOC->INDR & GPIO_INDR_IDR1 >> 1;

    // output bits

    // printf("color_temperature: %d, luminance: %d. W: %d, C: %d, ticks: %lu
    // \n",
    //        color_temperature, luminance, warm_value, cool_value,
    //        SysTick->CNT);

    printf("PD5, %d \n", GPIOD->INDR & GPIO_Pin_5 ? 1 : 0);

    Delay_Ms(250);
  }
}
