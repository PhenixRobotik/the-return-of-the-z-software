#pragma once

#include "theZ.h"

// POOLING Timer
#define STEPPER_TIM TIM15
// magic trick of timer 15 normaly timX_isr
#define timX_isr(void) tim1_brk_tim15_isr(void)
#define STEPPER_RCC_TIM RCC_TIM15
// normaly NVIC_TIMx_IRQ
#define STEPPER_NVIC_TIM_IRQ NVIC_TIM1_BRK_TIM15_IRQ
#define STEPPER_RST_TIM RST_TIM15
// change these 2 lines to setup interruption frequency
// #define ODOM_TIM_PRESCALER ((rcc_apb1_frequency * 2) / 5000)
#define STEPPER_TIM_PRESCALER 50
#define STEPPER_TIM_PERIOD 150

void setup_stepper_interrupt();
void stepper_set(int32_t goal);
int32_t stepper_get();
