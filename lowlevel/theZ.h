#pragma once

#include <stdbool.h>
#include <stdint.h>

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

// This API should be splitted to specific headers

/*****************************************************************************/
// Led,…
void gpio_setup();

void led_set_status(uint32_t status);
void led_toggle_status();

void led_test_loop();

typedef enum { STEPPER_DOWN=0, STEPPER_UP=1 } T_STEPPER_DIR ;

void disable_stepper();
void enable_stepper();

int get_top_es();//get top end stop
int get_bottom_es();//get bottom end stop

void set_stepper_dir(int dir);
void step_toggle();

void stepper_set(int32_t goal);
int32_t stepper_get();


void set_pump(uint8_t status);
void set_valve(uint8_t status);

void get_color(uint8_t *dt_red, uint8_t *dt_blue, uint8_t *dt_green);

void stepper_test_loop();

void pump_test_loop();

void valve_test_loop();

void pv_test_loop();

void color_test_loop();

/*****************************************************************************/
void adc_setup();
uint32_t adc_read();
void adc_test_loop();//need debug uart working
