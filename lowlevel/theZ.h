#pragma once

#include <stdbool.h>
#include <stdint.h>

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
