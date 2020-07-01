#include "theZ.h"

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>

typedef enum { STEPPER_DOWN=0, STEPPER_UP=1 } T_STEPPER_DIR ;

void gpio_setup()
{
  //start all the ports clocks
  rcc_periph_clock_enable(RCC_GPIOA);
  rcc_periph_clock_enable(RCC_GPIOB);
  rcc_periph_clock_enable(RCC_GPIOF);

  // status led
  gpio_mode_setup(GPIOF, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO0);//led pin

  //pump and valve gpios
  gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO9);
  gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO10);

  //stepper
  gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO8);//step
  gpio_mode_setup(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO0);//dir
  gpio_mode_setup(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO1);//enable
  disable_stepper();
  gpio_mode_setup(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_NONE , GPIO6);//TOP
  gpio_mode_setup(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_NONE , GPIO7);//DOWN

  //AX dir
  gpio_mode_setup(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO5);//enable


  gpio_mode_setup(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_NONE , GPIO1);
  gpio_mode_setup(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_NONE , GPIO2);
  gpio_mode_setup(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_NONE , GPIO3);
  gpio_mode_setup(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_NONE , GPIO4);
  gpio_mode_setup(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_NONE , GPIO5);
  gpio_mode_setup(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_NONE , GPIO0);
  gpio_mode_setup(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_NONE , GPIO15);



  // power input
  //gpio_mode_setup(GPIOF, GPIO_MODE_INPUT, GPIO_PUPD_PULLDOWN, GPIO1);
}


void disable_stepper()
{
  gpio_set(GPIOB, GPIO1);
}

void enable_stepper()
{
  gpio_clear(GPIOB, GPIO1);
}

//get top end stop
int get_top_es()
{
  return gpio_get(GPIOA, GPIO6);
}

//get bottom end stop
int get_bottom_es()
{
  return gpio_get(GPIOA, GPIO7);
}

void set_stepper_dir(int dir)
{
  if(dir == STEPPER_DOWN)
    gpio_clear(GPIOB, GPIO0);
  else
    gpio_set(GPIOB, GPIO0);
}

void step_toggle()
{
  gpio_toggle(GPIOA, GPIO8);//led
}



void led_toggle_status()
{
  gpio_toggle(GPIOF, GPIO0);//led
}

void led_set_status(uint32_t status) {
  if (status == 0)
    gpio_clear(GPIOF, GPIO0);
  else
    gpio_set  (GPIOF, GPIO0);
}





void led_test_loop()
{
  while(1)
  {
    led_toggle_status();
    delay_ms(1000);
  }
}


void stepper_test_loop()
{
  enable_stepper();
  while(1)
  {
    set_stepper_dir(STEPPER_UP);//go up
    while(!get_top_es())
    {
      step_toggle();
      delay_us(500);
    }

    set_stepper_dir(STEPPER_DOWN);//go down
    while(!get_bottom_es())
    {
      step_toggle();
      delay_us(500);
    }
  }
}


void pump_test_loop()
{
  while(1)
  {
    gpio_toggle(GPIOA, GPIO9);
    delay_ms(1000);
  }
}

void valve_test_loop()
{
  while(1)
  {
    gpio_toggle(GPIOA, GPIO10);
    delay_ms(1000);
  }
}
