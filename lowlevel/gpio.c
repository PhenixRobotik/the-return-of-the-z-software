#include "theZ.h"

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/cm3/nvic.h>

#include "clock.h"
#include "uart.h"

// CAN
#define GPIO_CAN_PORT   GPIOA
#define GPIO_CAN_RX_PIN GPIO11
#define GPIO_CAN_TX_PIN GPIO12
#define GPIO_CAN_AF     GPIO_AF9

void gpio_setup()
{
  //start all the ports clocks
  rcc_periph_clock_enable(RCC_GPIOA);
  rcc_periph_clock_enable(RCC_GPIOB);
  rcc_periph_clock_enable(RCC_GPIOF);

  // CAN RX
  gpio_mode_setup(GPIO_CAN_PORT, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_CAN_RX_PIN);
  gpio_set_af(GPIO_CAN_PORT, GPIO_CAN_AF, GPIO_CAN_RX_PIN);

  // CAN TX
  gpio_mode_setup(GPIO_CAN_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_CAN_TX_PIN);
  gpio_set_output_options(GPIO_CAN_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_CAN_TX_PIN);
  gpio_set_af(GPIO_CAN_PORT, GPIO_CAN_AF, GPIO_CAN_TX_PIN);


  // status led
  gpio_mode_setup(GPIOF, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO0);//led pin

  //pump and valve gpios
  gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO9);
  gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO10);

  //stepper
  gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO8);//step
  gpio_mode_setup(GPIOB, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO0);//track cuted
  gpio_mode_setup(GPIOB, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO1);//track cuted
  gpio_mode_setup(GPIOF, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE , GPIO1);//enable
  gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE , GPIO15);//dir
  disable_stepper();
  gpio_mode_setup(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_NONE , GPIO6);//TOP
  gpio_mode_setup(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_NONE , GPIO7);//DOWN

  //AX dir
  gpio_mode_setup(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO5);//enable


  gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE , GPIO2);//S0
  gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE , GPIO3);//S1
  gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE , GPIO4);//S2
  gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE , GPIO5);//S3
  //track cuted
  //gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE , GPIO15);//color led

  gpio_clear(GPIOA, GPIO2);
  gpio_clear(GPIOA, GPIO3);
  gpio_clear(GPIOA, GPIO4);
  gpio_clear(GPIOA, GPIO5);
  gpio_clear(GPIOA, GPIO15);

  gpio_mode_setup(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_NONE , GPIO0);//color output

}

void disable_stepper()
{
  gpio_set(GPIOF, GPIO1);
}

void enable_stepper()
{
  gpio_clear(GPIOF, GPIO1);
}

//get top end stop
int get_top_es()
{
  return gpio_get(GPIOA, GPIO7);
}

//get bottom end stop
int get_bottom_es()
{
  return gpio_get(GPIOA, GPIO6);
}

void set_stepper_dir(int dir)
{
  if(dir == STEPPER_DOWN)
    gpio_clear(GPIOA, GPIO15);
  else
    gpio_set(GPIOA, GPIO15);
}

void step_toggle()
{
  gpio_toggle(GPIOA, GPIO8);
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


void reach_top()
{
  enable_stepper();
  set_stepper_dir(STEPPER_UP);//go up
  while(!get_top_es())
  {
    step_toggle();
    delay_us(500);
  }
}

void reach_down()
{
  enable_stepper();
  set_stepper_dir(STEPPER_DOWN);//go down
  while(!get_bottom_es())
  {
    step_toggle();
    delay_us(500);
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

void set_pump(uint8_t status)
{
  if(status)
    gpio_set(GPIOA, GPIO9);
  else
    gpio_clear(GPIOA, GPIO9);
}

void set_valve(uint8_t status)
{
  if(status)
    gpio_set(GPIOA, GPIO10);
  else
    gpio_clear(GPIOA, GPIO10);
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

void pv_test_loop()
{
  gpio_set(GPIOA, GPIO9);
  while(1)
  {
    gpio_clear(GPIOA, GPIO10);
    delay_ms(1000);
    gpio_set(GPIOA, GPIO10);
    int i;
    for(i=0; i<5; i++)
    {
      uart_send_int(adc_read());
      uart_send_string("\r\n");
      delay_ms(1000);
    }
  }

}


uint32_t get_dt_color()
{
  while(!gpio_get(GPIOA, GPIO0))//wait for down
  {
    ;
  }
  uint32_t t0 = get_systicks();


  while(gpio_get(GPIOA, GPIO0) && (get_systicks()-t0) < 500)//wait for up
  {
    ;
  }
  while(!gpio_get(GPIOA, GPIO0) && (get_systicks()-t0) < 500)//wait for down
  {
    ;
  }

  uint32_t t1 = get_systicks();

  return t1-t0;
}

void get_color(uint8_t *dt_red, uint8_t *dt_blue, uint8_t *dt_green)
{
  gpio_clear(GPIOA, GPIO2);
  gpio_set(GPIOA, GPIO3);

  gpio_clear(GPIOA, GPIO4);
  gpio_clear(GPIOA, GPIO5);
  *dt_red = get_dt_color();

  gpio_clear(GPIOA, GPIO4);
  gpio_set(GPIOA, GPIO5);
  *dt_blue = get_dt_color();

  gpio_set(GPIOA, GPIO4);
  gpio_set(GPIOA, GPIO5);
  *dt_green = get_dt_color();
}


void color_test_loop()
{
  while(1)
  {
    gpio_clear(GPIOA, GPIO2);
    gpio_set(GPIOA, GPIO3);

    gpio_clear(GPIOA, GPIO4);
    gpio_clear(GPIOA, GPIO5);
    uint32_t dt_red = get_dt_color();

    gpio_clear(GPIOA, GPIO4);
    gpio_set(GPIOA, GPIO5);
    uint32_t dt_blue = get_dt_color();

    gpio_set(GPIOA, GPIO4);
    gpio_set(GPIOA, GPIO5);
    uint32_t dt_green = get_dt_color();

    uart_send_string("\n\rred: ");
    uart_send_int(dt_red);
    uart_send_string("\n\rblue: ");
    uart_send_int(dt_blue);
    uart_send_string("\n\rgreen: ");
    uart_send_int(dt_green);
  }


}
