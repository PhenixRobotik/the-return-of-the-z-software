#include "uart_AX.h"

#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>

#include <lowlevel/ax_12a.h>
#include <lowlevel/clock.h>



void ax_uart_setup()
{
  // Open GPIO for USART
  rcc_periph_clock_enable(AX_PORT_TX_RCC);
  gpio_mode_setup(AX_PORT_TX, GPIO_MODE_AF, GPIO_PUPD_NONE, AX_PIN_TX);
  gpio_set_af(AX_PORT_TX, AX_AF_TX, AX_PIN_TX);

  rcc_periph_clock_enable(AX_PORT_RX_RCC);
  gpio_mode_setup(AX_PORT_RX, GPIO_MODE_AF, GPIO_PUPD_NONE, AX_PIN_RX);
  gpio_set_af(AX_PORT_RX, AX_AF_RX, AX_PIN_RX);

  rcc_periph_clock_enable(AX_RCC_USART);

  usart_disable(AX_USART);

  usart_set_baudrate(AX_USART, AX_UART_SPEED);
  usart_set_databits(AX_USART, 8);
  usart_set_stopbits(AX_USART, USART_STOPBITS_1);
  usart_set_mode(AX_USART, USART_MODE_TX_RX);
  usart_set_parity(AX_USART, USART_PARITY_NONE);
  usart_set_flow_control(AX_USART, USART_FLOWCONTROL_NONE);

  usart_enable(AX_USART);
}

void ax_uart_send_string(char* chain)
{
  for (int i = 0; chain[i] != 0; i++) {
    usart_send_blocking(AX_USART, chain[i]);
  }
}




uint8_t send(uint8_t *buff, uint16_t len, uint32_t timeout)
{
  for (int i = 0; i<len; i++)
  {
    usart_send_blocking(AX_USART, buff[i]);
  }
  return 0;
}

uint8_t receive(uint8_t *buff, uint16_t len, uint32_t timeout)
{
  return 0;
}

void set_direction(AX_Direction dir)
{
  if( dir == AX_SEND)
  {
    delay_us(100);
    gpio_set(GPIOB, GPIO5);
    delay_us(100);
  }
  else
  {
    delay_us(100);
    gpio_clear(GPIOB, GPIO5);
    delay_us(100);
  }
}


void ax_uart_test_loop()
{
  AX_Interface interface;

  interface.receive = receive;
  interface.send = send;
  interface.set_direction = set_direction;
  interface.delay = delay_ms;

  AX servo;
  servo.id = 0x01;
  servo.interface = &interface;

  //gpio_set(GPIOB, GPIO5);
  while(1)
  {
    //ax_uart_send_string("Hello\n\r");
    AX_Set_LED( &servo, 1, AX_NOW);
    delay_ms(1000);
    AX_Set_LED( &servo, 0, AX_NOW);
    delay_ms(1000);
  }
}
