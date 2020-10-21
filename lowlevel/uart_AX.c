#include "uart_AX.h"

#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/cm3/nvic.h>

#include <lowlevel/ax_12a.h>
#include <lowlevel/clock.h>

//sorry for this
static AX_Interface interface;
static AX servo;
static AX servo_flag;
static AX servo_arm;


//functions for the AX interface
static uint8_t send(uint8_t *buff, uint16_t len, uint32_t timeout)
{
  for (int i = 0; i<len; i++)
  {
    usart_send_blocking(AX_USART, buff[i]);
  }
  return 0;
}

static volatile uint8_t *rx_buff;
static volatile uint8_t rx_size;
static volatile uint8_t rx_size_max;
static volatile uint8_t got_spamed;

void usart2_exti26_isr(void)
{
  uint8_t data = usart_recv(AX_USART);
  if(rx_size<rx_size_max)
  {
    rx_buff[rx_size] = data;
    rx_size += 1;
  }
  else
  {
    usart_disable_rx_interrupt(AX_USART);//we are spamed stop it now!
    got_spamed = 1;
  }
  USART_CR1(USART2) &= ~USART_ISR_CTSIF;//clear the interrupt
}

static uint8_t receive(uint8_t *buff, uint16_t len, uint32_t timeout)
{
  rx_buff = buff;
  rx_size = 0;
  rx_size_max = len;
  got_spamed = 0;
  uint32_t t0 = get_systicks();
  usart_enable_rx_interrupt(AX_USART);
  delay_ms(timeout);
  usart_disable_rx_interrupt(AX_USART);
  if(got_spamed)
  {
    usart_disable(AX_USART);
    usart_enable(AX_USART);
    return 1;
  }
  if(rx_size != len)
    return 1;
  return 0;
}


static void set_direction(AX_Direction dir)
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

  nvic_enable_irq(NVIC_USART2_EXTI26_IRQ);

  usart_enable(AX_USART);



  interface.receive = receive;
  interface.send = send;
  interface.set_direction = set_direction;
  interface.delay = delay_ms;

  servo.id = 0x03;
  servo.interface = &interface;

  servo_flag.id = 0x02;
  servo_flag.interface = &interface;


  servo_arm.id = 0x04;
  servo_arm.interface = &interface;

  //AX_Configure_ID(&servo_arm, 0x04);
  //AX_Configure_Angle_Limit(&servo, 0, 0x3FF);

}


void ax_uart_set_left()
{
  AX_Set_Goal_Position(&servo, 200, AX_NOW);
}

void ax_uart_set_right()
{
  AX_Set_Goal_Position(&servo, 800, AX_NOW);
}

void ax_uart_set_center()
{
  AX_Set_Goal_Position(&servo, 512, AX_NOW);
}




void flag_out()
{
  AX_Set_Goal_Position(&servo_flag, 200, AX_NOW);
}

void flag_in()
{
  AX_Set_Goal_Position(&servo_flag, 511, AX_NOW);
}

void flag_set(uint8_t status)
{
  if(status)
    flag_out();
  else
    flag_in();
}

uint8_t flag_get()
{
  uint16_t position;
  if(AX_Get_Current_Position(&servo_flag, &position) == 1)
    return 0;
  if(position<300)
    return 1;
  return 0;
}



void arm_set(uint16_t value)
{
  if(value > 800)
    value = 800;
  if(value < 325)
    value = 325;
  AX_Set_Goal_Position(&servo_arm, value, AX_NOW);
}

uint16_t arm_get()
{
  uint16_t position;
  if(AX_Get_Current_Position(&servo_arm, &position) == 1)
    return 1024;
  return position;
}


void angle_set(uint16_t value)
{
  if(value > 840)
    value = 840;
  if(value < 180)
    value = 180;
  AX_Set_Goal_Position(&servo, value, AX_NOW);
}

uint16_t angle_get()
{
  uint16_t position;
  if(AX_Get_Current_Position(&servo, &position) == 1)
    return 1024;
  return position;
}



void ax_uart_test_loop()
{

  //gpio_set(GPIOB, GPIO5);

  //Range : [0x00, 0x3FF] (0°, 300°);
  //AX_Configure_Angle_Limit(&servo, 0, 0x3FF);


  AX_Set_LED( &servo_arm, 1, AX_NOW);
  AX_Set_Goal_Speed_Join(&servo, 100, AX_NOW);
  AX_Set_Goal_Position(&servo_arm, 511, AX_NOW);


  AX_Set_LED( &servo_flag, 1, AX_NOW);
  AX_Set_Goal_Speed_Join(&servo, 300, AX_NOW);

  uint16_t position;
  while(1)
  {
    flag_out();
    AX_Set_LED( &servo, 1, AX_NOW);
    ax_uart_set_left();
    delay_ms(2000);
    AX_Get_Current_Position(&servo, &position);
    uart_send_string("main\n");
    uart_send_int(position);

    flag_in();
    ax_uart_set_right();
    AX_Set_LED( &servo, 0, AX_NOW);
    delay_ms(2000);
    AX_Get_Current_Position(&servo, &position);
    uart_send_string("main\n");
    uart_send_int(position);

    ax_uart_set_center();
    delay_ms(2000);
    AX_Get_Current_Position(&servo, &position);
    uart_send_string("main\n");
    uart_send_int(position);
  }
}
