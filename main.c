#include "lowlevel/theZ.h"
#include "lowlevel/clock.h"
#include "lowlevel/uart.h"
#include "lowlevel/uart_AX.h"
#include "can/can.h"

#include "actions/actions.h"

#include <libopencm3/stm32/can.h>


void can_rx_handler(uint8_t fifo, uint8_t pending, bool full, bool overrun)//mdr (issou)
{
  bool ext, rtr;
  uint8_t fmi, len = 0;
  uint16_t timestamp;
  uint32_t id;
  uint8_t data[8];

  // Receive the Great bits of the CAN, the First of their Frame,
  // Breaker of the CAN filters, Slayer of CPU time
  can_receive(CAN1,
	      0,
	      true, // release FIFO
	      &id,
	      &ext,
	      &rtr,
	      &fmi,
	      &len,
	      data,
	      &timestamp);

/*
  uart_send_string("\n\ndelimiter\n");
  uart_send_int(id);
  uart_send_string(" ext:");
  uart_send_int(ext);
  uart_send_string(" rtr:");
  uart_send_int(rtr);
  uart_send_string(" len:");
  uart_send_int(len);
  uart_send_string(" data:");
  for(int i=0; i<len; i++)
  {
    uart_send_string("-");
    uart_send_int(data[i]);
  }
  */

  // Don't care
  (void)full;
  (void)overrun;
  (void)ext;
  (void)rtr;
  (void)fmi;
  (void)timestamp;
}

int main() {
  clock_setup();
  gpio_setup();
  adc_setup();
  uart_setup();
  ax_uart_setup();
  can_setup();
  led_set_status(1);



  //init_Z();

  //led_test_loop();
  //color_test_loop();
  //ax_uart_test_loop();
  //uart_test_loop();
  //adc_test_loop();
  //stepper_test_loop();
  //pump_test_loop();
  //valve_test_loop();
  //pv_test_loop();

  uint8_t buffer[8];
  buffer[0] = 3;
  while(1)
  {
    led_toggle_status();
    can_transmit(CAN1 , 1233, 0, 0, 1, buffer);
    delay_ms(500);
  }

  while (1) {

    reach_top();
    ax_uart_set_left();
    delay_ms(1000);
    ax_uart_set_right();
    delay_ms(2000);
    ax_uart_set_center();
    delay_ms(1000);

    reach_down();
    ax_uart_set_left();
    delay_ms(1000);
    ax_uart_set_right();
    delay_ms(2000);
    ax_uart_set_center();
    delay_ms(1000);

    //led_toggle_status();
    //delay_ms(2000);
    //gpio_toggle(GPIOB, GPIO1);//nenable

  }
  return 0;
}
