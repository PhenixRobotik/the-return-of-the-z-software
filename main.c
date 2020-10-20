#include "lowlevel/theZ.h"
#include "lowlevel/clock.h"
#include "lowlevel/uart.h"
#include "lowlevel/uart_AX.h"
#include "can/can.h"
#include "can/can_defines.h"

#include "actions/actions.h"

#include <stdlib.h>
#include "can/canard_link.h"
#include "can/can_defines.h"

#include <libopencm3/stm32/can.h>

//#define RX_CAN_ID 0x002E002B
#define RX_CAN_ID ((CAN_ID_Z<<16) + CAN_ID_RPI)
#define CAN_ID_MASK (0x000F00FF)

static global_data data_g;

void can_rx_handler(uint8_t fifo, uint8_t pending, bool full, bool overrun)
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

  if((id & CAN_ID_MASK) != RX_CAN_ID)
  {
    // All you had to do was filtering the damn trame, ST !
    return;
  }

  CanardFrame received_frame;
  received_frame.timestamp_usec = 0;
  received_frame.extended_can_id = id;
  received_frame.payload_size = len;
  received_frame.payload = data;

  int result = process_can_rx(&data_g, &received_frame);

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
  init_can_link(&data_g);


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


  while (1)
  {
    //pump block
    set_pump(data_g.pump_order);
    set_valve(data_g.valve_order);
    data_g.adc_value = adc_read();

    //flag block
    if(!data_g.flag_order_sent)
    {
      data_g.flag_order_sent = 1;
      flag_set(data_g.flag_order);
    }
    data_g.flag_status = flag_get();

    //arm block
    if(!data_g.arm_order_sent)
    {
      data_g.arm_order_sent = 1;
      arm_set(data_g.arm_order);
    }
    data_g.arm_status = arm_get();

    //angle block
    if(!data_g.angle_order_sent)
    {
      data_g.angle_order_sent = 1;
      angle_set(data_g.angle_order);
    }
    data_g.angle_status = angle_get();


    (void) tx_feed_back( &data_g);
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
