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



void canard_test()
{

  while(1)
  {
    /*
    const CanardTransfer transfer = {
        .timestamp_usec = 0,      // Zero if transmission deadline is not limited.
        .priority       = CanardPriorityNominal,
        .transfer_kind  = CanardTransferKindMessage,
        .port_id        = Z_OUT,                       // This is the subject-ID.
        .remote_node_id = CANARD_NODE_ID_UNSET,       // Messages cannot be unicast, so use UNSET.
        .transfer_id    = my_message_transfer_id,
        .payload_size   = 11,
        .payload        = "hello stm32",
    };
    ++my_message_transfer_id;  // The transfer-ID shall be incremented after every transmission on this subject.
    int32_t result = canardTxPush(&ins, &transfer);

    for (const CanardFrame* txf = NULL; (txf = canardTxPeek(&ins)) != NULL;)  // Look at the top of the TX queue.
    {
      // Please ensure TX deadline not expired.
      // Send the frame. Redundant interfaces may be used here.

      can_transmit(CAN1, txf->extended_can_id, 1, 0, txf->payload_size, txf->payload);
                                   // If the driver is busy, break and retry later.
      canardTxPop(&ins);                         // Remove the frame from the queue after it's transmitted.
      ins.memory_free(&ins, (CanardFrame*)txf);  // Deallocate the dynamic memory afterwards.
    }*/
    led_toggle_status();
    delay_ms(500);
  }
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
    set_pump(data_g.pump_order);
    //TODO send back status
    delay_ms(1000);
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
