#include "lowlevel/theZ.h"
#include "lowlevel/clock.h"
#include "lowlevel/uart.h"
#include "lowlevel/uart_AX.h"
#include "can/can.h"

#include "actions/actions.h"

#include "libcanard/libcanard/canard.h"
#include <stdlib.h>

#include <libopencm3/stm32/can.h>

#define RX_CAN_ID 0x0000002B
#define CAN_ID_MASK 0x000000FF

static CanardInstance ins;
static CanardRxSubscription my_service_subscription;

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
  CanardTransfer transfer;
  const int8_t result = canardRxAccept(&ins,
                                       &received_frame,            // The CAN frame received from the bus.
                                       0,  // If the transport is not redundant, use 0.
                                       &transfer);
  uart_send_string("ID\n");
  uart_send_int(id);
  uart_send_string("ID\n");
  uart_send_int(received_frame.extended_can_id);
  uart_send_string("ID\n");
  if (result < 0)
  {
    uart_send_string("Got an error\n");
  }
  else if (result == 1)
  {
    uart_send_string("Go something\n");
    for(int i=0; i<transfer.payload_size; i++)
    {
      char to_send[2];
      to_send[0] = ((char*)transfer.payload)[i];
      to_send[1] = 0;
      uart_send_string(to_send);
    }
    uart_send_string("\n");

    ins.memory_free(&ins, (void*)transfer.payload);  // Deallocate the dynamic memory afterwards.
  }
  else
  {
    uart_send_string("nothing\n");

  }
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



static void* memAllocate(CanardInstance* const ins, const size_t amount)
{
    (void) ins;
    //return o1heapAllocate(my_allocator, amount);
    return malloc(amount);
}

static void memFree(CanardInstance* const ins, void* const pointer)
{
    (void) ins;
    //o1heapFree(my_allocator, pointer);
    free(pointer);
}

void canard_test()
{
  ins = canardInit(&memAllocate, &memFree);
  ins.mtu_bytes = CANARD_MTU_CAN_CLASSIC;  // Defaults to 64 (CAN FD); here we select Classic CAN.
  ins.node_id   = 42;                      // Defaults to anonymous; can be set up later at any point.

  (void) canardRxSubscribe(&ins,   // Subscribe to an arbitrary service response.
                         CanardTransferKindMessage,  // Specify that we want service responses, not requests.
                         1230,    // The Service-ID whose responses we will receive.
                         128,   // The extent (the maximum payload size); pick a huge value if not sure.
                         CANARD_DEFAULT_TRANSFER_ID_TIMEOUT_USEC,
                         &my_service_subscription);


  static uint8_t my_message_transfer_id;  // Must be static or heap-allocated to retain state between calls.

  while(1)
  {
    const CanardTransfer transfer = {
        .timestamp_usec = 0,      // Zero if transmission deadline is not limited.
        .priority       = CanardPriorityNominal,
        .transfer_kind  = CanardTransferKindMessage,
        .port_id        = 1233,                       // This is the subject-ID.
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
    }
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

  canard_test();

  uint8_t buffer[8];
  buffer[0] = 3;
  while(1)
  {
    led_toggle_status();
    can_transmit(CAN1, 1233, 0, 0, 1, buffer);
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
