#include "canard_link.h"
#include "can_defines.h"

#include <stdlib.h>

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

void init_can_link(global_data *pdata)
{
  pdata->pump_order = 0;

  pdata->can_ins = canardInit(&memAllocate, &memFree);
  pdata->can_ins.mtu_bytes = CANARD_MTU_CAN_CLASSIC;  // Defaults to 64 (CAN FD); here we select Classic CAN.
  pdata->can_ins.node_id   = CAN_ID_Z;

  (void) canardRxSubscribe(&pdata->can_ins,   // Subscribe to an arbitrary service response.
                         CanardTransferKindMessage,  // Specify that we want service responses, not requests.
                         Z_IN,    // The Service-ID whose responses we will receive.
                         128,   // The extent (the maximum payload size); pick a huge value if not sure.
                         CANARD_DEFAULT_TRANSFER_ID_TIMEOUT_USEC,
                         &pdata->z_in_subscription);
  (void) canardRxSubscribe(&pdata->can_ins,   // Subscribe to an arbitrary service response.
                         CanardTransferKindMessage,  // Specify that we want service responses, not requests.
                         Z_PUMP_SET,    // The Service-ID whose responses we will receive.
                         1,   // The extent (the maximum payload size); pick a huge value if not sure.
                         CANARD_DEFAULT_TRANSFER_ID_TIMEOUT_USEC,
                         &pdata->z_pump_subscription);
}


int process_can_rx(global_data *pdata, CanardFrame *preceived_frame)
{
  CanardTransfer transfer;
  const int8_t result = canardRxAccept(&pdata->can_ins,
                                       preceived_frame,            // The CAN frame received from the bus.
                                       0,  // If the transport is not redundant, use 0.
                                       &transfer);

  if (result < 0)
  {
    //uart_send_string("Got an error\n");
  }
  else if (result == 1)
  {
    decode_can_rx(pdata, &transfer);
    //uart_send_string("Got something\n");
    pdata->can_ins.memory_free(&pdata->can_ins, (void*)transfer.payload);  // Deallocate the dynamic memory afterwards.
  }
  else
  {
    //uart_send_string("nothing\n");

  }
  return 1;
}

int decode_can_rx(global_data *pdata, CanardTransfer *ptransfer)
{
  if( ptransfer->port_id == Z_IN )
  {
    for(int i=0; i<ptransfer->payload_size; i++)
    {
      char to_send[2];
      to_send[0] = ((char*)ptransfer->payload)[i];
      to_send[1] = 0;
      uart_send_string(to_send);
    }
    uart_send_string("\n");
  }
  else if( ptransfer->port_id == Z_PUMP_SET )
  {
    //uart_send_string("pump\n");
    if(ptransfer->payload_size != 1)
      return 0;
    pdata->pump_order = ((uint8_t *)ptransfer->payload)[0];
  }
  return 1;
}
