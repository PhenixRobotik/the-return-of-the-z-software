#include "canard_link.h"
#include "can_defines.h"

#include <stdlib.h>
#include <libopencm3/stm32/can.h>

int z_out_transfer_id;
int z_pump_transfer_id;
int z_valve_transfer_id;
int z_flag_transfer_id;
int z_adc_transfer_id;
int z_arm_transfer_id;
int z_angle_transfer_id;


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
  z_out_transfer_id = 0;
  z_pump_transfer_id = 0;
  z_valve_transfer_id = 0;
  z_flag_transfer_id = 0;
  z_adc_transfer_id = 0;
  z_arm_transfer_id = 0;
  z_angle_transfer_id = 0;

  pdata->pump_order = 0;
  pdata->valve_order = 0;
  pdata->flag_order = 0;
  pdata->flag_order_sent = 1;
  pdata->arm_order = 0;
  pdata->arm_order_sent = 1;
  pdata->angle_order = 0;
  pdata->angle_order_sent = 1;

  pdata->can_ins = canardInit(&memAllocate, &memFree);
  pdata->can_ins.mtu_bytes = CANARD_MTU_CAN_CLASSIC;  // Defaults to 64 (CAN FD); here we select Classic CAN.
  pdata->can_ins.node_id   = CAN_ID_Z;

  (void) canardRxSubscribe(&pdata->can_ins,   // Subscribe to an arbitrary service response.
                         CanardTransferKindMessage,  // Specify that we want service responses, not requests.
                         Z_TEXT_SET,    // The Service-ID whose responses we will receive.
                         128,   // The extent (the maximum payload size); pick a huge value if not sure.
                         CANARD_DEFAULT_TRANSFER_ID_TIMEOUT_USEC,
                         &pdata->z_in_subscription);
  (void) canardRxSubscribe(&pdata->can_ins,   // Subscribe to an arbitrary service response.
                         CanardTransferKindMessage,  // Specify that we want service responses, not requests.
                         Z_PUMP_SET,    // The Service-ID whose responses we will receive.
                         1,   // The extent (the maximum payload size); pick a huge value if not sure.
                         CANARD_DEFAULT_TRANSFER_ID_TIMEOUT_USEC,
                         &pdata->z_pump_subscription);
  (void) canardRxSubscribe(&pdata->can_ins,   // Subscribe to an arbitrary service response.
                         CanardTransferKindMessage,  // Specify that we want service responses, not requests.
                         Z_VALVE_SET,    // The Service-ID whose responses we will receive.
                         1,   // The extent (the maximum payload size); pick a huge value if not sure.
                         CANARD_DEFAULT_TRANSFER_ID_TIMEOUT_USEC,
                         &pdata->z_valve_subscription);
  (void) canardRxSubscribe(&pdata->can_ins,
                         CanardTransferKindMessage,
                         FLAGGY_SET,
                         1,
                         CANARD_DEFAULT_TRANSFER_ID_TIMEOUT_USEC,
                         &pdata->z_flag_subscription);
  (void) canardRxSubscribe(&pdata->can_ins,
                         CanardTransferKindMessage,
                         ARM_SET,
                         2,
                         CANARD_DEFAULT_TRANSFER_ID_TIMEOUT_USEC,
                         &pdata->z_arm_subscription);
  (void) canardRxSubscribe(&pdata->can_ins,
                         CanardTransferKindMessage,
                         Z_ANGLE_SET,
                         2,
                         CANARD_DEFAULT_TRANSFER_ID_TIMEOUT_USEC,
                         &pdata->z_angle_subscription);

}

int canard_send_tx_queue(CanardInstance *pins)
{
  for (const CanardFrame* txf = NULL; (txf = canardTxPeek(pins)) != NULL;)  // Look at the top of the TX queue.
  {
    // Please ensure TX deadline not expired.
    // Send the frame. Redundant interfaces may be used here.

    //trying to send, -1 is sending queue full
    while(can_transmit(CAN1, txf->extended_can_id, 1, 0, txf->payload_size, txf->payload) == -1);
                                 // If the driver is busy, break and retry later.
    canardTxPop(pins);                         // Remove the frame from the queue after it's transmitted.
    pins->memory_free(pins, (CanardFrame*)txf);  // Deallocate the dynamic memory afterwards.
  }
  return 1;
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
  if( ptransfer->port_id == Z_TEXT_SET )
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
  else if( ptransfer->port_id == Z_VALVE_SET )
  {
    //uart_send_string("pump\n");
    if(ptransfer->payload_size != 1)
      return 0;
    pdata->valve_order = ((uint8_t *)ptransfer->payload)[0];
  }
  else if( ptransfer->port_id == FLAGGY_SET )
  {
    if(ptransfer->payload_size != 1)
      return 0;
    pdata->flag_order = ((uint8_t *)ptransfer->payload)[0];
    pdata->flag_order_sent = 0;
  }
  else if( ptransfer->port_id == ARM_SET )
  {
    if(ptransfer->payload_size != 2)
      return 0;
    pdata->arm_order = ((uint16_t *)ptransfer->payload)[0];
    pdata->arm_order_sent = 0;
  }
  else if( ptransfer->port_id == Z_ANGLE_SET )
  {
    if(ptransfer->payload_size != 2)
      return 0;
    pdata->angle_order = ((uint16_t *)ptransfer->payload)[0];
    pdata->angle_order_sent = 0;
  }
  else
  {
    return 0;
  }
  return 1;
}


int tx_feed_back(global_data *pdata)
{
  int32_t result;

  uint8_t byte;

  CanardTransfer transfer = {
      .timestamp_usec = 0,      // Zero if transmission deadline is not limited.
      .priority       = CanardPriorityNominal,
      .transfer_kind  = CanardTransferKindMessage,
      .remote_node_id = CANARD_NODE_ID_UNSET,       // Messages cannot be unicast, so use UNSET.
  };

  /*transfer.port_id        = Z_TEXT_GET;
  transfer.transfer_id    = z_out_transfer_id;
  transfer.payload_size   = 11;
  transfer.payload        = "hello stm32";
  ++z_out_transfer_id;  // The transfer-ID shall be incremented after every transmission on this subject.
  result = canardTxPush(&pdata->can_ins, &transfer);
  canard_send_tx_queue(&pdata->can_ins);*/

  byte = pdata->pump_order;
  transfer.port_id        = Z_PUMP_GET;
  transfer.transfer_id    = z_pump_transfer_id;
  //transfer.payload_size   = 1;
  transfer.payload        = &byte;
  ++z_pump_transfer_id;  // The transfer-ID shall be incremented after every transmission on this subject.
  result = canardTxPush(&pdata->can_ins, &transfer);
  canard_send_tx_queue(&pdata->can_ins);


  byte = pdata->valve_order;
  transfer.port_id        = Z_VALVE_GET;
  transfer.transfer_id    = z_valve_transfer_id;
  transfer.payload_size   = 1;
  transfer.payload        = &byte;
  ++z_valve_transfer_id;  // The transfer-ID shall be incremented after every transmission on this subject.
  result = canardTxPush(&pdata->can_ins, &transfer);
  canard_send_tx_queue(&pdata->can_ins);

  byte = pdata->flag_status;
  transfer.port_id        = FLAGGY_GET;
  transfer.transfer_id    = z_flag_transfer_id;
  //transfer.payload_size   = 1;
  transfer.payload        = &byte;
  ++z_flag_transfer_id;  // The transfer-ID shall be incremented after every transmission on this subject.
  result = canardTxPush(&pdata->can_ins, &transfer);
  canard_send_tx_queue(&pdata->can_ins);

  transfer.port_id        = Z_PRESS_GET;
  transfer.transfer_id    = z_adc_transfer_id;
  transfer.payload_size   = 4;
  transfer.payload        = &(pdata->adc_value);
  ++z_adc_transfer_id;  // The transfer-ID shall be incremented after every transmission on this subject.
  result = canardTxPush(&pdata->can_ins, &transfer);
  canard_send_tx_queue(&pdata->can_ins);

  transfer.port_id        = ARM_GET;
  transfer.transfer_id    = z_arm_transfer_id;
  transfer.payload_size   = 2;
  transfer.payload        = &(pdata->arm_status);
  ++z_arm_transfer_id;  // The transfer-ID shall be incremented after every transmission on this subject.
  result = canardTxPush(&pdata->can_ins, &transfer);
  canard_send_tx_queue(&pdata->can_ins);

  transfer.port_id        = Z_ANGLE_GET;
  transfer.transfer_id    = z_angle_transfer_id;
  transfer.payload_size   = 2;
  transfer.payload        = &(pdata->angle_status);
  ++z_angle_transfer_id;  // The transfer-ID shall be incremented after every transmission on this subject.
  result = canardTxPush(&pdata->can_ins, &transfer);
  canard_send_tx_queue(&pdata->can_ins);



  return 1;
}
