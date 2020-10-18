#pragma once

#include "libcanard/libcanard/canard.h"

typedef struct{
  CanardInstance can_ins;

  CanardRxSubscription z_in_subscription;
  uint8_t z_out_transfer_id;

  CanardRxSubscription z_pump_subscription;
  uint8_t pump_order;

  CanardRxSubscription z_valve_subscription;
  uint8_t valve_order;

  CanardRxSubscription z_flag_subscription;
  uint8_t flag_order;
  uint8_t flag_order_sent;

}global_data;
