#pragma once

#include "libcanard/libcanard/canard.h"

typedef struct{
  CanardInstance can_ins;

  CanardRxSubscription z_in_subscription;
  uint8_t z_out_transfer_id;

  CanardRxSubscription z_pump_subscription;
  uint8_t pump_order;

}global_data;
