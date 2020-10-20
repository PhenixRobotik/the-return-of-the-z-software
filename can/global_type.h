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
  uint8_t flag_status;

  uint32_t adc_value;

  CanardRxSubscription z_arm_subscription;
  uint16_t arm_order;
  uint8_t arm_order_sent;
  uint16_t arm_status;

  CanardRxSubscription z_angle_subscription;
  uint16_t angle_order;
  uint8_t angle_order_sent;
  uint16_t angle_status;

}global_data;
