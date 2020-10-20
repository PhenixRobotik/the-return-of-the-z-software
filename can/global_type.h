#pragma once

#include "libcanard/libcanard/canard.h"

typedef struct{
  CanardInstance can_ins;

  uint8_t z_out_transfer_id;

  uint8_t pump_order;

  uint8_t valve_order;

  uint8_t flag_order;
  uint8_t flag_order_sent;
  uint8_t flag_status;

  uint32_t adc_value;

  uint16_t arm_order;
  uint8_t arm_order_sent;
  uint16_t arm_status;

  uint16_t angle_order;
  uint8_t angle_order_sent;
  uint16_t angle_status;

  uint8_t dt_red;
  uint8_t dt_blue;
  uint8_t dt_green;

  int32_t pos_order;
  uint8_t pos_order_sent;
  int32_t z_pos;

}global_data;
