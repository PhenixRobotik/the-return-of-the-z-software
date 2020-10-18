#pragma once

#define CAN_SPEED 500000

enum CAN_ID {
    CAN_ID_RPI  = 3,
    CAN_ID_TSMR = 4,
    CAN_ID_UPS  = 5,
    CAN_ID_Z    = 6,
};

enum MessageType {
  Z_TEXT_GET  = (CAN_ID_Z << 8) +   1,
  Z_TEXT_SET  = (CAN_ID_Z << 8) +   2,
  Z_PUMP_GET  = (CAN_ID_Z << 8) +   3,
  Z_PUMP_SET  = (CAN_ID_Z << 8) +   4,
  Z_VALVE_GET = (CAN_ID_Z << 8) +   5,
  Z_VALVE_SET = (CAN_ID_Z << 8) +   6,
  Z_ZPOS_GET  = (CAN_ID_Z << 8) +   7,
  Z_ZPOS_SET  = (CAN_ID_Z << 8) +   8,
  Z_ANGLE_GET = (CAN_ID_Z << 8) +   9,
  Z_ANGLE_SET = (CAN_ID_Z << 8) +  10,
  FLAGGY_GET  = (CAN_ID_Z << 8) +  11,
  FLAGGY_SET  = (CAN_ID_Z << 8) +  12,
  ARM_GET     = (CAN_ID_Z << 8) +  13,
  ARM_SET     = (CAN_ID_Z << 8) +  14,

  Z_VERSION   = (CAN_ID_Z << 8) + 255,
};
