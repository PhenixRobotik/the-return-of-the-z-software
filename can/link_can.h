#ifndef LINK_CAN_H
#define LINK_CAN_H

#include <stdint.h>

#define LOCAL_ID     0x01
#define RAGEBERRY_ID 0x00
#define TX_CAN_ID ((uint32_t) ((((LOCAL_ID) & 0x3F) << 6) | ((RAGEBERRY_ID) & 0x3F)))
#define RX_CAN_ID ((uint32_t) ((((RAGEBERRY_ID) & 0x3F) << 6) | ((LOCAL_ID) & 0x3F)))
#define CAN_ID_MASK 0x7FF
/**
 * Note : The lower the CAN ID is, the higher the priority
 * is. Therefore we will set the local ID as most significant bits and
 * remote ID as least significant bits.
 */

// We'll receive in FIFO0 only
#define CAN_RX_FIFO 0

#include "robotronik_protocol.h"
extern RP_Interface pi_iface;

void setup_com();
uint8_t rp_send(void* link_handler, uint32_t can_id, uint8_t* data, uint16_t len, uint32_t timeout);

#endif
