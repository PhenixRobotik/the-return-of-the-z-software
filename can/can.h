#ifndef CAN_H
#define CAN_H

#include <stdint.h>
#include <stdbool.h>

#include <libopencm3/stm32/can.h>

#define CAN_ENABLE_IRQ_RX_0 1
#define CAN_ENABLE_IRQ_RX_1 0
#define CAN_ENABLE_IRQ_TX   0
#define CAN_ENABLE_IRQ_ERR  1

int can_setup(void);
/**
 * Setup and start the bxCAN peripheral.
 * The CAN GPIOs must be already configured.
 */

/* User interrupt handlers */
void can_error_handler(uint32_t can_esr);
/**
 * It is triggered on a bit set in the CAN Error Status Register, on
 * wakeup and entry to sleep mode. See libopencm3/stm32/can.h for
 * CAN_ESR values.
 */
// TODO : helper to check for sleep mode, wakeup condition

void can_rx_handler(uint8_t fifo, uint8_t pending, bool full, bool overrun);
/**
 * Tiggered when :
 * - A message have been received. The number of pending messages is @pending.
 * - The FIFO is full, @full is true.
 * - The FIFO was overrun, @overrun is true.
 */

void can_tx_handler(uint8_t mailbox);
/**
 * Triggered when a pending message is being sent, ie. its associated
 * mailbox becomes empty. The 3 lsb bits of mailbox gives the
 * completed status of the three corresponding mailboxes in bxCAN,
 * ie. bit 2 of mailbox at 1 means the mailbox2 is empty.
 */

/* CAN_ESR helpers */
typedef enum CAN_Error_Code_E{
  CAN_NO_ERROR,
  CAN_STUFF_ERROR,
  CAN_FORM_ERROR,
  CAN_ACK_ERROR,
  CAN_RECESSIVE_ERROR,
  CAN_DOMINANT_ERROR,
  CAN_CRC_ERROR,
  CAN_SOFT_ERROR // set by software
}CAN_Error_Code;
#define CAN_RX_ERROR_COUNTER(can_esr) ((uint8_t)(((can_esr) & CAN_ESR_REC_MASK) >> 24))
#define CAN_TX_ERROR_COUNTER(can_esr) ((uint8_t)(((can_esr) & CAN_ESR_TEC_MASK) >> 16))
#define CAN_ERROR_BUSOFF(can_esr)     ((uint8_t)(((can_esr) & CAN_ESR_BOFF)     >>  2))
#define CAN_ERROR_PASSIVE(can_esr)    ((uint8_t)(((can_esr) & CAN_ESR_EPVF)     >>  1))
#define CAN_ERROR_WARNING(can_esr)    ((uint8_t)(((can_esr) & CAN_ESR_EWGF)     >>  0))
#define CAN_LAST_ERROR_CODE(can_esr)  ((CAN_Error_Code) ((((can_esr) >> 2) & 0b111)))

#endif
