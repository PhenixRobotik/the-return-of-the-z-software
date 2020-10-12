#include "can.h"

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/f3/nvic.h>
#include <libopencm3/stm32/can.h>

#include "common_defs.h"

#include "link_can.h"

static void can_enable_irqs();

int can_setup(void){
  /* Enable bxCAN clock */
  rcc_periph_clock_enable(RCC_CAN1);

  /* Reset bxCAN */
  can_reset(CAN1);

  /* Init and start CAN */
  /**
   * Note on the bitrate :
   * Given the frequency of APB1 (f), we have :
   * bitrate = f / (nb_tq * prescaler)
   * So if f = 32MHz, and we have 16 TQ, a prescaler of 4 would mean 500 kbps
   */
  int err = can_init(CAN1,
		     false, // Time-triggered communication mode
		     true,  // Automatically recover from bus-off state
		     false, // Automatically wake up from sleep mode
		     false, // No automatic retransmission (false = automatic)
		     false, // FIFO lock function
		     true, // FIFO priority mode (false = id priority mode)
		     CAN_BTR_SJW_1TQ, // 1
		     CAN_BTR_TS1_9TQ, // 9
		     CAN_BTR_TS2_6TQ, // 6
		     4,
		     false, // Silent mode
		     false  // Loopback mode
    );
  // TODO : baud rate enum ?
  // TODO : add parameters for baud rate

  if(err){
    return -1;
  } else {
    // be happy :)
  }

  /* Set the CAN filter */
  #if 0
  can_filter_id_mask_32bit_init(0,      /* Filter ID */
				RX_CAN_ID, /* CAN ID */
				CAN_ID_MASK,  /* ID mask */
				CAN_RX_FIFO,      /* FIFO */
				true);
  #endif

  /* CAN filter 0 init */
  can_filter_id_mask_32bit_init(
    0,     /* Filter ID */
    0,     /* CAN ID */
    0,     /* CAN ID mask */
    0,     /* FIFO assignment (here: FIFO0) */
    true); /* Enable the filter. */


  /* Enable interrupts */
  can_enable_irqs();

  return 0;
}

static void can_enable_irqs() {
  /* For each interrupt, setup NVIC and set interrupt enable bits */

  // RX FIFO 0 interrupt
#if CAN_ENABLE_IRQ_RX_0 == 1
  nvic_enable_irq(NVIC_USB_LP_CAN1_RX0_IRQ);
  nvic_set_priority(NVIC_USB_LP_CAN1_RX0_IRQ, 2);

  can_enable_irq(CAN1, CAN_IER_FMPIE0);
  can_enable_irq(CAN1, CAN_IER_FFIE0);
  can_enable_irq(CAN1, CAN_IER_FOVIE0);
#endif

  // RX FIFO 1 interrupt
#if CAN_ENABLE_IRQ_RX_1 == 1
  nvic_enable_irq(NVIC_CAN1_RX1_IRQ);
  nvic_set_priority(NVIC_CAN1_RX1_IRQ, 2);

  can_enable_irq(CAN1, CAN_IER_FMPIE1);
  can_enable_irq(CAN1, CAN_IER_FFIE1);
  can_enable_irq(CAN1, CAN_IER_FOVIE1);
#endif

  // TX Interrupt
#if CAN_ENABLE_IRQ_TX == 1
  nvic_enable_irq(NVIC_USB_HP_CAN1_TX_IRQ);
  nvic_set_priority(NVIC_USB_HP_CAN1_TX_IRQ, 2);

  can_enable_irq(CAN1, CAN_IER_TMEIE);
#endif

  // Status Change and Errors interrupt
#if CAN_ENABLE_IRQ_ERR == 1
  nvic_enable_irq(NVIC_CAN1_SCE_IRQ);
  nvic_set_priority(NVIC_CAN1_SCE_IRQ, 2);

  can_enable_irq(CAN1, CAN_IER_ERRIE); // error detection
  can_enable_irq(CAN1, CAN_IER_EWGIE); // warning limit reached
  can_enable_irq(CAN1, CAN_IER_EPVIE); // passive limit reached
  can_enable_irq(CAN1, CAN_IER_BOFIE); // bus-off
  can_enable_irq(CAN1, CAN_IER_LECIE); // last error code
  // TODO : support Wakeup and Sleep in error handler
  //can_enable_irq(CAN1, CAN_IER_WKUIE);
  //can_enable_irq(CAN1, CAN_IER_SLKIE);
#endif

  // TODO : check priority relevance
}

/* System interrupt handlers */
void usb_hp_can1_tx_isr(void) {
  uint32_t tsr = CAN_TSR(BX_CAN1_BASE);

  /* Clear flags */
  if(IS_SET(tsr, CAN_TSR_RQCP0)) {
    CLEAR(CAN_TSR(BX_CAN1_BASE), CAN_TSR_RQCP0);
  }

  if(IS_SET(tsr, CAN_TSR_RQCP1)) {
    CLEAR(CAN_TSR(BX_CAN1_BASE), CAN_TSR_RQCP1);
  }

  if(IS_SET(tsr, CAN_TSR_RQCP2)) {
    CLEAR(CAN_TSR(BX_CAN1_BASE), CAN_TSR_RQCP2);
  }

  //TODO : check if the transmission succeeded or failed
  can_tx_handler((uint8_t) ((tsr & 1)                |    //mailbox 0 (@ bit 0 ->  bit 0)
			    ((tsr >> (8-1)) & 0b10)  |    //mailbox 1 (@ bit 8 ->  bit 1)
			    ((tsr >> (8*2 - 2)) & 0b100)) //mailbox 2 (@ bit 16 -> bit 2)
    );
}

static inline void can1_rx_isr(uint8_t fifo, uint32_t rfr){
  can_rx_handler(fifo,                            // Fifo number
		 (rfr & CAN_RF0R_FMP0_MASK) >> 0, // pending messages
		 (rfr & CAN_RF0R_FULL0)     >> 3, // is full
		 (rfr & CAN_RF0R_FOVR0)     >> 4);// is overrun
}

void usb_lp_can1_rx0_isr(void) {
  uint32_t rfr = CAN_RF0R(BX_CAN1_BASE);

  /* Clear flags */
  if(IS_SET(rfr, CAN_RF0R_FOVR0)) {
    CLEAR(CAN_RF0R(BX_CAN1_BASE), CAN_RF0R_FOVR0);
  }

  if(IS_SET(rfr, CAN_RF0R_FULL0)) {
    CLEAR(CAN_RF0R(BX_CAN1_BASE), CAN_RF0R_FULL0);
  }

  can1_rx_isr(0, rfr);
}

void can1_rx1_isr(void) {
  uint32_t rfr = CAN_RF1R(BX_CAN1_BASE);

  /* Clear flags */
  if(IS_SET(rfr, CAN_RF1R_FOVR1)) {
    CLEAR(CAN_RF1R(BX_CAN1_BASE), CAN_RF1R_FOVR1);
  }

  if(IS_SET(rfr, CAN_RF1R_FULL1)) {
    CLEAR(CAN_RF1R(BX_CAN1_BASE), CAN_RF1R_FULL1);
  }

  can1_rx_isr(1, rfr);
}

void can1_sce_isr(void) {
  uint32_t msr = CAN_MSR(BX_CAN1_BASE);

  /* Clear flags */
  if(CAN_LAST_ERROR_CODE(msr) != 0) {
    CLEAR(CAN_ESR(BX_CAN1_BASE), 0x70);
  }
  // ERRI
  CLEAR(CAN_MSR(BX_CAN1_BASE), 0x102);

  can_error_handler(CAN_ESR(BX_CAN1_BASE));
  //TODO : support sleep and wakeup
}

/* User interrupt handlers */
__attribute__((weak)) void can_error_handler(uint32_t can_esr);
__attribute__((weak)) void can_rx_handler(uint8_t fifo, uint8_t pending, bool full, bool overrun);
__attribute__((weak)) void can_tx_handler(uint8_t mailbox);
