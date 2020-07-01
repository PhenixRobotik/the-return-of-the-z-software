#include "lowlevel/theyseemerolling.h"
#include "lowlevel/clock.h"
#include "lowlevel/uart.h"
#include "lowlevel/uart_AX.h"
#include "lowlevel/motors.h"

#include "fsm/fsm_asser.h"
#include "asservissement/odometry.h"
#include "asservissement/pid.h"
#include "asservissement/calibration.h"


int main() {
  clock_setup();
  gpio_setup();
  uart_setup();
  ax_uart_setup();


  //led_test_loop();
  led_set_status(1);

  ax_uart_test_loop();
  //uart_test_loop();
  //ax_uart_test_loop();
  //stepper_test_loop();
  //pump_test_loop();
  //valve_test_loop();


  while (1) {
    //led_toggle_status();
    //delay_ms(2000);
    //gpio_toggle(GPIOB, GPIO1);//nenable

  }
  return 0;
}
