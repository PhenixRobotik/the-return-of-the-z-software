#include "lowlevel/theZ.h"
#include "lowlevel/clock.h"
#include "lowlevel/uart.h"
#include "lowlevel/uart_AX.h"

#include "actions/actions.h"


int main() {
  clock_setup();
  gpio_setup();
  adc_setup();
  uart_setup();
  ax_uart_setup();

  led_set_status(1);



  //init_Z();

  //led_test_loop();
  //color_test_loop();
  //ax_uart_test_loop();
  //uart_test_loop();
  //adc_test_loop();
  //stepper_test_loop();
  //pump_test_loop();
  //valve_test_loop();


  while (1) {

    reach_top();
    ax_uart_set_left();
    delay_ms(1000);
    ax_uart_set_right();
    delay_ms(2000);
    ax_uart_set_center();
    delay_ms(1000);

    reach_down();
    ax_uart_set_left();
    delay_ms(1000);
    ax_uart_set_right();
    delay_ms(2000);
    ax_uart_set_center();
    delay_ms(1000);

    //led_toggle_status();
    //delay_ms(2000);
    //gpio_toggle(GPIOB, GPIO1);//nenable

  }
  return 0;
}
