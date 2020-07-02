#include "actions.h"

#include <lowlevel/theZ.h>
#include <lowlevel/clock.h>

uint8_t init_Z()
{
  enable_stepper();

  set_stepper_dir(STEPPER_UP);
  while(!get_top_es())
  {
    step_toggle();
    delay_us(500);
  }

  return 0;
}
