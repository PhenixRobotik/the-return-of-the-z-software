#include <libopencm3/stm32/timer.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/rcc.h>

#include "clock.h"

#include "stepper.h"

static volatile int32_t goal_;
static volatile int32_t pos_;
static volatile int32_t goal_masked_;
static volatile int32_t goal_filtered_;

void setup_stepper_interrupt()
{
	rcc_periph_clock_enable(STEPPER_RCC_TIM);
	rcc_periph_reset_pulse(STEPPER_RST_TIM);
	timer_set_mode(STEPPER_TIM, TIM_CR1_CKD_CK_INT,TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
	timer_set_prescaler(STEPPER_TIM, STEPPER_TIM_PRESCALER);
	timer_disable_preload(STEPPER_TIM);
	timer_continuous_mode(STEPPER_TIM);
	timer_set_period(STEPPER_TIM, STEPPER_TIM_PERIOD);
	timer_set_oc_value(STEPPER_TIM, TIM_OC1,0);//compare to zero
	timer_enable_counter(STEPPER_TIM);
  nvic_enable_irq(STEPPER_NVIC_TIM_IRQ);

  pos_ = 0;
}

void stepper_set(int32_t goal)
{
  goal_ = goal;
  goal_masked_ = goal_ & 0xC0000000;
  goal_filtered_ = goal_ & 0x3FFFFFFF;

  if(goal_masked_ == (int32_t)0xC0000000)
  {
    timer_disable_irq(STEPPER_TIM, TIM_DIER_CC1IE);
    disable_stepper();
  }
  else
  {
    enable_stepper();
    timer_enable_irq(STEPPER_TIM, TIM_DIER_CC1IE);
  }

}

int32_t stepper_get()
{
  int32_t ret = pos_;
  if(get_top_es())
    ret = ret | 0x80000000;
  if(get_bottom_es())
    ret = ret | 0x40000000;
  return ret;
}

void timX_isr(void)
{
  if (timer_get_flag(STEPPER_TIM, TIM_SR_CC1IF))
  {
    // Clear compare interrupt flag.
    timer_clear_flag(STEPPER_TIM, TIM_SR_CC1IF);

    if(goal_masked_ == (int32_t)0x80000000)//reach top
    {
      if(!get_top_es())
      {
        pos_++;
        set_stepper_dir(STEPPER_UP);
        step_toggle();
      }
      else
      {
        timer_disable_irq(STEPPER_TIM, TIM_DIER_CC1IE);
      }
    }

    else if(goal_masked_ == 0x40000000)//rech bottom and init pos
    {
      if(!get_bottom_es())
      {
        pos_--;
        set_stepper_dir(STEPPER_DOWN);
        step_toggle();
      }
      else
      {
        pos_ = 0;
        timer_disable_irq(STEPPER_TIM, TIM_DIER_CC1IE);
      }
    }

    else//go where we need
    {
      if(goal_filtered_ < pos_)//we need to down
      {
        if(!get_bottom_es())//check end stop
        {
          pos_--;
          set_stepper_dir(STEPPER_DOWN);
          step_toggle();
        }
        else
        {
          pos_ = 0;
          timer_disable_irq(STEPPER_TIM, TIM_DIER_CC1IE);
        }
      }
      else if(goal_filtered_ > pos_)//we need to go up
      {
        if(!get_top_es())//check end stop
        {
          pos_++;
          set_stepper_dir(STEPPER_UP);
          step_toggle();
        }
        else
        {
          timer_disable_irq(STEPPER_TIM, TIM_DIER_CC1IE);
        }
      }
      else
      {
        timer_disable_irq(STEPPER_TIM, TIM_DIER_CC1IE);
      }
    }


  }
}
