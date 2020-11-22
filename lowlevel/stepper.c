#include <libopencm3/stm32/timer.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/rcc.h>
#include <stdlib.h>
#include <math.h>

#include "uart.h"
#include "clock.h"

#include "stepper.h"

#define T1 0.3
#define T2 0.3
//change this if you don't want to count in steps
#define dx_step 1


static volatile int32_t goal_;
static volatile int32_t pos_;
static volatile int32_t goal_masked_;
static volatile int32_t goal_filtered_;

static volatile double initial_pos;
static volatile double t1, t2, t3;
static volatile double a1, a2;
static volatile double x1, x2;
static volatile double v;

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
		timer_set_period(STEPPER_TIM, STEPPER_TIM_PERIOD);//restore default speed

		if(goal_masked_ == 0)//not an endstop hit
		{
			//set direction
			if(goal_filtered_ < pos_)//we need to go down
				set_stepper_dir(STEPPER_DOWN);
			else
				set_stepper_dir(STEPPER_UP);

			initial_pos = pos_;
			double dx = fabs((double)goal - (double)initial_pos);


			//compute trapezoid parameters
			v = 4266;//steps per second, always positive

			t3 = dx/(v*(1-0.5*T1-0.5*T2));
			t1 = T1*t3;
			t2 = t3*(1-T2);

			a1 = v/t1;
			a2 = v/(t3-t2);

			x1 = 0.5*a1*(t1*t1);
			x2 = v*(t2-t1)+x1;

			/*
			uart_send_string("t1: ");
			uart_send_int(t1*10);
			uart_send_string("\n");
			uart_send_string("t2: ");
			uart_send_int(t2*10);
			uart_send_string("\n");
			uart_send_string("t3: ");
			uart_send_int(t3*10);
			uart_send_string("\n");

			uart_send_string("x1: ");
			uart_send_int(x1);
			uart_send_string("\n");
			uart_send_string("x2: ");
			uart_send_int(x2);
			uart_send_string("\n");
			*/
		}



    enable_stepper();
    timer_enable_irq(STEPPER_TIM, TIM_DIER_CC1IE);
  }

}

int32_t get_timer_period()
{
	double period;
	double dx = fabs((double)pos_ - (double)initial_pos);
	if (dx<x1)
    period = dx_step / sqrt(2*(dx+dx_step)*a1);
  else if (dx<x2)
    period = dx_step / v;
  else
	{
    double tmp = 2*a2*(fabs((double)goal_-(double)pos_)+dx_step);
    period = dx_step / sqrt(tmp);
	}
	return period * 64*1e+6 / 2 / STEPPER_TIM_PRESCALER;//convertion to timer period
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
					timer_set_period(STEPPER_TIM, get_timer_period());
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
					timer_set_period(STEPPER_TIM, get_timer_period());
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
