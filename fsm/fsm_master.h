#pragma once

#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/timer.h>

#include "lowlevel/clock.h"

typedef enum FSM_Status_E{
  FSM_RUNNING,
  FSM_SUCCESS,
  FSM_ERROR
}FSM_Status;

struct FSM_Instance_S;
typedef void (*FSM_Procedure)(struct FSM_Instance_S*);

typedef struct FSM_Instance_S{
  FSM_Procedure run;
  FSM_Status status;
  uint32_t time_limit;
}FSM_Instance;

#define FSM_SET_TIME_BUDGET(p_instance, t_ms) {p_instance->time_limit = get_systicks() + MILLIS_TO_SYSTICK(t_ms);}
#define FSM_TIMEOUT_REACHED(p_instance) (get_systicks() > p_instance->time_limit)
#define FSM_NEXT(p_instance, next_state, t_ms) {p_instance->run = next_state; FSM_SET_TIME_BUDGET(p_instance, t_ms);}

void FSM_NOP(FSM_Instance *fsm);
