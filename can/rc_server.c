#include "rc_server.h"

#include "link_can.h"
#include "fsm/fsm_asser.h"
#include "asservissement/odometry.h"
#include "main.h"

RC_Server server;

static const char fprots[NB_FUNS][2][RC_FMT_SIZE] =
  {
   [SET_STOP]              = {"",     ""   },
   [SET_THETA]             = {"F",    "u"   },
   [SET_THETA_SPEED]       = {"F",    "u"   },
   [SET_TRANSLATION]       = {"F",    "u"   },
   [SET_TRANSLATION_SPEED] = {"F",    "u"   },
   [SET_X_Y_THETA]         = {"FFFu", "u"   },
   [GET_ODOMETRY]          = {"",     "FFF"},
   [GET_STATE]             = {"",     "u"  },
   [ENABLE]                = {"",     ""},
   [DISABLE]               = {"",     ""}
  };

typedef void (*rc_fun)(struct RC_Server_S*);
void rc_set_stop(RC_Server*);
void rc_set_theta(RC_Server*);
void rc_set_theta_speed(RC_Server*);
void rc_set_translation(RC_Server*);
void rc_set_translation_speed(RC_Server*);
void rc_set_X_Y_theta(RC_Server*);
void rc_get_odometry(RC_Server*);
void rc_get_state(RC_Server*);
void rc_enable(RC_Server*);
void rc_disable(RC_Server*);

static rc_fun funs[NB_FUNS] = //mdr
  {
   [SET_STOP]              = rc_set_stop,
   [SET_THETA]             = rc_set_theta,
   [SET_THETA_SPEED]       = rc_set_theta_speed,
   [SET_TRANSLATION]       = rc_set_translation,
   [SET_TRANSLATION_SPEED] = rc_set_translation_speed,
   [SET_X_Y_THETA]         = rc_set_X_Y_theta,
   [GET_ODOMETRY]          = rc_get_odometry,
   [GET_STATE]             = rc_get_state,
   [ENABLE]                = rc_enable,
   [DISABLE]               = rc_disable
  };
typedef enum{PARAMS, RETURN}fprot_field;

typedef enum{ASSER_DONE, ASSER_BUSY}Asser_State;

int init_server(){
  RC_Server_Init(&server, &pi_iface);

  // Test function
#if 0
  int r = RC_Server_Add_Function(&server,
				 TEST_FUNCTION,
				 test_function,
				 "sbBuifFs",
				 "sbBuifFs",
				 RC_IMMEDIATE);
#endif

  int i;
  for(i = 0; i < NB_FUNS; ++i){
    if(RC_Server_Add_Function(&server,
			      i,
			      funs[i],
			      fprots[i][PARAMS],
			      fprots[i][RETURN],
			      RC_IMMEDIATE) != 0){
      return -1;
    }
  }
  
  return 0;
}


#define CHECK_STATE(fsm) ((uint8_t) (((fsm).instance.run == FSM_NOP)?(ASSER_DONE):(ASSER_BUSY)))

void rc_set_stop(RC_Server *server){
  set_stop(&fsm_asser);
  RC_Server_Return(server);
}

void rc_set_theta(RC_Server *server){
  uint8_t state = CHECK_STATE(fsm_asser);
  if(state == ASSER_DONE){
    double theta;
    if(RC_Server_Get_Args(server, &theta) == 0){
      set_theta(&fsm_asser, theta);
    }
  }
  RC_Server_Return(server, state);
}

void rc_set_theta_speed(RC_Server *server){
  uint8_t state = CHECK_STATE(fsm_asser);
  if(state == ASSER_DONE){
    double vth;
    if(RC_Server_Get_Args(server, &vth) == 0){
      set_theta_speed(&fsm_asser, vth);
    }
  }
  RC_Server_Return(server, state);
}

void rc_set_translation(RC_Server *server){
  uint8_t state = CHECK_STATE(fsm_asser);
  if(state == ASSER_DONE){
    double t;
    if(RC_Server_Get_Args(server, &t) == 0){
      set_translation(&fsm_asser, t);
    }
  }
  RC_Server_Return(server, state);
}

void rc_set_translation_speed(RC_Server *server){
  uint8_t state = CHECK_STATE(fsm_asser);
  if(state == ASSER_DONE){
    double vt;
    if(RC_Server_Get_Args(server, &vt) == 0){
      set_translation_speed(&fsm_asser, vt);
    }
  }
  RC_Server_Return(server, state);
}

void rc_set_X_Y_theta(RC_Server *server){
  uint8_t state = CHECK_STATE(fsm_asser);
  if(state == ASSER_DONE){
    double x, y, theta;
    uint8_t back;
    if(RC_Server_Get_Args(server, &x, &y, &theta, &back) == 0){
      set_X_Y_theta(&fsm_asser, x, y, theta, (int)back);
    }
  }
  RC_Server_Return(server, state);
}

void rc_get_odometry(RC_Server *server){
  odometry odo = odometry_get_position();
  RC_Server_Return(server, odo.x, odo.y, odo.theta);
}

void rc_get_state(RC_Server *server){
  uint8_t state = CHECK_STATE(fsm_asser);
  RC_Server_Return(server, state);
}

void rc_enable(RC_Server *server){
  enable = true;
  RC_Server_Return(server);
}

void rc_disable(RC_Server *server){
  enable = false;
  RC_Server_Return(server);
}

void test_function(RC_Server *server){
  char str_start[RC_STR_SIZE], str_end[RC_STR_SIZE];
  uint8_t byte;
  uint16_t half_word;
  uint32_t u_word;
  int word;
  float simpleFloat;
  double doubleFloat;

  int r = RC_Server_Get_Args(server,
			     str_start,
			     &byte,
			     &half_word,
			     &u_word,
			     &word,
			     &simpleFloat,
			     &doubleFloat,
			     str_end);
  if(r != 0){
    //    led_on();
    //let the other end timeout
    return;
  }

  /* Change values and send */
  byte++;
  half_word++;
  u_word++;
  word = -(word + ((word > 0)?(1):(-1)));
  simpleFloat += 0.01;
  doubleFloat -= 0.01;

  RC_Server_Return(server, str_start, byte, half_word, u_word, word, simpleFloat, doubleFloat, str_end);
}
