#ifndef RC_SERVER_H
#define RC_SERVER_H

#include "remote_call.h"

extern RC_Server server;

int init_server();

#define NB_FUNS 10
typedef enum Server_Functions_E{
  SET_STOP,
  SET_THETA,
  SET_THETA_SPEED,
  SET_TRANSLATION,
  SET_TRANSLATION_SPEED,
  SET_X_Y_THETA,
  GET_ODOMETRY,
  GET_STATE,
  ENABLE,
  DISABLE,
}Server_Functions;

void test_function(RC_Server *server);

#endif
