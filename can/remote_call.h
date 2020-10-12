/**
 * Library Remote Call Client/Server @Robotronik
 * remote_call.h
 * This library is meant to run a remote call server between µC. 
 *
 * Copyright 2018 Antonin Hirschy
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#ifndef __RC_SERVER_H_
#define __RC_SERVER_H_

#include <stdlib.h>
#include <stdint.h>
#include <stdarg.h>
#include "robotronik_protocol.h"

#define RC_NB_FUNCTIONS 32
#define RC_MAX_DATA RP_MAX_PACKET_SIZE-1//-1 because one byte is used for id
#define RC_FMT_SIZE 9
#define RC_STR_SIZE 64
#define RC_TRANSFERT_TIMEOUT 100
#define RC_CALL_TIMEOUT 100

//GCC macro
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
#define RC_LITTLE_ENDIAN
#elif __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
#define RC_BIG_ENDIAN
#else
#error "RC_SERVER_H : Unsupported endianness"
#endif

#if RC_NB_FUNCTIONS > 256 || RC_NB_FUNCTIONS < 0
#error "RC_NB_FUNCTIONS must be between 0 and 256"
#endif

//==================================================
//          Global definitions
//==================================================

typedef enum RC_Type_E{
  RC_UINT8_T = 'b', //uint8_t
  RC_UINT16_T = 'B', //uint16_t
  RC_UINT32_T = 'u', //uint32_t
  RC_INT = 'i', //int
  RC_FLOAT = 'f', //float
  RC_DOUBLE = 'F', //double
  RC_STRING = 's' //string
}RC_Type;
/*
 * These are the supported types.
 */

//==================================================
//            Server definitions
//==================================================

typedef enum RC_Call_Type_E{
  RC_IMMEDIATE, RC_DELAYED
}RC_Call_Type;
/*
 * This enumeration is used by a server.  It tells wether a function
 * should be called directly when the request is processed (eg. within
 * an interrupt execution) (RC_IMMEDIATE) or later by polling the
 * server (RC_DELAYED) to know if a request is ready to be
 * executed. See RC_Server_Get_Request() & RC_Server_Poll().
 */

struct RC_Server_S;

typedef struct RC_Server_Function_S{
  //Address of the function
  void (*call)(struct RC_Server_S *server);
  
  //"Virtual" prototype
  char params_fmt[RC_FMT_SIZE];
  char return_fmt[RC_FMT_SIZE];
  /*
   * As the protocol needs a fixed prototype for each function, we
   * need a mechanism to simulate custom prototypes, ie. we must
   * define how data from and for remote functions should be
   * packed/unpacked from link layer frames. This then allows us to
   * have ready to use functions for such operations and prevent the
   * users from having to do it manually for each
   * function. Concretely, we define our virtual prototype as a suite
   * of parameters among authorized types. Packing/unpacking such a
   * suite which is defined on the fly reminds us what printf and
   * scanf are actually doing. We don't want to use these functions
   * directly, because we need a binary-oriented packing, and not
   * text-oriented (doing so would take 3 bytes to write 250 instead
   * of one).
   *
   * This RC lib then has its own format conventions. The format is a
   * suite of authorized characters whose list and meaning are defined
   * in RC_Type enum. Any other character will trigger an error.
   *
   * For example, a valid value for params_fmt and return_fmt could
   * be respectively "ifFs" and "Fs".  This means "The virtual
   * prototype of the function is (int, float, double, const char*)
   * and it returns (double, const char*)".  Please note that the
   * combination of types is limited by RC_MAX_DATA, RC_STR_SIZE, and
   * RC_FMT_SIZE. See RC_(Server|Client)_Add_Function for more
   * informations.
   */
  
  RC_Call_Type call_type;
  /*
   * See RC_Call_Type enum.
   */
}RC_Server_Function;

typedef struct RC_Request_S{
  int id_client; //id of the client that made the request
  int id;
  uint8_t *data;
  int len;
}RC_Request;

typedef struct RC_Server_S{
  RP_Interface *interface;
  RC_Server_Function functions[RC_NB_FUNCTIONS];
  RC_Request request;
  uint8_t pending;
}RC_Server;

//==================================================
//            Client definitions
//==================================================

typedef struct RC_Client_Function_S{
  bool used;
  char params_fmt[RC_FMT_SIZE];
  char return_fmt[RC_FMT_SIZE];
}RC_Client_Function;

typedef struct RC_Client_S{
  RP_Interface *interface;
  RC_Client_Function functions[RC_NB_FUNCTIONS];
  int id_server; //id of the associated server
}RC_Client;

//==================================================
//              Errors
//==================================================

/*
 * When a function fails, it returns -1 and set the appropriate error
 * code that can be checked with RC_Get_Error() function.
 */
typedef enum RC_Error_E{
  RC_BAD_ID, //The given ID of the function is'nt a valid one.
  RC_WRONG_FORMAT, //The given format isn't valid (see
		   //RC_(Server|Client)_Add_Function for
		   //explanations).
  RC_UNDEFINED_FUNCTION, //The wanted function isn't defined.
  RC_LINK_ERROR, //There was an error of communication at link layer
		 //level.
  RC_ERR_CALL_TIMEOUT, //The allowed time between call and return has
		       //been exceeded.
  RC_INVALID_RETURN, //The return of the function was incorrect.
}RC_Error;

int RC_Get_Error();
/**
 * Returns the error code of the last error. The error code and their
 * meaning are defined in RC_Error enum.
 */

//==================================================//
//            RC Server functions                   //
//==================================================//

//Init functions
void RC_Server_Init(RC_Server *server, RP_Interface *interface);
/**
 * Init the server, ie. set all functions to undefined.
 */

int RC_Server_Add_Function(RC_Server *server,
			   int id,
			   void (*function)(RC_Server *server),
			   const char params_fmt[],
			   const char return_fmt[],
			   RC_Call_Type call_type);
/**
 * Initializes the function n°id of the server with the given
 * informations. Returns 0 on success, -1 on error.  On error,
 * RC_Get_Error() can give additionnal informations.
 */

//Request management
int RC_Server_Get_Request(RC_Server *server, RP_Packet *packet);
int RC_Server_Poll(RC_Server *server);
/**
 * Poll the server to check if there is a pending request. If true,
 * the request is executed and the return number is 1. If there is no
 * request, the function returns 0.
 */

int RC_Server_Get_Args(RC_Server *server, ...);
int RC_Server_Return(RC_Server *server, ...);

//==================================================//
//            RC Client functions                   //
//==================================================//

void RC_Client_Init(RC_Client *client, RP_Interface *interface, int id_server);
int RC_Client_Add_Function(RC_Client *client,
			   int id,
			   const char params_fmt[],
			   const char return_fmt[]);
int RC_Call(RC_Client *client, int id, ...);

#endif
