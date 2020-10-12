/**
 * Library Remote Call Client/Server @Robotronik
 * remote_call.c
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
#include "remote_call.h"

//Error code
static int err = 0;

//Private functions
static int RC_Copy_Format(char *dst, const char fmt[]);
static int RC_Pack_Vars(const char *fmt, uint8_t *out, int out_len,  va_list *args);
static int RC_Unpack_Vars(const char *fmt, uint8_t *in, int in_len,  va_list *args);

//Macros
#define CHECK_ID(id) ((id >= 0) && (id < RC_NB_FUNCTIONS))

#define RC_CHECK_TYPE(type, count)					\
  ({									\
    int valid = 1;							\
    switch(type){							\
    case RC_UINT8_T: count += sizeof(uint8_t); break;			\
    case RC_UINT16_T: count += sizeof(uint16_t); break;			\
    case RC_UINT32_T: count += sizeof(uint32_t); break;			\
    case RC_INT: count += sizeof(int); break;				\
    case RC_FLOAT: count += sizeof(float); break;			\
    case RC_DOUBLE: count += sizeof(double); break;			\
    case RC_STRING: count += RC_STR_SIZE; break;			\
    default: valid = 0; break;						\
    }									\
    valid;								\
  })

#ifdef RC_LITTLE_ENDIAN
#define RC_PACK_VAR_START(var) ((uint8_t*) (&var))
#define RC_UNPACK_VAR_START(args, type, promoted) ((uint8_t*) va_arg(args, promoted))
#define RC_VAR_GET_BYTE(ptr) *(ptr++)
#else
#define RC_PACK_VAR_START(var) ((uint8_t*) (&var) + (sizeof(var) - 1))
#define RC_UNPACK_VAR_START(args, type, promoted) (((uint8_t*) va_arg(args, promoted)) + (sizeof(type) - 1))
#define RC_VAR_GET_BYTE(ptr) *(ptr--)
#endif

#define RC_PACK_VAR(args, type, promoted, out, out_len, success)	\
  {									\
    type var = (type) va_arg(args, promoted);				\
    uint8_t *byte  = RC_PACK_VAR_START(var);				\
    int _end = out_len - sizeof(var);					\
    success = (_end >= 0);						\
    									\
    if(success){							\
      for(; out_len > _end; out_len--){					\
	*(out++) = RC_VAR_GET_BYTE(byte);				\
      }									\
    }									\
  }								

#define RC_PACK_STRING(args, out, out_len, success)			\
  {									\
    char *str = va_arg(args, char*);					\
    char c = '*';							\
    int count = RC_STR_SIZE;						\
    /* Copy a character from str to out until the end of str or out is reached */ \
    while(out_len != 0 && (c = *(out++) = *(str++)) != '\0' && (--count)){ \
      out_len--;							\
    }									\
    /* if the last character was '\0', out_len must be decremented */	\
    (c == '\0')?(out_len--, success = 1):(success = 0);			\
  }						

#define RC_UNPACK_VAR(args, type, promoted, in, in_len, success)	\
  {									\
    uint8_t *w_ptr = RC_UNPACK_VAR_START(args, type, promoted);		\
    int _end = in_len - sizeof(type);					\
    success = (_end >= 0);						\
									\
    if(success){							\
      for(; in_len > _end; in_len--){					\
	RC_VAR_GET_BYTE(w_ptr) = *(in++);				\
      }									\
    }									\
  }

#define RC_UNPACK_STRING(args, in, in_len, success)			\
  {									\
    char *w_ptr = va_arg(args, char*);					\
    char c = '*';							\
    int count = RC_STR_SIZE;						\
    while(in_len != 0 && (c = *(w_ptr++) = *(in++)) != '\0' && (--count)){ \
      in_len--;								\
    }									\
    (c == '\0')?(in_len--, success = 1):(success = 0);			\
  }								

static int RC_Copy_Format(char *dst, const char fmt[]){
  int i;
  char c;
  int max_size = 0;
  
  for(i = 0; i < RC_FMT_SIZE && (c = dst[i] = fmt[i]) != '\0' && RC_CHECK_TYPE(c, max_size); i++);
  
  if(c != '\0' || max_size > RC_MAX_DATA){
    err = RC_WRONG_FORMAT;
    return -1;
  }

  return 0;
}

static int RC_Pack_Vars(const char *fmt, uint8_t *out, int out_len,  va_list *args){
  int out_len_tmp = out_len;

  char c;
  int success = 1;
  while((c = *(fmt++)) != '\0' && success){
    if(c == RC_UINT8_T){
      RC_PACK_VAR(*args, uint8_t, int, out, out_len, success);
    }
    else if(c == RC_UINT16_T){
      RC_PACK_VAR(*args, uint16_t, int, out, out_len, success);
    }
    else if(c == RC_UINT32_T){
      RC_PACK_VAR(*args, uint32_t, int, out, out_len, success);      
    }
    else if(c == RC_INT){
      RC_PACK_VAR(*args, int, int, out, out_len, success);
    }
    else if(c == RC_FLOAT){
      RC_PACK_VAR(*args, float, double, out, out_len, success);
    }
    else if(c == RC_DOUBLE){
      RC_PACK_VAR(*args, double, double, out, out_len, success);      
    }
    else if(c == RC_STRING){
      RC_PACK_STRING(*args, out, out_len, success);
    }else{
      //Should not happen because format is checked when the function
      //is initialized.
      err = RC_WRONG_FORMAT;
      return -1;
    }
  }
  
  return (success)?(out_len_tmp - out_len):-1;
}								

static int RC_Unpack_Vars(const char *fmt, uint8_t *in, int in_len,  va_list *args){
  char c;
  int success = 1;
  
  while((c = *(fmt++)) != '\0' && success){
    if(c == RC_UINT8_T){
      RC_UNPACK_VAR(*args, uint8_t, uint8_t*, in, in_len, success);
    }
    else if(c == RC_UINT16_T){
      RC_UNPACK_VAR(*args, uint16_t, uint16_t*, in, in_len, success);
    }
    else if(c == RC_UINT32_T){
      RC_UNPACK_VAR(*args, uint32_t, uint32_t*, in, in_len, success);
    }
    else if(c == RC_INT){
      RC_UNPACK_VAR(*args, int, int*, in, in_len, success);
    }
    else if(c == RC_FLOAT){
      RC_UNPACK_VAR(*args, float, float*, in, in_len, success);
    }
    else if(c == RC_DOUBLE){
      RC_UNPACK_VAR(*args, double, double*, in, in_len, success);
    }
    else if(c == RC_STRING){
      RC_UNPACK_STRING(*args, in, in_len, success);
    }else{
      //Should not happen because format is checked when the function
      //is initialized.
      err = RC_WRONG_FORMAT;
      return -1;
    }
  }
  
  return (success && in_len == 0)?0:-1;
}

//==================================================//
//               ERROR ACCESSOR                     //
//==================================================//
int RC_Get_Error(){
  return err;
}

//==================================================//
//                  RC SERVER                       //
//==================================================//

void RC_Server_Init(RC_Server *server, RP_Interface *interface){
  int i;

  server->interface = interface;
  for(i = 0; i < RC_NB_FUNCTIONS; i++){
    server->functions[i].call = 0;
  }  
}

int RC_Server_Add_Function(RC_Server *server,
			   int id,
			   void (*function)(RC_Server *server),
			   const char params_fmt[],
			   const char return_fmt[],
			   RC_Call_Type call_type){
  //Check ID range
  if(!CHECK_ID(id)){
    err = RC_BAD_ID;
    return -1;
  }

  //Get function struct by id
  RC_Server_Function *fun = &server->functions[id];

  //Params format
  if(RC_Copy_Format(fun->params_fmt, params_fmt) == -1){
    return -1;
  }

  //Return format
  if(RC_Copy_Format(fun->return_fmt, return_fmt) == -1){
    return -1;
  }

  //Function pointer
  fun->call = function;
  
  //Call type
  fun->call_type = call_type;

  return 0;
}

int RC_Server_Get_Request(RC_Server *server, RP_Packet *packet){
  if(packet->len < 1){
    err = RC_LINK_ERROR;
    return -1;
  }
  
  //Extracting packet informations
  const int id = packet->data[0];
  const int len = packet->len - 1;
  uint8_t *const data = &packet->data[1];


  //Checking basic informations
  if(!CHECK_ID(id)){
    err = RC_BAD_ID;
    return -1;
  }

  RC_Server_Function *const fun = &server->functions[id];
  if(fun->call == 0){
    err = RC_UNDEFINED_FUNCTION;
    return -1;
  }
  
  //Filling request informations
  server->request.id = id;
  server->request.data = data;
  server->request.len = len;

  switch(fun->call_type){
  case RC_IMMEDIATE:
    server->pending = 0;
    fun->call(server);
    break;
  case RC_DELAYED:
    server->pending = 1;
    break;
  default:
    break;
  }

  return 0;
}

int RC_Server_Poll(RC_Server *server){
  if(!server->pending){
    return 0;
  }

  server->pending = 0;
  server->functions[server->request.id].call(server);
  
  return 1;
}

int RC_Server_Get_Args(RC_Server *server, ...){
  va_list args;
  va_start(args, server);

  RC_Server_Function *const fun = &server->functions[server->request.id];
  
  int r = RC_Unpack_Vars(fun->params_fmt, server->request.data, server->request.len, &args);

  va_end(args);

  return r;
}


int RC_Server_Return(RC_Server *server, ...){
  va_list args;
  va_start(args, server);

  RP_Packet *packet = &server->interface->s_packet;

  packet->data[0] = server->request.id;
  packet->len = 1 + RC_Pack_Vars(server->functions[server->request.id].return_fmt,
			       &packet->data[1],
			       RC_MAX_DATA,
			       &args);

  if(packet->len == 0){
    err = RC_INVALID_RETURN;
    return -1;
  }
  
  va_end(args);
  return RP_Send(server->interface, packet, RC_TRANSFERT_TIMEOUT);
}

//==================================================//
//                  RC CLIENT                       //
//==================================================//
void RC_Client_Init(RC_Client *client, RP_Interface *interface, int id_server){
  int i;

  client->interface = interface;
  client->id_server = id_server;
  for(i = 0; i < RC_NB_FUNCTIONS; i++){
    client->functions[i].used = false;
  }  
}

int RC_Client_Add_Function(RC_Client *client,
			   int id,
			   const char params_fmt[],
			   const char return_fmt[]){
   //Check ID range
  if(!CHECK_ID(id)){
    err = RC_BAD_ID;
    return -1;
  }

  //Get function struct by id
  RC_Client_Function *fun = &client->functions[id];

  //Params format
  if(RC_Copy_Format(fun->params_fmt, params_fmt) == -1){
    return -1;
  }

  //Return format
  if(RC_Copy_Format(fun->return_fmt, return_fmt) == -1){
    return -1;
  }

  //Used flag
  fun->used = true;

  return 0;
}

int RC_Call(RC_Client *client, int id, ...){
  va_list args;
  va_start(args, id);

  //Check ID
  if(!CHECK_ID(id)){
    err = RC_BAD_ID;
    return -1;
  }

  RC_Client_Function *const fun = &client->functions[id];

  if(fun->used == false){
    err = RC_UNDEFINED_FUNCTION;
    return -1;
  }

  //Build the packet
  RP_Packet *packet = &client->interface->s_packet;
  packet->data[0] = id;
  packet->len = 1 + RC_Pack_Vars(client->functions[id].params_fmt,
				 &packet->data[1],
				 RC_MAX_DATA,
				 &args);
  if(packet->len == 0){
    err = RC_WRONG_FORMAT;
    return -1;
  }

  //Send the request
  if(RP_Send(client->interface, packet, RC_TRANSFERT_TIMEOUT)){
    err = RC_LINK_ERROR;
    return -1;
  }

  //Wait for return
  if(RP_Wait_Packet(client->interface, RC_CALL_TIMEOUT)){
    err = RC_ERR_CALL_TIMEOUT;
    return -1;
  }

  //Check return packet
  RP_Packet *const r_packet = &client->interface->r_packet;
  if(r_packet->len < 1 || r_packet->data[0] != id){
    err = RC_INVALID_RETURN;
    return -1;
  }

  //Extract return informations
  RC_Unpack_Vars(client->functions[id].return_fmt,
			 &r_packet->data[1],
			 r_packet->len - 1,
			 &args);
  va_end(args);

  return 0;
}



