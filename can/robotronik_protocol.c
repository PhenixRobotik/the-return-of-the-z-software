/**
 * Library Robotronik Protocol, CAN GANGE EDITION
 * robotronik_protocol.c
 *
 * This library is meant to implement a light and reliable protocol
 * for communication between two MCUs. The protocol was designed for
 * request-response oriented communications. It is why it focuses on
 * performance for asynchronous reception.
 * 
 * It uses the Consistent Overhead Byte Stuffing algorithm to delimit
 * the frames and for synchronization purposes. CRC-16/BUYPASS is
 * used to detect corrupted frames.
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
#include "robotronik_protocol.h"

#define RP_EOF 0x00
#define WRITE_BUFF(p_buffer,byte) *(p_buffer++) = byte;
#define ENCODE_BYTE(byte,p_buffer,p_code) {\
    if(byte == RP_EOF){			   \
      *p_code = p_buffer - p_code;	   \
      p_code = p_buffer++;		   \
    }					   \
    else{				   \
      *(p_buffer++) = byte;		   \
    }					   \
  }

static uint16_t err;

static inline void RP_FSM_INIT(RP_Interface *interface);
static inline void RP_FSM_DATA(RP_Interface *interface);

//==================================================
//           Initialization functions
//==================================================

void RP_Init_Interface(RP_Interface *interface,
		       void* link_handler,
		       uint32_t can_id,
		       uint8_t (*send)(void*, uint32_t, uint8_t *, uint16_t, uint32_t),
		       uint32_t (*get_tick)()){
  interface->link_handler = link_handler;
  interface->can_id = can_id;
  interface->send = send;
  interface->get_tick = get_tick;
  interface->received = false;
  
  //FSM
  interface->update_state = RP_FSM_INIT;
  interface->p_in = interface->buffer_in;
  interface->p_out = interface->r_packet.data;
}

void __attribute__((weak)) RP_Packet_Received(RP_Interface* interface, RP_Packet* packet){
  //Avoid compiler warnings.
  (void)interface;
  (void)packet;
}

void __attribute__((weak)) RP_Error_Handler(RP_Interface* interface, uint16_t err){
  //Avoid compiler warnings.
  (void)interface;
  (void)err;
 }

//==================================================
//           Sending functions
//==================================================

int RP_Build_Frame(RP_Packet *packet, uint8_t buffer[RP_BUFFER_SIZE]){
  if((packet->len <= 0) || (packet->len + 2 > RP_BUFFER_SIZE) || (packet->len > RP_MAX_PACKET_SIZE)){
    err =  RP_ERR_INTERNAL | RP_ERR_ILLEGAL_ARGUMENTS;
    return -1;
  }

  uint8_t *p_buffer = buffer;
  uint8_t *p_code = p_buffer++;
  
  int i;
  for(i = 0; i < packet->len; i++){
    const uint8_t byte = packet->data[i];
    ENCODE_BYTE(byte, p_buffer, p_code);
  }
  
  //RP_EOF
  *p_code = p_buffer - p_code;
  *(p_buffer++) = RP_EOF;

  return p_buffer - buffer;
}

int RP_Sync(RP_Interface *interface, uint32_t timeout){
  uint8_t byte = RP_EOF;

  if(interface->send(interface->link_handler, interface->can_id, &byte, 1, timeout) != 0){
    err = RP_ERR_LINK | RP_ERR_TIMEOUT;
    return -1;
  }
  
  return 0;
}

int RP_Send(RP_Interface *interface, RP_Packet *packet, uint32_t timeout){
  int len = RP_Build_Frame(packet, interface->buffer_out);

  if(len == -1){
    return -1;
  }

  if(interface->send(interface->link_handler, interface->can_id, interface->buffer_out, len, timeout) != 0){
    err = RP_ERR_LINK | RP_ERR_TIMEOUT;
    return -1;
  }
  
  return 0;
}

//==================================================
//           Receiving functions
//==================================================

void RP_Process_Data(RP_Interface *interface, uint8_t *data, uint16_t len){
  interface->p_in = data;

  while(len--){
    interface->update_state(interface);
  }
}

int RP_Wait_Packet(RP_Interface *interface, uint32_t timeout_ms){
  int start = interface->get_tick();
  
  //interface->received = false;
  while(!interface->received && (interface->get_tick() - start < timeout_ms));
  
  if(!interface->received){
    return -1;
  }
  
  interface->received = false;
  
  return 0;
}

#define FSM_UPDATE(fsm,state_function) fsm->update_state = state_function

#define FSM_GET_BYTE(fsm) uint8_t byte = *(fsm->p_in++);

#define FSM_BYTE byte

#define FSM_RESET(fsm) {						\
    FSM_UPDATE(fsm, RP_FSM_INIT);					\
    fsm->p_out = fsm->r_packet.data;					\
    return;								\
  }

#define FSM_ERR(fsm, err_code) {		\
  err = err_code;				\
  RP_Error_Handler(fsm, err);			\
  FSM_RESET(fsm);				\
  }

#define FSM_CHECK_ERR(fsm, err_code) {		\
    if(FSM_BYTE == RP_EOF){			\
      FSM_ERR(fsm, err_code);			\
    }						\
  }

#define FSM_CHECK_EOF(fsm) {			\
    if(FSM_BYTE == RP_EOF) {FSM_RESET(fsm);}	\
  }
/**
 * Reinit the FSM when EOF is reached.
 * Set the 'valid' flag of FSM
 * and the error code.
 */

#define FSM_DECODE_BYTE(fsm, byte)			\
  (((--fsm->bs_count) == 0)?				\
   (fsm->bs_count = byte, (uint8_t) RP_EOF):		\
   (byte))
/**
 * Update the COBS counter and decode the current byte. This macro can
 * be used once and only once in an fsm state. If the macro is not
 * used and the FSM is not reset, bs_count MUST be updated manually.
 */

static inline void RP_FSM_INIT(RP_Interface *interface){
  FSM_GET_BYTE(interface);

  // Reset FSM if the current byte is an EOF
  FSM_CHECK_EOF(interface);

  /*
   * The first byte initializes the COBS algorithm.
   */
  interface->bs_count = FSM_BYTE;
  interface->tmp_size = 0;

  FSM_UPDATE(interface, RP_FSM_DATA);
}

static inline void RP_FSM_DATA(RP_Interface *interface){
  FSM_GET_BYTE(interface);
  
  // An EOF flag means end of packet
  if(FSM_BYTE == 0x00){
    interface->received = true;
    interface->r_packet.len = interface->tmp_size;
    RP_Packet_Received(interface, &interface->r_packet);
    FSM_RESET(interface);
  } else {
    FSM_BYTE = FSM_DECODE_BYTE(interface, FSM_BYTE);
    *(interface->p_out++) = FSM_BYTE;
    interface->tmp_size++;
  }
}

uint16_t RP_Get_Error(){
  return err;
}
