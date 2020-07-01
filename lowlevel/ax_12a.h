/*
 * Library for AX-12A @Robotronik
 * ax_12a.h
 * Written for a little-endian µC
 * You may freely rage with this software
 */
#ifndef AX_12A_HEADER
#define AX_12A_HEADER

#include <stdint.h>

#define AX_BUFFER_SIZE 32
#define AX_DEFAULT_TIMEOUT 1
#define AX_BROADCAST 0xFE

//==================================================
//                INSTRUCTIONS
//==================================================
typedef enum AX_Instruction_E{
  AX_PING = 0x01,
  AX_READ = 0x02,
  AX_WRITE = 0x03,
  AX_REG_WRITE = 0x04,
  AX_ACTION = 0x05,
  AX_FACTORY_RESET = 0x06,
  AX_SYNC_WRITE = 0x83,
}AX_Instruction;

//==================================================
//           INTERFACE, RECEPTION, SENDING
//==================================================
typedef struct AX_Instruction_Packet_S{
  uint8_t id;
  AX_Instruction instruction;
  uint8_t nb_params;
  uint8_t *params;
}AX_Instruction_Packet;

typedef struct AX_Status_Packet_S{
  uint8_t id;
  uint8_t err;
  uint8_t nb_params;
  uint8_t params[AX_BUFFER_SIZE];
}AX_Status_Packet;

/*
 * Compute checksum for the data between data_blk_ptr and
 * data_blk_ptr+(data_blk_size-1).
 */
uint8_t AX_Compute_Checksum(uint8_t *data_blk_ptr, uint16_t data_blk_size);

//Final state machine used for reception
typedef struct AX_Receiver_FSM_S{
  void (*update_state)(struct AX_Receiver_FSM_S*);
  uint8_t remaining_bytes;//Remaining bytes while receiving.
  uint8_t *buffer;
  uint8_t *p_buffer;//Current position in the buffer
  uint8_t done;
}AX_Receiver_FSM;

void AX_FSM_HEADER_0(AX_Receiver_FSM *fsm);
void AX_FSM_HEADER_1(AX_Receiver_FSM *fsm);
void AX_FSM_ID(AX_Receiver_FSM *fsm);
void AX_FSM_LENGTH(AX_Receiver_FSM *fsm);
void AX_FSM_RECEIVING(AX_Receiver_FSM *fsm);

typedef enum AX_Direction_E{
  AX_SEND, AX_RECEIVE
}AX_Direction;

typedef struct AX_Interface_S{
  uint8_t (*send)(uint8_t *, uint16_t, uint32_t);//data, size, timeout_ms
  uint8_t (*receive)(uint8_t *, uint16_t, uint32_t);//data, size, timeout ms
  void (*set_direction)(AX_Direction);
  void (*delay)(uint32_t);//ms
  AX_Receiver_FSM fsm;
  uint8_t buffer[AX_BUFFER_SIZE];
  AX_Status_Packet status;
}AX_Interface;
/*
 * Note :
 * The interface's buffer is used both for sending and receiving packets.
 */

uint8_t AX_Extract_Status_Packet(AX_Status_Packet *packet, uint8_t frame[AX_BUFFER_SIZE], uint16_t packet_size);
/*
 * Extract a status packet from a valid frame wich total length is 'packet_size'.
 * Return 0 on success, 1 if it fails.
 */

uint8_t AX_Receive(AX_Interface *interface, uint16_t packet_size, uint32_t timeout);
/*
 * Receive a packet from *interface in the given time (timeout).
 * The received packet is stored in interface->status.
 * packet_size is the effective length of the packet, from header to checksum included.
 * This function should return 0 on success, 1 otherwise.
 */

//Sending a packet
uint8_t AX_Build_Frame(AX_Instruction_Packet *packet, uint8_t buffer[AX_BUFFER_SIZE]);
/*
 * Build a frame from an instruction packet and write it on the
 * buffer.  This function return the effective elngth of the built
 * frame on success, 0 if an error occured.
 */

uint8_t AX_Send(AX_Interface *interface, AX_Instruction_Packet *packet, uint32_t timeout);


//==================================================
//                     AX-12A
//==================================================
typedef struct AX_S{
  uint8_t id;
  AX_Interface *interface;
}AX;

typedef enum AX_Baud_Rate_E{
  AX_BAUD_RATE_9600 = 207,
  AX_BAUD_RATE_57600 = 34,
  AX_BAUD_RATE_115200 = 16,
  AX_BAUD_RATE_1MBPS = 1
}AX_Baud_Rate;

typedef enum AX_Return_Level_E{
  AX_PING_RETURN = 0, AX_READ_RETURN = 1, AX_ALL_RETURN = 2
}AX_Return_Level;

typedef enum AX_LED_State_E{
  AX_LED_ON = 1, AX_LED_OFF = 0
}AX_LED_State;

//======================================
//    DEFINITIONS TABLE DE CONTROLE       
//======================================

#define AX_NOW 1
#define AX_LATER 0

#define AX_NB_FIELDS 32
typedef enum AX_Field_E{
  //EEPROM - Configuration  
  AX_ID,
  AX_BAUD_RATE,
  AX_RETURN_DELAY_TIME,
  AX_CW_ANGLE_LIMIT,
  AX_CCW_ANGLE_LIMIT,
  AX_LIMIT_TEMPERATURE,
  AX_LOWER_LIMIT_VOLTAGE,
  AX_UPPER_LIMIT_VOLTAGE,
  AX_MAX_TORQUE,
  AX_RETURN_LEVEL,
  AX_ALARM_LED,
  AX_ALARM_SHUTDOWN,
  //EEPROM - Info
  AX_MODEL_NUMBER,
  AX_FIRMWARE_VERSION,
  //RAM - Contrôle
  AX_TORQUE_ENABLE,
  AX_LED,
  AX_CW_COMPLIANCE_MARGIN,
  AX_CCW_COMPLIANCE_MARGIN,
  AX_CW_COMPLIANCE_SLOPE,
  AX_CCW_COMPLIANCE_SLOPE,
  AX_GOAL_POSITION,
  AX_MOVING_SPEED,
  AX_TORQUE_LIMIT,
  AX_EEPROM_LOCK,
  AX_PUNCH,
  //RAM - Info
  AX_CURRENT_POSITION,
  AX_CURRENT_SPEED,
  AX_CURRENT_LOAD,
  AX_CURRENT_VOLTAGE,
  AX_CURRENT_TEMPERATURE,
  AX_REGISTERED_INSTRUCTION,
  AX_MOVING,
}AX_Field;

//======================================
//        INSTRUCTION SET
//======================================
uint8_t AX_Write(AX *servo, AX_Field field, uint8_t *data, uint8_t size, uint8_t now);
/*
 * Write the data from data to data+(size-1) to the control table the
 * servo, starting at the address given by the field. This function
 * should not be called by a user, but is used by others library
 * functions.
 */

uint8_t AX_Ping(AX *servo);
/*
 * Send a ping request to the servo and retrieve status in
 * servo->interface->status. Return 0 if it succeeds, 1 if it fails.
 */

uint8_t AX_Discover(AX_Interface *interface, AX *buffer_servos, uint8_t len_buffer, uint16_t *nb_servos);
/*
 * Send a broadcast ping request and retrieve a table of at most
 * len_buffer servos which have responded. Update nb_servo to the
 * number of discovered servos.
 */

uint8_t AX_Say_Hello(AX *servo);
/*
 * Check the existence of the servo and make its LED blink as most
 * times as his ID number, with a frequency of 1Hz.
 */

uint8_t AX_Read(AX *servo, AX_Field field, uint16_t *data);
/*
 * Read a field in the control table of the servo. If it succeeds, the
 * return status is stored in the interface of the servo and the data
 * is written in "data". Return 0 in case of success, -1 if it fails.
 * Note : the AX_Status_Packet of an interface is valid only before
 * the next action.
 */

uint8_t AX_Action(AX *servo);
/*
 * Launch a previously stored instruction in servo with a REG_WRITE
 * instruction (AX_Write(...) with now = AX_LATER). Return 0 on
 * success, -1 otherwise.
 */

uint8_t AX_Factory_Reset(AX *servo);
/*
 * Reset all the EEPROM fields to its factory value.  CAUTION : This
 * will also set the ID to 1 and the baudrate to 1Mbps.
 */

//==================================================
//              ERROR HANDLING
//==================================================
//Error types
typedef enum AX_Error_Type_E{
  AX_ERR_INTERNAL = 0x0000,
  AX_ERR_LINK = 0x0100,
  AX_ERR_STATUS = 0x0200,
}AX_Error_Type;

//Internal errors (library)
typedef enum AX_Internal_Error_E{
  AX_ERR_ILLEGAL_ARGUMENTS, //When the user did a mistake
  AX_ERR_BUFFER_OVERFLOW, //When the buffer is too small
}AX_Internal_Error;

//Communication errors (UART)
typedef enum AX_Link_Error_E{
  AX_ERR_BAD_FRAME, //When we receive an incorrect frame
  AX_ERR_TIMEOUT, //On timeout
}AX_Link_Error;

//Status errors
typedef enum AX_Status_Error_E{
  AX_VOLTAGE_ERROR = 0x01,
  AX_ANGLE_LIMIT_ERROR = 0x02,
  AX_OVERHEATING_ERROR = 0x04,
  AX_RANGE_ERROR = 0x08,
  AX_CHECKSUM_ERROR = 0x10,
  AX_OVERLOAD_ERROR = 0x20,
  AX_INSTRUCTION_ERROR = 0x40,
}AX_Status_Error;

//Macros on errors
#define AX_ERROR_TYPE(err) (AX_Error_Type) (err & 0xFF00)
#define AX_ERROR_CODE(err) (err & 0xFF)
#define AX_STATUS_ERROR(err) (AX_Status_Error) (AX_ERROR_CODE(err) & 0b0111111)
#define AX_INTERNAL_ERROR(err) (AX_Internal_Error) AX_ERROR_CODE(err)
#define AX_LINK_ERROR(err) (AX_Link_Error) AX_ERROR_CODE(err)

//Function to retrieve an error
uint16_t AX_Get_Error();
/*
 * Return the last catched error. The error type can be checked using
 * the macro AX_ERROR_TYPE. To gather more detailed informations
 * knowing the error type, you can use the others macros.
 */

uint8_t AX_Check_Status(AX *servo);
/*
 * This function should not be called by a user. It is used right
 * after the reception of a status packet in order to check for
 * errors, and return 1 if an error is catched, 0 otherwise.
*/

//======================================
//       CONFIGURATION EEPROM   
//======================================

uint8_t AX_Configure_ID(AX *servo, uint8_t id);
/*
 * Set the ID of a servo.
 * The range is between 0x00 and 0xFC (252).
 */

uint8_t AX_Configure_Baud_Rate(AX *servo, AX_Baud_Rate baud_rate);
/*
 * Set the transmission baud rate of the servo.
 * This library allows only the common values defined by AX_Baud_Rate enum.
 */

uint8_t AX_Configure_Return_Delay_Time(AX *servo, uint8_t delay);
/*
 * Set the delay time between an instruction packet and a status packet.
 * Values : [0x00, 0xFE]
 * Unit : 2 µs.
 */

uint8_t AX_Configure_Angle_Limit(AX *servo, uint16_t cw_angle, uint16_t ccw_angle);
/*
 * Set the angle limits of the servo.  See the documentation for
 * further informations.  Note that if both cw_angle and ccw_angle are
 * 0, the servo is set to WHEEL mode. If neither are 0, it is set to
 * JOIN mode.
 * Range : [0x00, 0x3FF] (0°, 300°);
 */

uint8_t AX_Configure_Limit_Temperature(AX *servo, uint8_t temp);
/*
 * Set the max operating temperature of the servo before catching an
 * error AX_OVERHEATING_ERROR.  This function should not exist.
 */

uint8_t AX_Configure_Lower_Limit_Voltage(AX *servo, uint8_t voltage);
/*
 * Configure the lower limit voltage before catching an error AX_VOLTAGE_ERROR.
 * Values : [50, 250]
 * Unit : 0.1 V
 */

uint8_t AX_Configure_Upper_Limit_Voltage(AX *servo, uint8_t voltage);
/*
 * Configure the upper limit voltage before catching an error AX_VOLTAGE_ERROR.
 * Values : [50, 250]
 * Unit : 0.1 V
 */

uint8_t AX_Configure_Max_Torque(AX *servo, uint16_t max_torque);
/*
 * Set the max torque.
 * Values : [0, 1023]
 * Unit : approx. 0.1%
 */

uint8_t AX_Configure_Return_Level(AX *servo, AX_Return_Level level);
/*
 * Set the return level.
 * Values :
 * 0 : No return, except for PING
 * 1 : return only for READ instructions
 * 2 : always return
 * NOTE : this library need to be used with the parameter 2.
 */

uint8_t AX_Configure_Alarm_LED(AX *servo, AX_Status_Error errors);
/*
 * Set for which errors an Alarm LED should be triggered.
 * Values : Combinations (OR) of AX_Status_Error enums.
 */

uint8_t AX_Configure_Alarm_Shutdown(AX *servo, AX_Status_Error errors);
/*
 * Set for which errors an Alarm LED should be triggered.
 * Values : Combinations (OR) of AX_Status_Error enums.
 */

//======================================
//       CONTROL SERVOMOTEUR       
//======================================

uint8_t AX_Power_On(AX *servo, uint8_t now);
/*
 * Turn on the motor.
 */

uint8_t AX_Power_Off(AX *servo, uint8_t now);
/*
 * Turn off the motor.
 */

uint8_t AX_Set_LED(AX *servo, AX_LED_State state, uint8_t now);
/*
 * Set the led state according to "state".
 */

typedef struct AX_Compliance_S{
  uint8_t cw_margin;
  uint8_t ccw_margin;
  uint8_t cw_slope;
  uint8_t ccw_slope;
}AX_Compliance;
uint8_t AX_Set_Compliance(AX *servo, AX_Compliance compliance, uint8_t now);
/*
 * Defines the compliance scheme of the servo.
 * Margins : [0, 255], 0.29°/value
 * Slope : [0, 6],
 *         0 : no flexibility
 *         6 : max flexibility
 * See the doc for more detailed informations
 */

uint8_t AX_Set_Goal_Position(AX *servo, uint16_t position, uint8_t now);
/*
 * Modify the goal position.
 * Values : 0 -> 1023 (0x3FF)
 * Unit : 0.29°
 * Note : This value is used only in Join mode.
 */

typedef enum AX_Wheel_Direction_E{
  AX_CLOCKWISE, AX_COUNTERCLOCKWISE
}AX_Wheel_Direction;

uint8_t AX_Set_Goal_Speed_Join(AX *servo, uint16_t speed, uint8_t now);
/* Change the goald speed when the servo is in JOIN mode.
 * Range : 0 -> 1023
 * Unit : 0.111 RPM
 * 0 : no speed control.
 */

uint8_t AX_Set_Goal_Speed_Wheel(AX *servo, uint16_t speed, AX_Wheel_Direction dir, uint8_t now);
/*
 * Change the goald speed when the servo is in Wheel mode.
 * Range :
 * speed : 0 -> 1023, unit : 0.1% of maximum speed
 * dir : AX_CLOCKWISE or else AX_COUNTERCLOCKWISE
 * Note : it just control the output of the motor.
 */

uint8_t AX_Set_Torque_Limit(AX *servo, uint16_t torque_limit, uint8_t now);
/*
 * Change the torque limit.
 * Range : 0 -> 1023
 * Unit : ~0.1%
 * Note :
 * It is set to 0 when an alarm shutdown occurs.
 */

uint8_t AX_Set_Punch(AX *servo, uint16_t punch, uint8_t now);
/*
 * Set the minimum current to drive the motor.
 * Range : 0x20 -> 0x3FF
 * Unit : ?
 */

uint8_t AX_Lock_ROM(AX *servo, uint8_t now);
/*
 * Locks the ROM. This ensures that the ROM can't be changed anymore
 * while the servo is working. Note : the power must be turner off and
 * then on in order to unlock the ROM after a lock.
 */

uint8_t AX_Unlock_ROM(AX *servo, uint8_t now);
/*
 * Unlock the ROM.
 */

//======================================
//       LECTURE D'INFORMATIONS
//======================================
uint8_t AX_Get_Current_Position(AX *servo, uint16_t *position);
/*
 * Reads the current position.
 * Range : 0 (0°) -> 1023 (300°)
 * Unit : 0.29°
 */

uint8_t AX_Get_Current_Speed(AX *servo, AX_Wheel_Direction *direction, uint16_t *speed);
/*
 * Reads the current speed.
 * Range : 0 -> 1023
 * Direction : AX_CLOCKWISE or else AX_COUNTERCLOCKWISE
 * Unit (Join Mode) : 0.111 rpm
 * Unit (Wheel Mode) : approx. 0.1% of maximum
 */

uint8_t AX_Get_Current_Load(AX *servo, AX_Wheel_Direction *direction, uint16_t *load);
/*
 * Reads the current load.
 * Range : 0 -> 1023.
 * Direction : AX_CLOCKWISE or else AX_COUNTERCLOCKWISE
 * Unit : 0.1% of maximum
 */

uint8_t AX_Get_Current_Voltage(AX *servo, uint16_t *voltage);
/*
 * Reads the current voltage.
 * Unit : 0.1V
 */

uint8_t AX_Get_Current_Temperature(AX *servo, uint16_t *temperature);
/*
 * Reads the current temperature.
 * Unit : 1°C.
 */

uint8_t AX_Is_Working(AX *servo, uint16_t *working);
/*
 * Reads the field Torque Enable to figure out if the servo is working.
 * Values : 1 or 0 respectively if the motor is working or not.
 */

uint8_t AX_Is_Moving(AX *servo, uint16_t *moving);
/*
 * Reads the field Moving to figure out if the servo is moving.
 * Values : 1 or 0 respectively if the motor is moving or not.
 */

/*
 * Note : Some fields can be read but doesn't need a dedicated
 * function. They can be easily read with the function AX_Read.
 */

#endif
