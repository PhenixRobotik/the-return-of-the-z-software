#pragma once

/***************************************
simple uart configuration for debugging
***************************************/

// DEFINES FOR DEBUG_UART
#define DEBUG_RCC_USART RCC_USART1
#define DEBUG_USART USART1
#define DEBUG_UART_SPEED (9600*2)
// Bug in libopencm3 ? Need to *2 because of APB2/APB1 weird things.

#define DEBUG_PORT_TX GPIOB
#define DEBUG_PORT_TX_RCC RCC_GPIOB // clock of GPIO port
#define DEBUG_PIN_TX GPIO6
#define DEBUG_AF_TX GPIO_AF7

#define DEBUG_PORT_RX GPIOB
#define DEBUG_PORT_RX_RCC RCC_GPIOB
#define DEBUG_PIN_RX GPIO7
#define DEBUG_AF_RX GPIO_AF7

void uart_setup();
void uart_send_string(char* chain); // NULL-ended string
void uart_send_int(int integer);

//rec=usart_recv_blocking(DEBUG_USART);//to receive a byte
