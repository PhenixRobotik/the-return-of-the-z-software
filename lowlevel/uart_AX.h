#pragma once


// DEFINES FOR AX
#define AX_RCC_USART RCC_USART2
#define AX_USART USART2
//#define AX_UART_SPEED (9600)
#define AX_UART_SPEED (1000000)

#define AX_PORT_TX GPIOB
#define AX_PORT_TX_RCC RCC_GPIOB // clock of GPIO port
#define AX_PIN_TX GPIO3
#define AX_AF_TX GPIO_AF7

#define AX_PORT_RX GPIOB
#define AX_PORT_RX_RCC RCC_GPIOB
#define AX_PIN_RX GPIO4
#define AX_AF_RX GPIO_AF7

void ax_uart_setup();
void ax_uart_send_string(char* chain); // NULL-ended string
