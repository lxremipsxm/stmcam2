/*Siddharth Vasudevan
UART/src/lib/uart.h
Header file for user-defined library
*/

#ifndef UART_H

#define UART_H

#include "stm32f4xx.h"
void uart_com_init(void);
void send_com_string(char message[]);
void send_com_char(char character);

#endif //UART_H