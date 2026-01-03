/*Siddharth Vasudevan
UART/src/lib/delays.h
Header file for delays library

*/
#ifndef DELAYS_H

#define DELAYS_H

#include <stdint.h>

void delay_init();
void delay_s(uint32_t sec);
void delay_ms(uint32_t msec);
void delay_us(uint32_t usec);


#endif //DELAYS_H