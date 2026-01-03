/*Siddharth Vasudevan
OV7670/src/main.c

This code contains test code for the OV7670 camera module using the OV7670 Adafruit library

Comments
--------
6/2/25: Created file for testing OV7670
*/

#include "stm32f4xx.h"
#include "lib/delays.h"
#include "lib/uart.h"


void main(void){
    uart_com_init();
    delay_init();

    while(1){

        char strs[3][10] = {"interior", "crocodile", "alligator"};
        for (int i=0; i<3; i++){
            send_com_string(strs[i]);
            send_com_string("\r\n");
            delay_ms(500);
        }
        
    }
}