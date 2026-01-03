/*Siddharth Vasudevan
UART/src/lib/uart.c
Testing + developing usart user-defined library
Ideally, output from this code should be readable by putty once flashed onto STM32F4

Comments
--------
5/29/25: Test successful
*/

#include "stm32f4xx.h"
#include "uart.h"
#include <stdint.h> 


void uart_com_init(void) {    
//Set up peripheral clock on GPIO A
RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

//Enable USART2 clock
RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

//Enable alternate function (USART) on GPIO
GPIOA->MODER |= ((0x2) << (2*2)); //Selects Alternate function mode (10 * 2^18)
GPIOA->AFR[0] |= (0x7 << (2)*4); //Selects Alternate function 7 on pin A2, USART

//Program M bit for 9 bits
//USART2->CR1 |= USART_CR1_M;

//Set baud rate with BRR register
USART2->BRR = 0x683;

//Enable USART with UE bit
USART2->CR1 |= USART_CR1_UE;

//Set TE in CR1 to send idle frame as first transmission
USART2->CR1 |= USART_CR1_TE;}


void send_com_char(char character){
    while(!(USART2->SR & USART_SR_TXE)){/*wait*/}
        USART2->DR = character;
}


void send_com_string(char message[]) {

    for (int i=0; message[i] != '\0'; i++){
        send_com_char(message[i]);
    }
}

