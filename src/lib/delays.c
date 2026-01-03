/*Siddharth Vasudevan
UART/src/lib/delays.c

Comments
--------
5/29/25: Test successful, but delays not accurate at all. Will improve with STM32 TIM based delays
5/30/25: Recreating delay_ms with TIM2
*/

#include "stm32f4xx.h"
#include <stdint.h>
#include "delays.h"

void delay_init(){
    //Enable TIM2 General purpose timer clock
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

    //set clock prescaler
    TIM2->PSC = (SystemCoreClock/1000000)-1; //16000000/s / 16 => 1/us

    //reset timer config entirely
    TIM2->CR1 = 0;
}

void delay_us(volatile uint32_t usec){
    //set max
    TIM2->ARR = usec;

    //Reset flags
    TIM2->SR &= ~TIM_SR_UIF;

    //set count to 0
    TIM2->CNT = 0;

    //enable update flag
    //TIM2->EGR |= TIM_EGR_UG;

    //enable counter
    TIM2->CR1 |= TIM_CR1_CEN;

    while (!(TIM2->SR & TIM_SR_UIF)){/*wait for overflow*/}

    TIM2->CR1 &= ~TIM_CR1_CEN;

}

void delay_ms(volatile uint32_t msec) {
    while (msec--){
        delay_us(1000);
    }
}

void delay_s(volatile uint32_t sec) {
    while(sec--){
        delay_ms(1000);
    }
}