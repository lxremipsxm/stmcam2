/*Siddharth Vasudevan
OV7670/ov7670_stm32.c

This file provides architecture specific data so that the ov7670 can be interfaced
with my STM32F401RE. I've named the file ov7670_stm32.c (as opposed to just stm32f401re.c, or something similar)
only to avoid file name similarities between this file and the other dependencies I need to compile 
and flash code to the board. Additionally, I want it to be clear that this is not one of the main dependencies
for having code compile correctly for the board, but rather a requirement for the ov7670 to work.
This code will likely only work with stm32f401xe boards. For now, I will work in the main project directory, 
but I will move it into inc/ later.

Comments
--------
6/10/25: Created file

License and Credit
------------------
SPDX-FileCopyrightText: 2020 P Burgess for Adafruit Industries
SPDX-License-Identifier: MIT
*/

#include "inc/stm32f4xx.h"
#if defined(STM32F4)
#include "ov7670.h"
#include "ov7670_stm32.h"
#include "src/lib/delays.h"
#endif




/*
Additional methods I must define(ensure they are defined before the two OV7670 methods):
- OV7670_delay_ms(x)
- OV7670_pin_output(pin)
- OV7670_pin_write(pin, hi)
- OV7670_disable_interrupts() - tentative
- OV7670_enable_interrupts() - tentative*/


/*I also need to decide how to connect this thing to my STM32. 

Pins on OV7670 (Total 18)
-------------------------
Digital: {D0,...,D7}, I2C: {SCL, SDA}, Clock in: {XCLK}
Clock out: {PLK}, Power: {3.3V, DGND, PWDN}, Reset: {RET}
Sync: {VS, HS}

Need a timer/PWM pin for XCLK, 8 digital pins, Default I2C (SDA, SCL), unsure about clock out (PCLK) for now
Power is obvious; one GPIO should be dedicated to PWDN and RET each

In all, it seems I need 10 GPIOs, 8 input from OV7670 and 2 output to OV7670. 
This is how I'll connect the OV7670:

 Left side
-----------
  3.3V 3.3V
  scl 
  vsync 
  pclk PA6 - TIMER IN
  d7 PB7
  d5 PB5
  d3 PB3
  d1 PB1
  ret PB10


Right side
----------
  DGND GND
  sda 
  hsync 
  xclk PA8 TIMER OUT
  d6 
  d4 
  d2 
  d0 
  enable

*/


void OV7670_delay_ms(int x){
  //Assume delay_init() is already called. Ensure this is done in the main file.
  delay_ms(x);
}


void OV7670_pin_output(int pin){
  // sets a pin to output

  // Looking at the OV7670_pins type in ov7670.h, I need to worry about {enable, reset, xclk, pclk, vsync, hsync,
  // data[8], sda, and scl}. This represents all the pins minus the power pins. However, the only actual pins that
  // ever need to be set to output are the reset and enable pins, so I can be pretty lax about how I do this.
  // I'm thinking a boolean-eque either-this-or-that kind of output setting, and do the same for pin_write.
  
  //Both enable and reset are on GPIO B, so I'll enable the clock for it outside the if block
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;

  if (pin == 0){ //enable pin
    GPIOB->MODER &= ~((0x3)<<(2*1)); //clears mode register
    GPIOB->MODER |= ((0x1) << (2*1)); //sets B1 to output
  }

  else if (pin == 1){ //using an else if for reset for robustness
    GPIOB->MODER &= ~((0x3) << (2*10)); //clears mode register
    GPIOB->MODER |= ((0x1) << (2*10)); //sets B10 to output
  }
}


void OV7670_pin_write(int pin, int hi){

  //Again, since the only pins that are declared output are enable and reset, this will be
  //quite straightforward.

  if (pin == 0){
    if (hi == 0){GPIOB->ODR &= ~(1<<1);} //set PB1 to low
    else {GPIOB->ODR |= (1<<1);} //set PB1 to high
  } 

  else if (pin == 1){
    if (hi == 0){GPIOB->ODR &= ~(1<<10);} //set PB10 to low
    else {GPIOB->ODR |= (1<<10);} //set PB10 to high
  }
}


void OV7670_disable_interrupts(void){}


void Ov7670_enable_interrupts(void){}


OV7670_status OV7670_arch_begin(OV7670_host *host) {

    //Setup timer as required: This is not flexible, and will be defined for a single timer
    //as that is all I need to supply to XCLK

    //The code for the SAMD5 uses PCC, or parallel capture controller. I am not aware of whether 
    //STM32F01RE has a PCC equivalent, so I will do some more digging to find an equivalent or alternative

    //XCLK on PA8, Timer 1, channel 1
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN; //enable clock on TIM1
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; //enable clock on GPIOA
    
    GPIOA->MODER &= ~(0x3 << (2*8)); //clear mode register
    GPIOA->MODER |= ((0x2) << (2*8)); //Sets Alternate function mode on A8
    
    GPIOA->AFR[1] &= ~((0xF) << (4*(8-8))); //Clear alternate function register
    GPIOA->AFR[1] |= ((0x1)<< (4*(8-8))); //set alternate function to TIM1, channel 1


    //For 20MHz, we need 2000000 pulses a second. According to the original author, a 50% duty cycle is
    //recommended. I'll look at the original code to get an idea of how the author did this for SAMD51
    //It seems their peripheral clock had a frequency of 48MHz, which they did not prescale, and then 
    //used the value OV7670_XCLK_HZ in 'ov7670.h' to calculate the period in one line. I'll replicate this. 
    //The frequency of APB2 peripherals is 84MHz according to the datasheet.

    TIM1->PSC = 0; //no prescaler
    TIM1->CR1 = 0; //reset control register
    TIM1->CNT = 0; //reset count
    uint16_t period = 84000000 / OV7670_XCLK_HZ - 1;



    TIM1->EGR |= TIM_EGR_UG; //STM32 docs say to enable update events 

    TIM1->ARR = period; //set auto-reload register
    TIM1->CCR1 = (period+1)/2; //set capture/compare register. I'm directly copying the calculation the og author used 
    
    TIM1->CCMR1 &= ~((0x7) << 4); //clear OC1M bits in capture/compare mode register
    TIM1->CCMR1 |= ((0x6) << 4); //set to PWM mode
    
    TIM1->CCER |= TIM_CCER_CC1E; //enable channel 1
    TIM1->CCMR1 |= TIM_CCMR1_OC1PE; //set preload enable
    TIM1->CR1 |= TIM_CR1_CEN; //start counter channel 1
    TIM1->BDTR |= TIM_BDTR_MOE; //enable output (PA8 will output this as it is configured to output)


    //set up DMA peripheral-to-memory mode. DMA should be triggered by a timer that toggles based on PCLK (PA6), then grab 8 bits from 
    //GPIO B, preferably PORTB[7:0], and send to memory via double-buffering mode. This should enable the highest possible 
    //frame rate from the camera.

    //Set up PCLK input capture
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN; //Enable DMA1 clock
    DMA1_Stream4->CR &= ~DMA_SxCR_EN; //Disable DMA1 
    DMA1_Stream4->CR &= ~((0x3)<<DMA_SxCR_DIR_Pos); //Peripheral-to-memory mode

    //Set up PA6 input capture mode: Takes input from PCLK
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN; //Enable timer 3
    TIM3->SMCR |= (0x7<<TIM_SMCR_SMS); // Enable external clock source mode 1
    TIM3->CCMR1 &= ~(0b11<<TIM_CCMR1_CC2S); //Clear CC2S bits
    TIM3->CCMR1 |= 0b01<<TIM_CCMR1_CC2S; //Set CC2S bits for detecting TI2 input capture
    TIM3->CCMR1 &= ~TIM_CCMR1_IC2F;//No filter for now

    TIM3->CCER &= ~TIM_CCER_CC1P; //Set rising edge polarity
    TIM3->CCER &= ~TIM_CCER_CC1NP; //Also for setting rising edge polarity
    
    TIM3->SMCR &= ~(0x7<<TIM_SMCR_TS); //Clearing TS bits
    TIM3->SMCR |= (0x6<<TIM_SMCR_TS); //Setting TS bits to 110
    
    TIM3->CR1 |= TIM_CR1_CEN; //Tim3 enabled. In this mode, at every rising edge, TIF flag is set. 

    //Setting up DMA1
    //The number of DMA captures that will be made is stored in NDTR. The camera has a resolution of 640x480, so that's 307 200 pixels.
    //Since I'm capturing line-by-line, and each pixel is 8 bits, I need to capture 640 bytes of information. Each DMA transfer
    //transfers one byte from data[7:0] to memory until 640 pixels have been transferred. 
    //This repeats until the number of rows matches the height of the resolution (480 in my case)
    //Then, DMA switches over to the second memory buffer. While DMA is filling the second buffer, I can drain the first one.

    DMA1_Stream4->CR |= DMA_SxCR_DBM; //turn on double buffer mode

    //Setting Memory and Peripheral widths
    DMA1_Stream4->CR &= ~(0x3 << DMA_SxCR_MSIZE); //One byte
    DMA1_Stream4->CR &= ~(0x3 << DMA_SxCR_PSIZE); //One byte

    DMA1_Stream4->PAR = (uint32_t)&GPIOB->IDR; //Peripheral Location


    //Memory Buffers for DBM
    volatile uint8_t buf0[640];
    volatile uint8_t buf1[640];

    DMA1_Stream4->M0AR = buf0;
    DMA1_Stream4->M1AR = buf1;
    
    DMA1_Stream4->NDTR = 640; //total number of beats to be captured by DMA

    DMA1_Stream4->CR |= DMA_SxCR_TCIE; //Enable transfer complete interrupt

  return OV7670_STATUS_OK;
}


void OV7670_capture(uint32_t *dest, uint16_t width, uint16_t height,
                    volatile uint32_t *vsync_reg, uint32_t vsync_bit,
                    volatile uint32_t *hsync_reg, uint32_t hsync_bit) {

   
}

