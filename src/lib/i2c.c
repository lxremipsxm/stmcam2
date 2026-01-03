/*OV7670/src/lib/i2c.c*/
#include "stm32f401xe.h"


void i2c_init(){
    /* 
    Program peripheral input clock >= 4 MHz CR2
    config control clock registers
    config rise time reg
    program i2c_cr1 reg en
    set start bit in i2c_cr1 to generate start condition  
    */

    //Enable I2C 1 clock
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
    
    I2C1->CR1 &= ~(I2C_CR1_PE); //disable I2C for setting values


    I2C1->CR2 = 42; //APB1 clock speed

    //config control clock registers and rise time here

    /*
    Fast mode, DUTY = 0

    PCLK = 42 MHz
    Tpclk = 1/42000000 = 2.38e-8s = 23.8 ns
    if we want 400kHz, we need CCR*Tpclk = 1/3*400000 = 2.5e-6/3 s= 2500 ns/3 = 833 ns

    then CCR = 833.33/23.8 ~= 35.01  

    For TRISE:

    tmax/Tpclk + 1 = 300/23.8 + 1 ~= 13

    */

    I2C1->CCR = 0; //reset CCR entirely

   
    I2C1->CCR &= ~(1 << I2C_CCR_DUTY); //set DUTY to 0
    
    I2C1->CCR |= (1<< I2C_CCR_FS); //Set Fast mode
    I2C1->CCR |= (35 & 0x0FFF); //as calculated above for a SCL frequency of 400kHz
    

    I2C1->TRISE = (13 & 0x3F); //trise=13

    //Enable peripheral
    I2C1->CR1 |= I2C_CR1_PE; //enable

}

void i2c_tx(uint8_t device_addr, uint8_t reg_address, uint8_t data){
    //assumes i2c_init() is already called

    //generate start condition
    I2C1->CR1 |= (1<<I2C_CR1_START);

    //wait for sb flag in status register to be set by hardware
    while(!(I2C1->SR1 & I2C_SR1_SB));

    //send address of device to comm with
    I2C1->DR = (device_addr << 1) | 0;

    while(!(I2C1->SR1 & I2C_SR1_ADDR)); //wait for flag to set

    volatile uint32_t tmp1 = I2C1->SR1;
    volatile uint32_t tmp2 = I2C1->SR2; //this forces ADDR to 0

    I2C1->DR = reg_address; //send address of register

    while(!(I2C1->SR1 & I2C_SR1_TXE)); //wait for transmission to finish

    I2C1->DR = data;

    while(!(I2C1->SR1 & I2C_SR1_BTF)); //send data to register

    I2C1->CR1 |= I2C_CR1_STOP; //stop transmission 

}


uint8_t i2c_rx(uint8_t device_addr, uint8_t reg_address){
    //assumes i2c_init() is already called
    
    I2C1->CR1 |= I2C_CR1_START; //start condition
    
    while(!(I2C1->SR1 & I2C_SR1_SB));

    I2C1->DR = (device_addr << 1) | 0; //write

    while(!(I2C1->SR1 & I2C_SR1_ADDR));

    uint32_t tmp = I2C1->SR1;
    uint32_t tmp = I2C1->SR2; //clears ADDR


    I2C1->DR = reg_address; //send reg to comm with

    while(!(I2C1->SR1 & I2C_SR1_TXE));


    I2C1->CR1 |= I2C_CR1_START; //repeated start for receive
    while(!(I2C1->SR1 & I2C_SR1_SB));


    I2C1->DR = (device_addr << 1) | 1; //send device addr
    while(!(I2C1->SR1 & I2C_SR1_ADDR));

    tmp = I2C1->SR1;
    tmp = I2C1->SR2;//clears addr


    I2C1->CR1 &= ~I2C_CR1_ACK; //clear ACK

    I2C1->CR1 |= I2C_CR1_STOP; //stop bit

    while(!(I2C1->SR1 & I2C_SR1_RXNE)); //wait for reception

    return I2C1->DR;
}