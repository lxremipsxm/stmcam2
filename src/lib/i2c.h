/*OV7670/src/lib/i2c.h*/

#ifndef I2C_H
#define I2C_H
#include <stdint.h>

void i2c_init();

void i2c_tx(uint8_t device_addr, uint8_t reg_address, uint8_t data);

uint8_t i2c_rx(uint8_t device_addr, uint8_t reg_address);

#endif //I2C