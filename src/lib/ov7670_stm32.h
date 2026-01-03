/*Siddharth Vasudevan

OV7670/ov7670_stm32.h

Header for ov7670_stm32.c, adapted from Adafruit_OV7670:
https://github.com/adafruit/Adafruit_OV7670/tree/master


Comments
--------
6/10/25: Created file, figuring out details. I will most likely move this file 
into the inc/ folder directly so I don't have to adjust my entire working directory.

License and Credit
------------------
SPDX-FileCopyrightText: 2020 P Burgess for Adafruit Industries
SPDX-License-Identifier: MIT
*/

#pragma once

#include "inc/stm32f4xx.h"

#if defined(STM32F4)
#if defined(ARDUINO)
#include <Arduino.h>
#else
#include <stdbool.h>
#include <stdint.h>
#endif // end platforms

typedef int8_t OV7670_pin;

// OV7670 datasheet claims 10-48 MHz clock input, with 24 MHz typical.
// If any trouble, try dialing this down to 16 or 12 MHz.
//---------------^paraphrased from previous author^------------------------

// I'll start with a ~20MHz clock. Since APB1 peripherals have a max 
//clock of 42MHz, I can prescale it by 2 to get 21MHz. If I have trouble with it,
//I'll decrease it.

#define OV7670_XCLK_HZ 21000000 ///< XCLK to camera, 8-24 MHz

// Device-specific structure attached to the OV7670_host.arch pointer.
typedef struct {
  void *timer;    ///< TC or TCC peripheral base address for XCLK out
  bool xclk_pdec; ///< If true, XCLK needs special PDEC pin mux
} OV7670_arch;


#ifdef __cplusplus
extern "C" {
#endif

extern void OV7670_capture(uint32_t *dest, uint16_t width, uint16_t height,
                           volatile uint32_t *vsync_reg, uint32_t vsync_bit,
                           volatile uint32_t *hsync_reg, uint32_t hsync_bit);

#ifdef __cplusplus
};
#endif

#endif // STM32F4