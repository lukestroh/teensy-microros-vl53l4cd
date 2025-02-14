/**
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */


#ifndef _PLATFORM_H_
#define _PLATFORM_H_
#pragma once

#include <stdint.h>
#include <string.h>

#include <Arduino.h>
#include <Wire.h>

/**
* VL53L4CD device instance.
*/

typedef uint16_t Dev_t;

/**
 * @brief Error instance.
 */
typedef uint8_t VL53L4CD_Error;

/**
 * @brief If the macro below is defined, the device will be programmed to run
 * with I2C Fast Mode Plus (up to 1MHz). Otherwise, default max value is 400kHz.
 */

//#define VL53L4CD_I2C_FAST_MODE_PLUS

typedef struct {
  // I2C address
  uint16_t address;
  // TwoWire* dev_i2c;
} VL53L4CD_Platform;

uint8_t VL53L4CDduino_I2CRead(VL53L4CD_Platform* p_platform, uint8_t deviceAddr, uint16_t registerAddr, uint8_t* p_values, uint32_t size);


/**
 * @brief Read 32 bits through I2C.
 */

uint8_t VL53L4CD_RdDWord(VL53L4CD_Platform* p_platform, uint16_t registerAddr, uint32_t *value);
/**
 * @brief Read 16 bits through I2C.
 */

uint8_t VL53L4CD_RdWord(VL53L4CD_Platform* p_platform, uint16_t registerAddr, uint16_t *value);

/**
 * @brief Read 8 bits through I2C.
 */

uint8_t VL53L4CD_RdByte(VL53L4CD_Platform* p_platform, uint16_t registerAddr, uint8_t *value);

/**
 * @brief Write 8 bits through I2C.
 */

uint8_t VL53L4CD_WrByte(VL53L4CD_Platform* p_platform, uint16_t registerAddr, uint8_t value);

/**
 * @brief Write 16 bits through I2C.
 */

uint8_t VL53L4CD_WrWord(VL53L4CD_Platform* p_platform, uint16_t RegisterAdress, uint16_t value);

/**
 * @brief Write 32 bits through I2C.
 */

uint8_t VL53L4CD_WrDWord(VL53L4CD_Platform* p_platform, uint16_t RegisterAdress, uint32_t value);

/**
 * @brief Wait during N milliseconds.
 */

uint8_t VL53L4CD_WaitMs(VL53L4CD_Platform* p_platform, uint32_t TimeMs);

#endif	// _PLATFORM_H_