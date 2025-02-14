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


#include "platform.h"

uint8_t VL53L4CDduino_I2CRead(uint16_t dev, uint8_t deviceAddr, uint16_t registerAddr, uint8_t* p_values, uint32_t size){
	int status = 0;
	uint8_t buffer[2];

	
}


uint8_t VL53L4CD_RdDWord(uint16_t dev, uint16_t RegisterAdress, uint32_t *value)
{
	uint8_t status = 0;
	uint8_t buffer[4] = {0,0,0,0};

	
	
	/* To be filled by customer. Return 0 if OK */
	/* Warning : For big endian platforms, fields 'RegisterAdress' and 'value' need to be swapped. */
	
	return status;
}

uint8_t VL53L4CD_RdWord(uint16_t dev, uint16_t RegisterAdress, uint16_t *value)
{
	uint8_t status = 255;
	
	/* To be filled by customer. Return 0 if OK */
	/* Warning : For big endian platforms, fields 'RegisterAdress' and 'value' need to be swapped. */
	
	return status;
}

uint8_t VL53L4CD_RdByte(uint16_t dev, uint16_t RegisterAdress, uint8_t *value)
{
	uint8_t status = 255;
	
	/* To be filled by customer. Return 0 if OK */
	/* Warning : For big endian platforms, fields 'RegisterAdress' and 'value' need to be swapped. */
	
	return status;
}

uint8_t VL53L4CD_WrByte(uint16_t dev, uint16_t RegisterAdress, uint8_t value)
{
	uint8_t status = 255;

	/* To be filled by customer. Return 0 if OK */
	/* Warning : For big endian platforms, fields 'RegisterAdress' and 'value' need to be swapped. */
	
	return status;
}

uint8_t VL53L4CD_WrWord(uint16_t dev, uint16_t RegisterAdress, uint16_t value)
{
	uint8_t status = 255;
	
	/* To be filled by customer. Return 0 if OK */
	/* Warning : For big endian platforms, fields 'RegisterAdress' and 'value' need to be swapped. */
	
	return status;
}

uint8_t VL53L4CD_WrDWord(uint16_t dev, uint16_t RegisterAdress, uint32_t value)
{
	uint8_t status = 255;

	/* To be filled by customer. Return 0 if OK */
	/* Warning : For big endian platforms, fields 'RegisterAdress' and 'value' need to be swapped. */
	
	return status;
}

uint8_t VL53L4CD_WaitMs(uint16_t dev, uint32_t TimeMs)
{
	uint8_t status = 255;
	/* To be filled by customer */
	return status;
}
