/**
  ******************************************************************************

  Inter - Intergrated Circuit module 1 Setup For STM32F446RE
  Author:   Sang Truong Tan
  Updated:  August 23rd, 2021

  ******************************************************************************
  Copyright (C) 2021 Truong Tan Sang

  This is a free software under the GNU license, you can redistribute it and/or modify it under the terms
  of the GNU General Public License version 3 as published by the Free Software Foundation.
  This software library is shared with public for educational purposes, without WARRANTY and Author is not liable for any damages caused directly
  or indirectly by this software, read more about this on the GNU General Public License.

  ******************************************************************************
*/

/**
  * @brief  Intergrated Circuit module 1
  *         The system I2C1 is configured as follow : 
	*					Standard mode
	*					GPIOB
	*					SCL -> PINB 8
	*					SDA -> PINB 9
  * @param  None
  * @retval None
  */


#include "stm32f030x6.h"
#include "stdlib.h"


void I2C1_Config (void);
static void I2C1_Start (void);
static int I2C1_Write (uint8_t* dataW, uint8_t size);
static void I2C1_SendAdd (uint8_t add);
static void I2C1_Read ( uint8_t *dataR, uint8_t size);
static void I2C1_Stop (void) __attribute__((unused));

void I2C1_Transmit (uint8_t add, uint8_t* dataW, uint8_t size);
void I2C1_Receive (uint8_t add, uint8_t* dataR, uint8_t size);
void I2C1_Mem_Read (uint8_t add, uint8_t reg, uint8_t* dataR, uint8_t size);
void I2C1_Mem_Write (uint8_t add, uint8_t reg, uint8_t* dataW, uint8_t size);
