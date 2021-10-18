/**
  ******************************************************************************

  I2C1 Setup For STM32F446RE
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
#include "I2C1_F0.h"
void I2C1_Mem_Write (uint8_t add, uint8_t reg, uint8_t* dataW, uint8_t size) {
	uint8_t* ptr =(uint8_t*) malloc(size + 1);
	uint8_t i;
	ptr[0] = reg;
	for(i = 1; i < size + 1; i++)
	{
		ptr[i] = dataW[i - 1];
	}
	if(I2C1->ISR & I2C_ISR_TIMEOUT)
		I2C1->ICR = I2C_ICR_TIMOUTCF;
	if(!(I2C1->ISR & I2C_ISR_BUSY)) {
		I2C1->CR2 = 0x00;
		I2C1->CR2 |= (size + 1) << I2C_CR2_NBYTES_Pos;
		I2C1->CR2 &= ~I2C_CR2_AUTOEND;
		I2C1->CR2 &= ~I2C_CR2_RD_WRN;		//Write
		I2C1_SendAdd(add);	//Write Condition		//Must be set up address
		I2C1_Start();
		I2C1_Write(ptr, size + 1);
		while((!(I2C1->ISR & I2C_ISR_TC)) && (!(I2C1->ISR & I2C_ISR_TIMEOUT)));
		I2C1_Stop();
	}
	else {
		I2C1->CR1 &= ~I2C_CR1_PE;
		__NOP();
		__NOP();
		__NOP();
		I2C1->CR1 |= I2C_CR1_PE;
	}
	free(ptr);
}
void I2C1_Mem_Read (uint8_t add, uint8_t reg, uint8_t* dataR, uint8_t size) {
	if(I2C1->ISR & I2C_ISR_TIMEOUT)
		I2C1->ICR = I2C_ICR_TIMOUTCF;
	if(!(I2C1->ISR & I2C_ISR_BUSY)) {
		I2C1->CR2 = 0x00;
		I2C1->CR2 |= 1 << I2C_CR2_NBYTES_Pos;
		I2C1->CR2 &= ~I2C_CR2_AUTOEND;
		I2C1->CR2 &= ~I2C_CR2_RD_WRN;		//Write
		I2C1_SendAdd(add);	//Write Condition		//Must be set up address
		I2C1_Start();
		I2C1_Write(&reg, 1);
		while((!(I2C1->ISR & I2C_ISR_TC)) && (!(I2C1->ISR & I2C_ISR_TIMEOUT)));
		I2C1->CR2 = 0x00;
		I2C1->CR2 |=size << I2C_CR2_NBYTES_Pos;		
		I2C1->CR2 &= ~I2C_CR2_AUTOEND;	
		I2C1->CR2 |= I2C_CR2_RD_WRN;		//Read
		I2C1_SendAdd(add);	//Read Condition		//Must be set up address
		I2C1_Start();
		I2C1_Read(dataR, size - 1);
		I2C1_Stop();
		I2C1_Read(&dataR[size - 1], 1);
		while((!(I2C1->ISR & I2C_ISR_STOPF)) && (!(I2C1->ISR & I2C_ISR_TIMEOUT)));
		I2C1->ICR = I2C_ICR_STOPCF;
	}
	else {
		I2C1->CR1 &= ~I2C_CR1_PE;
		__NOP();
		__NOP();
		__NOP();
		I2C1->CR1 |= I2C_CR1_PE;
	}
}


void I2C1_Transmit (uint8_t add, uint8_t* dataW, uint8_t size) {
	I2C1->CR2 = 0x00;
	I2C1->CR2 |=size << I2C_CR2_NBYTES_Pos;		
	I2C1->CR2 |= I2C_CR2_AUTOEND;
	I2C1->CR2 &= ~I2C_CR2_RD_WRN;		//Write
	I2C1_SendAdd(add);	//Write Condition		//Must be set up address
	I2C1_Start();
	I2C1_Write(dataW, size);
//	I2C1_Stop();
}


void I2C1_Receive (uint8_t add, uint8_t* dataR, uint8_t size) {
	I2C1->CR2 |=size << I2C_CR2_NBYTES_Pos;		
	I2C1->CR2 |= I2C_CR2_AUTOEND;	
	I2C1->CR2 |= I2C_CR2_RD_WRN;		//Read
	I2C1_SendAdd(add);	//Read Condition		//Must be set up address
	I2C1_Start();
	I2C1_Read(dataR, size);
}

static void I2C1_Stop (void) {
	I2C1->CR2 |= I2C_CR2_STOP; //Stop generation after current byte
	
}

static void I2C1_Read ( uint8_t *dataR, uint8_t size) {
	/**** STEPS FOLLOWED  ************
	1. If only 1 BYTE needs to be Read
		a) Write the slave Address, and wait for the ADDR bit (bit 1 in SR1) to be set
		b) the Acknowledge disable is made during EV6 (before ADDR flag is cleared) and the STOP condition generation is made after EV6
		c) Wait for the RXNE (Receive Buffer not Empty) bit to set
		d) Read the data from the DR

	2. If Multiple BYTES needs to be read
		a) Write the slave Address, and wait for the ADDR bit (bit 1 in SR1) to be set
		b) Clear the ADDR bit by reading the SR1 and SR2 Registers
		c) Wait for the RXNE (Receive buffer not empty) bit to set
		d) Read the data from the DR 
		e) Generate the Acknowlegment by settint the ACK (bit 10 in SR1)
		f) To generate the nonacknowledge pulse after the last received data byte, the ACK bit must be cleared just after reading the 
			second last data byte (after second last RxNE event)
		g) In order to generate the Stop/Restart condition, software must set the STOP/START bit 
			after reading the second last data byte (after the second last RxNE event)
	*/
	int remaining = size;

	
	while(remaining >= 1) {
			//c) Wait for the RXNE (Receive buffer not empty) bit to set
	while((!(I2C1->ISR & I2C_ISR_RXNE)) && (!(I2C1->ISR & I2C_ISR_TIMEOUT)));
//		while((!(I2C1->ISR & I2C_ISR_RXNE)));
	if(I2C1->ISR & I2C_ISR_TIMEOUT)
		break;
			//d) Read the data from the DR 
			dataR[size - remaining] = (uint8_t) I2C1->RXDR;
			remaining -=  1;
		}
	//3. Wait for data transmission successful
}


static void I2C1_SendAdd (uint8_t add) {
	/**** STEPS FOLLOWED  ************
	1. Send the Slave Address to the DR Register
	2. Wait for the ADDR (bit 1 in SR1) to set. This indicates the end of address transmission
	3. clear the ADDR by reading the SR1 and SR2
	*/
	
	//1. Send the Slave Address to the DR Register
	I2C1->CR2 |= add << 1;
	
	
}
static int I2C1_Write (uint8_t* dataW, uint8_t size) {
	/**** STEPS FOLLOWED  ************
	1. Wait for the TXE (bit 7 in SR1) to set. This indicates that the DR is empty
	2. Send the DATA to the DR Register
	3. Wait for the BTF (bit 2 in SR1) to set. This indicates the end of LAST DATA transmission
	*/
	int remaining  =  size;
	//Clear timout flag
//	if(I2C1->ISR & I2C_ISR_TIMEOUT)
//		I2C1->ICR = I2C_ICR_TIMOUTCF;		//Clear flag
	//Set number of byte to Write
	while(remaining >= 1) {
		
	//1. Wait for TX Register empty
	while((!(I2C1->ISR & I2C_ISR_TXIS)) && (!(I2C1->ISR & I2C_ISR_TIMEOUT)));
//	while((!(I2C1->ISR & I2C_ISR_TXIS)));
	if(I2C1->ISR & I2C_ISR_TIMEOUT)
		return 0;
	//2. Send the DATA to the DR Register
	I2C1->TXDR = dataW[size - remaining];
	
	remaining -= 1;
		if (I2C1->ISR & I2C_ISR_NACKF) {
			I2C1->ICR = I2C_ICR_NACKCF;
			return 0;
		}
	}
	//3. Wait for data transmission successful
	return 1;
}

static void I2C1_Start (void) {
	/**** STEPS FOLLOWED  ************
	1. Enable the ACK
	2. Send the START condition 
	3. Wait for the SB ( Bit 0 in SR1) to set. This indicates that the start condition is generated
	*/
	//Set bit Start
	I2C1->CR2 |= I2C_CR2_START;	//Start bit
	
}

void I2C1_Config (void) {
	/**** STEPS FOLLOWED  ************
	1. Enable the I2C CLOCK and GPIO CLOCK
	2. Configure the I2C PINs for ALternate Functions
		a) Select Alternate Function in MODER Register
		b) Select Open Drain Output 
		c) Select High SPEED for the PINs
		d) Select Pull-up for both the Pins
		e) Configure the Alternate Function in AFR Register
	3. Reset the I2C 	
	4. Program the peripheral input clock in I2C_CR2 Register in order to generate correct timings
	5. Configure the clock control registers
	6. Configure the rise time register
	7. Program the I2C_CR1 register to enable the peripheral
	*/
	
	//1. Enable the I2C CLOCK and GPIO CLOCK
	RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;		//1 << 21
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN; 	//1 << 1;
	
	//2. Configure the I2C PINs for ALternate Functions
	//a) Select Alternate Function in MODER Register
	GPIOA->MODER |= GPIO_MODER_MODER9_1;	//Alternate Function
	GPIOA->MODER |= GPIO_MODER_MODER10_1;	//Alternate Function
	
	//b) Select Open Drain Output 
	GPIOA->OTYPER |= GPIO_OTYPER_OT_9;		//Open Drain
	GPIOA->OTYPER |= GPIO_OTYPER_OT_10;
	//c) Select High SPEED for the PINs
	GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR9;	//High Speed
	GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR10;
	
	//d) Select Pull-up for both the Pins
	GPIOA->PUPDR |= GPIO_PUPDR_PUPDR9_0;	//Pull up
	GPIOA->PUPDR |= GPIO_PUPDR_PUPDR10_0;
	//e) Configure the Alternate Function in AFR Register
	GPIOA->AFR[1] |= 4 << GPIO_AFRH_AFRH1_Pos;		//
	GPIOA->AFR[1] |= 4 << GPIO_AFRH_AFRH2_Pos;
	
	//3. Reset the I2C 
//	I2C1->CR1 |= (1 << 15);
//	I2C1->CR1 &= ~ (uint32_t) (1 << 15);
//	
	//Config Clock
//	I2C1->TIMINGR |= 3 << I2C_TIMINGR_PRESC_Pos;
//	I2C1->TIMINGR |= 4 <<I2C_TIMINGR_SCLDEL_Pos;
//	I2C1->TIMINGR |= 2 << I2C_TIMINGR_SDADEL_Pos;
//	I2C1->TIMINGR |= 0x0F << I2C_TIMINGR_SCLH_Pos;
//	I2C1->TIMINGR |= 0x13 << I2C_TIMINGR_SCLL_Pos;
	I2C1->TIMINGR = 0x2000090E;
	
	//Config Timout 
	I2C1->TIMEOUTR |= I2C_TIMEOUTR_TIMOUTEN;	///Enable timout
	I2C1->TIMEOUTR |= 99 << 1; //Set timeout 100*2048*ti2cclk
	//7. Program the I2C_CR1 register to enable the peripheral
	I2C1->CR1 |= I2C_CR1_PE; 		//1 << 0
	
}
