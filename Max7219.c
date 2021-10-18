#include "Max7219.h"

/* Variable */
uint8_t Led_code[11]= {0x7E, 0x30, 0x6D, 0x79, 0x33, 0x5B, 0x5F, 0x70, 0x7F, 0x7B, 0x00};
uint8_t Led_codeI[11] = {0x7E, 0x06, 0x6D, 0x4F, 0x17, 0x5B, 0x7B, 0x0E, 0x7F, 0x5F, 0x00};


void max_set_brightness (uint8_t brightness) {
	write_max_cmd(0x0A, brightness);		//Intensity for LED, Max 0x0F
}
void write_data (uint8_t num, uint8_t val)
{
	CS_Pin(0);
	write_byte(num);
	write_byte(val);
	CS_Pin(1);
}
void max_clear (void)
{
	uint8_t i;
	CS_Pin(0);
	for(i = 1; i < 5; i++)
	{
		write_byte(i);
		write_byte(0x00);
	}
	CS_Pin(1);
}
void max_init (uint8_t brightness)
{
	GPIO_Config();
	write_max_cmd(0x09, 0x00);		//No decode mode
	write_max_cmd(0x0A, brightness);		//Intensity for LED, Max 0x0F
	write_max_cmd(0x0B, 0x03);				//Scan Limit 4 LED
	write_max_cmd(0x0C, 0x01);				//Normal Operation
	write_max_cmd(0x0F, 0x00);				//No Test Display
	max_clear();

}
void write_max_cmd (uint8_t address, uint8_t cmd)
{
	CS_Pin(0);
	write_byte(address);
	write_byte(cmd);
	CS_Pin(1);
}
void write_byte (uint8_t byte)
{
	uint8_t i;
	for ( i =0; i<8; i++)
	{
		CLK_Pin(0);  // pull the clock pin low
		DIN_Pin((byte&0x80));  // write the MS0b bit to the data pin
		byte = byte<<1;  // shift left
		CLK_Pin(1);   // pull the clock pin HIGH
	}
}

static void GPIO_Config(void) {
		/* PA0 PA1 PA2 as Output */
	//1. Enable GPIOA clock
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;			//1 << 17
	
	//2. Set the PIN PA5 as output
	
	GPIOA->MODER |= GPIO_MODER_MODER0_0 | GPIO_MODER_MODER1_0 | GPIO_MODER_MODER2_0 ;	
	
	//3. Configure the output mode i.e state, speed, and pull
	GPIOA->OSPEEDR |= GPIO_OSPEEDR_OSPEEDR0 | GPIO_OSPEEDR_OSPEEDR1 | GPIO_OSPEEDR_OSPEEDR2;	//	High Speed
}

