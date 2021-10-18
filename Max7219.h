#include "stm32f030x6.h"

/* Macro */
#define CS_Pin(x) ((x == 1) ? (GPIOA->ODR |= GPIO_ODR_2) : (GPIOA->ODR &= ~GPIO_ODR_2))
#define CLK_Pin(x) ((x == 1) ? (GPIOA->ODR |= GPIO_ODR_1) : (GPIOA->ODR &= ~GPIO_ODR_1))
#define DIN_Pin(x) ((x) ? (GPIOA->ODR |= GPIO_ODR_0) : (GPIOA->ODR &= ~GPIO_ODR_0))
//#define CS_Pin(x) ((x == 1) ? 1 : 2)
void max_init (uint8_t brightness);
void max_clear(void);
void write_max_cmd (uint8_t address, uint8_t cmd);
void write_byte (uint8_t byte);
void write_data (uint8_t num, uint8_t val);
void max_set_brightness (uint8_t brightness);
static void GPIO_Config(void);
