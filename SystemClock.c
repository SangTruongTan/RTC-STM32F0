#include "SystemClock.h"

void SysClockConfig (void)
	{
		/*************>>>>>>> STEPS FOLLOWED <<<<<<<<************
	
	1. ENABLE HSE and wait for the HSE to become Ready
	2. Set the POWER ENABLE CLOCK and VOLTAGE REGULATOR
	3. Configure the FLASH PREFETCH and the LATENCY Related Settings
	4. Configure the PRESCALARS HCLK, PCLK1, PCLK2
	5. Configure the MAIN PLL
	6. Enable the PLL and wait for it to become ready
	7. Select the Clock Source and wait for it to be set
	
	********************************************************/
	
	

	// 1. ENABLE HSE and wait for the HSE to become Ready
	RCC->CR |= RCC_CR_HSEON;  // RCC->CR |= 1<<16; 
	while (!(RCC->CR & RCC_CR_HSERDY));  // while (!(RCC->CR & (1<<17)));
	
	// 2. Set the POWER ENABLE CLOCK and VOLTAGE REGULATOR
	RCC->APB1ENR |= RCC_APB1ENR_PWREN;  // RCC->APB1ENR |= 1<<28;
	
	
	// 3. Configure the FLASH PREFETCH and the LATENCY Related Settings
	FLASH->ACR |= FLASH_ACR_PRFTBE;		//Prefetch buffer enable
//	FLASH->ACR &= ~FLASH_ACR_LATENCY;		Enable if SYSCLK > 24Mhz
	
	// 4. Configure the PRESCALARS HCLK, PCLK1, PCLK2
	// AHB PR
	RCC->CFGR |= RCC_CFGR_HPRE_DIV1;  // RCC->CFGR &= ~(1<<4);
	
	// APB1 PR
	RCC->CFGR |= RCC_CFGR_PPRE_DIV1;  // RCC->CFGR |= (5<<10);
	
	
	
	// 5. Configure the MAIN PLL
	RCC->CFGR |= RCC_CFGR_PLLMUL2;
	
	RCC->CFGR |= RCC_CFGR_PLLSRC_HSE_PREDIV;	//1 << 22
	
	// 6. Enable the PLL and wait for it to become ready
	RCC->CR |= RCC_CR_PLLON;  // RCC->CR |= (1<<24);
	while (!(RCC->CR & RCC_CR_PLLRDY));  // while (!(RCC->CR & (1<<25)));
	
		
	// 7. Select the Clock Source and wait for it to be set, Select PLL Clock Source
	RCC->CFGR |= RCC_CFGR_SW_PLL;  // RCC->CFGR |= (2<<0);
	while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);  // while (!(RCC->CFGR & (2<<2))); 
}
	
