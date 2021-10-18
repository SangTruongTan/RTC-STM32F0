#include "stm32f030x6.h"
#include "SystemClock.h"
#include "I2C1_F0.h"
#include "Max7219.h"
#include "stdlib.h"
#include "stdbool.h"
#include "stdio.h"
#include "string.h"
/* User define */
#define DS_add (uint8_t) 0x68
/* Typedef */
typedef struct {
	uint8_t seconds;
	uint8_t minutes;
	uint8_t hours;
	uint8_t tempH;
	uint8_t tempL;
}time_t;

typedef enum {
	Display = 0,
	Config_h,
	Config_m,
	
}State_t;

typedef struct {
	time_t* time;
	State_t state;
}control_t;

/* Variable */
time_t Time_Global;
State_t State;
time_t Time_Buff;
uint16_t time_count;
control_t control;
uint8_t* tptr;
extern uint8_t Led_code[10];
extern uint8_t Led_codeI[10];
int count;
uint8_t dataW[3] = {0x00,0x15, 0x20};
uint8_t dataR[3];



/* Function prototype */
uint8_t dec2bcd (uint8_t data);
uint8_t bcd2dec (uint8_t data);
void GPIOConfig (void);
void Timer3Config (void);
void TIM3_IRQHandler (void); 
void Timer14Config (void);
void TIM14_IRQHandler (void);
void ExtiConfig (void);
void EXTI4_15_IRQHandler (void);
void Limit_time (control_t* control);
void delay (uint32_t time);
void getTime (time_t* time, uint8_t reg, uint8_t size);
void setTime (uint8_t* buff, uint8_t reg, uint8_t size);
void display (time_t Time, bool point_en);
void Calib_brightness (time_t time);
void Display_temp (time_t Time_Temp);
void Set_SLEEPONEXIT (bool sc);

void Set_SLEEPONEXIT (bool sc) {
	uint32_t SCR = 0xE000ED10;			//SCR adress
	uint32_t buff;
	buff = *(__IO uint32_t*)SCR;
	if (sc == true) {
		buff |=  1 << SCB_SCR_SLEEPONEXIT_Pos;
	}
	else {
		buff &= ~ (uint32_t) (1 << SCB_SCR_SLEEPONEXIT_Pos);
	}
	*(__IO uint32_t*)SCR  = buff;
}

void Display_temp (time_t Time_Temp) {
	uint8_t buf[3];
	buf[0] = (Time_Temp.tempH >> 4)&0x0F;
	buf[1] = (Time_Temp.tempH&0x0F);
	buf[2] = (Time_Temp.tempL >> 4)&0x0F;
	write_data(1, Led_code[buf[0]]);
//	write_data(2, Led_code[buf[1]] | 0x80);	
//	write_data(3, Led_codeI[buf[2]]);
//	write_data(4, 0x4E);	//C character without point
	write_data(2, Led_code[buf[1]] );	
 	write_data(3, 0x78 | 0x80 ); 	//add point and Charracter C inverter
	write_data(4, 0x00 ); // 
}




int main () {
	
	SysClockConfig();
	Timer3Config();
	Timer14Config();
	GPIOConfig();
	I2C1_Config();
	max_init(0x04);
	ExtiConfig();
	/* Init time */
	getTime(&Time_Global, 0x00, 3);
	display(Time_Global, true);
	Calib_brightness(Time_Global);
	Set_SLEEPONEXIT(true);
	__WFI();
	
	while(1) {
		if(State != Display) {
			getTime(&Time_Buff, 0x00, 3);
			while(1) {
				if(!(GPIOA->IDR & GPIO_IDR_3)) {
					*tptr = bcd2dec(*tptr);
					*tptr += 1;
					*tptr = dec2bcd(*tptr);
					Limit_time(&control);
					display(Time_Buff, false);
					time_count = 0;
					delay(800000);
				}
				if(State == Display) {
					break;
					
				}
			}
		}
	}
}


void TIM3_IRQHandler (void) {
	time_t time;
	uint8_t buff[5];
	uint8_t buf[2];
	uint8_t tempBuf[2];
	static bool conv = true;
	static bool blink = true;
	static uint16_t Count_state = 0;
	if (TIM3->SR & TIM_SR_UIF) {
		TIM3->SR &= ~TIM_SR_UIF;
		if (State == Display) {
			Count_state += 1;
			if(Count_state <= 60) {
				getTime(&Time_Global, 0x00, 3);
				display(Time_Global, true);
				Calib_brightness(Time_Global);
				if((Count_state >= 20) && (conv == true)) {
						I2C1_Mem_Read(DS_add, 0x0E, buff, 2);
						if(!(buff[1] & (1 << 2))) {
							conv = false;
							buff[0] |= 1 << 5;
							I2C1_Mem_Write(DS_add, 0x0E, buff, 1);
						}
				}
			}
			else if (Count_state > 60 && Count_state <= 63) {
				if(conv == false) {
					I2C1_Mem_Read(DS_add, 0x0E, buff, 5);
					if(!(buff[0] & (1 << 5))) {
//						I2C1_Mem_Read(DS_add, 0x11, buff, 3);
						tempBuf[0] = dec2bcd(buff[3]);
						tempBuf[1] = dec2bcd((buff[4] >> 6)*25);
						Time_Global.tempH = tempBuf[0];
						Time_Global.tempL = tempBuf[1];
						conv = true;
					}
				}
				Display_temp(Time_Global);
			}
			else if (Count_state > 67) {
				Count_state = 0;
			}
		}
		else {
			time_count += 1;
			if(time_count > 2) {
				memcpy((void*)&time, &Time_Buff, sizeof(time_t)); 
				if (blink == true) {
					if(control.state == Config_h)
						time.hours = 0xAA; 	//no led on
					if(control.state == Config_m)
						time.minutes = 0xAA;
				}
				blink = !blink;
				display(time, false);
			}
		}
	}
}

void TIM14_IRQHandler (void) {
	if(TIM14->SR & TIM_SR_UIF) {
		TIM14->SR &= ~TIM_SR_UIF;
		TIM14->CR1 &= ~TIM_CR1_CEN;
		TIM14->DIER &= ~TIM_DIER_UIE;
		
		EXTI->IMR |= EXTI_IMR_IM4;
	}
}
void EXTI4_15_IRQHandler (void)  {
	/*************>>>>>>> STEPS FOLLOWED <<<<<<<<************
	
	1. Check the Pin, which trgerred the Interrupt
	2. Clear the Interrupt Pending Bit
	
	********************************************************/
	uint8_t buf[3];
	//1. Check the Pin, which trgerred the Interrupt
	if (EXTI->PR & EXTI_PR_PR4) {
			EXTI->PR = EXTI_PR_PR4;		//Clear Pending bit
		EXTI->IMR &= ~EXTI_IMR_IM4;
		
		/* Change State machine */
		count ++;
		switch (State) {
			case Display:
				State = Config_h;
				control.state = Config_h;
				control.time = &Time_Buff;
				tptr = &Time_Buff.hours;
				time_count = 5;
				Set_SLEEPONEXIT(false);		//Disable sleep on exit
				break;
			case Config_h:
				State = Config_m;
				control.state = Config_m;
				control.time = &Time_Buff;
				tptr = &Time_Buff.minutes;
				time_count = 5;
				break;
			case Config_m:
				buf[0] = control.time->seconds;
				buf[1] = control.time->minutes;
				buf[2] = control.time->hours;
				setTime(buf, 0x00, 3);
				State = Display;
				Set_SLEEPONEXIT(true);		//Enable Sleep on Exit
				break;
		}
		
		/*Config timer to debounce */
		TIM14->CNT = 0;
		TIM14->CR1 |= TIM_CR1_CEN;
		while(!(TIM14->SR & TIM_SR_UIF));
		TIM14 -> SR &= ~TIM_SR_UIF;
		TIM14->DIER |= TIM_DIER_UIE;
	}
}

void getTime (time_t* time, uint8_t reg, uint8_t size) {
	uint8_t buff[3];
	I2C1_Mem_Read(DS_add, 0x00, buff, 3);
	time->seconds = buff[0];
	time->minutes = buff[1];
	time->hours = buff[2];
}


void setTime (uint8_t* buff, uint8_t reg, uint8_t size) {
	I2C1_Mem_Write(DS_add, reg, buff, size);
}

void ExtiConfig (void) {
	/*************>>>>>>> STEPS FOLLOWED <<<<<<<<************
	
	1. Enable the SYSCFG/AFIO bit in RCC register 
	2. Configure the EXTI configuration Register in the SYSCFG/AFIO
	3. Disable the EXTI Mask using Interrupt Mask Register (IMR)
	4. Configure the Rising Edge / Falling Edge Trigger
	5. Set the Interrupt Priority
	6. Enable the interrupt
	
	********************************************************/
	
	//1. Enable the SYSCFG/AFIO bit in RCC register 
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN; 		//1 << 0
	
	//2. Configure the EXTI configuration Register in the SYSCFG/AFIO
	SYSCFG->EXTICR[1] |= SYSCFG_EXTICR2_EXTI4_PA; 
	
	//3. Disable the EXTI Mask using Interrupt Mask Register (IMR)
	EXTI->IMR |= EXTI_IMR_MR4; 
	
	//4. Configure the Rising Edge / Falling Edge Trigger
	EXTI->RTSR &= ~EXTI_RTSR_TR4; 
	EXTI->FTSR |= EXTI_FTSR_TR4; 
	
	//5. Set the Interrupt Priority
	NVIC_SetPriority (EXTI4_15_IRQn, 1);	//Set priority to  1
	NVIC_EnableIRQ(EXTI4_15_IRQn);		//Enable IRQ
	
	
}

void Calib_brightness (time_t time) {
	if(time.hours >= 0x06 && time.hours <= 0x22)
		max_set_brightness(0x05);
	else
		max_set_brightness(0x00);
}

void Limit_time (control_t* control) {
	switch (control->state) {
		case Config_h:
			if(control->time->hours == 0x24)
				control->time->hours = 0x00;
			if(control->time->hours == 0x95)
				control->time->hours = 0x23;
			break;
		case Config_m:
			if(control->time->minutes == 0x60)
				control->time->minutes = 0;
			if(control->time->minutes== 0x95)
				control->time->minutes = 0x59;
			break;
	}
}

void display (time_t Time, bool point_en) {
	uint8_t buf[3];
	static uint8_t point = 0x00;
	buf[0] = (Time.hours >> 4)&0x0F;
	buf[1] = (Time.hours&0x0F);
	write_data(1, Led_code[buf[0]]);
	write_data(2, Led_code[buf[1]] | point);
	
	buf[0] = ((Time.minutes >> 4)&0x0F);
	buf[1] = Time.minutes&0x0F;
	write_data(3, Led_codeI[buf[0]] | point);
	write_data(4, Led_code[buf[1]]);
	if (point_en == true) {
		if(point == 0x80)
			point = 0x00;
		else
			point = 0x80;
	}
}



void Timer14Config (void) {
	/************** STEPS TO FOLLOW *****************
	1. Enable Timer clock
	2. Set the prescalar and the ARR
	3. Enable the Timer, and wait for the update Flag to set
	************************************************/
	
	//Reset and Enable Timer clock
	RCC->APB1RSTR |= RCC_APB1RSTR_TIM14RST;		//Reset Timer 3
	RCC->APB1RSTR &= ~ RCC_APB1RSTR_TIM14RST;
	RCC->APB1ENR |= RCC_APB1ENR_TIM14EN;
	
	//Set the prescalar and the ARR
	TIM14->PSC = 300 - 1;
	TIM14->ARR = 40000-1;
	
	//Enable the Timer, and wait for the update Flag to set
//	TIM14->CR1 |= TIM_CR1_CEN;
//	while (!(TIM14->SR & (1<<0)));  // UIF: Update interrupt flag..  This bit is set by hardware when the registers are updated
//	TIM14->SR &=  ~TIM_SR_UIF;	// Write 1 to bit 0
	
	//Enable Interrupt
	TIM14->CR1 |= TIM_CR1_URS;
//	TIM14->DIER |= TIM_DIER_UIE;	  //1 << 0
	NVIC_SetPriority(TIM14_IRQn,1);
	NVIC_EnableIRQ(TIM14_IRQn);
	
}



void Timer3Config (void) {
	/************** STEPS TO FOLLOW *****************
	1. Enable Timer clock
	2. Set the prescalar and the ARR
	3. Enable the Timer, and wait for the update Flag to set
	************************************************/
	
	//Reset and Enable Timer clock
	RCC->APB1RSTR |= RCC_APB1RSTR_TIM3RST;		//Reset Timer 3
	RCC->APB1RSTR &= ~ RCC_APB1RSTR_TIM3RST;
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
	
	//Set the prescalar and the ARR
	TIM3->PSC = 200 - 1;
	TIM3->ARR = 40000-1;
	
	//Enable the Timer, and wait for the update Flag to set
	TIM3->CR1 |= TIM_CR1_CEN;
	while (!(TIM3->SR & (1<<0)));  // UIF: Update interrupt flag..  This bit is set by hardware when the registers are updated
	TIM3->SR &=  ~TIM_SR_UIF;	// Write 1 to bit 0
	
	//Enable Interrupt
	TIM3->CR1 |= TIM_CR1_URS;
	TIM3->DIER |= TIM_DIER_UIE;	  //1 << 0
	NVIC_SetPriority(TIM3_IRQn,1);
	NVIC_EnableIRQ(TIM3_IRQn);
	
}


void GPIOConfig (void) {
	/************** STEPS TO FOLLOW *****************
	1. Enable GPIOA clock
	2. Set the PIN  as output
	3. Configure the output mode i.e state, speed, and pull
	************************************************/
	/* PA9 as Output */
	//1. Enable GPIOA clock
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;			//1 << 17
	
	//2. Set the PIN PA5 as output
	
//	GPIOA->MODER |= GPIO_MODER_MODER9_0;	//1 << 18
	
	//3. Configure the output mode i.e state, speed, and pull
//	GPIOA->OSPEEDR |= GPIO_OSPEEDR_OSPEEDR9;	//	3 << 18
	
	/* PA3, PA4 as Input */
	GPIOA->MODER &= ~ (uint32_t) (3 << 6);
	GPIOA->MODER &= ~ (uint32_t) (3 << 8);
	
	GPIOA->PUPDR |= GPIO_PUPDR_PUPDR3_0;	//1 << 6
	GPIOA->PUPDR |= GPIO_PUPDR_PUPDR4_0;  //1 << 8
}

void delay (uint32_t time) {
	uint32_t i;
	for (i = 0; i < time; i++);
}

uint8_t dec2bcd (uint8_t data) {
	uint8_t buf = ((data/10) << 4) | (data%10);
	return buf;
}

uint8_t bcd2dec (uint8_t data) {
	uint8_t buf = ((data >> 4)& 0x0F) *10 + (data&0x0F);
	return buf;
}
