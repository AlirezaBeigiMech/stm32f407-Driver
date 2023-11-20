#include "stm32f407xx.h"

int main() {
	
	/*IO port D & A clock enable*/
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;	/*Bit3*/
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;	/*Bit0*/
	
	/*Clear LED fields*/
	GPIOD->MODER &= ~(GPIO_MODER_MODER12);
	GPIOD->MODER &= ~(GPIO_MODER_MODER13);
	GPIOD->MODER &= ~(GPIO_MODER_MODER14);
	GPIOD->MODER &= ~(GPIO_MODER_MODER15);
	
	/*Set LEDs to general purpose output mode*/
	GPIOD->MODER |= GPIO_MODER_MODE12_0;	/*Bit24*/
	GPIOD->MODER |= GPIO_MODER_MODE13_0;	/*Bit26*/
	GPIOD->MODER |= GPIO_MODER_MODE14_0;	/*Bit28*/
	GPIOD->MODER |= GPIO_MODER_MODE15_0;	/*Bit30*/
	
	/*Clear Button fields & set A0 to pull down*/
	GPIOA->MODER &= ~(GPIO_MODER_MODER0);
	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR0);
	GPIOA->PUPDR |= GPIO_PUPDR_PUPDR0_1;	/*Bit1*/
	
	while(1){
		if(GPIOA->IDR & GPIO_IDR_ID0){
			/*if Button is active, turn on LEDs*/
			GPIOD->BSRR |= GPIO_BSRR_BS12;	/*Bit12*/
			GPIOD->BSRR |= GPIO_BSRR_BS13;	/*Bit13*/
			GPIOD->BSRR |= GPIO_BSRR_BS14;	/*Bit14*/
			GPIOD->BSRR |= GPIO_BSRR_BS15;	/*Bit15*/
		}
		else{
			/*turn off LEDs*/
			GPIOD->BSRR |= GPIO_BSRR_BR12;	/*Bit28*/
			GPIOD->BSRR |= GPIO_BSRR_BR13;	/*Bit29*/
			GPIOD->BSRR |= GPIO_BSRR_BR14;	/*Bit30*/
			GPIOD->BSRR |= GPIO_BSRR_BR15;	/*Bit31*/
		}
	}
}
