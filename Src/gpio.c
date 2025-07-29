/*
 * gpio.c
 *
 *  Created on: Jul 28, 2025
 *      Author: Akshit
 */


#include "gpio.h"
#include "stm32f4xx.h"

#define GPIOA_EN (1U<<0)

void gpio_init(void){

	//PORT A CLOCK ACCES
	RCC->AHB1ENR |= GPIOA_EN;

	//PA5 IN OUTPUT MODE (LED)
	GPIOA->MODER |= (1U<<10);
	GPIOA->MODER &=~(1U<<11); //BIT 11-10 are set 01 for output mode

	//PA6 IN OUTPUT MODE (BUZZER)
	GPIOA->MODER |= (1U<<12);
	GPIOA->MODER &=~(1U<<13); //BIT 13-12 are set 01 for output mode

	//PA7 IN OUTPUT MODE (FAN)
	GPIOA->MODER |= (1U << 14);
	GPIOA->MODER &= ~(1U << 15);


}
