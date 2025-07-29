/*
 * adc.c
 *
 *  Created on: Jul 28, 2025
 *      Author: Akshit
 */


#include "stm32f4xx.h"
#include "adc.h"

#define ADC1_CLK_EN 	(1U<<8)
#define ADC_CH1_AT_SEQ1 (1U<<0)
#define ADC1_SEQ_LEN_2  (1U<<20) //23-20 bit is set 0001 for 2 conversion
#define ADC1_SEQ_LEN_1  (0x00)   //23-20 bit si set 0000 for 1 conversion
#define ADC1_CR2_ADON 	(1U<<0)
#define ADC_CR2_SWSTART	(1U<<30)
#define SR_EOC			(1U<<1)
#define CR2_CONT		(1U<<1)
#define CR1_EOCIE		(1U<<5)


/*adc configured with 2 channel, ch0 ch1
 * first ch0
 * second ch1
 * therefore SQ1 is ch0 SQ2 is ch1
 */

void pa0_pa1_adc_interrupt_init(void){
	/*enable clock access to adc pin port*/ /*here it PA0 and PA1*/
		RCC->AHB1ENR |=(1U<<0);
	/*set the mode of PA0 to analog*/
		GPIOA->MODER |= (1U<<0);
		GPIOA->MODER |= (1U<<1);
	/*set the mode of PA1 to analog*/
		GPIOA->MODER |= (1U<<2);
		GPIOA->MODER |= (1U<<3);
	/*********configure adc module*********/
	/*enable clock access to adc1*/
		RCC->APB2ENR |= ADC1_CLK_EN;
	/****configure adc parameters****/
	/*enable adc end of conversion interrupt*/
		ADC1->CR1 |=CR1_EOCIE;
		/*enable adc interrupt in NVIC*/
		NVIC_EnableIRQ(ADC_IRQn);
		/* Set conversion sequence: SQ1 -> CH0 (PA0), SQ2 -> CH1 (PA1) */
		ADC1->SQR3 = (0U<<0) | (1U<<5); //BIT 4-0 PRIORITY1, BIT 9-5 PRIORITY2
		/*conversion sequence length*/
		ADC1->SQR1 = ADC1_SEQ_LEN_2;
		/*enable adc module*/
		ADC1->CR2 |= ADC1_CR2_ADON;
}

void pa1_adc_interrupt_init(void){
	/*enable clock access to adc pin port*/ /*here it PA1*/
	RCC->AHB1ENR |=(1U<<0);
	/*set the mode of PA1 to analog*/
	GPIOA->MODER |= (1U<<2);
	GPIOA->MODER |= (1U<<3);

	/*********configure adc module*********/
	/*enable clock access to adc1*/
	RCC->APB2ENR |= ADC1_CLK_EN;
	/****configure adc parameters****/
	/*enable adc end of conversion interrupt*/
	ADC1->CR1 |=CR1_EOCIE;
	/*enable adc interrupt in NVIC*/
	NVIC_EnableIRQ(ADC_IRQn);
	/*conversion sequence start*/
	ADC1->SQR3 = ADC_CH1_AT_SEQ1; //we want only 1 ch and single conversion
	/*conversion sequence length*/
	ADC1->SQR1 = ADC1_SEQ_LEN_1;
	/*enable adc module*/
	ADC1->CR2 |= ADC1_CR2_ADON;

}


void pa1_adc_init(void){
	/*enable clock access to adc pin port*/ /*here it PA1*/
	RCC->AHB1ENR |=(1U<<0);
	/*set the mode of PA1 to analog*/
	GPIOA->MODER |= (1U<<2);
	GPIOA->MODER |= (1U<<3);

	/*********configure adc module*********/
	/*enable clock access to adc1*/
	RCC->APB2ENR |= ADC1_CLK_EN;
	/****configure adc parameters****/
	/*conversion sequence start*/
	ADC1->SQR3 = ADC_CH1_AT_SEQ1; //we want only 1 ch and single conversion
	/*conversion sequence length*/
	ADC1->SQR1 = ADC1_SEQ_LEN_1;
	/*enable adc module*/
	ADC1->CR2 |= ADC1_CR2_ADON;

}

void start_conversion(void){
	/*enable continuous conversion*/
	ADC1->CR2 |= CR2_CONT;
	/*start the conversion*/
	ADC1->CR2 |= ADC_CR2_SWSTART;

}

uint32_t adc_read(void){
	/*wait till conversion is complete*/
	while(!(ADC1->SR & SR_EOC)){}
	/* reads the value*/
	return (ADC1->DR);
}





