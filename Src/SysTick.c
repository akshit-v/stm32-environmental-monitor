/*
 * SysTick.c
 *
 *  Created on: Jul 28, 2025
 *      Author: Akshit
 */

#include "stm32f4xx.h"
#include "SysTick.h"

#define SysTick_load_val 	16000 //16000 cycles in 1ms
#define CTRL_ENABLE			(1U<<0)
#define CTRL_CLKSRC			(1U<<2)
#define CTRL_COUNTERFLAG	(1U<<16)



void SysTickDelayms(int delay){
	/*configure systick*/
	/*reload with no. of clocks per ms*/
	SysTick->LOAD =SysTick_load_val;
	/*clear systick current val register*/
	SysTick->VAL =0;
	/*enable systick and enable internal clck src*/
	SysTick->CTRL = CTRL_ENABLE | CTRL_CLKSRC;

	for(int i=0; i<delay; i++){
		/*wait until the count flag is set*/
		while((SysTick->CTRL & CTRL_COUNTERFLAG) ==0){}
	}

	SysTick->CTRL =0; //disable the counter
}

