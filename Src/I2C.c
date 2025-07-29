/*
 * I2C.c
 *
 *  Created on: Jul 28, 2025
 *      Author: Akshit
 */


#include "stm32f4xx.h"
#include "I2C.h"

#define GPIOB_EN 	(1U<<1)
#define I2C1_EN		(1U<<21)
#define I2C_100KHZ 	80 //0b 0101 0000 = decimal 80
#define STD_MODE_MAX_RISE_TIME	17
#define CR1_PE		(1U<<0)

#define SR2_BUSY	(1U<<1)
#define CR1_START	(1U<<8)
#define SR1_SB		(1U<<0)
#define SR1_ADDR	(1U<<1)
#define SR1_TXE 	(1U<<7)
#define CR1_ACK 	(1U<<10)
#define CR1_STOP	(1U<<9)
#define SR1_RXNE	(1U<<6)
#define SR1_BTF 	(1U<<2)


/*PB9 is SDA PB8 is SCL in alternate mode AF04 */


void I2C1_init(void){
	/*enable clock access to GPIOB*/
	RCC->AHB1ENR |= GPIOB_EN;
	/*set PB8 and PB9 in alternate mode*/
	GPIOB->MODER &=~ (1U<<16);
	GPIOB->MODER |= (1U<<17); //PB8 IN ALTERNATE MODE

	GPIOB->MODER &=~ (1U<<18);
	GPIOB->MODER |=  (1U<<19);// PB9 IN ALTERNATE MODE
	/*set the alternate function mode to af04*/
	GPIOB->AFR[1] &=~ (1U<<0);
	GPIOB->AFR[1] &=~ (1U<<1);
	GPIOB->AFR[1] |= (1U<<2);
	GPIOB->AFR[1] &=~ (1U<<3);// 3-0 are for PB8 and 0100 for af04

	GPIOB->AFR[1] &=~ (1U<<4);
	GPIOB->AFR[1] &=~ (1U<<5);
	GPIOB->AFR[1] |= (1U<<6);
	GPIOB->AFR[1] &=~ (1U<<7);// 7-4 are for PB9 and 0100 for af04
	/*select PB8 and PB9 output type to open drain*/
	GPIOB->OTYPER |= (1U<<8); // PIN 8 IN open drain
	GPIOB->OTYPER |= (1U<<9); // PIN 9 in open drain
	/*enable pull-up for PB8 and PB9*/
	GPIOB->PUPDR |= (1U<<16);
	GPIOB->PUPDR &=~ (1U<<17); //01 at bit 17-16 to select pull up for PB8

	GPIOB->PUPDR |= (1U<<18);
	GPIOB->PUPDR &=~ (1U<<19);//01 at bit 19-18 to select pull up for PB9

	/*Enable clock access to I2C1*/
	RCC->APB1ENR |= I2C1_EN;
	/*enter reset mode*/
	I2C1->CR1 |= (1U<<15); // SWRST pin is bit 15
	/*come out of reset mode*/
	I2C1->CR1 &=~ (1U<<15);
	/*set peripheral clock frequency*/
	I2C1->CR2 |= (1U<<4); //16MHz first 6 bits are for frequency 010000
	/*set 12C to standard mode, 100khz clock*/
	I2C1->CCR = I2C_100KHZ;
	/*set the rise time*/
	I2C1->TRISE = STD_MODE_MAX_RISE_TIME;
	/*enable I2C*/
	I2C1->CR1 |= CR1_PE;

}

void I2C1_CommandWrite(char sl_addr,uint8_t* data,int n){
	volatile int temp;

	/*make sure I2C is not busy*/
	while(I2C1->SR2 & (SR2_BUSY)) {}
	/*generate the start condition*/
	I2C1->CR1 |= CR1_START;
	/*wait until start flag is set*/
	while(!(I2C1->SR1 & (SR1_SB))) {}
	/*transmit slave address + write*/
	I2C1->DR = sl_addr << 1;
	/*wait until address flag is set*/
	while(!(I2C1->SR1 & (SR1_ADDR))) {}
	/*clear address flag by reading contents of SR2*/
	temp = I2C1->SR2;

	for(int i=0; i<n ; i++){
		/*wait until DR is empty*/
		while(!(I2C1->SR1 & (SR1_TXE))) {}
		/*transmit memory address*/
		I2C1->DR = data[i];
	}

	/*wait until transfer is finished*/
	while(!(I2C1->SR1 & (SR1_BTF))){}
	/*generate stop*/
	I2C1->CR1 |= CR1_STOP;
}

void I2C1_CommandRead(uint8_t sl_addr, uint8_t* data, int n) {
    volatile int temp;

    // Wait until I2C is not busy
    while (I2C1->SR2 & SR2_BUSY) {}

    // Generate START condition
    I2C1->CR1 |= CR1_START;

    // Wait until START flag is set
    while (!(I2C1->SR1 & SR1_SB)) {}

    // Send slave address + READ (LSB = 1)
    I2C1->DR = (sl_addr << 1) | 1;

    // Wait until ADDR is set
    while (!(I2C1->SR1 & SR1_ADDR)) {}

    /*clear address flag by reading contents of SR2*/
    temp = I2C1->SR2;
    /*enable the ACK*/
    I2C1->CR1 |= CR1_ACK;

    while(n> 0U){
    	/*if one byte is left*/
    	if(n==1U){
    		//disable the ACK
    		I2C1->CR1 &=~ CR1_ACK;
    		/*generate stop*/
    		I2C1->CR1 |= CR1_STOP;
    		/*wait until RXNE flag is set*/
    		while(!(I2C1->SR1 & (SR1_RXNE))){}
    		/*read data from DR*/
    		*data++ = I2C1->DR;
    		break;
    	}
    	//if more than one byte is there to receive
    	else{
    		/*wait until RXNE flag is set*/
    		while(!(I2C1->SR1 & (SR1_RXNE))){}
    		/*read data from DR*/
    		*data++ = I2C1->DR;
   			n--; //1 byte is read from n bytes
    	}
    }
}

