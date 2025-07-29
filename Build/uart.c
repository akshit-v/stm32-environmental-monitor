/*
 * uart.c
 *
 *  Created on: Jul 28, 2025
 *      Author: Akshit
 */

#include "uart.h"


#define GPIOAEN (1U<<0)

#define USART2EN (1U<<17)

#define SYS_FREQ 16000000
#define APB1_CLK SYS_FREQ
#define USART_BAUDRATE 115200

#define CR1_RE (1U<<2)
#define CR1_TE (1U<<3)
#define CR1_UE (1U<<13)
#define SR_TXE (1U<<7)
#define SR_RXNE (1U<<5)

//for interrupt
#define CR1_RXNEIE	(1U<<5) //if its set it will enable usart receive interrupt

static void usart_set_baudrate(USART_TypeDef *USARTx,uint32_t PeriphClk,uint32_t BaudRate);
static uint16_t compute_usart_bd(uint32_t PeriphClk,uint32_t BaudRate);

void usart2_write(int ch);

int __io_putchar(int ch){
	usart2_write(ch);
	return ch;
}


void usart2_rx_interrupt_init(void){
	/*************configure usart gpio pin****************/
	/*ENABLE CLOCK ACCESS TO GPIOA*/
		RCC->AHB1ENR |=GPIOAEN;
	/*SET PA2 MODE TO ALTERNATE FUNC MODE*/
		GPIOA->MODER &=~(1U<<4); //4TH BIT 0
		GPIOA->MODER |=(1U<<5); //5TH BIT 1

	/*SET PA2 AF TYPE UART _TX (AF07)*/
		GPIOA->AFR[0] |=(1U<<8);
		GPIOA->AFR[0] |=(1U<<9);
		GPIOA->AFR[0] |=(1U<<10);
		GPIOA->AFR[0] &=~(1U<<11);
	/*SET PA3 MODE TO ALTERNATE FUNC MODE*/
		GPIOA->MODER &=~(1U<<6); //6TH BIT 0
		GPIOA->MODER |=(1U<<7); //7TH BIT 1

	/*SET PA3 AF TYPE UART _TX (AF07)*/
		GPIOA->AFR[0] |=(1U<<12);
		GPIOA->AFR[0] |=(1U<<13);
		GPIOA->AFR[0] |=(1U<<14);
		GPIOA->AFR[0] &=~(1U<<15);

	/****************configure usart module**************/
	/*ENABLE CLOCK ACCES TO UART2*/
		RCC->APB1ENR |=USART2EN;
	/*CONFIGURE UART BAUD RATE*/
		usart_set_baudrate(USART2,APB1_CLK,USART_BAUDRATE);
	/*CONFIGURE TRANSFER DIRECTION*/
		USART2->CR1 = (CR1_TE|CR1_RE); //PURPOSELY "=" FOR MAKING TX & RX ENABLE AND OTHERS TO DEFAULT
		USART2->CR1 |=CR1_RXNEIE; //ENABLED RCV INTERRUPT IN REGISTER
		/*ENABLE UART2 INTERRUPT IN NVIC*/
		NVIC_EnableIRQ(USART2_IRQn);
	/*ENABLE UART MODULE*/
		USART2->CR1 |=CR1_UE;
}

void usart2_rxtx_init(void){
	/*************configure usart gpio pin****************/
	/*ENABLE CLOCK ACCESS TO GPIOA*/
		RCC->AHB1ENR |=GPIOAEN;
	/*SET PA2 MODE TO ALTERNATE FUNC MODE*/
		GPIOA->MODER &=~(1U<<4); //4TH BIT 0
		GPIOA->MODER |=(1U<<5); //5TH BIT 1

	/*SET PA2 AF TYPE UART _TX (AF07)*/
		GPIOA->AFR[0] |=(1U<<8);
		GPIOA->AFR[0] |=(1U<<9);
		GPIOA->AFR[0] |=(1U<<10);
		GPIOA->AFR[0] &=~(1U<<11);
	/*SET PA3 MODE TO ALTERNATE FUNC MODE*/
		GPIOA->MODER &=~(1U<<6); //6TH BIT 0
		GPIOA->MODER |=(1U<<7); //7TH BIT 1

	/*SET PA3 AF TYPE UART _TX (AF07)*/
		GPIOA->AFR[0] |=(1U<<12);
		GPIOA->AFR[0] |=(1U<<13);
		GPIOA->AFR[0] |=(1U<<14);
		GPIOA->AFR[0] &=~(1U<<15);

	/****************configure usart module**************/
	/*ENABLE CLOCK ACCES TO UART2*/
		RCC->APB1ENR |=USART2EN;
	/*CONFIGURE UART BAUD RATE*/
		usart_set_baudrate(USART2,APB1_CLK,USART_BAUDRATE);
	/*CONFIGURE TRANSFER DIRECTION*/
		USART2->CR1 = (CR1_TE|CR1_RE); //PURPOSELY "=" FOR MAKING TX & RX ENABLE AND OTHERS TO DEFAULT
	/*ENABLE UART MODULE*/
		USART2->CR1 |=CR1_UE;
}

void usart2_tx_init(void){
	/*************configure usart gpio pin****************/
	/*ENABLE CLOCK ACCESS TO GPIOA*/
		RCC->AHB1ENR |=GPIOAEN;
	/*SET PA2 MODE TO ALTERNATE FUNC MODE*/
		GPIOA->MODER &=~(1U<<4); //4TH BIT 0
		GPIOA->MODER |=(1U<<5); //5TH BIT 1

	/*SET PA2 AF TYPE UART _TX (AF07)*/
		GPIOA->AFR[0] |=(1U<<8);
		GPIOA->AFR[0] |=(1U<<9);
		GPIOA->AFR[0] |=(1U<<10);
		GPIOA->AFR[0] &=~(1U<<11);

	/****************configure usart module**************/
	/*ENABLE CLOCK ACCES TO UART2*/
		RCC->APB1ENR |=USART2EN;
	/*CONFIGURE UART BAUD RATE*/
		usart_set_baudrate(USART2,APB1_CLK,USART_BAUDRATE);
	/*CONFIGURE TRANSFER DIRECTION*/
		USART2->CR1 = CR1_TE; //PURPOSELY "=" FOR MAKING TX ENABLE AND OTHERS TO DEFAULT
	/*ENABLE UART MODULE*/
		USART2->CR1 |=CR1_UE;
}

char usart2_read(void){
	/*make sure receive data register is not empty*/
	while(!(USART2->SR & SR_RXNE)){} //we will be stuck in this line until TXE bit in SR becomes 1
		/*read data register*/
		return USART2->DR;
}

void usart2_write(int ch){
	/*make sure transmit data register is empty*/
	while(!(USART2->SR & SR_TXE)){} //we will be stuck in this line until TXE bit in SR becomes 1
	/*write to transmit data register*/
	USART2->DR= (ch & 0xFF);
}
static void usart_set_baudrate(USART_TypeDef *USARTx,uint32_t PeriphClk,uint32_t BaudRate){

	USARTx->BRR= compute_usart_bd(PeriphClk,BaudRate);
}

static uint16_t compute_usart_bd(uint32_t PeriphClk,uint32_t BaudRate){

	return ((PeriphClk+(BaudRate/2U))/BaudRate);
}
