/*
 * uart.h
 *
 *  Created on: Jul 28, 2025
 *      Author: Akshit
 */

#ifndef UART_H_
#define UART_H_

#include "stm32f4xx.h"
#include <stdint.h>

void usart2_tx_init(void);
char usart2_read(void);
void usart2_rxtx_init(void);
void usart2_rx_interrupt_init(void);

#define SR_RXNE	(1U<<5)

#endif /* UART_H_ */
