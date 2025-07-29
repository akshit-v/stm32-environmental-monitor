/*
 * adc.h
 *
 *  Created on: Jul 28, 2025
 *      Author: Akshit
 */

#ifndef ADC_H_
#define ADC_H_

#include <stdint.h>

void pa1_adc_init();
void start_conversion(void);
uint32_t adc_read(void);
void pa1_adc_interrupt_init(void);
#define SR_EOC	(1U<<1)


void pa0_pa1_adc_interrupt_init(void);

#endif /* ADC_H_ */





