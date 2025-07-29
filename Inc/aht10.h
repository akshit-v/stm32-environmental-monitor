/*
 * aht10.h
 *
 *  Created on: Jul 28, 2025
 *      Author: Akshit
 */

#ifndef AHT10_H_
#define AHT10_H_

void AHT10_init(void);
void AHT10_trigger_measurement(void);
void AHT10_read_data(/*float* temperature,*/ float* humidity);

#endif /* AHT10_H_ */
