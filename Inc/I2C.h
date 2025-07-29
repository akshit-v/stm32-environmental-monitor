/*
 * I2C.h
 *
 *  Created on: Jul 28, 2025
 *      Author: Akshit
 */

#ifndef I2C_H_
#define I2C_H_
#include <stdint.h>

void I2C1_init(void);
void I2C1_CommandWrite(char sl_addr,uint8_t* data,int n);
void I2C1_CommandRead(uint8_t sl_addr, uint8_t* data, int n);

#endif /* I2C_H_ */
