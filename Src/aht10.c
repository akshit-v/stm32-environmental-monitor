/*
 * aht10.c
 *
 *  Created on: Jul 28, 2025
 *      Author: Akshit
 */

#include "aht10.h"
#include "I2C.h"
#include "stm32f4xx.h"

#define AHT10_ADDR 0x38

void AHT10_init(void) {
    uint8_t cmd = 0xE1; // Optional calibration enable
    I2C1_CommandWrite(AHT10_ADDR, &cmd, 1);
}

void AHT10_trigger_measurement(void) {
    uint8_t trigger[3] = {0xAC, 0x33, 0x00};
    I2C1_CommandWrite(AHT10_ADDR, trigger, 3);
}

void AHT10_read_data(/*float* temperature,*/ float* humidity) {
    uint8_t data[6];
    I2C1_CommandRead(AHT10_ADDR, data, 6);

    // Parse 20-bit values from 6 bytes
    uint32_t raw_humidity = ((data[1] << 12) | (data[2] << 4) | (data[3] >> 4));
    //uint32_t raw_temp     = ((data[3] & 0x0F) << 16) | (data[4] << 8) | data[5];

    *humidity = (raw_humidity / 1048576.0f) * 100.0f;
    //*temperature = (raw_temp / 1048576.0f) * 200.0f - 50.0f;
}

