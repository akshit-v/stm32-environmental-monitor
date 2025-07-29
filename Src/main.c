#include "stm32f4xx.h"
#include <stdint.h>
#include <stdio.h>

#include "gpio.h"
#include "adc.h"
#include "uart.h"
#include "I2C.h"
#include "aht10.h"
#include "SysTick.h"
#include "lcd.h"

/*VARIABLES FOR ADC*/
volatile uint16_t adc_values[2] = {0};  // adc_values[0] = AQI, adc_values[1] = Temp
volatile uint8_t adc_channel_index = 0;
volatile uint8_t adc_data_ready = 0;
float r0_voltage = 0.0f;
float temperature = 0;
float aqi_ratio = 0;

/*VARIABLES FOR AHT10*/
float humidity = 0;

/*ADC FUNCTIONS*/
static void adc_callback(uint16_t value){
	adc_values[adc_channel_index++] = value;
	if(adc_channel_index >= 2){
	 adc_channel_index = 0;
	 adc_data_ready=1;
	 // Optional: Add logic to process full set of values
	 // For example: trigger an LCD update
	}

}

void ADC_IRQHandler(void){
	if (ADC1->SR & SR_EOC){
	    uint16_t val = ADC1->DR;  // Read clears EOC
	    adc_callback(val);
	}
}

float get_voltage(uint16_t raw_adc) {
    return (raw_adc * 3.3f) / 4095.0f;
}

void calibrate_mq135() {
    float sum_voltage = 0;
    int num_samples = 100;

    for (int i = 0; i < num_samples; i++) {
    	// Wait for a new ADC conversion to complete
    	while (!adc_data_ready);  // Wait for interrupt-driven conversion
    	adc_data_ready = 0;       // Reset for next round
        sum_voltage += get_voltage(adc_values[0]); // PA0 → AQI

    }

    r0_voltage = sum_voltage / num_samples;

}


int main(void){

	gpio_init();
	usart2_rxtx_init();
	pa0_pa1_adc_interrupt_init();
	start_conversion();
	calibrate_mq135(); // Calibrate MQ135 once
	SysTickDelayms(1000); //delay after calibration for 1 second
	I2C1_init();
	AHT10_init();
	lcd_init();
	lcd_set_cursor(0, 0);
	lcd_send_string("System Init...");
	SysTickDelayms(1000);
	lcd_send_cmd(0x01); // Clear display

	while(1){

		/*ADC MODULE FOR TEMP AND AQI*/
		if (adc_data_ready) {

			adc_data_ready = 0;

		    // Read values
		    uint16_t aqi_raw = adc_values[0];
		    uint16_t temp_raw = adc_values[1];

		    float temp_voltage = (temp_raw * 3.3f) / 4095.0f;
		    temperature = temp_voltage * 100.0f;

		    float aqi_voltage = (aqi_raw * 3.3f) / 4095.0f;
		    aqi_ratio = aqi_voltage / r0_voltage;

		    printf("Temp: %.2f°C | AQI V: %.2fV\n\r",temperature,aqi_ratio);

		    //ACTION ACC TO AQI LEVEL
		    if (aqi_ratio < 1.1f) {
		        printf("Air Quality: Good\r\n");
		        GPIOA->ODR &= ~(1U<<6); //TURN OFF BUZZER
		    } else if (aqi_ratio < 1.5f) {
		        printf("Air Quality: Moderate\r\n");
		        GPIOA->ODR &= ~(1U<<6); //TURN OFF BUZZER
		    } else {
		        printf("Air Quality: Poor\r\n");
		        GPIOA->ODR |= (1U<<6); //TURN ON BUZZER
		    }
		    //ACTION ACC TO TEMP LEVELS
		    if(temperature > 35.0f) {
		    	GPIOA->ODR |= (1U << 5);  // turn on LED at PA5
		    }
		    else{
		    	GPIOA->ODR &= ~(1U << 5); // turn off LED at PA5
		    }
		}

		/*AHT10 MODULE FOR HUMIDITY*/
		AHT10_trigger_measurement();
		SysTickDelayms(100); //DELAY FOR 100ms (recommended was 80ms)
		AHT10_read_data(&humidity);
		printf("Humidity: %.2f%%\n\r", humidity);

		//ACTION ACC TO HUMIDITY LEVELS
		if (humidity > 60.0f) {
		    GPIOA->ODR |= (1U << 7);  // Turn ON Fan at PA7
		} else {
		    GPIOA->ODR &= ~(1U << 7); // Turn OFF Fan at PA7
		}

		/*LCD MODULE TO DISPLAY DATA ON SCREEN*/
		char buffer[16];

		lcd_send_cmd(0x01);  // Clear screen
		SysTickDelayms(2);   // LCD clear needs delay

		// First line: Temperature and AQI
		lcd_set_cursor(0, 0);
		snprintf(buffer, sizeof(buffer), "T:%.1fC AQI:%.2f", temperature, aqi_ratio);
		lcd_send_string(buffer);

		// Second line: Humidity
		lcd_set_cursor(1, 0);
		snprintf(buffer, sizeof(buffer), "H:%.1f%%", humidity);
		lcd_send_string(buffer);

		SysTickDelayms(1000); //1 SEC DELAY

	}


}



