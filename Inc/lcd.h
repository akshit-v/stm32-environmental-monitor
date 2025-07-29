/*
 * lcd.h
 *
 *  Created on: Jul 29, 2025
 *      Author: Akshit
 */

#ifndef LCD_H_
#define LCD_H_
#include <stdint.h>

void lcd_gpio_init(void);
void lcd_pulse_enable(void);
void lcd_send_nibble(uint8_t nibble);
void lcd_send_byte(uint8_t data, uint8_t is_data);
void lcd_init(void);
void lcd_send_cmd(uint8_t cmd);
void lcd_send_data(uint8_t data);
void lcd_send_string(char *str);
void lcd_set_cursor(uint8_t row, uint8_t col);

#endif /* LCD_H_ */
