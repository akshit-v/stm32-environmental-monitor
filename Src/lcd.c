/*
 * lcd.c
 *
 *  Created on: Jul 29, 2025
 *      Author: Akshit
 */

#include "lcd.h"
#include "stm32f4xx.h"
#include "SysTick.h"

// LCD control pins
#define RS_PIN (1U << 0)   // PB0
#define EN_PIN (1U << 1)   // PB1

// LCD data pins (D4–D7) as PB2–PB5
#define D4_PIN (1U << 2)
#define D5_PIN (1U << 3)
#define D6_PIN (1U << 4)
#define D7_PIN (1U << 5)

void lcd_gpio_init(void) {
    // Enable GPIOB clock
    RCC->AHB1ENR |= (1U << 1);

    // Set PB0–PB5 as general purpose outputs
    for (int pin = 0; pin <= 5; ++pin) {
        GPIOB->MODER &= ~(3U << (pin * 2));
        GPIOB->MODER |=  (1U << (pin * 2));
    }
}

void lcd_pulse_enable(void) {
    GPIOB->ODR |= EN_PIN;
    SysTickDelayms(1);
    GPIOB->ODR &= ~EN_PIN;
    SysTickDelayms(1);
}

void lcd_send_nibble(uint8_t nibble) {
    // Clear data bits
    GPIOB->ODR &= ~(D4_PIN | D5_PIN | D6_PIN | D7_PIN);

    if (nibble & 0x01) GPIOB->ODR |= D4_PIN;
    if (nibble & 0x02) GPIOB->ODR |= D5_PIN;
    if (nibble & 0x04) GPIOB->ODR |= D6_PIN;
    if (nibble & 0x08) GPIOB->ODR |= D7_PIN;

    lcd_pulse_enable();
}

void lcd_send_byte(uint8_t data, uint8_t is_data) {
    if(is_data){
        GPIOB->ODR |= RS_PIN;
    }
    else{
    	GPIOB->ODR &= ~RS_PIN;
    }

    lcd_send_nibble(data >> 4);   // Send high nibble
    lcd_send_nibble(data & 0x0F); // Send low nibble

    SysTickDelayms(2);
}

void lcd_init(void) {
    lcd_gpio_init();
    SysTickDelayms(40);  // Wait for LCD to power up

    // Initial 8-bit mode commands to switch to 4-bit
    lcd_send_nibble(0x03);
    SysTickDelayms(5);
    lcd_send_nibble(0x03);
    SysTickDelayms(150);
    lcd_send_nibble(0x03);
    lcd_send_nibble(0x02);  // Set to 4-bit mode

    lcd_send_cmd(0x28);  // 4-bit, 2 lines, 5x8 dots
    lcd_send_cmd(0x0C);  // Display on, cursor off
    lcd_send_cmd(0x01);  // Clear display
    SysTickDelayms(2);
    lcd_send_cmd(0x06);  // Entry mode set
}

void lcd_send_cmd(uint8_t cmd) {
    lcd_send_byte(cmd, 0);
}

void lcd_send_data(uint8_t data) {
    lcd_send_byte(data, 1);
}

void lcd_send_string(char *str) {
    while (*str) {
        lcd_send_data(*str++);
    }
}

void lcd_set_cursor(uint8_t row, uint8_t col) {
    uint8_t pos = (row == 0) ? (0x80 + col) : (0xC0 + col);
    lcd_send_cmd(pos);
}



