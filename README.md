# STM32 Environmental Monitoring System

This project implements a real-time environmental monitoring system using the STM32F411RE microcontroller. It collects data from multiple sensors, processes it, displays results on a 16x2 LCD, and provides alerts through GPIO-driven outputs such as LED, buzzer, and fan.

---

## Features

- **Temperature Monitoring**
  - Measured using analog temperature sensor.
  - ADC configured with interrupt-based multi-channel sampling.
  - LED warning when temperature exceeds 35°C.

- **Air Quality Index (AQI) Detection**
  - Utilizes MQ135 gas sensor.
  - Calibrated using Ro reference voltage at startup.
  - Buzzer alerts for poor air quality.

- **Humidity Sensing**
  - Integrated via AHT10 sensor over I2C protocol.
  - Fan activates automatically if humidity exceeds 60%.

- **Real-time LCD Display**
  - Displays live temperature, humidity, and AQI ratio.
  - Custom LCD driver written in bare-metal C using SysTick-based delays.

- **Serial Debugging**
  - UART support for live sensor data visualization on serial monitor using `printf()`.

---

## System Architecture

- **Microcontroller**: STM32F411RE (bare-metal programming)
- **Programming Language**: Embedded C
- **Development Tools**: STM32CubeIDE, OpenOCD, STM32 Programmer
- **Communication Protocols**: I2C (AHT10), UART (debugging), ADC (temperature, AQI)
- **Peripherals Used**: GPIO, ADC1, USART2, I2C1, SysTick Timer

---

## Pin Configuration

| Function        | GPIO Pin |
|----------------|----------|
| Temperature (ADC) | PA1      |
| AQI Sensor (ADC)  | PA0      |
| Buzzer            | PA6      |
| Fan Control       | PA7      |
| Status LED        | PA5      |
| I2C (AHT10)       | PB8 (SCL), PB9 (SDA) |
| LCD Control/Data  | PB0-PB5 |

---

## Functional Flow

1. **Initialization**
   - Configure GPIO, ADC, I2C, UART, SysTick.
   - Calibrate MQ135 AQI sensor using Ro voltage average.

2. **Sensor Sampling**
   - ADC samples temperature and AQI alternately using interrupt-based switching.
   - AHT10 triggered over I2C and returns humidity data.

3. **Data Processing**
   - Temperature is scaled from voltage.
   - AQI is calculated as a ratio of live voltage to Ro voltage.
   - Humidity is parsed from AHT10 data frame.

4. **Outputs**
   - Status messages sent to serial monitor.
   - Warnings triggered via buzzer, LED, and fan.
   - 16x2 LCD updated with fresh sensor readings every second.

---

## Calibration

- **MQ135 Ro Calibration**:
  - Runs once at startup.
  - Takes 100 ADC samples from AQI channel and computes average reference voltage `Ro`.

---



