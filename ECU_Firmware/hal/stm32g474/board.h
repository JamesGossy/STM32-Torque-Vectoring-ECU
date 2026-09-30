/* Pin map of the ECU board (U11, STM32G474CEU6), taken from ECU_Hardware/schematics.
   PC4 (IMU_INT) and PA8 (GPS_PPS) are wired but not used yet. */
#ifndef BOARD_H
#define BOARD_H

#include "stm32g4xx.h"

/* ---- pin map ----

   PA0 TEMP_SENSE ADC1_IN1    PA4 IMU_CS_N GPIO       PA9  GPS_TX    USART1 AF7
   PA1 APPS_1     ADC1_IN2    PA5 IMU_SCK  SPI1 AF5   PA10 GPS_RX    USART1 AF7
   PA2 APPS_2     ADC1_IN3    PA6 IMU_MISO SPI1 AF5   PB4  CAN_FAULT input
   PA3 BPPS       ADC1_IN4    PA7 IMU_MOSI SPI1 AF5   PB5  CAN_RX    FDCAN2 AF9
   PB0 STEERING   ADC1_IN15   PB9 HEARTBEAT LED       PB6  CAN_TX    FDCAN2 AF9
   PB1 VIN_SENSE  ADC1_IN12   PA11/PA12 USB */

#define LED_PIN       9 // PB9
#define IMU_CS_PIN    4 // PA4
#define CAN_FAULT_PIN 4 // PB4, TCAN337G FAULT goes high on a transceiver fault

enum { PIN_IN, PIN_OUT, PIN_AF, PIN_ANALOG };

static inline void pin_mode(GPIO_TypeDef *port, uint32_t pin, uint32_t mode, uint32_t af)
{
    port->MODER = (port->MODER & ~(3u << (pin * 2))) | (mode << (pin * 2));
    if (mode == PIN_AF) {
        port->OSPEEDR |= 3u << (pin * 2);
        uint32_t shift      = (pin & 7) * 4;
        port->AFR[pin >> 3] = (port->AFR[pin >> 3] & ~(0xFu << shift)) | (af << shift);
    }
}

void board_init(void);

#endif
