/* The small peripheral drivers under hal_stm32.c: ADC, SPI, UART and FDCAN. */
#ifndef DRIVERS_H
#define DRIVERS_H

#include <stddef.h>
#include <stdint.h>

/* ---- adc.c ---- */

void adc_init(void);
float adc_volts(int channel); // hal.h channel order

/* ---- spi.c ---- */

void spi_init(void);
void spi_transfer(const uint8_t *tx, uint8_t *rx, size_t len);

/* ---- uart.c ---- */

void uart_init(void);
size_t uart_read(uint8_t *data, size_t max);

/* ---- fdcan.c ---- */

void fdcan_init(void);
int fdcan_send(uint32_t id, const uint8_t *data, uint8_t len);
int fdcan_recv(uint32_t *id, uint8_t *data, uint8_t *len);
int fdcan_ok(void);

#endif
