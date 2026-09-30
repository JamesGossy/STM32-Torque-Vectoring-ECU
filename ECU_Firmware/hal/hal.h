/* Everything the ECU application needs from a board. hal/stm32g474 is the real
   board and sim/ is the host build used by the tests and the HIL simulator.
   The app/ code only ever includes this file. */
#ifndef HAL_H
#define HAL_H

#include <stddef.h>
#include <stdint.h>

/* ---- start-up and time ---- */

void hal_init(void);
void hal_start(void); // starts the watchdog
uint32_t hal_millis(void);

/* ---- analog inputs ---- */

enum { ADC_TEMP, ADC_APPS1, ADC_APPS2, ADC_BPPS, ADC_STEERING, ADC_VIN, ADC_COUNT };

float hal_adc_volts(int channel); // volts at the MCU pin, averaged

/* ---- IMU on SPI ---- */

// Chip select stays low for the whole transfer.
void hal_imu_transfer(const uint8_t *tx, uint8_t *rx, size_t len);

/* ---- CAN: standard 11-bit ids, up to 8 bytes, both return 1 on success ---- */

int hal_can_send(uint32_t id, const uint8_t *data, uint8_t len);
int hal_can_recv(uint32_t *id, uint8_t *data, uint8_t *len);
int hal_can_ok(void); // 0 when bus-off or the transceiver reports a fault

/* ---- GPS UART and USB console ---- */

size_t hal_gps_read(uint8_t *data, size_t max);
int hal_serial_write(const void *data, size_t len);
size_t hal_serial_read(uint8_t *data, size_t max);

/* ---- misc ---- */

void hal_led_set(int on);
void hal_wdg_kick(void);

#endif
