/* hal.h for the ECU board: thin wrappers around the drivers in this folder. */
#include "hal.h"
#include "board.h"
#include "drivers.h"
#include "system.h"
#include "usb_cdc.h"

void hal_init(void)
{
    system_init();
    board_init();
    adc_init();
    spi_init();
    uart_init();
    usb_init();
    fdcan_init();
}

void hal_start(void)
{
    wdg_init();
}

uint32_t hal_millis(void)
{
    return ms_ticks;
}
float hal_adc_volts(int channel)
{
    return adc_volts(channel);
}
void hal_imu_transfer(const uint8_t *tx, uint8_t *rx, size_t len)
{
    spi_transfer(tx, rx, len);
}

int hal_can_send(uint32_t id, const uint8_t *data, uint8_t len)
{
    return fdcan_send(id, data, len);
}
int hal_can_recv(uint32_t *id, uint8_t *data, uint8_t *len)
{
    return fdcan_recv(id, data, len);
}
int hal_can_ok(void)
{
    return fdcan_ok();
}

size_t hal_gps_read(uint8_t *data, size_t max)
{
    return uart_read(data, max);
}
int hal_serial_write(const void *data, size_t len)
{
    return usb_write(data, (uint32_t)len);
}
size_t hal_serial_read(uint8_t *data, size_t max)
{
    return usb_read(data, max);
}

void hal_led_set(int on)
{
    if (on) {
        GPIOB->BSRR = 1u << LED_PIN;
    } else {
        GPIOB->BRR = 1u << LED_PIN;
    }
}

void hal_wdg_kick(void)
{
    wdg_kick();
}
