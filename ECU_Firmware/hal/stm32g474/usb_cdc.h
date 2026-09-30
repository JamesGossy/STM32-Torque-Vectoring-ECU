/*
 * usb_cdc.h - USB virtual COM port used as the serial link.
 */
#pragma once
#include <stdint.h>
#include <stddef.h>

void usb_init(void);
int usb_write(const void *data, uint32_t len);     /* returns 0 if not connected or the buffer is full */
size_t usb_read(uint8_t *data, size_t max);
