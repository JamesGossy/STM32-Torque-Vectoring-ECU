/*
 * usb_desc.h - USB descriptors for the CDC ACM (virtual COM port) device.
 */
#pragma once
#include <stdint.h>

/* Look up a descriptor from a GET_DESCRIPTOR wValue. Returns its length and
   sets *desc, or returns 0 if there is no such descriptor. */
int usb_get_descriptor(uint16_t value, const uint8_t **desc);
