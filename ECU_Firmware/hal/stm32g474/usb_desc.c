/*
 * usb_desc.c - device, configuration and string descriptors.
 *
 * A standard CDC ACM device (Windows 10+ loads its built-in driver):
 *   interface 0: communications, notification endpoint 0x82
 *   interface 1: data, bulk OUT 0x01 and bulk IN 0x81 (64 bytes)
 * Uses ST's virtual COM port VID/PID.
 */
#include "usb_desc.h"
#include "stm32g4xx.h"

static const uint8_t device_desc[18] = {
    18, 1,              /* length, DEVICE */
    0x00, 0x02,         /* USB 2.0 */
    0x02, 0x00, 0x00,   /* class CDC */
    64,                 /* EP0 packet size */
    0x83, 0x04,         /* VID 0x0483 */
    0x40, 0x57,         /* PID 0x5740 */
    0x00, 0x02,         /* device release */
    1, 2, 3,            /* manufacturer, product, serial strings */
    1,                  /* one configuration */
};

static const uint8_t config_desc[67] = {
    9, 2, 67, 0, 2, 1, 0, 0x80, 50,         /* configuration: 2 interfaces, bus powered, 100 mA */
    9, 4, 0, 0, 1, 0x02, 0x02, 0x00, 0,     /* interface 0: CDC ACM, 1 endpoint */
    5, 0x24, 0x00, 0x10, 0x01,              /* CDC header */
    5, 0x24, 0x01, 0x00, 0x01,              /* call management */
    4, 0x24, 0x02, 0x02,                    /* ACM: line coding supported */
    5, 0x24, 0x06, 0x00, 0x01,              /* union: interface 0 controls 1 */
    7, 5, 0x82, 0x03, 8, 0, 16,             /* EP 0x82 interrupt IN */
    9, 4, 1, 0, 2, 0x0A, 0x00, 0x00, 0,     /* interface 1: CDC data, 2 endpoints */
    7, 5, 0x01, 0x02, 64, 0, 0,             /* EP 0x01 bulk OUT */
    7, 5, 0x81, 0x02, 64, 0, 0,             /* EP 0x81 bulk IN */
};

static const uint8_t language_desc[4] = {4, 3, 0x09, 0x04};    /* English (US) */

static uint8_t string_buf[64];

/* Build a UTF-16 string descriptor from ASCII. */
static int make_string(const char *s)
{
    uint8_t n = 0;
    while (s[n] && n < 30) {
        string_buf[2 + 2 * n] = (uint8_t)s[n];
        string_buf[3 + 2 * n] = 0;
        n++;
    }
    string_buf[0] = 2 + 2 * n;
    string_buf[1] = 3;
    return string_buf[0];
}

/* Serial number: the chip's unique id folded into 8 hex digits. */
static int make_serial(void)
{
    const uint32_t *uid = (const uint32_t *)UID_BASE;
    uint32_t u = uid[0] ^ uid[1] ^ uid[2];
    char hex[9];
    for (int i = 0; i < 8; i++) hex[i] = "0123456789ABCDEF"[(u >> (28 - 4 * i)) & 0xF];
    hex[8] = 0;
    return make_string(hex);
}

int usb_get_descriptor(uint16_t value, const uint8_t **desc)
{
    uint8_t type = value >> 8, index = value & 0xFF;

    if (type == 1) { *desc = device_desc; return sizeof device_desc; }
    if (type == 2) { *desc = config_desc; return sizeof config_desc; }
    if (type != 3) return 0;

    *desc = string_buf;
    switch (index) {
    case 0: *desc = language_desc; return sizeof language_desc;
    case 1: return make_string("STM32G474 ECU");
    case 2: return make_string("Torque Vectoring ECU");
    case 3: return make_serial();
    default: return 0;
    }
}
