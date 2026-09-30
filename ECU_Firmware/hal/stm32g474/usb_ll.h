/*
 * usb_ll.h - low level helpers for the STM32 USB FS device peripheral.
 *
 * Endpoint registers have "toggle" and "clear on write 0" bits, so they
 * can't be set with plain read-modify-write. These helpers do it properly.
 * The packet memory (PMA) is 1 KB, accessed as 16-bit words.
 */
#pragma once
#include <stdint.h>
#include "stm32g4xx.h"

#define EPR(n)  (*(volatile uint16_t *)(USB_BASE + 4u * (n)))
#define PMA(o)  (*(volatile uint16_t *)(USB_PMAADDR + (o)))

/* packet memory layout (buffer descriptor table at 0) */
#define EP0_TX  0x40
#define EP0_RX  0x80
#define EP1_TX  0xC0
#define EP1_RX  0x100
#define EP2_TX  0x140
#define RX64    0x8400      /* receive count field meaning "64 byte buffer" */

/* buffer descriptor table entries for endpoint n */
#define BT_TX_COUNT(n)  PMA((n) * 8 + 2)
#define BT_RX_COUNT(n)  PMA((n) * 8 + 6)

/* Copy bytes into packet memory. */
static inline void pma_write(uint16_t offset, const uint8_t *src, uint16_t len)
{
    for (uint16_t i = 0; i < len; i += 2) {
        uint16_t w = src[i];
        if (i + 1 < len) w |= (uint16_t)src[i + 1] << 8;
        PMA(offset + i) = w;
    }
}

/* Copy bytes out of packet memory. */
static inline void pma_read(uint16_t offset, uint8_t *dst, uint16_t len)
{
    for (uint16_t i = 0; i < len; i += 2) {
        uint16_t w = PMA(offset + i);
        dst[i] = (uint8_t)w;
        if (i + 1 < len) dst[i + 1] = (uint8_t)(w >> 8);
    }
}

/* Set the TX status (VALID, NAK, STALL). Status bits toggle when written as 1. */
static inline void ep_set_tx(int ep, uint16_t status)
{
    uint16_t v = EPR(ep) & (USB_EPREG_MASK | USB_EPTX_STAT);
    EPR(ep) = (v ^ status) | USB_EP_CTR_RX | USB_EP_CTR_TX;
}

/* Set the RX status. */
static inline void ep_set_rx(int ep, uint16_t status)
{
    uint16_t v = EPR(ep) & (USB_EPREG_MASK | USB_EPRX_STAT);
    EPR(ep) = (v ^ status) | USB_EP_CTR_RX | USB_EP_CTR_TX;
}

/* Acknowledge a completed transfer (CTR bits clear when written as 0). */
static inline void ep_clear_rx(int ep) { EPR(ep) = (EPR(ep) & USB_EPREG_MASK & ~USB_EP_CTR_RX) | USB_EP_CTR_TX; }
static inline void ep_clear_tx(int ep) { EPR(ep) = (EPR(ep) & USB_EPREG_MASK & ~USB_EP_CTR_TX) | USB_EP_CTR_RX; }

/* Configure an endpoint's type and address, reset its data toggles to 0 and
   set its RX/TX status. */
static inline void ep_init(int ep, uint16_t type, uint16_t rx_status, uint16_t tx_status)
{
    uint16_t v = EPR(ep);
    uint16_t w = type | (uint16_t)ep | USB_EP_CTR_RX | USB_EP_CTR_TX;
    w |= v & (USB_EP_DTOG_RX | USB_EP_DTOG_TX);
    w |= (v & (USB_EPRX_STAT | USB_EPTX_STAT)) ^ (rx_status | tx_status);
    EPR(ep) = w;
}
