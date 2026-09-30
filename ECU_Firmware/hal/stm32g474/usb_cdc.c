/*
 * usb_cdc.c - minimal USB CDC ACM device.
 *
 * Handles enumeration on endpoint 0 and streams data on endpoint 1.
 * Transmit data goes through a ring buffer: the main loop fills it and the
 * USB interrupt empties it 64 bytes at a time. Received bytes go into a
 * second ring buffer that the command parser reads.
 */
#include "usb_cdc.h"
#include "usb_ll.h"
#include "usb_desc.h"
#include "system.h"

#define TX_SIZE 2048u
#define RX_SIZE 256u

static uint8_t tx_buf[TX_SIZE];
static volatile uint32_t tx_head, tx_tail;
static volatile uint8_t tx_busy;

static uint8_t rx_buf[RX_SIZE];
static volatile uint32_t rx_head, rx_tail;

static uint8_t configured, pending_address, expect_line_coding;
static uint8_t line_coding[7] = {0x00, 0x10, 0x0E, 0x00, 0, 0, 8};   /* 921600 8N1 (ignored) */

/* endpoint 0 transfer in progress */
static const uint8_t *ep0_data;
static uint16_t ep0_left;
static uint8_t ep0_needs_zlp;

/* Send the next (up to 64 byte) packet of the endpoint 0 transfer. */
static void ep0_send_next(void)
{
    uint16_t n = ep0_left > 64 ? 64 : ep0_left;
    pma_write(EP0_TX, ep0_data, n);
    BT_TX_COUNT(0) = n;
    ep0_data += n;
    ep0_left -= n;
    ep_set_tx(0, USB_EP_TX_VALID);
}

/* Start sending a reply on endpoint 0, truncated to what the host asked for.
   A reply that is shorter than requested and a multiple of 64 bytes must be
   terminated with a zero length packet. */
static void ep0_send(const uint8_t *data, uint16_t len, uint16_t requested)
{
    if (len > requested) len = requested;
    ep0_data = data;
    ep0_left = len;
    ep0_needs_zlp = len && len < requested && (len % 64) == 0;
    ep0_send_next();
}

/* Zero length status packet. */
static void ep0_ack(void) { ep0_send(0, 0, 0); }

/* Standard requests (enumeration). Returns 0 if unsupported. */
static int standard_request(uint8_t request, uint16_t value, uint16_t length)
{
    static const uint8_t zero[2];
    const uint8_t *desc;
    int n;

    switch (request) {
    case 0x06:  /* GET_DESCRIPTOR */
        n = usb_get_descriptor(value, &desc);
        if (!n) return 0;
        ep0_send(desc, (uint16_t)n, length);
        return 1;
    case 0x05:  /* SET_ADDRESS: applied after the status stage completes */
        pending_address = value & 0x7F;
        ep0_ack();
        return 1;
    case 0x09:  /* SET_CONFIGURATION */
        configured = (uint8_t)value;
        if (configured) {
            ep_init(1, USB_EP_BULK, USB_EP_RX_VALID, USB_EP_TX_NAK);
            ep_init(2, USB_EP_INTERRUPT, 0, USB_EP_TX_NAK);
            tx_busy = 0;
        }
        ep0_ack();
        return 1;
    case 0x08:  /* GET_CONFIGURATION */
        ep0_send(&configured, 1, length);
        return 1;
    case 0x0A:  /* GET_INTERFACE */
        ep0_send(zero, 1, length);
        return 1;
    case 0x00:  /* GET_STATUS */
        ep0_send(zero, 2, length);
        return 1;
    case 0x01: case 0x03: case 0x0B:    /* CLEAR/SET_FEATURE, SET_INTERFACE */
        ep0_ack();
        return 1;
    }
    return 0;
}

/* CDC class requests. Returns 0 if unsupported. */
static int class_request(uint8_t request, uint16_t length)
{
    switch (request) {
    case 0x20:  /* SET_LINE_CODING: 7 data bytes follow */
        expect_line_coding = 1;
        return 1;
    case 0x21:  /* GET_LINE_CODING */
        ep0_send(line_coding, 7, length);
        return 1;
    case 0x22: case 0x23:   /* SET_CONTROL_LINE_STATE, SEND_BREAK */
        ep0_ack();
        return 1;
    }
    return 0;
}

/* A SETUP packet arrived on endpoint 0. */
static void handle_setup(void)
{
    uint8_t s[8];
    pma_read(EP0_RX, s, 8);
    uint8_t type = s[0] & 0x60, request = s[1];
    uint16_t value = s[2] | (s[3] << 8);
    uint16_t length = s[6] | (s[7] << 8);
    expect_line_coding = 0;

    int ok = 0;
    if (type == 0x00) ok = standard_request(request, value, length);
    else if (type == 0x20) ok = class_request(request, length);

    if (!ok) ep_set_tx(0, USB_EP_TX_STALL);
}

/* Endpoint 0 events. */
static void handle_ep0(uint16_t epr)
{
    if (epr & USB_EP_CTR_TX) {
        ep_clear_tx(0);
        if (pending_address) {
            USB->DADDR = USB_DADDR_EF | pending_address;
            pending_address = 0;
        }
        if (ep0_left) {
            ep0_send_next();
        } else if (ep0_needs_zlp) {
            ep0_needs_zlp = 0;
            ep0_send_next();
        }
    }

    if (epr & USB_EP_CTR_RX) {
        if (epr & USB_EP_SETUP) {
            handle_setup();
        } else if (expect_line_coding) {
            uint16_t n = BT_RX_COUNT(0) & 0x3FF;
            pma_read(EP0_RX, line_coding, n < 7 ? n : 7);
            expect_line_coding = 0;
            ep0_ack();
        }
        ep_clear_rx(0);
        ep_set_rx(0, USB_EP_RX_VALID);
    }
}

/* Endpoint 1 events: IN packet sent, or OUT data received. */
static void handle_ep1(uint16_t epr)
{
    if (epr & USB_EP_CTR_TX) {
        ep_clear_tx(1);
        tx_busy = 0;
    }

    if (epr & USB_EP_CTR_RX) {
        uint8_t packet[64];
        uint16_t n = BT_RX_COUNT(1) & 0x3FF;
        if (n > 64) n = 64;
        pma_read(EP1_RX, packet, n);
        for (uint16_t i = 0; i < n; i++) {
            uint32_t next = (rx_head + 1) % RX_SIZE;
            if (next == rx_tail) break;     /* full: drop the rest */
            rx_buf[rx_head] = packet[i];
            rx_head = next;
        }
        ep_clear_rx(1);
        ep_set_rx(1, USB_EP_RX_VALID);
    }
}

/* If endpoint 1 is free, send the next chunk of the TX ring buffer. */
static void tx_kick(void)
{
    if (!configured || tx_busy || tx_head == tx_tail) return;

    uint8_t packet[64];
    uint16_t n = 0;
    uint32_t tail = tx_tail;
    while (tail != tx_head && n < 64) {
        packet[n++] = tx_buf[tail];
        tail = (tail + 1) % TX_SIZE;
    }

    pma_write(EP1_TX, packet, n);
    BT_TX_COUNT(1) = n;
    tx_tail = tail;
    tx_busy = 1;
    ep_set_tx(1, USB_EP_TX_VALID);
}

/* Bus reset: set up the buffer table and endpoint 0, address 0. */
static void bus_reset(void)
{
    USB->BTABLE = 0;
    PMA(0) = EP0_TX;  PMA(2) = 0;  PMA(4) = EP0_RX;  PMA(6) = RX64;
    PMA(8) = EP1_TX;  PMA(10) = 0; PMA(12) = EP1_RX; PMA(14) = RX64;
    PMA(16) = EP2_TX; PMA(18) = 0; PMA(20) = 0;      PMA(22) = 0;

    ep_init(0, USB_EP_CONTROL, USB_EP_RX_VALID, USB_EP_TX_NAK);
    USB->DADDR = USB_DADDR_EF;

    configured = pending_address = expect_line_coding = 0;
    tx_busy = 0;
    tx_tail = tx_head;
}

/* USB interrupt. Also triggered by usb_write() to start transmission. */
void USB_LP_IRQHandler(void)
{
    if (USB->ISTR & USB_ISTR_RESET) {
        USB->ISTR = (uint16_t)~USB_ISTR_RESET;
        bus_reset();
        return;
    }

    uint16_t istr;
    while ((istr = USB->ISTR) & USB_ISTR_CTR) {
        int ep = istr & USB_ISTR_EP_ID;
        uint16_t epr = EPR(ep);
        if (ep == 0) {
            handle_ep0(epr);
        } else if (ep == 1) {
            handle_ep1(epr);
        } else {
            if (epr & USB_EP_CTR_TX) ep_clear_tx(ep);
            if (epr & USB_EP_CTR_RX) ep_clear_rx(ep);
        }
    }

    tx_kick();
}

/* Start the USB peripheral and connect to the host. */
void usb_init(void)
{
    RCC->APB1ENR1 |= RCC_APB1ENR1_USBEN | RCC_APB1ENR1_CRSEN;
    (void)RCC->APB1ENR1;
    CRS->CR |= CRS_CR_AUTOTRIMEN | CRS_CR_CEN;     /* trim HSI48 to the host's SOF packets */

    USB->CNTR = USB_CNTR_FRES;      /* power up, still in reset */
    delay_us(10);
    USB->CNTR = 0;
    USB->ISTR = 0;
    USB->CNTR = USB_CNTR_CTRM | USB_CNTR_RESETM;
    USB->BCDR |= USB_BCDR_DPPU;     /* D+ pull-up: tell the host we're here */

    NVIC_SetPriority(USB_LP_IRQn, 3);
    NVIC_EnableIRQ(USB_LP_IRQn);
}

/* Queue data for the host. All-or-nothing, so frames are never split by a full buffer. */
int usb_write(const void *data, uint32_t len)
{
    if (!configured) return 0;

    uint32_t head = tx_head;
    uint32_t space = (tx_tail + TX_SIZE - head - 1) % TX_SIZE;
    if (len > space) return 0;

    const uint8_t *d = data;
    for (uint32_t i = 0; i < len; i++) {
        tx_buf[head] = d[i];
        head = (head + 1) % TX_SIZE;
    }
    __DMB();
    tx_head = head;
    NVIC_SetPendingIRQ(USB_LP_IRQn);    /* let the interrupt start sending */
    return 1;
}

/* Take received bytes. */
size_t usb_read(uint8_t *data, size_t max)
{
    size_t n = 0;
    while (n < max && rx_tail != rx_head) {
        data[n++] = rx_buf[rx_tail];
        rx_tail = (rx_tail + 1) % RX_SIZE;
    }
    return n;
}
