/* FDCAN2 at 1 Mbit/s classic CAN, matching the motor controllers. The hardware
   only queues 3 frames each way, so received frames are moved to a ring buffer
   by interrupt and transmitted frames wait in a software queue. */
#include "board.h"
#include "drivers.h"
#include <string.h>

// FDCAN2 is the second instance in message RAM. Each instance has a fixed layout.
#define MSG_RAM  (SRAMCAN_BASE + 0x350u)
#define RX_FIFO0 (MSG_RAM + 0x0B0u)
#define TX_FIFO  (MSG_RAM + 0x278u)
#define ELEMENT  72u
#define RING     64u

typedef struct {
    uint32_t id;
    uint8_t len;
    uint32_t words[2];
} Frame;

static Frame rx_ring[RING], tx_ring[RING];
static volatile uint32_t rx_head, rx_tail;
static uint32_t tx_head, tx_tail;

void fdcan_init(void)
{
    RCC->APB1ENR1 |= RCC_APB1ENR1_FDCANEN;
    (void)RCC->APB1ENR1;

    FDCAN2->CCCR |= FDCAN_CCCR_INIT;
    while (!(FDCAN2->CCCR & FDCAN_CCCR_INIT)) { }
    FDCAN2->CCCR |= FDCAN_CCCR_CCE;
    FDCAN2->CCCR &= ~(FDCAN_CCCR_FDOE | FDCAN_CCCR_BRSE | FDCAN_CCCR_DAR); // classic, auto retry

    for (uint32_t *p = (uint32_t *)MSG_RAM; p < (uint32_t *)(MSG_RAM + 0x350u); p++)
        *p = 0;

    // 16 MHz crystal, 16 time quanta per bit: sync 1 + seg1 12 + seg2 3 (81 % sample point)
    FDCAN2->NBTP = (2u << FDCAN_NBTP_NSJW_Pos) | (0u << FDCAN_NBTP_NBRP_Pos)
        | (11u << FDCAN_NBTP_NTSEG1_Pos) | (2u << FDCAN_NBTP_NTSEG2_Pos);
    FDCAN2->RXGFC = FDCAN_RXGFC_RRFS | FDCAN_RXGFC_RRFE; // reject remote frames, keep the rest
    FDCAN2->TXBC  = 0;                                   // TX FIFO mode

    FDCAN2->IE  = FDCAN_IE_RF0NE;
    FDCAN2->ILE = FDCAN_ILE_EINT0;
    NVIC_SetPriority(FDCAN2_IT0_IRQn, 2);
    NVIC_EnableIRQ(FDCAN2_IT0_IRQn);

    FDCAN2->CCCR &= ~FDCAN_CCCR_INIT;
    while (FDCAN2->CCCR & FDCAN_CCCR_INIT) { }
}

void FDCAN2_IT0_IRQHandler(void)
{
    FDCAN2->IR = FDCAN_IR_RF0N;
    while (FDCAN2->RXF0S & FDCAN_RXF0S_F0FL_Msk) {
        uint32_t index       = (FDCAN2->RXF0S >> FDCAN_RXF0S_F0GI_Pos) & 3u;
        volatile uint32_t *e = (volatile uint32_t *)(RX_FIFO0 + index * ELEMENT);
        uint32_t header      = e[0];
        Frame f       = { (header >> 18) & 0x7FFu, (uint8_t)((e[1] >> 16) & 0xFu), { e[2], e[3] } };
        FDCAN2->RXF0A = index;

        uint32_t next = (rx_head + 1) % RING;
        if (header & (1u << 30) || next == rx_tail) continue; // extended id, or no room
        if (f.len > 8) f.len = 8;
        rx_ring[rx_head] = f;
        rx_head          = next;
    }
}

// Moves queued frames into the hardware FIFO while it has room.
static void kick(void)
{
    // after bus-off the controller sets INIT, and clearing it starts the recovery.
    // Queued frames are old by then, so drop them rather than send stale setpoints late.
    if (FDCAN2->CCCR & FDCAN_CCCR_INIT) {
        tx_tail = tx_head;
        FDCAN2->CCCR &= ~FDCAN_CCCR_INIT;
    }

    while (tx_tail != tx_head && !(FDCAN2->TXFQS & FDCAN_TXFQS_TFQF)) {
        const Frame *f       = &tx_ring[tx_tail];
        uint32_t index       = (FDCAN2->TXFQS >> FDCAN_TXFQS_TFQPI_Pos) & 3u;
        volatile uint32_t *e = (volatile uint32_t *)(TX_FIFO + index * ELEMENT);
        e[0]                 = (f->id & 0x7FFu) << 18;
        e[1]                 = (uint32_t)f->len << 16;
        e[2]                 = f->words[0];
        e[3]                 = f->words[1];
        FDCAN2->TXBAR        = 1u << index;
        tx_tail              = (tx_tail + 1) % RING;
    }
}

int fdcan_send(uint32_t id, const uint8_t *data, uint8_t len)
{
    uint32_t next = (tx_head + 1) % RING;
    if (next == tx_tail) return 0;
    Frame *f    = &tx_ring[tx_head];
    f->id       = id;
    f->len      = len > 8 ? 8 : len;
    f->words[0] = f->words[1] = 0;
    if (data && f->len) memcpy(f->words, data, f->len);
    tx_head = next;
    kick();
    return 1;
}

int fdcan_recv(uint32_t *id, uint8_t *data, uint8_t *len)
{
    kick();
    if (rx_tail == rx_head) return 0;
    const Frame *f = &rx_ring[rx_tail];
    *id            = f->id;
    *len           = f->len;
    memcpy(data, f->words, 8);
    rx_tail = (rx_tail + 1) % RING;
    return 1;
}

int fdcan_ok(void)
{
    int bus_off           = (FDCAN2->PSR & FDCAN_PSR_BO) != 0;
    int transceiver_fault = (GPIOB->IDR & (1u << CAN_FAULT_PIN)) != 0;
    return !bus_off && !transceiver_fault;
}
