/* USART1 receives NMEA from the GPS at its default 9600 baud. The interrupt
   fills a ring buffer that the main loop empties. */
#include "board.h"
#include "drivers.h"
#include "system.h"

#define RX_SIZE 512u

static uint8_t rx_buf[RX_SIZE];
static volatile uint32_t rx_head, rx_tail;

void uart_init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
    USART1->BRR = F_SYS / 9600u;
    USART1->CR1 = USART_CR1_UE | USART_CR1_RE | USART_CR1_TE | USART_CR1_RXNEIE_RXFNEIE;
    NVIC_SetPriority(USART1_IRQn, 2);
    NVIC_EnableIRQ(USART1_IRQn);
}

void USART1_IRQHandler(void)
{
    if (USART1->ISR & USART_ISR_ORE) USART1->ICR = USART_ICR_ORECF; // lost byte: bad checksum
    while (USART1->ISR & USART_ISR_RXNE_RXFNE) {
        uint8_t byte  = (uint8_t)USART1->RDR;
        uint32_t next = (rx_head + 1) % RX_SIZE;
        if (next != rx_tail) {
            rx_buf[rx_head] = byte;
            rx_head         = next;
        }
    }
}

size_t uart_read(uint8_t *data, size_t max)
{
    size_t count = 0;
    while (count < max && rx_tail != rx_head) {
        data[count++] = rx_buf[rx_tail];
        rx_tail       = (rx_tail + 1) % RX_SIZE;
    }
    return count;
}
