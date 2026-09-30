/* SPI1 for the IMU: mode 3, 8-bit frames, 170 MHz / 32 = 5.3 MHz (the sensor
   allows 10 MHz). Transfers are short and blocking. */
#include "board.h"
#include "drivers.h"

void spi_init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
    SPI1->CR2 = (7u << SPI_CR2_DS_Pos) | SPI_CR2_FRXTH; // 8-bit, RXNE after one byte
    SPI1->CR1 = SPI_CR1_MSTR | SPI_CR1_SSM | SPI_CR1_SSI | (4u << SPI_CR1_BR_Pos) | SPI_CR1_CPOL
        | SPI_CR1_CPHA | SPI_CR1_SPE;
}

void spi_transfer(const uint8_t *tx, uint8_t *rx, size_t len)
{
    GPIOA->BRR = 1u << IMU_CS_PIN;
    for (size_t i = 0; i < len; i++) {
        while (!(SPI1->SR & SPI_SR_TXE)) { }
        *(volatile uint8_t *)&SPI1->DR = tx[i]; // byte access, or the SPI sends 16 bits
        while (!(SPI1->SR & SPI_SR_RXNE)) { }
        rx[i] = *(volatile uint8_t *)&SPI1->DR;
    }
    while (SPI1->SR & SPI_SR_BSY) { }
    GPIOA->BSRR = 1u << IMU_CS_PIN;
}
