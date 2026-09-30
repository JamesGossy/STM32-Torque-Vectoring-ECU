/* GPIO setup. Analog pins are left in their reset state, which is analog. */
#include "board.h"

void board_init(void)
{
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOBEN | RCC_AHB2ENR_GPIOCEN;

    GPIOA->BSRR = 1u << IMU_CS_PIN; // deselect the IMU before the pin becomes an output
    pin_mode(GPIOA, IMU_CS_PIN, PIN_OUT, 0);
    pin_mode(GPIOB, LED_PIN, PIN_OUT, 0);

    GPIOB->PUPDR &= ~(3u << (CAN_FAULT_PIN * 2)); // PB4 has a JTAG pull-up from reset
    pin_mode(GPIOB, CAN_FAULT_PIN, PIN_IN, 0);

    pin_mode(GPIOA, 5, PIN_AF, 5);  // SPI1 SCK
    pin_mode(GPIOA, 6, PIN_AF, 5);  // SPI1 MISO
    pin_mode(GPIOA, 7, PIN_AF, 5);  // SPI1 MOSI
    pin_mode(GPIOA, 9, PIN_AF, 7);  // USART1 TX
    pin_mode(GPIOA, 10, PIN_AF, 7); // USART1 RX
    pin_mode(GPIOB, 5, PIN_AF, 9);  // FDCAN2 RX
    pin_mode(GPIOB, 6, PIN_AF, 9);  // FDCAN2 TX
}
