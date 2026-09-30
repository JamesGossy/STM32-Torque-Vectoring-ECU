/* Clock tree: 16 MHz crystal -> PLL -> 170 MHz for the core and both APB buses.
   FDCAN runs straight from the crystal, USB from HSI48 trimmed by CRS. A crash
   just waits for the watchdog, because the motor controllers stop by
   themselves 250 ms after their setpoints stop. */
#include "system.h"

volatile uint32_t ms_ticks;
uint32_t SystemCoreClock = F_SYS;

// Called by the startup code before main().
void SystemInit(void)
{
    SCB->CPACR |= (0xFu << 20); // FPU on
    SCB->VTOR = FLASH_BASE;
}

void SysTick_Handler(void)
{
    ms_ticks++;
}

void NMI_Handler(void)
{
    for (;;) { }
}
void HardFault_Handler(void)
{
    for (;;) { }
}
void MemManage_Handler(void)
{
    for (;;) { }
}
void BusFault_Handler(void)
{
    for (;;) { }
}
void UsageFault_Handler(void)
{
    for (;;) { }
}

void system_init(void)
{
    RCC->APB1ENR1 |= RCC_APB1ENR1_PWREN;
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
    PWR->CR3 |= PWR_CR3_UCPD_DBDIS; // USB-C dead battery pull-downs sit on PB4 and PB6 (CAN)
    PWR->CR5 &= ~PWR_CR5_R1MODE;    // range 1 boost, needed above 150 MHz
    FLASH->ACR = FLASH_ACR_DBG_SWEN | FLASH_ACR_LATENCY_4WS | FLASH_ACR_PRFTEN | FLASH_ACR_ICEN
        | FLASH_ACR_DCEN; // DBG_SWEN keeps SWD working after boot

    RCC->CR |= RCC_CR_HSEON;
    while (!(RCC->CR & RCC_CR_HSERDY)) { }

    // 16 MHz / M4 * N85 / R2 = 170 MHz
    RCC->PLLCFGR = RCC_PLLCFGR_PLLSRC_HSE | (3u << RCC_PLLCFGR_PLLM_Pos)
        | (85u << RCC_PLLCFGR_PLLN_Pos) | RCC_PLLCFGR_PLLREN;
    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY)) { }

    // the reference manual asks for AHB/2 for 1 us when jumping above 80 MHz
    RCC->CFGR = RCC_CFGR_HPRE_DIV2 | RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL) { }
    for (volatile int i = 0; i < 100; i++) { }
    RCC->CFGR = RCC_CFGR_SW_PLL;

    RCC->CRRCR |= RCC_CRRCR_HSI48ON;
    while (!(RCC->CRRCR & RCC_CRRCR_HSI48RDY)) { }

    // kernel clocks: FDCAN = HSE, USB = HSI48, ADC = SYSCLK, USART1 = PCLK2
    RCC->CCIPR = (0u << RCC_CCIPR_FDCANSEL_Pos) | (0u << RCC_CCIPR_CLK48SEL_Pos)
        | (2u << RCC_CCIPR_ADC12SEL_Pos) | (0u << RCC_CCIPR_USART1SEL_Pos);

    SysTick_Config(F_SYS / 1000u);
    NVIC_SetPriority(SysTick_IRQn, 1);
}

// Busy wait, roughly calibrated for 170 MHz.
void delay_us(uint32_t us)
{
    uint32_t n = us * (F_SYS / 4000000u);
    while (n--)
        __NOP();
}

// Independent watchdog, 100 ms. Frozen while halted in the debugger.
void wdg_init(void)
{
    DBGMCU->APB1FZR1 |= DBGMCU_APB1FZR1_DBG_IWDG_STOP;
    IWDG->KR  = 0xCCCC; // start
    IWDG->KR  = 0x5555; // unlock
    IWDG->PR  = 2;      // 32 kHz / 16 = 2 kHz
    IWDG->RLR = 200;
    while (IWDG->SR) { }
    IWDG->KR = 0xAAAA;
}
