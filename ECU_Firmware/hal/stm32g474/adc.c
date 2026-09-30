/* ADC1 scans the six board inputs over and over, with DMA writing the results
   into a buffer, so a reading is always ready. Long sample times suit the high
   impedance dividers, and 16x oversampling averages out noise. */
#include "board.h"
#include "drivers.h"
#include "hal.h"
#include "system.h"

#define ADC_CR_SET_ONLY                                                                            \
    (ADC_CR_ADCAL | ADC_CR_JADSTP | ADC_CR_ADSTP | ADC_CR_JADSTART | ADC_CR_ADSTART | ADC_CR_ADDIS \
        | ADC_CR_ADEN) // writing these back as 1 would start things
#define SMP_247_5   6u
#define DMAMUX_ADC1 5u

static const uint32_t CHANNELS[ADC_COUNT] = { 1, 2, 3, 4, 15, 12 }; // hal.h order
static volatile uint16_t results[ADC_COUNT];

static void cr_set(uint32_t bit)
{
    ADC1->CR = (ADC1->CR & ~ADC_CR_SET_ONLY) | bit;
}

static void set_sample_time(uint32_t channel)
{
    if (channel < 10) {
        ADC1->SMPR1 |= SMP_247_5 << (3 * channel);
    } else {
        ADC1->SMPR2 |= SMP_247_5 << (3 * (channel - 10));
    }
}

// Sequence slot 1..6 gets a channel. SQR1 holds slots 1-4, SQR2 slots 5-9.
static void set_slot(uint32_t slot, uint32_t channel)
{
    if (slot <= 4) {
        ADC1->SQR1 |= channel << (6 * slot);
    } else {
        ADC1->SQR2 |= channel << (6 * (slot - 5));
    }
}

void adc_init(void)
{
    RCC->AHB2ENR |= RCC_AHB2ENR_ADC12EN;
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN | RCC_AHB1ENR_DMAMUX1EN;
    ADC12_COMMON->CCR = 3u << ADC_CCR_CKMODE_Pos; // HCLK/4 = 42.5 MHz

    // 1. power up and calibrate
    ADC1->CR = 0; // leave deep power down
    ADC1->CR = ADC_CR_ADVREGEN;
    delay_us(30);
    cr_set(ADC_CR_ADCAL);
    while (ADC1->CR & ADC_CR_ADCAL) { }
    delay_us(5);
    ADC1->ISR = ADC_ISR_ADRDY;
    cr_set(ADC_CR_ADEN);
    while (!(ADC1->ISR & ADC_ISR_ADRDY)) { }

    // 2. continuous scan of all six channels, 16x oversampled back to 12 bits
    ADC1->CFGR  = ADC_CFGR_CONT | ADC_CFGR_DMAEN | ADC_CFGR_DMACFG | ADC_CFGR_OVRMOD;
    ADC1->CFGR2 = ADC_CFGR2_ROVSE | (3u << ADC_CFGR2_OVSR_Pos) | (4u << ADC_CFGR2_OVSS_Pos);
    ADC1->SMPR1 = 0;
    ADC1->SMPR2 = 0;
    ADC1->SQR1  = (ADC_COUNT - 1) << ADC_SQR1_L_Pos;
    ADC1->SQR2  = 0;
    for (uint32_t i = 0; i < ADC_COUNT; i++) {
        set_sample_time(CHANNELS[i]);
        set_slot(i + 1, CHANNELS[i]);
    }

    // 3. DMA copies every result into the buffer, wrapping round forever
    DMAMUX1_Channel0->CCR = DMAMUX_ADC1;
    DMA1_Channel1->CPAR   = (uint32_t)&ADC1->DR;
    DMA1_Channel1->CMAR   = (uint32_t)results;
    DMA1_Channel1->CNDTR  = ADC_COUNT;
    DMA1_Channel1->CCR    = DMA_CCR_MINC | DMA_CCR_CIRC | (1u << DMA_CCR_PSIZE_Pos)
        | (1u << DMA_CCR_MSIZE_Pos) | DMA_CCR_EN;

    cr_set(ADC_CR_ADSTART);
}

float adc_volts(int channel)
{
    return results[channel] * (3.3f / 4095.0f);
}
