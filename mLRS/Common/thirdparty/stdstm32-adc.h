//*******************************************************
// Copyright (c) OlliW, OlliW42, www.olliw.eu
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// my stripped down ADC standard library
//*******************************************************
// Interface:
//
// #define ADC_USE_DMA
//
// #define ADC_SAMPLINGTIME
// #define ADC_DMA_PRIORITY
//
//*******************************************************
#ifndef STDSTM32_LL_ADC_H
#define STDSTM32_LL_ADC_H
#ifdef __cplusplus
extern "C" {
#endif


// only F1, G4, F7, WL, F0 supported currently
#if defined STM32F1 || defined STM32G4 || defined STM32F7 || defined STM32WL || defined STM32F0

#if defined ADC_USE_DMA && (defined STM32WL || defined STM32F0)
  #error DMA not supported for STM32WL!
#endif


//-------------------------------------------------------
// Defines
//-------------------------------------------------------

#ifndef ADC_SAMPLINGTIME
  #ifdef STM32F1
    #define ADC_SAMPLINGTIME    LL_ADC_SAMPLINGTIME_239CYCLES_5
  #elif defined STM32F7
    #define ADC_SAMPLINGTIME    LL_ADC_SAMPLINGTIME_480CYCLES
  #elif defined STM32G4
    #define ADC_SAMPLINGTIME    LL_ADC_SAMPLINGTIME_640CYCLES_5
  #elif defined STM32WL
    #define ADC_SAMPLINGTIME    LL_ADC_SAMPLINGTIME_160CYCLES_5
  #elif defined STM32F0
    #define ADC_SAMPLINGTIME    LL_ADC_SAMPLINGTIME_239CYCLES_5
  #endif
#endif


#ifndef ADC_DMA_PRIORITY
  #define ADC_DMA_PRIORITY      LL_DMA_PRIORITY_MEDIUM
#endif


//-------------------------------------------------------
// helper routines
//-------------------------------------------------------

#ifdef STM32F1
void delay_us(uint32_t us);

void _adc_ADC_SelfCalibrate(ADC_TypeDef* ADCx)
{ /*
    ADC_ResetCalibration(ADCx);
    while (ADC_GetResetCalibrationStatus(ADCx)) {};
    ADC_StartCalibration(ADCx);
    while (ADC_GetCalibrationStatus(ADCx)) {}; */
    delay_us(100); // LL_ADC_DELAY_DISABLE_CALIB_ADC_CYCLES
    LL_ADC_StartCalibration(ADCx);
    while (LL_ADC_IsCalibrationOnGoing(ADCx)) {}
}
#else
void _adc_ADC_SelfCalibrate(ADC_TypeDef* ADCx) {}
#endif


//-------------------------------------------------------
// Sequencer routines
//-------------------------------------------------------

void adc_init_one_channel(ADC_TypeDef* ADCx)
{
LL_ADC_InitTypeDef ADC_InitStruct = {};
LL_ADC_REG_InitTypeDef ADC_REG_InitStruct = {};

    ADC_InitStruct.DataAlignment = LL_ADC_DATA_ALIGN_RIGHT;
#if defined STM32F1
    ADC_InitStruct.SequencersScanMode = LL_ADC_SEQ_SCAN_DISABLE;
#elif defined STM32G4
    ADC_InitStruct.Resolution = LL_ADC_RESOLUTION_12B;
    ADC_InitStruct.LowPowerMode = LL_ADC_LP_MODE_NONE;
#elif defined STM32F7
    ADC_InitStruct.Resolution = LL_ADC_RESOLUTION_12B;
    ADC_InitStruct.SequencersScanMode = LL_ADC_SEQ_SCAN_DISABLE;
#elif defined STM32WL || defined STM32F0
    ADC_InitStruct.Clock = LL_ADC_CLOCK_SYNC_PCLK_DIV4;
    ADC_InitStruct.Resolution = LL_ADC_RESOLUTION_12B;
    ADC_InitStruct.LowPowerMode = LL_ADC_LP_MODE_NONE;
#endif
    LL_ADC_Init(ADCx, &ADC_InitStruct);

#if defined STM32F1 || defined STM32G4 || defined STM32F7
LL_ADC_CommonInitTypeDef ADC_CommonInitStruct = {};

    ADC_CommonInitStruct.Multimode = LL_ADC_MULTI_INDEPENDENT;
#if defined STM32F7 || defined STM32G4
    // F7: ADC clock is derived from PCLK2, which is 108 MHz, but ADC seems to be limited to 36 MHz, hence div by 4 = 27 MHz
    ADC_CommonInitStruct.CommonClock = LL_ADC_CLOCK_SYNC_PCLK_DIV4;
#endif
    LL_ADC_CommonInit(__LL_ADC_COMMON_INSTANCE(ADCx), &ADC_CommonInitStruct);
#endif

#ifdef STM32WL
    LL_ADC_REG_SetSequencerConfigurable(ADCx, LL_ADC_REG_SEQ_CONFIGURABLE);
    while (!LL_ADC_IsActiveFlag_CCRDY(ADCx)) {} // poll for ADC channel configuration ready
    LL_ADC_ClearFlag_CCRDY(ADCx);
#endif

    ADC_REG_InitStruct.TriggerSource = LL_ADC_REG_TRIG_SOFTWARE;
#ifndef STM32F0
    ADC_REG_InitStruct.SequencerLength = LL_ADC_REG_SEQ_SCAN_DISABLE;
#endif
    ADC_REG_InitStruct.SequencerDiscont = LL_ADC_REG_SEQ_DISCONT_DISABLE;
    ADC_REG_InitStruct.ContinuousMode = LL_ADC_REG_CONV_CONTINUOUS;
    ADC_REG_InitStruct.DMATransfer = LL_ADC_REG_DMA_TRANSFER_NONE;
#if defined STM32G4 || defined STM32WL || defined STM32F0
    ADC_REG_InitStruct.Overrun = LL_ADC_REG_OVR_DATA_OVERWRITTEN; //xxLL_ADC_REG_OVR_DATA_PRESERVED;
#endif
    LL_ADC_REG_Init(ADCx, &ADC_REG_InitStruct);
}


#ifdef ADC_USE_DMA

void adc_init_dma(ADC_TypeDef* ADCx, uint8_t no_of_channels, DMA_TypeDef* DMAx, uint32_t dma_channel, uint32_t m_ptr)
{
LL_DMA_InitTypeDef DMA_InitStruct = {};

#if defined STM32F1 || defined STM32G4
    LL_DMA_DeInit(DMAx, dma_channel);
#elif defined STM32F7
    LL_DMA_DeInit(DMA, DMA_STREAM);
#endif

    DMA_InitStruct.PeriphOrM2MSrcAddress = LL_ADC_DMA_GetRegAddr(ADCx, LL_ADC_DMA_REG_REGULAR_DATA);
    DMA_InitStruct.PeriphOrM2MSrcDataSize = LL_DMA_PDATAALIGN_HALFWORD;
    DMA_InitStruct.MemoryOrM2MDstDataSize = LL_DMA_MDATAALIGN_HALFWORD;
    DMA_InitStruct.PeriphOrM2MSrcIncMode = LL_DMA_PERIPH_NOINCREMENT;
    DMA_InitStruct.Mode = LL_DMA_MODE_CIRCULAR;
    DMA_InitStruct.Priority = ADC_DMA_PRIORITY;
    DMA_InitStruct.NbData = no_of_channels;

    DMA_InitStruct.Direction = LL_DMA_DIRECTION_PERIPH_TO_MEMORY;
    DMA_InitStruct.MemoryOrM2MDstAddress = m_ptr; //(uint32_t)&(_adc_hal.potreadings);
    DMA_InitStruct.MemoryOrM2MDstIncMode = LL_DMA_MEMORY_INCREMENT;
#ifdef STM32F7
    DMA_InitStruct.FIFOMode = LL_DMA_FIFOMODE_DISABLE;
    DMA_InitStruct.Channel = DMA_CHANNEL;
#endif
#ifdef STM32G4
    // DMA_InitStruct.PeriphRequest = DMAMUX_REQ_ADC;
    if (ADCx == ADC1) { DMA_InitStruct.PeriphRequest = LL_DMAMUX_REQ_ADC1; } else
    if (ADCx == ADC2) { DMA_InitStruct.PeriphRequest = LL_DMAMUX_REQ_ADC2; } else
    if (ADCx == ADC3) { DMA_InitStruct.PeriphRequest = LL_DMAMUX_REQ_ADC3; } else { while (1) {} }
#endif

#if defined STM32F1 || defined STM32G4
    LL_DMA_Init(DMAx, dma_channel, &DMA_InitStruct);
    LL_DMA_EnableChannel(DMAx, dma_channel);
#elif defined STM32F7
    LL_DMA_Init(DMAx, DMA_STREAM, &DMA_InitStruct);
    LL_DMA_EnableStream(DMAx, DMA_STREAM);
#endif
}


void adc_init_scan_dma(ADC_TypeDef* ADCx, uint8_t no_of_channels)
{
LL_ADC_InitTypeDef ADC_InitStruct = {};
LL_ADC_CommonInitTypeDef ADC_CommonInitStruct = {};
LL_ADC_REG_InitTypeDef ADC_REG_InitStruct = {};

    ADC_InitStruct.DataAlignment = LL_ADC_DATA_ALIGN_RIGHT;
#if defined STM32F1
    ADC_InitStruct.SequencersScanMode = LL_ADC_SEQ_SCAN_ENABLE;
#elif defined STM32G4
    ADC_InitStruct.Resolution = LL_ADC_RESOLUTION_12B;
    ADC_InitStruct.LowPowerMode = LL_ADC_LP_MODE_NONE;
#elif defined STM32F7
    ADC_InitStruct.Resolution = LL_ADC_RESOLUTION_12B;
    ADC_InitStruct.SequencersScanMode = LL_ADC_SEQ_SCAN_ENABLE;
#endif
    LL_ADC_Init(ADCx, &ADC_InitStruct);

    ADC_CommonInitStruct.Multimode = LL_ADC_MULTI_INDEPENDENT;
#if defined STM32F7 || defined STM32G4
    // F7: ADC clock is derived from PCLK2, which is 108 MHz, but ADC seems to be limited to 36 MHz, hence div by 4 = 27 MHz
    ADC_CommonInitStruct.CommonClock = LL_ADC_CLOCK_SYNC_PCLK_DIV4;
#endif
    LL_ADC_CommonInit(__LL_ADC_COMMON_INSTANCE(ADCx), &ADC_CommonInitStruct);

    ADC_REG_InitStruct.TriggerSource = LL_ADC_REG_TRIG_SOFTWARE;
    ADC_REG_InitStruct.SequencerLength = LL_ADC_REG_SEQ_SCAN_DISABLE;
    switch (no_of_channels) {
        case 2: ADC_REG_InitStruct.SequencerLength = LL_ADC_REG_SEQ_SCAN_ENABLE_2RANKS; break;
        case 3: ADC_REG_InitStruct.SequencerLength = LL_ADC_REG_SEQ_SCAN_ENABLE_3RANKS; break;
        case 4: ADC_REG_InitStruct.SequencerLength = LL_ADC_REG_SEQ_SCAN_ENABLE_4RANKS; break;
    }
    ADC_REG_InitStruct.SequencerDiscont = LL_ADC_REG_SEQ_DISCONT_DISABLE;
    ADC_REG_InitStruct.ContinuousMode = LL_ADC_REG_CONV_CONTINUOUS;
    ADC_REG_InitStruct.DMATransfer = LL_ADC_REG_DMA_TRANSFER_UNLIMITED;
#ifdef STM32G4
    ADC_REG_InitStruct.Overrun = LL_ADC_REG_OVR_DATA_OVERWRITTEN;
#endif
    LL_ADC_REG_Init(ADCx, &ADC_REG_InitStruct);
}

#endif // ADC_USE_DMA


void adc_config_channel(ADC_TypeDef* ADCx, uint32_t Rank, uint32_t Channel, GPIO_TypeDef* GPIOx, uint32_t GPIO_Pin)
{
#ifndef STM32F0
    LL_ADC_REG_SetSequencerRanks(ADCx, Rank, Channel);
#else
    LL_ADC_REG_SetSequencerScanDirection(ADCx, LL_ADC_REG_SEQ_SCAN_DIR_FORWARD);
    LL_ADC_REG_SetSequencerChAdd(ADCx, Channel);
    //LL_ADC_REG_SetSequencerChannels(ADCx, Channel);
#endif

#ifdef STM32WL
    while (!LL_ADC_IsActiveFlag_CCRDY(ADCx)) {} // poll for ADC channel configuration ready
    LL_ADC_ClearFlag_CCRDY(ADCx);
#endif

#if defined STM32F1 || defined STM32G4 || defined STM32F7
    LL_ADC_SetChannelSamplingTime(ADCx, Channel, ADC_SAMPLINGTIME);
#elif defined STM32F0
    LL_ADC_SetSamplingTimeCommonChannels(ADCx, ADC_SAMPLINGTIME);
#else
    LL_ADC_SetChannelSamplingTime(ADCx, Channel, LL_ADC_SAMPLINGTIME_COMMON_1);
#endif

#ifdef STM32G4
    LL_ADC_SetChannelSingleDiff(ADCx, Channel, LL_ADC_SINGLE_ENDED);
#endif

    gpio_init(GPIOx, GPIO_Pin, IO_MODE_INPUT_ANALOG, IO_SPEED_DEFAULT);
}


//-------------------------------------------------------
// INIT routines
//-------------------------------------------------------

void adc_init_begin(ADC_TypeDef* ADCx)
{
#ifdef STM32F0
    // ADC disable sequence from datasheet, needed to avoid getting stuck in enable
    // ADCx->CR |= ADC_CR_ADSTP;                   // stop any ongoing conversion
    // while ((ADCx->CR & ADC_CR_ADSTP) != 0) {}   // wait until conversion is stopped
    // ADCx->CR |= ADC_CR_ADDIS;                   // disable ADC
    // while ((ADCx->CR & ADC_CR_ADEN) != 0) {}    // wait until ADC is fully disabled
    LL_ADC_REG_StopConversion(ADCx);
    while (LL_ADC_REG_IsStopConversionOngoing(ADCx)) {}
    LL_ADC_Disable(ADCx);
    //while (LL_ADC_IsEnabled(ADCx)) {}
    while (LL_ADC_IsDisableOngoing(ADCx)) {}
#endif
#ifdef STM32G4
    LL_RCC_SetADCClockSource(LL_RCC_ADC12_CLKSOURCE_SYSCLK);
#endif
    rcc_init_afio();
    rcc_init_adc(ADCx);
}


void adc_enable(ADC_TypeDef* ADCx)
{
#if defined STM32G4
    LL_ADC_SetGainCompensation(ADCx, 0);
    LL_ADC_SetOverSamplingScope(ADCx, LL_ADC_OVS_DISABLE);
    LL_ADC_DisableDeepPowerDown(ADCx);
#elif defined STM32WL
    LL_ADC_SetOverSamplingScope(ADCx, LL_ADC_OVS_DISABLE);
    LL_ADC_SetSamplingTimeCommonChannels(ADCx, LL_ADC_SAMPLINGTIME_COMMON_1, ADC_SAMPLINGTIME);
    LL_ADC_SetSamplingTimeCommonChannels(ADCx, LL_ADC_SAMPLINGTIME_COMMON_2, ADC_SAMPLINGTIME);
    //LL_ADC_DisableIT_EOC(ADCx);
    //LL_ADC_DisableIT_EOS(ADCx);
    //LL_ADC_SetTriggerFrequencyMode(ADCx, LL_ADC_TRIGGER_FREQ_HIGH);
#endif

#if defined STM32G4 || defined STM32WL
    LL_ADC_EnableInternalRegulator(ADCx);
    uint32_t wait_loop_index;
    wait_loop_index = ((LL_ADC_DELAY_INTERNAL_REGUL_STAB_US * (SystemCoreClock / (100000 * 2))) / 10);
    while (wait_loop_index != 0) { wait_loop_index--; }
#endif

#ifndef STM32F0
    LL_ADC_Enable(ADCx);
#else
    // ADC enable sequence from datasheet
    // if ((ADCx->ISR & ADC_ISR_ADRDY) != 0) { ADCx->ISR |= ADC_ISR_ADRDY; } // ensure ADRDY = 0
    // ADCx->CR |= ADC_CR_ADEN;                                              // enable ADC
    // while ((ADCx->ISR & ADC_ISR_ADRDY) == 0) {}                           // wait until ADC read
    LL_ADC_ClearFlag_ADRDY(ADCx);
    LL_ADC_Enable(ADCx);
    while (!LL_ADC_IsActiveFlag_ADRDY(ADCx)) {} // makes it stuck after restart if not disable before
#endif

    _adc_ADC_SelfCalibrate(ADCx);
}


void adc_start_conversion(ADC_TypeDef* ADCx)
{
#if defined STM32F1 || defined STM32F7
    LL_ADC_REG_StartConversionSWStart(ADCx);
#elif defined STM32G4 || defined STM32WL || defined STM32F0
    LL_ADC_REG_StartConversion(ADCx);
#endif
}


#endif // defined STM32F1 || defined STM32G4 || defined STM32F7 || defined STM32F0

//-------------------------------------------------------
#ifdef __cplusplus
}
#endif
#endif // STDSTM32_LL_ADC_H
