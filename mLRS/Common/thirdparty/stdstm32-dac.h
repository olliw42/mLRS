//*******************************************************
// Copyright (c) OlliW, OlliW42, www.olliw.eu
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// my stripped down DAC standard library
//*******************************************************
// Interface:
//
//
//*******************************************************
#ifndef STDSTM32_LL_DAC_H
#define STDSTM32_LL_DAC_H
#ifdef __cplusplus
extern "C" {
#endif


// only F3 supported currently
#if defined STM32F3


//-------------------------------------------------------
// Defines
//-------------------------------------------------------



//-------------------------------------------------------
// helper routines
//-------------------------------------------------------


void dac_config_channel(DAC_TypeDef* DACx, uint32_t Channel, GPIO_TypeDef* GPIOx, uint32_t GPIO_Pin)
{
    gpio_init(GPIOx, GPIO_Pin, IO_MODE_INPUT_ANALOG, IO_SPEED_DEFAULT);

    LL_DAC_InitTypeDef DAC_InitStruct = {};

    DAC_InitStruct.TriggerSource = LL_DAC_TRIG_SOFTWARE;
    DAC_InitStruct.WaveAutoGeneration = LL_DAC_WAVE_AUTO_GENERATION_NONE;
    DAC_InitStruct.WaveAutoGenerationConfig = LL_DAC_WAVE_AUTO_GENERATION_NONE;
    DAC_InitStruct.OutputBuffer = LL_DAC_OUTPUT_BUFFER_ENABLE;
    LL_DAC_Init(DACx, Channel, &DAC_InitStruct);
    LL_DAC_DisableTrigger(DACx, Channel);

    LL_DAC_Enable(DACx, Channel);
}


void dac_write_channel(DAC_TypeDef* DACx, uint32_t Channel, uint16_t value)
{
    LL_DAC_ConvertData12RightAligned(DACx, Channel, value);
}


//-------------------------------------------------------
// INIT routines
//-------------------------------------------------------

void dac_init(DAC_TypeDef* DACx)
{
    rcc_init_dac(DACx);
}




#endif // defined STM32F3

//-------------------------------------------------------
#ifdef __cplusplus
}
#endif
#endif // STDSTM32_LL_DAC_H
