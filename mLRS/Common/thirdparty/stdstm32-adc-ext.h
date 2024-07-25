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
#ifndef STDSTM32_LL_ADC_EXT_H
#define STDSTM32_LL_ADC_EXT_H
#ifdef __cplusplus
extern "C" {
#endif


void adc_config_channel_tempsensor(ADC_TypeDef* ADCx, uint32_t Rank)
{
#ifdef STM32G4
    LL_ADC_REG_SetSequencerRanks(ADCx, Rank, LL_ADC_CHANNEL_TEMPSENSOR_ADC1);
    LL_ADC_SetChannelSamplingTime(ADCx, LL_ADC_CHANNEL_TEMPSENSOR_ADC1, LL_ADC_SAMPLINGTIME_640CYCLES_5); // slowest sampling time
    LL_ADC_SetChannelSingleDiff(ADCx, LL_ADC_CHANNEL_TEMPSENSOR_ADC1, LL_ADC_SINGLE_ENDED);

    //MODIFY_REG(ADCxy_COMMON->CCR, ADC_CCR_VREFEN | ADC_CCR_VSENSESEL | ADC_CCR_VBATSEL, PathInternal);
    LL_ADC_SetCommonPathInternalCh(__LL_ADC_COMMON_INSTANCE(ADC1), LL_ADC_PATH_INTERNAL_TEMPSENSOR);

    //SET_BIT(ADCxy_COMMON->CCR, PathInternal);
    //LL_ADC_SetCommonPathInternalChAdd(__LL_ADC_COMMON_INSTANCE(ADC1), LL_ADC_PATH_INTERNAL_TEMPSENSOR);
#endif
}


int16_t adc_tempsensor_convert(uint16_t ts_data)
{
#ifdef STM32G4
    // RM0440, 21.4.31
    // T in °C( ) = (TS_CAL2_TEMP - TS_CAL1_TEMP)/(TS_CAL2 - TS_CAL1) * (TS_DATA - TS_CAL1) + TS_CAL1_TEMP
    // DS13122, 3.18.1, table 5
    #define TS_CAL1_TEMP  30 // @ VDDA= VREF+ = 3.0 V
    #define TS_CAL2_TEMP  130
    #define TS_CAL1_ADR   0x1FFF75A8 // e.g.: 1042
    #define TS_CAL2_ADR   0x1FFF75CA //       1386

    int16_t TS_CAL1 = *(int16_t *)TS_CAL1_ADR;
    int16_t TS_CAL2 = *(int16_t *)TS_CAL2_ADR;

    int32_t t = (TS_CAL2_TEMP - TS_CAL1_TEMP);
    t *= ((int32_t)ts_data * 11 - (int32_t)TS_CAL1 * 10); // 3.3 V / 3.0 V = 11 / 10
    t /= (TS_CAL2 - TS_CAL1) * 10;
    t += TS_CAL1_TEMP;

    return t;
#endif
    return 0;
}


//-------------------------------------------------------
#ifdef __cplusplus
}
#endif
#endif // STDSTM32_LL_ADC_EXT_H
