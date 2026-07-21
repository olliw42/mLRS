//*******************************************************
// Copyright (c) OlliW, OlliW42, www.olliw.eu
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// my stripped down TIM standard library
//*******************************************************
// Interface:
//*******************************************************
#ifndef STDSTM32_LL_TIM_EXT_H
#define STDSTM32_LL_TIM_EXT_H
#ifdef __cplusplus
extern "C" {
#endif


void tim_config_oc(TIM_TypeDef* TIMx, uint32_t Channels)
{
#ifdef STM32G4
LL_TIM_OC_InitTypeDef TIM_OC_InitTypeDef = {};

    TIM_OC_InitTypeDef.OCMode       = LL_TIM_OCMODE_PWM1;
    TIM_OC_InitTypeDef.OCState      = LL_TIM_OCSTATE_ENABLE;
    TIM_OC_InitTypeDef.OCNState     = LL_TIM_OCSTATE_DISABLE;
    TIM_OC_InitTypeDef.CompareValue = 0;
    TIM_OC_InitTypeDef.OCPolarity   = LL_TIM_OCPOLARITY_HIGH;
    TIM_OC_InitTypeDef.OCNPolarity  = LL_TIM_OCPOLARITY_HIGH;
    TIM_OC_InitTypeDef.OCIdleState  = LL_TIM_OCIDLESTATE_LOW;
    TIM_OC_InitTypeDef.OCNIdleState = LL_TIM_OCIDLESTATE_LOW;
    LL_TIM_OC_Init(TIMx, Channels, &TIM_OC_InitTypeDef);

    LL_TIM_OC_EnablePreload(TIMx, Channels);
    LL_TIM_CC_EnableChannel(TIMx, Channels);
#endif
}


void tim_oc_enable(TIM_TypeDef* TIMx)
{
    if (IS_TIM_BREAK_INSTANCE(TIMx)) {
        LL_TIM_EnableAllOutputs(TIMx);
    }
}


//-------------------------------------------------------
#ifdef __cplusplus
}
#endif
#endif // STDSTM32_LL_TIM_EXT_H
