//*******************************************************
// Copyright (c) OlliW, OlliW42, www.olliw.eu
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// my stripped down WS2812 standard library
//*******************************************************
// Interface:
// #define WS2812_NUMBER_OF_LEDS
// #define WS2812_IO               IO_PC8
// #define WS2812_IO_AF            IO_AF_4
// #define WS2812_TIMx             TIM8
// #define WS2812_TIMno            8
// #define WS2812_CHno             3
// #define WS2812_DMAx             DMA2
// #define WS2812_DMA_CHANNEL_x    LL_DMA_CHANNEL_3
//*******************************************************
#ifndef STDSTM32_WS2812_H
#define STDSTM32_WS2812_H
#ifdef __cplusplus
extern "C" {
#endif


//-------------------------------------------------------
// WS2812 Defines
//-------------------------------------------------------

#define _APPEND(x,y)          x ## y
#define APPEND(x,y)           _APPEND(x,y)

#define _APPEND4(x,y,z,w)     x ## y ## z ## w
#define APPEND4(x,y,z,w)      _APPEND4(x,y,z,w)


#define WS2812_TIM_CHANNEL_CHx        APPEND(LL_TIM_CHANNEL_CH, WS2812_CHno)
#define WS2812_TIM_CCRx               APPEND(CCR, WS2812_CHno)

#define WS2812_DMAMUX_REQ_TIMx_CHy    APPEND4(LL_DMAMUX_REQ_TIM, WS2812_TIMno, _CH, WS2812_CHno)

#define WS2812_TIM_EnableDMAReq_CCx   APPEND(LL_TIM_EnableDMAReq_CC, WS2812_CHno)
#define WS2812_TIM_DisableDMAReq_CCx  APPEND(LL_TIM_DisableDMAReq_CC, WS2812_CHno)
#define WS2812_DMA_ClearFlag_TCx      APPEND(LL_DMA_ClearFlag_TC, WS2812_CHno)
#define WS2812_DMA_IsActiveFlag_TCx   APPEND(LL_DMA_IsActiveFlag_TC, WS2812_CHno)


//-------------------------------------------------------
// WS2812 Peripheral stuff
//-------------------------------------------------------
// TIM clock = 170 MHz
// 1 tick = 5.88 ns
// 1.25 µs / 5.88 ns ≈ 212.5 ticks

//#define WS2812_TIMLOW            13+8 //was +5  // 0.35 us, measured to ca. 0.2us -> correction
//#define WS2812_TIMHIGH           32-2  //was +3 // 0.90 us, measured to ca. 0.64us
//#define WS2812_PERIOD_US         5   // shouldn't be larger than 9us //WRONG, seems not to matter

#define WS2812_TIMLOW             70 // ~ 0,41 us
#define WS2812_TIMHIGH           140 // ~ 0.82 us
#define WS2812_PERIOD            212


void ws2812_periph_init(uint32_t buf_adr)
{
    rcc_init_afio();

    gpio_init_af(WS2812_IO, IO_MODE_OUTPUT_ALTERNATE_PP, WS2812_IO_AF, IO_SPEED_FAST);

    tim_config_up(WS2812_TIMx, WS2812_PERIOD, TIMER_BASE_MAX);

LL_TIM_OC_InitTypeDef TIM_OC_InitTypeDef = {};
    TIM_OC_InitTypeDef.OCMode       = LL_TIM_OCMODE_PWM1;
    TIM_OC_InitTypeDef.OCState      = LL_TIM_OCSTATE_ENABLE;
    TIM_OC_InitTypeDef.OCNState     = LL_TIM_OCSTATE_DISABLE;
    TIM_OC_InitTypeDef.CompareValue = 0;
    TIM_OC_InitTypeDef.OCPolarity   = LL_TIM_OCPOLARITY_HIGH;
    TIM_OC_InitTypeDef.OCNPolarity  = LL_TIM_OCPOLARITY_HIGH;
    TIM_OC_InitTypeDef.OCIdleState  = LL_TIM_OCIDLESTATE_LOW;
    TIM_OC_InitTypeDef.OCNIdleState = LL_TIM_OCIDLESTATE_LOW;
    LL_TIM_OC_Init(WS2812_TIMx, WS2812_TIM_CHANNEL_CHx, &TIM_OC_InitTypeDef);

    LL_TIM_OC_EnablePreload(WS2812_TIMx, WS2812_TIM_CHANNEL_CHx);
    LL_TIM_CC_EnableChannel(WS2812_TIMx, WS2812_TIM_CHANNEL_CHx);

    rcc_init_dma(WS2812_DMAx);

LL_DMA_InitTypeDef DMA_InitTypeDef = {};
    DMA_InitTypeDef.PeriphOrM2MSrcAddress  = (uint32_t)&(WS2812_TIMx->WS2812_TIM_CCRx);
    DMA_InitTypeDef.MemoryOrM2MDstAddress  = buf_adr; //(uint32_t)(ws2812handler.buf);
    DMA_InitTypeDef.Direction              = LL_DMA_DIRECTION_MEMORY_TO_PERIPH;
    DMA_InitTypeDef.Mode                   = LL_DMA_MODE_NORMAL;
    DMA_InitTypeDef.PeriphOrM2MSrcIncMode  = LL_DMA_PERIPH_NOINCREMENT;
    DMA_InitTypeDef.MemoryOrM2MDstIncMode  = LL_DMA_MEMORY_INCREMENT;
    DMA_InitTypeDef.PeriphOrM2MSrcDataSize = LL_DMA_PDATAALIGN_HALFWORD;
    DMA_InitTypeDef.MemoryOrM2MDstDataSize = LL_DMA_MDATAALIGN_HALFWORD;
    DMA_InitTypeDef.NbData                 = WS2812_NUMBER_OF_LEDS * 24 + 2;
    DMA_InitTypeDef.Priority               = LL_DMA_PRIORITY_VERYHIGH;

    LL_DMA_DeInit(WS2812_DMAx, WS2812_DMA_CHANNEL_x);
    LL_DMA_Init(WS2812_DMAx, WS2812_DMA_CHANNEL_x, &DMA_InitTypeDef);

    LL_DMA_SetPeriphRequest(WS2812_DMAx, WS2812_DMA_CHANNEL_x, WS2812_DMAMUX_REQ_TIMx_CHy);

    LL_TIM_EnableAllOutputs(WS2812_TIMx);
    LL_TIM_EnableCounter(WS2812_TIMx);
}



void ws2812_dma_stop(void)
{
    WS2812_TIM_DisableDMAReq_CCx(WS2812_TIMx);
    LL_DMA_DisableChannel(WS2812_DMAx, WS2812_DMA_CHANNEL_x);
    WS2812_DMA_ClearFlag_TCx(WS2812_DMAx);
}


void ws2812_dma_start(void)
{
    LL_DMA_SetDataLength(WS2812_DMAx, WS2812_DMA_CHANNEL_x, WS2812_NUMBER_OF_LEDS * 24 + 2);
    WS2812_DMA_ClearFlag_TCx(WS2812_DMAx);
    LL_DMA_EnableChannel(WS2812_DMAx, WS2812_DMA_CHANNEL_x);
    WS2812_TIM_EnableDMAReq_CCx(WS2812_TIMx);
}


uint16_t ws2812_dma_isready(void)
{
    if (!WS2812_DMA_IsActiveFlag_TCx(WS2812_DMAx)) return 0; // not yet done  // CHANNEL!

    WS2812_TIM_DisableDMAReq_CCx(WS2812_TIMx);
    LL_DMA_DisableChannel(WS2812_DMAx, WS2812_DMA_CHANNEL_x);
    WS2812_DMA_ClearFlag_TCx(WS2812_DMAx);
    return 1;
}


//-------------------------------------------------------
// WS2812 Handler stuff
//-------------------------------------------------------

typedef uint32_t tWs2812Color; // is GRB !!

#define WS2812_YELLOW   0x3f3f00
#define WS2812_PURPLE   0x003f3f
#define WS2812_CYAN     0x3f003f
#define WS2812_RED      0x00ff00 //0x003f00
#define WS2812_GREEN    0x1f0000 //0x3f0000;
#define WS2812_BLUE     0x00003f



tWs2812Color ws2812_rgbcolor(uint32_t r, uint32_t g, uint32_t b)
{
    return (r << 8) + (g << 16) + (b << 0);
}


tWs2812Color ws2812_rgbcolor_wbrightness(uint32_t r, uint32_t g, uint32_t b, uint16_t brightness)
{
    if (brightness > 100) brightness = 100;
    brightness *= 10;
    r = ((r * brightness + 500) / 1000 ) & 0x000000FF;
    g = ((g * brightness + 500) / 1000 ) & 0x000000FF;
    b = ((b * brightness + 500) / 1000 ) & 0x000000FF;
    return (r << 8) + (g << 16) + (b << 0);
}


tWs2812Color ws2812_scalecolor(tWs2812Color color, uint16_t brightness)
{
    uint32_t r = (color >>  8) & 0x000000FF;
    uint32_t g = (color >> 16) & 0x000000FF;
    uint32_t b = (color >>  0) & 0x000000FF;
    return ws2812_rgbcolor_wbrightness(r, g, b, brightness);
}


typedef struct {
    uint16_t state;
    uint16_t tlast_send_us;
    uint16_t buf[WS2812_NUMBER_OF_LEDS * 24 + 8];
} tWs2812Handler;

tWs2812Handler ws2812handler;

void ws2812_fill_all(tWs2812Color grb1);


void ws2812_init(void)
{
    ws2812_periph_init((uint32_t)(ws2812handler.buf));

    ws2812handler.state = 0;
    ws2812handler.tlast_send_us = 0;

    memset(ws2812handler.buf, 0, sizeof(ws2812handler.buf));
    ws2812_fill_all(0);
}


void ws2812_fill(uint8_t idx, tWs2812Color grb)
{
    if (idx >= WS2812_NUMBER_OF_LEDS) idx = WS2812_NUMBER_OF_LEDS - 1;

    uint16_t p = idx * 24;
    for (uint16_t i = 0; i < 24; i++) {
        ws2812handler.buf[p++] = (grb & 0x00800000) ? WS2812_TIMHIGH : WS2812_TIMLOW;
        grb <<= 1;
    }
}


void ws2812_fill_all(tWs2812Color grb1)
{
uint16_t p;

    p = 0;
    for (uint16_t n = 0; n < WS2812_NUMBER_OF_LEDS; n++) {
        tWs2812Color grb = grb1;
        for (uint16_t i = 0; i < 24; i++) {
            ws2812handler.buf[p++] = (grb & 0x00800000) ? WS2812_TIMHIGH : WS2812_TIMLOW;
            grb <<= 1;
        }
    }
/*    ws2812handler.buf[p++] = 0;
    ws2812handler.buf[p++] = 0;
    ws2812handler.buf[p++] = 0;
    ws2812handler.buf[p++] = 0;
    ws2812handler.buf[p++] = 0;
    ws2812handler.buf[p++] = 0; */
}


void ws2812_send(void)
{
    ws2812handler.state = 1;

    ws2812_dma_stop();
    ws2812_dma_start();
}


void ws2812_trigger(void)
{
    if (ws2812handler.state > 0) return; // ERROR
    ws2812handler.state = 1;

    ws2812_dma_start();
}


// call in the loop at high frequency
void ws2812_do(void)
{
    if (ws2812handler.state == 0) return; // not DMA running

    if (ws2812_dma_isready()) {
        ws2812handler.state = 0;
    }
}


//-------------------------------------------------------
#ifdef __cplusplus
}
#endif
#endif // STDSTM32_WS2812_H
