//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// OTA, the radio link, as tx relay and rx do it with the sx driver, see ota_protocol.h
//*******************************************************
// LoRa, explicit header, radio crc on, on a fixed frequency per band, see ota_link_freq_reg()
// 2.4 GHz: SF5 BW800
// sub-GHz: SF5 BW500, but SF7 BW500 if a SX127x can be on the other side, i.e. in 19 Hz 7x mode
//   a SX127x can do SF6 only with implicit header, SF7 is the lowest it can do with explicit header
//*******************************************************
#ifndef OTA_LINK_H
#define OTA_LINK_H
#pragma once


#include "ota_protocol.h"


// OTA_SX(NAME) gives the SX126X_NAME, SX1276_NAME, SX1280_NAME, LR11XX_NAME or LR20XX_NAME constant of the chip
#if defined DEVICE_HAS_SX128x
  #define OTA_SX(name)              SX1280_##name
  #define OTA_SX_IRQ_RX_ERROR       (SX1280_IRQ_CRC_ERROR | SX1280_IRQ_HEADER_ERROR)
#elif defined DEVICE_HAS_LR11xx
  #define OTA_SX(name)              LR11XX_##name
  #define OTA_SX_IRQ_RX_ERROR       (LR11XX_IRQ_PACKET_ERROR | LR11XX_IRQ_HEADER_ERROR)
#elif defined DEVICE_HAS_LR20xx
  #define OTA_SX(name)              LR20XX_##name
  #define OTA_SX_IRQ_RX_ERROR       (LR20XX_IRQ_CRC_ERROR | LR20XX_IRQ_LORA_HEADER_ERR)
#elif defined DEVICE_HAS_SX127x
  #define OTA_SX(name)              SX1276_##name
  #define OTA_SX_IRQ_RX_ERROR       SX1276_IRQ_PAYLOAD_CRC_ERROR
#else
  #define OTA_SX(name)              SX126X_##name
  #define OTA_SX_IRQ_RX_ERROR       (SX126X_IRQ_CRC_ERROR | SX126X_IRQ_HEADER_ERROR)
#endif

#define OTA_LINK_IS_2P4             (Config.Sx.FrequencyBand == SX_FHSS_FREQUENCY_BAND_2P4_GHZ)
#define OTA_LINK_IS_SF7             (Config.Mode == MODE_19HZ_7X) // a SX127x is only in this mode

#define OTA_LORA_CONFIGURATION(sf, bw, cr) \
    { .SpreadingFactor = sf, .Bandwidth = bw, .CodingRate = cr, .PreambleLength = 12, \
      .HeaderType = OTA_SX(LORA_HEADER_EXPLICIT), .PayloadLength = OTA_PACKET_LEN_MAX, \
      .CrcEnabled = OTA_SX(LORA_CRC_ENABLE), .InvertIQ = OTA_SX(LORA_IQ_NORMAL) }

// dual band chips: 2.4 GHz, sub-GHz, sub-GHz with a SX127x
#if defined DEVICE_HAS_SX128x
const tSxLoraConfiguration ota_lora_configuration[] = {
    OTA_LORA_CONFIGURATION(SX1280_LORA_SF5, SX1280_LORA_BW_800, SX1280_LORA_CR_LI_4_5) };
#elif defined DEVICE_HAS_SX127x
const tSxLoraConfiguration ota_lora_configuration[] = {
    OTA_LORA_CONFIGURATION(SX1276_LORA_SF7, SX1276_LORA_BW_500, SX1276_LORA_CR_4_5) };
#elif defined DEVICE_HAS_LR11xx
const tSxLoraConfiguration ota_lora_configuration[] = {
    OTA_LORA_CONFIGURATION(LR11XX_LORA_SF5, LR11XX_LORA_BW_800, LR11XX_LORA_CR_LI_4_5),
    OTA_LORA_CONFIGURATION(LR11XX_LORA_SF5, LR11XX_LORA_BW_500, LR11XX_LORA_CR_4_5),
    OTA_LORA_CONFIGURATION(LR11XX_LORA_SF7, LR11XX_LORA_BW_500, LR11XX_LORA_CR_4_5) };
#elif defined DEVICE_HAS_LR20xx
const tSxLoraConfiguration ota_lora_configuration[] = {
    OTA_LORA_CONFIGURATION(LR20XX_LORA_SF5, LR20XX_LORA_BW_812, LR20XX_LORA_CR_LI_4_5),
    OTA_LORA_CONFIGURATION(LR20XX_LORA_SF5, LR20XX_LORA_BW_500, LR20XX_LORA_CR_4_5),
    OTA_LORA_CONFIGURATION(LR20XX_LORA_SF7, LR20XX_LORA_BW_500, LR20XX_LORA_CR_4_5) };
#else
const tSxLoraConfiguration ota_lora_configuration[] = {
    OTA_LORA_CONFIGURATION(SX126X_LORA_SF5, SX126X_LORA_BW_500, SX126X_LORA_CR_4_5) };
#endif


const tSxLoraConfiguration* ota_link_lora_configuration(void)
{
#if defined DEVICE_HAS_LR11xx || defined DEVICE_HAS_LR20xx
    if (OTA_LINK_IS_2P4) return &ota_lora_configuration[0];
    return &ota_lora_configuration[(OTA_LINK_IS_SF7) ? 2 : 1];
#else
    return &ota_lora_configuration[0];
#endif
}


// the frequency of the band, the same as fhss' bind channel, 0 if sx can't be on the band
// sx is the sub-GHz chip on dual band devices, and only for 868/915 MHz on dual SX126x
#if defined DEVICE_HAS_SX128x
  #define OTA_FREQ_GHZ_TO_REG(f)    SX1280_FREQ_GHZ_TO_REG(f)
#elif defined DEVICE_HAS_LR11xx
  #define OTA_FREQ_GHZ_TO_REG(f)    LR11XX_FREQ_GHZ_TO_REG(f)
  #define OTA_FREQ_MHZ_TO_REG(f)    LR11XX_FREQ_MHZ_TO_REG(f)
#elif defined DEVICE_HAS_LR20xx
  #define OTA_FREQ_GHZ_TO_REG(f)    LR20XX_FREQ_GHZ_TO_REG(f)
  #define OTA_FREQ_MHZ_TO_REG(f)    LR20XX_FREQ_MHZ_TO_REG(f)
#elif defined DEVICE_HAS_SX127x
  #define OTA_FREQ_MHZ_TO_REG(f)    SX127X_FREQ_MHZ_TO_REG(f)
#else
  #define OTA_FREQ_MHZ_TO_REG(f)    SX126X_FREQ_MHZ_TO_REG(f)
#endif

uint32_t ota_link_freq_reg(uint8_t band)
{
    switch (band) {
#if defined FREQUENCY_BAND_2P4_GHZ && defined OTA_FREQ_GHZ_TO_REG
    case SX_FHSS_FREQUENCY_BAND_2P4_GHZ: return OTA_FREQ_GHZ_TO_REG(2.447);
#endif
#if defined OTA_FREQ_MHZ_TO_REG
  #if defined FREQUENCY_BAND_915_MHZ_FCC
    case SX_FHSS_FREQUENCY_BAND_915_MHZ_FCC: return OTA_FREQ_MHZ_TO_REG(913.8);
  #endif
  #if defined FREQUENCY_BAND_868_MHZ
    case SX_FHSS_FREQUENCY_BAND_868_MHZ: return OTA_FREQ_MHZ_TO_REG(863.275);
  #endif
  #if defined FREQUENCY_BAND_866_MHZ_IN
    case SX_FHSS_FREQUENCY_BAND_866_MHZ_IN: return OTA_FREQ_MHZ_TO_REG(865.375);
  #endif
  #if defined FREQUENCY_BAND_433_MHZ && !defined DEVICE_HAS_DUAL_SX126x_SX126x
    case SX_FHSS_FREQUENCY_BAND_433_MHZ: return OTA_FREQ_MHZ_TO_REG(433.360);
  #endif
  #if defined FREQUENCY_BAND_70_CM_HAM && !defined DEVICE_HAS_DUAL_SX126x_SX126x
    case SX_FHSS_FREQUENCY_BAND_70_CM_HAM: return OTA_FREQ_MHZ_TO_REG(436.4);
  #endif
#endif
    }
    return 0;
}


// dio is set for done, timeout and errors, the irq status is read only then
// the WL radio has no dio pin, the pending bit of its disabled irq is the dio
static inline bool ota_link_dio_is_set(void)
{
#ifdef SX_DIO
    return gpio_read_activehigh(SX_DIO);
#else
    return NVIC_GetPendingIRQ(SX_DIO_EXTI_IRQn);
#endif
}


// takes the radio over: the isrs are off, as they do spi and dio is polled, only sx is used, sx2 is held in reset
// as idle is not off for all chips. The restart of the controller, or the reboot, undoes it.
void ota_link_sx_only(void)
{
#ifdef ESP8266 // sx_dio_init_exti_isroff() is empty on these, and they have no sx2
    detachInterrupt(SX_DIO);
#else
    sx_dio_init_exti_isroff();
  #ifndef SX_DIO
    NVIC_DisableIRQ(SX_DIO_EXTI_IRQn); // the WL's isroff doesn't disable the irq
  #endif
  #ifdef USE_SX2
    sx2_dio_init_exti_isroff();
  #endif
#endif
    sx.SetToIdle(); // only after the isrs are off, they do spi
    sx2.SetToIdle();
#if defined USE_SX2 && defined SX2_RESET
    gpio_low(SX2_RESET);
  #ifdef SX2_RX_EN
    gpio_low(SX2_RX_EN); // reset doesn't switch off its front end
  #endif
  #ifdef SX2_TX_EN
    gpio_low(SX2_TX_EN);
  #endif
#endif
}


// on the WL the pending bit is cleared only after the irq status, else it pends again
uint32_t ota_link_get_and_clear_irq_status(void)
{
    uint32_t irq = sx.GetAndClearIrqStatus(OTA_SX(IRQ_ALL));
#ifndef SX_DIO
    NVIC_ClearPendingIRQ(SX_DIO_EXTI_IRQn);
#endif
    return irq;
}


void ota_link_set_packet_len(uint8_t len)
{
    const tSxLoraConfiguration* config = ota_link_lora_configuration();
#ifdef DEVICE_HAS_LR20xx
    sx.SetLoraPacketParams(config->PreambleLength, config->HeaderType, len, config->CrcEnabled, config->InvertIQ);
#else
    sx.SetPacketParams(config->PreambleLength, config->HeaderType, len, config->CrcEnabled, config->InvertIQ);
#endif
}


void ota_link_start(uint32_t sx_freq_reg)
{
#ifdef DEVICE_HAS_SX127x
    sx.SetStandby(); // it is in LoRa mode
#elif defined DEVICE_HAS_LR20xx
    sx.SetStandby(LR20XX_STANDBY_MODE_RC);
    delay_us(1000);
    sx.SetPacketType(LR20XX_PACKET_TYPE_LORA); // it may be in FLRC or GFSK mode
#else
    sx.SetStandby(OTA_SX(STDBY_CONFIG_STDBY_RC));
    delay_us(1000);
    sx.SetPacketType(OTA_SX(PACKET_TYPE_LORA)); // it may be in FLRC or GFSK mode
#endif
    sx.SetRfFrequency(sx_freq_reg);
    sx.SetLoraConfiguration(ota_link_lora_configuration());

    // also the errors, else a bad packet is taken as good, dio is set for done and timeout
#if defined DEVICE_HAS_SX128x
    sx.SetDioIrqParams(SX1280_IRQ_ALL, SX1280_IRQ_RX_DONE | SX1280_IRQ_TX_DONE | SX1280_IRQ_RX_TX_TIMEOUT | OTA_SX_IRQ_RX_ERROR,
                       SX1280_IRQ_NONE, SX1280_IRQ_NONE);
#elif defined DEVICE_HAS_LR11xx
    sx.SetDioIrqParams(LR11XX_IRQ_TX_DONE | LR11XX_IRQ_RX_DONE | LR11XX_IRQ_TIMEOUT | OTA_SX_IRQ_RX_ERROR, 0);
#elif defined DEVICE_HAS_LR20xx
    sx.SetDioIrqConfig(SX_USE_IRQ_DIO_NO, LR20XX_IRQ_TX_DONE | LR20XX_IRQ_RX_DONE | LR20XX_IRQ_TIMEOUT | OTA_SX_IRQ_RX_ERROR);
#elif defined DEVICE_HAS_SX127x
    sx.SetDioIrqParams(SX1276_IRQ_TX_DONE | SX1276_IRQ_RX_DONE | SX1276_IRQ_RX_TIMEOUT | OTA_SX_IRQ_RX_ERROR,
                       SX1276_DIO0_MAPPING_RX_TX_DONE, SX1276_DIO1_MAPPING_RX_TIMEOUT);
#else
    sx.SetDioIrqParams(SX126X_IRQ_ALL, SX126X_IRQ_RX_DONE | SX126X_IRQ_TX_DONE | SX126X_IRQ_RX_TX_TIMEOUT | OTA_SX_IRQ_RX_ERROR,
                       SX126X_IRQ_NONE, SX126X_IRQ_NONE);
#endif
    ota_link_get_and_clear_irq_status();
}


// reads what was received into buf, returns the length of the packet, 0 if it is of no use
uint8_t ota_link_read(uint32_t irq, uint8_t* buf)
{
    if (!(irq & OTA_SX(IRQ_RX_DONE)) || (irq & OTA_SX_IRQ_RX_ERROR)) return 0;
#ifdef DEVICE_HAS_LR20xx
    // it has a rx fifo, and the driver's ReadBuffer() is only for the few bytes of its isr
    uint16_t rx_len = sx.GetRxPktLength();
    if (rx_len < OTA_PACKET_HEADER_LEN || rx_len > OTA_PACKET_LEN_MAX) return 0;
    sx.ReadRadioRxFifo(buf, rx_len);
    return rx_len;
#else
    uint8_t rx_len, rx_start;
    sx.GetRxBufferStatus(&rx_len, &rx_start);
    if (rx_len < OTA_PACKET_HEADER_LEN || rx_len > OTA_PACKET_LEN_MAX) return 0;
    sx.ReadBuffer(rx_start, buf, rx_len);
    return rx_len;
#endif
}


static inline uint16_t ota_session_id(const uint8_t* buf) { return buf[1] | ((uint16_t)buf[2] << 8); }
static inline void ota_set_session_id(uint8_t* buf, uint16_t session_id) { buf[1] = session_id; buf[2] = session_id >> 8; }


#endif // OTA_LINK_H
