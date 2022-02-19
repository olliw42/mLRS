//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
// OlliW @ www.olliw.eu
//*******************************************************
// device configuration splicer
//*******************************************************

// enter define into "MCU G++ Compiler"->"Preprocessor" !!!
// for devices with I2C, un-comment #define HAL_I2C_MODULE_ENABLED in Core/Inc/stm32xxxx_hal_conf.h


#ifdef RX_SIYI_F373CC
  #define DEVICE_IS_RECEIVER
  #define DEVICE_HAS_SX128x
  #define FREQUENCY_BAND_2P4_GHZ
#endif

#ifdef TX_SIYI_F103C8
  #define DEVICE_IS_TRANSMITTER
  #define DEVICE_HAS_SX128x
  #define FREQUENCY_BAND_2P4_GHZ
#endif


#ifdef RX_DIY_BOARD01_F103CB
  #define DEVICE_IS_RECEIVER
  #define DEVICE_HAS_SX128x
  #define FREQUENCY_BAND_2P4_GHZ
#endif

#ifdef RX_DIY_E28_BOARD01_F103CB
  #define DEVICE_IS_RECEIVER
  #define DEVICE_HAS_SX128x
  #define FREQUENCY_BAND_2P4_GHZ
#endif


#ifdef TX_DIY_E28_BOARD01_F103CB
  #define DEVICE_IS_TRANSMITTER
  #define DEVICE_HAS_SX128x
  #define FREQUENCY_BAND_2P4_GHZ
#endif

#ifdef TX_DIY_MODULE01_G491RE
  #define DEVICE_IS_TRANSMITTER
  #define DEVICE_HAS_SX128x
  #define FREQUENCY_BAND_2P4_GHZ
#endif


#ifdef RX_R9MM_868_F103RB
  #define DEVICE_IS_RECEIVER
  #define DEVICE_HAS_SX127x
  #define FREQUENCY_BAND_868_MHZ
#endif

#ifdef TX_R9M_868_F103C8
  #define DEVICE_IS_TRANSMITTER
  #define DEVICE_HAS_SX127x
  #define FREQUENCY_BAND_868_MHZ
#endif


