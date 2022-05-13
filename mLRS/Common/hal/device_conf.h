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
  #define DEVICE_NAME "Siyi FM30 RX"
  #define DEVICE_IS_RECEIVER
  #define DEVICE_HAS_SX128x
  #define FREQUENCY_BAND_2P4_GHZ
#endif

#ifdef TX_SIYI_F103C8
  #define DEVICE_NAME "Siyi FM30 TX"
  #define DEVICE_IS_TRANSMITTER
  #define DEVICE_HAS_SX128x
  #define FREQUENCY_BAND_2P4_GHZ
#endif


#ifdef RX_DIY_BOARD01_F103CB
  #define DEVICE_NAME "DIY DualSX F103CB"
  #define DEVICE_IS_RECEIVER
  #define DEVICE_HAS_SX128x
  #define FREQUENCY_BAND_2P4_GHZ
#endif

#ifdef RX_DIY_E28DUAL_BOARD02_F103CB
  #define DEVICE_NAME "DIY DualE28 F103CB"
  #define DEVICE_IS_RECEIVER
  #define DEVICE_HAS_SX128x
  #define FREQUENCY_BAND_2P4_GHZ
#endif


#ifdef TX_DIY_E28DUAL_BOARD02_F103CB
  #define DEVICE_NAME "DIY DualE28 F103CB"
  #define DEVICE_IS_TRANSMITTER
  #define DEVICE_HAS_SX128x
  #define FREQUENCY_BAND_2P4_GHZ
#endif

#ifdef TX_DIY_BOARD01_G491RE
  #define DEVICE_NAME "DIY DualSX G491RE"
  #define DEVICE_IS_TRANSMITTER
  #define DEVICE_HAS_SX128x
  #define FREQUENCY_BAND_2P4_GHZ
#endif

#ifdef TX_DIY_E28DUAL_MODULE02_G491RE
  #define DEVICE_NAME "DIY DualE28 G491RE"
  #define DEVICE_IS_TRANSMITTER
  #define DEVICE_HAS_SX128x
  #define FREQUENCY_BAND_2P4_GHZ
#endif


#ifdef RX_R9MX_868_L433CB
  #define DEVICE_NAME "Frsky R9MX"
  #define DEVICE_IS_RECEIVER
  #define DEVICE_HAS_SX127x
  #define FREQUENCY_BAND_868_915_MHZ
  //#define FREQUENCY_BAND_868_MHZ
  //#define FREQUENCY_BAND_915_MHZ_FCC
#endif

#ifdef RX_R9MM_868_F103RB
  #define DEVICE_NAME "Frsky R9MM"
  #define DEVICE_IS_RECEIVER
  #define DEVICE_HAS_SX127x
  #define FREQUENCY_BAND_868_915_MHZ
  //#define FREQUENCY_BAND_868_MHZ
  //#define FREQUENCY_BAND_915_MHZ_FCC
#endif

#ifdef TX_R9M_868_F103C8
  #define DEVICE_NAME "Frsky R9"
  #define DEVICE_IS_TRANSMITTER
  #define DEVICE_HAS_SX127x
  #define FREQUENCY_BAND_868_915_MHZ
  //#define FREQUENCY_BAND_868_MHZ
  //#define FREQUENCY_BAND_915_MHZ_FCC
#endif


