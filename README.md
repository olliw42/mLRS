<p align="left"><a href="https://raw.githubusercontent.com/olliw42/mLRS-docu/master/logos/mLRS_logo_long_w_slogan_1280x768.png"><img src="https://raw.githubusercontent.com/olliw42/mLRS-docu/master/logos/mLRS_logo_long_w_slogan_1280x768.png" align="center" height="153" width="256" ></a>

# mLRS #

The mLRS project offers an open source 2.4 GHz & 915/868 MHz & 433 MHz/70 cm LoRa-based high-performance long-range radio link, which provides bidirectional serial connection combined with full remote control.

The 'm' in the project name alludes to 'MAVLink', as it has features which optimizes performance for MAVLink systems. However, it always will also provide a transparent serial link and hence will be of wider use and not be limited to MAVLink systems only. The 'LR' in the project name alludes to 'long range', which however should not be understood in terms of an absolute range, like 100 km, but - of course - as the best possible range under the given conditions. Physical laws simply say that the higher the data rate the shorter the range. So, mLRS cannot compete range-wise with systems which achieve their range by reducing data rate to a minimum, at the cost of the resulting compromises. The goal of mLRS is to achieve a high range under the condition of a relatively high data rate. 

Typical specs could be 'plenty' of full-resolution RC channels, with 50 Hz update rate and serial data rates of about 3-5 kBytes/s at 2.4 GHz.

Many LRS or radio links with telemetry exist, among them open source projects such as SiK radios, OpenLRS, ExpressLRS, but also befinitiv wifibroadcast based projects like OpenHD or Ruby, closed source hobbyist projects such as UltimateLRS, QczekLRS, as well as commercial systems such as DragonLink, RFD900, Dronee Zoon, Siyi, and also TBS Crossfire and alike.

However, while all these systems are truely excellent and achieve their goals, and some of them are indeed close to what the project aims at, none of them checks all boxes, like 
- relatively cheap
- 2.4 GHz, 915/868 MHz, 433 MHz/70 cm
- LoRa
- full-duplex serial link with sufficient data rate
- plenty full-size RC channels
- open source
- rich features and outstanding performance for MAVLink systems

The mLRS project fills this gap.

## Disclaimer ##

You of course use the project fully at your own risk.

## Project Status ##

There is still plenty of room for ideas and improvement, and in this sense the project is work in progress. It is however fair to call it stable and robust, and to perform quite well. 

The mLRS system also provides a high level of usability such as a variety of options for input/output, parameter setting via the mLRS transmitter, optimization for ArduPilot/PX4 systems, wireless connection to ground control stations like MissionPlanner or QGC, or support of the Yaapu telemetry app without extra hazzles. It also integrates well with the mTX (formerly MAVLink for OpenTx) project, which yields a most fluid user experience.

It supports the SX1280/1, SX1276, SX1262 and LLCC68 Semtech chips, and thus the 2.4 GHz, 915/868 MHz and 433 MHz/70 cm frequency bands.

It provides 16 RC channels with the following layout (layout is equal in all operation modes):
- CH1 - CH8: 8 channels with 11 bit resolution (CH1 - CH4 have a higher reliability margin)
- CH9 - CH12: 4 channels with 8 bit resolution
- CH13 - CH16: 4 channels with three steps (CH13, CH14 have a higher reliability margin) 

It provides these operation modes:

|  | 50 Hz | 31 Hz | 19 Hz | FLRC (111 Hz) | FSK (50 Hz) |
| --- | --- | --- | --- | --- | --- |
| frequency<br>bands | 2.4 GHz | 2.4 GHz<br>915/868 MHz<br>433 MHz/70 cm | 2.4 GHz<br>915/868 MHz<br>433 MHz/70 cm | 2.4 GHz | 915/868 MHz<br>433 MHz/70 cm |
| chip sets | SX128x | SX128x,<br>SX126x/LLCC68 | SX128x,<br>SX126x/LLCC68,<br>SX1276 | SX128x | SX126x/LLCC68 |
| downlink<br>serial rate | 4100 Bytes/sec | 2562 Bytes/sec | 1547 Bytes/sec | 9111 Bytes/sec | 4100 Bytes/sec |
| uplink<br>serial rate | 3200 Bytes/sec | 2000 Bytes/sec | 1207 Bytes/sec | 7111 Bytes/sec | 3200 Bytes/sec |  
| receiver<br>sensitivity | -105 dBm | -108 dBm | -112 dBm | not for LR | not for LR |

Further features:
- full diversity: mLRS transmitters and receivers which feature two Semtech Lora chips provide full diversity, for both receiving and transmitting. This really improves link quality in the far range, and allows advanced dual-antenna setups on the transmitter side.
- dual band: mLRS transmitters and receivers with two Semtech Lora chips working in different RF bands are supported (e.g. 2.4 GHz and 915/868 MHz). These provide full diversity, with simultaneous transmission on both RF bands in addition. 
- adaptive ARQ/retransmission.
- the receiver parameters can be set from the mLRS transmitter or radio; no need to mess with the receiver for configuration in any way.
- the transmitter and receiver parameters can be set via a Lua script, CLI, or an OLED display. There is no need to recompile or reflash the firmware in order to change an option or parameter setting. 
- bind mode for binding "unknown" receivers to a transmitter.
- the mLRS system can be configured through the parameters for a wide range of applications and use cases, for details see [Documentation](https://github.com/olliw42/mLRS-docu).
- 10 model configurations stored in the mLRS transmitter, selected by "Receiver" number in OpenTx/EdgeTx radios.  
- support of CRSF and ArduPilot passthrough protocol; enables using the Yaapu Telemetry app on standard radios (out of the box, no need for extra dongles!).
- optimizations for ArduPilot and PX4 autopilot systems.
- technologies introduced by mLRS:
    - innovative flow control for MAVLink for smoother and more robust data flow 
    - MavlinkX for reduced packet loss and data compression
- support of MSP and optimizations for INAV autopilot systems. Enables using the INAV telemetry widget, in-flight connection to the INAV configurator, supports MSP-RC, and introduces MspX for reduced packet loss.
- "except" and "ortho" features
- support for buzzer, OLED display & five-way button, serial2. 
- support of ESP32 and ESP8266 modules for wireless connection to a ground control station.
- support of plenty platforms: STM32F103, STM32G4, STM32L4, STM32F3, STM32WLE5, Wio-E5, ESP8285, ESP32, E28, E22, E77, SX1280, SX1262, SX1276, LLCC68.

## Community ##

Discussion thread at rcgroups: https://www.rcgroups.com/forums/showthread.php?4037943-mLRS-Lora-based-Mavlink-oriented-open-source-radio-link

Discord server: https://discord.gg/vwjzCD6ws5

Facebook group: https://www.facebook.com/groups/mlrslink/

## Range ##

The range which one may expect can be estimated from the standard math; the [ImmersionRc RF Link Range](https://www.immersionrc.com/rf-calculators/) calculator comes in very handy here. Let's assume: power = 20 dBm (100 mW), antenna gain = 2 dBi, link margin = 12 dB (note: 12 dB link margin is conservative). Then, for the three LoRa modes:

| | 50 Hz | 31 Hz | 19 Hz
| --- | --- | --- | ---        
| 2.4 GHz | 7 km | 10 km | 15 km
| 868/915 MHz | - | 26 km | 42 km
| 433 MHz/70 cm | - | 55 km | 87 km

For the 2.4 GHz band, the available range test reports consistently exceed the above estimated ranges (e.g., [8.3 km were reported](https://www.rcgroups.com/forums/showpost.php?p=50964339&postcount=1721) for 2.4 GHz, 50 Hz, 9 dBm (8 mW), which translates to 29 km at 100 mW). For the other frequency bands less information is available (e.g. [5.4 km and 9.2 km were reported](https://discord.com/channels/1005096100572700794/1005096101239603232/1267115117145751694) for 868 MHz, 31 Hz and 19 Hz, 0 dBm (1 mW), which would translate to 54 km and 92 km at 100 mW). Note that mLRS supports full diversity, which when enabled has been found to significantly improve performance at lower link budget, i.e., allows you to operate at larger ranges.

The FLRC and FSK modes are not intended for long range.

## Hardware ##

The STM32 chipsets were chosen as main platform, and a good number of STM32 based devices are supported. The widely available ExpressLRS hardware, which uses ESP chipsets, is not ideal for mLRS, but also a good number of ExpressLRS devices are supported.

The code currently supports:
- MatekSys mLRS boards (2.4 GHz, 868/915 MHz)
- ExpressLRS transmitter modules and receivers (2.4 GHz and 868/915 MHz*)
- Frsky R9M and R9M Lite Pro transmitter modules and R9 MX, R9 MM and R9 Mini receivers (868/915 MHz*)
- SeeedStudio Wio-E5 Mini and Grove Wio-E5 boards (868/915 MHz, 433 MHz/70 cm)
- EByte E77 MBL board (868/915 MHz, 433 MHz/70 cm)
- Flysky FRM303 transmitter module (2.4 GHz)
- several DIY boards you can find in https://github.com/olliw42/mLRS-hardware

MatekSys offers a selection of quality mLRS boards, which are currently the best option available. They are specifically designed for mLRS, exploiting its full potential feature-wise. They support the 2.4 GHz and 868/915 MHz frequency bands, offer up to 1 W transmit power, and employ TCXOs. Furthermore, they use comparatively beefy STM32 MCUs, and are also very well suited for tinkering since all usable pins are broken out and easily accessible.

ExpressLRS hardware also provides a readily available entry into mLRS. The RadioMaster Bandit and Ranger or the BetaFPV 1W Micro modules make reasonably good mLRS Tx modules offering up to 1 W. Also the internal modules in a number of popular EdgeTx/ExpressLRS radios from RadioMaster and Jumper are supported. The various ExpressLRS receivers can also be good options; some of them provide up to 500 mW transmission power, and some others are cheaply available. Note though that not every ExpressLRS hardware is supported by mLRS. A main downside of the ExpressLRS 900 MHz gear is that they only support the 19 Hz mode, and are incompatible with the MatekSys and Wio-E5/E77/E22 based gear ([link](https://github.com/olliw42/mLRS-docu/blob/master/docs/SX126x_SX127x_INCOMPATIBILITY.md)).

In the 915/868 MHz range, the Frsky R9 system can be an option too. The Frsky R9M transmitter module makes a reasonable mLRS Tx module offering up to 1 W (the R9M Lite Pro cannot be used as transmitter). The R9 receivers are good options but provide low transmission power (50 mW). This can be mitigated by using a R9M Lite Pro or R9M as receiver (1 W). Here too, the downside is that the R9 devices only support the 19 Hz mode, and are incompatible with the MatekSys and Wio-E5/E77/E22 based gear ([link](https://github.com/olliw42/mLRS-docu/blob/master/docs/SX126x_SX127x_INCOMPATIBILITY.md)).

The SeeedStudio Wio-E5 boards and the EByte E77-MBL board are yet another readily available and reasonable options for entering mLRS. The "easy-to-solder" board, which uses an EByte E77 module, is a simple and cheap DIY option for building mLRS receivers and transmitters. It can also be used for building mLRS full diversity devices, as well as dual-band devices working simultaneously in e.g. the 868/915 MHz and 2.4 GHz ranges. All these boards are based on the STM32WL5E chip and thus provide all advantages of the SX126x chipset, such as the 31 Hz mode. Their maximum power is 22 dBm, and they can be used in the 915/868 MHz and 433 MHz/70 cm frequency ranges. 

Don't hesitate to join the discussion thread at rcgroups or the discord channel for more details.

## Firmware: Flashing ##

Most devices which are supported can be flashed with the [mLRS Desktop App](https://github.com/olliw42/mLRS-Flasher).  Additionally, Matek devices can be flashed using a web browser with the [mLRS Web Flasher](https://mlrs.xyz/flash).

Once your hardware is flashed, you can then use the mLRS transmitter to configure your needs via the CLI, the mLRS Configuration Lua script, or the OLED display if available. The mLRS receiver is configured by connecting to the transmitter, and then configuring it through the transmitter, exactly like the transmitter is configured.

## Software: Installation Bits and Bops ##

mLRS uses STM32CubeIDE for STM32 targets, and PlatformIO with VSCode for ESP32 and ESP8285 targets. For details see:

- [STM32 Development](https://github.com/olliw42/mLRS-docu/blob/master/docs/STM32_DEVELOPMENT.md)
- [ESP Development](https://github.com/olliw42/mLRS-docu/blob/master/docs/ESP_DEVELOPMENT.md)

In case of issues with the procedures, don't hesitate to join the discussion thread at rcgroups or the discord channel.

## Further Documentation ##

You find many more information here:

[mLRS Documentation](https://github.com/olliw42/mLRS-docu/blob/master/README.md)
