# mLRS #

The mLRS project offers an open source 2.4 GHz & 915/868 MHz & 433 MHz/70 cm LoRa-based high-performance long-range radio link, which provides bidirectional serial connection combined with full remote control.

The 'm' in the project name alludes to 'Mavlink', as it will have features which optimizes performance for Mavlink systems. However, it always will also provide a transparent serial link and hence will be of wider use and not be limited to Mavlink systems only. The 'LR' in the project name alludes to 'long range', which however should not be understood in terms of an absolute range, like 100 km, but - of course - as the best possible range under the given conditions. Physical laws simply say that the higher the data rate the shorter the range. So, mLRS cannot compete range-wise with systems which achieve their range by reducing data rate to a minimum, at the cost of the resulting compromises. The goal of mLRS is to achieve a high range under the condition of a relatively high data rate. 

Typical specs could be 'plenty' of full-resolution RC channels, with 50 Hz update rate and serial data rates of about 3-5 kBytes/s at 2.4 GHz.

Many LRS or radio links with telemetry exist, among them open source projects such as SiK radios, OpenLRS, ExpressLRS, but also befinitiv wifibroadcast based projects like OpenHD or Ruby, closed source hobbyist projects such as UltimateLRS, QczekLRS, as well as commercial systems such as DragonLink, RFD900, Dronee Zoon, Siyi, and also TBS Crossfire and alike.

However, while all these systems are truely excellent and achieve their goals, and some of them are indeed close to what the project aims at, none of them checks all boxes, like 
- relatively cheap
- 2.4 GHz, 915/868 MHz, 433 MHz/70 cm
- LoRa
- full-duplex serial link with sufficient data rate
- plenty full-size RC channels
- open source
- rich features and outstanding performance for Mavlink systems

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
- dual band: mLRS transmitters and receivers with two Semtech Lora chips working in different RF bands are supported (e.g. 2.4 GHz and 915/868 MHz). These provide full diversity, where in addition transmission is simultaneoulsy on both RF bands. 
- the receiver parameters can be set from the mLRS transmitter or radio; no need to mess with the receiver for configuration in any way.
- the transmitter and receiver parameters can be set via a Lua script, a CLI, or an OLED display. There is no need to recompile or reflash the firmware in order to change an option or parameter setting. 
- bind mode for binding "unknown" receivers to a transmitter.
- the mLRS system can be configured through the parameters for a wide range of applications and use cases. For a pictoral representation of some typical examples see [mLRS Setup examples](https://www.rcgroups.com/forums/showpost.php?p=48821735&postcount=332), and for more details [Documentation](https://github.com/olliw42/mLRS-docu).
- 10 model configurations stored in the mLRS transmitter, selected by "Receiver" number in OpenTx/EdgeTx radios.  
- support of CRSF and ArduPilot passthrough protocol; enables using the Yaapu Telemetry app on standard radios (out of the box, no need for extra dongles!).
- optimizations for ArduPilot and PX4 autopilot systems.
- technologies introduced by mLRS:
    - innovative flow control for smoother and robust data flow 
    - MavlinkX for reduced packet loss and data compression
- "except" and "ortho" features
- support for buzzer, OLED display & five-way button, serial2. 
- support of ESP32 modules for wireless connection to a ground control station.
- support of plenty platforms: STM32F103, STM32G4, STM32L4, STM32F3, STM32WLE5, Wio-E5, ESP8285, ESP32, E28, E22, E77, SX1280, SX1262, SX1276, LLCC68.

## Community ##

Discussion thread at rcgroups: https://www.rcgroups.com/forums/showthread.php?4037943-mLRS-Lora-based-Mavlink-oriented-open-source-radio-link

Discord server by LELE2022: https://discord.gg/vwjzCD6ws5

## Range ##

The range which one may expect can be estimated from the standard math; the [ImmersionRc RF Link Range](https://www.immersionrc.com/rf-calculators/) calculator comes in very handy here. Let's assume: power = 20 dBm (100 mW), antenna gain = 2 dBi, link margin = 12 dB (note: 12 dB link margin is conservative). Then, for the three LoRa modes:

| | 50 Hz | 31 Hz | 19 Hz
| --- | --- | --- | ---        
| 2.4 GHz | 7 km | 10 km | 15 km
| 868/915 MHz | - | 26 km | 42 km
| 433 MHz/70 cm | - | 55 km | 87 km

For the 2.4 GHz band, the available range test reports consistently exceed the above estimated ranges (e.g., [8.3 km were reported](https://www.rcgroups.com/forums/showpost.php?p=50964339&postcount=1721) for 2.4 GHz, 50 Hz, 9 dBm (8 mW), which translates to 29 km at 100 mW). For the other frequency bands less information is available. Note that mLRS supports full diversity, which when enabled has been found to significantly improve performance at lower link budget, i.e., allows you to operate at larger ranges.

The FLRC and FSK modes are not intended for long range.

## Hardware ##

Hardware is still a problem. Off-the-shelf hardware which is taylored at mLRS and exploits its full potential is not available. One could think that the ExpressLRS hardware available commercially since a while should be good platforms, but this is unfortunately not entirely so. The ESP chipsets which they use has several restrictions and lack peripherals which are desired for mLRS transmitters, and STM32 was hence chosen as main platform. A good number of STM32 based devices as well as some ExpressLRS hardware are supported.

The code currently supports:
- Frsky R9M and R9M Lite Pro transmitter modules and R9 MX, R9 MM and R9 Mini receivers (868/915 MHz)
- Flysky FRM303 transmitter module (2.4 GHz)
- SeeedStudio Wio-E5 Mini and Grove Wio-E5 boards (868/915 MHz, 433 MHz/70 cm)
- EByte E77 MBL board (868/915 MHz, 433 MHz/70 cm)
- ExpressLRS receivers (2.4 GHz and 868/915 MHz) (support for ExpressLRS transmitter modules is in development)
- several DIY boards you can find in https://github.com/olliw42/mLRS-hardware

In the 915/868 MHz range, the Frsky R9 system and ExpressLRS receivers provide a simple and readily available entry into mLRS. The disadvantage of the R9 receivers is their low transmission power (50 mW). This can be mitigated by using the R9M Lite Pro or R9M transmitter modules as receivers, which offer 1 W. The R9 receivers can also be cumbersome to flash. A better option is often the various ExpressLRS receivers, some of which provide up to 500 mW transmission power, and some of which are cheaply available. The combination of a Frsky R9M transmitter module and a ExpressLRS receiver is probably the best option available currently in the 900 MHz range. The downside of all these gear is that they only support the 19 Hz mode.

The SeeedStudio Wio-E5 boards and the EByte E77-MBL board are also readily available, and hence good options too for entering mLRS. The "easy-to-solder" board, which uses a EByte E77 module, is a simple DIY option for building mLRS receivers and transmitters. It can also be used to build mLRS dual-band devices. All these boards are based on the STM32WL5E chip and thus provide all advantages of the SX1262, like the 31 Hz mode. Their maximum power is 22 dBm, and they can be used in the 915/868 MHz and 433 MHz/70 cm frequency ranges. 

In the 2.4 GHz range, the Flysky FRM303 transmitter module is readily available, albeit quite expensive. mLRS supports using it as transmitter as well as receiver. Concerning receivers, the easiest option is ExpressLRS receivers. They are readily available, can provide up to 100 mW power, and some support full diversity. If the full potential of mLRS is however desired, the DIY options are the way to go (they are all based on STM32). For instance, they offer the most capable mLRS transmitters available.

Don't hesitate to join the discussion thread at rcgroups or the discord channel for more details.

## Firmware: Flashing ##

Ready-to-flash firmware can be found in the "firmware" folder. All you need to do is to flash the binary file appropriate for your board into the device (it is not required to install the software for compiling as described in the next chapter or the docs). The mLRS transmitter can then be configured to your needs via the CLI, the mLRS Configuration Lua script, or the OLED display if available. The mLRS receiver is configured by first binding it to the transmitter, and then configuring it through the transmitter, exactly like the transmitter is configured.

## Software: Installation Bits and Bops ##

mLRS uses STM32CubeIDE for STM32 targets, and PlatformIO with VSCode for ESP targets. For STM32 targets this procedure should work (for ESP targets see [ESP Development](https://github.com/olliw42/mLRS-docu/blob/master/docs/ESP_DEVELOPMENT.md)):

Let's assume that the project should be located in the folder C:/Me/Documents/Github/mlrs.
 
**I. Clone and setup the project files**
1. open a command line processor
2. cd into `C:/Me/Documents/Github` (not C:/Me/Documents/Github/mlrs !)
3. `git clone https://github.com/olliw42/mLRS.git mlrs`
4. `cd mlrs`
5. run `run_setup.py`. This does three steps: Initializes submodules (git submodule --init --recursive), copies ST HAL and LL drivers to the target folders, and generates the MAVLink library files.
    - ***Note***: Ensure that all three steps are executed completely.

For cloning you of course can use any other tool you like.

**II. STM32CubeIDE (for STM32 targets)**
1. download and install STM32CubeIDE
    - ***Note***: Install into the default folder if possible.
2. start STM32CubeIDE
3. in Launcher select Workspace by hitting [Browse...] button, and browse to `C:/Me/Documents/Github/mlrs/mLRS`. Hit [Launch] button.
    - ***Note***: It is not C:/Me/Documents/Github/mlrs but C:/Me/Documents/Github/mlrs/mLRS! If you proceed with the wrong path then there will be a compile error "undefined reference to main_main()"!
4. in the IDE's top bar go to `File->Open Projects from File System`
5. in the Importer select Import source by hitting [Directory...] button, and browse to the desired project. E.g. select `C:/Me/Documents/Github/mlrs/mLRS/rx-diy-board01-f103cb`. Hit [Finish] button.
6. change from Debug to Release configuration: Go to the 'hammer' icon in the top icon bar, click on the down arrow right to it, and select `Release`.
    - ***Note***: If you don't do that then there will be a compile error "undefined reference to main_main()"!
7. open the file `mlrs-rx.cpp` or `mlrs-tx.cpp` into the editor
8. compiling should work now: Go to the green 'right-pointing triangle' icon in the top icon bar and click it
9. repeat steps 4. - 8. for each board you are interested in

<img src="https://user-images.githubusercontent.com/6089567/154903396-25f62bf6-573a-4b80-9720-a0ad4a21f291.jpg" width="480">

The STM32CubeIDE has its weirdness, so you may have to get used to it. 

In case of issues with this procedure, don't hesitate to join the discussion thread at rcgroups or the discord channel, or submit an issue in the github repository.

#### Dependencies ####

You need to have git and Python3 installed. Depending on the Python3 distribution you may need to install further libraries.

## Further Documentation ##

You find many more information here:

[mLRS Documentation](https://github.com/olliw42/mLRS-docu/blob/master/README.md)
