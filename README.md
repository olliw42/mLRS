This is the dual-band branch of mLRS. 

It is experimental. It works well on the bench and the author has also test flown it without issues. It is experimental however since it hasn't seen yet as wide use as it should have seen in order to ensure that all little issues and edge cases have been all weeded out.

The two targets it is supposed to be used with are 'rx-easy-solder-E77-E22-dual-wle5cc' and 'tx-easy-solder-E77-E22-dual-wle5cc'. For these two targets the binaries are included, for other targets you need to compile the code yourself. 

Pl ask in the Discord channel for help and advice if needed. 

# mLRS #

The mLRS project offers an open source 2.4 GHz & 915/868 MHz & 433 MHz/70 cm LoRa-based high-performance long-range radio link, which provides transparent bidirectional serial connection combined with full remote control.

The 'm' in the project name alludes to 'Mavlink', as it will have features which optimizes performance for Mavlink systems. However, it always will also provide a transparent serial link and hence will be of wider use and by no means limited to Mavlink systems only. The 'LR' in the project name alludes to 'long range', which however should not be understood in terms of an absolute range, like 100 km, but - of course - as the best possible range under the given conditions. Physical laws simply say that the higher the data rate the shorter the range. So, mLRS cannot compete range-wise with systems which achieve their range by reducing data rate to the minimal, at the cost of the associated compromises. The goal of mLRS is to achieve a high range under the condition of a relatively high data rate. 

Typical specs could be 'plenty' of full-resolution RC channels, with 50 Hz update rate and serial data rates of about 3-5 kBytes/s at 2.4 GHz.

Many LRS or radio links with telemetry exist, among them open source projects such as SiK radios, OpenLRS, ExpressLRS, but also befinitiv wifibroadcast based projects like OpenHD or Ruby, closed source hobbyist projects such as UltimateLRS, QczekLRS, as well as commercial systems such as DragonLink, RFD900, Dronee Zoon, Siyi, but also TBS Crossfire and alike.

However, while all these systems are truely excellent and achieve their goals, and some of them are indeed close to what the project aims at, none of them checks all points, like 
- relatively cheap
- 2.4 GHz, 915/868 MHz, 433 MHz/70 cm
- LoRa
- full-duplex serial link with sufficient data rate
- plenty full-size RC channels
- open source
- rich features for Mavlink systems

The mLRS project fills this gap.

## Disclaimer ##

You of course use the project fully at your own risk.

## Project Status ##

The project is work in progress, and there is still plenty of room for ideas and improvement.

The essential features, however, i.e., the RC link and the serial (MAVLink) data link, are stable and robust, and perform quite well. 

The mLRS system also provides a high level of usability such as a variety of options for input/output, parameter setting via the mLRS transmitter, optimization for ArduPilot/PX4 systems, wireless connection to ground control stations like MissionPlanner or QGC, or support of the Yaapu telemetry app without extra hazzles. It also integrates well with the MAVLink for OpenTx project, which yields a most fluid user experience.

It supports the SX1280, SX1276, SX1262 and LLCC68 Semtech chips, and thus the 2.4 GHz, 915/868 MHz and 433 MHz/70 cm frequency bands.

It provides 16 RC channels with the following layout (layout is equal in all operation modes):
- CH1 - CH8: 8 channels with 11 bit resolution (CH1 - CH4 have a higher reliability margin)
- CH9 - CH12: 4 channels with 8 bit resolution
- CH13 - CH16: 4 channels with three steps (CH13, CH14 have a higher reliability margin) 

It provides these operation modes:
- 50 Hz Mode<br>
  frequency bands: 2.4 GHz (SX1280 chip)<br>
  uplink serial rate: 3200 Bytes/sec<br>
  downlink serial rate: 4100 Bytes/sec<br>
  receiver sensitivity: -105 dBm
- 31 Hz Mode<br>
  frequency bands: 2.4 GHz, 915/868 MHz, 433 MHz/70 cm (SX1280 and SX1262/LLCC68 chips)<br>
  uplink serial rate: 2000 Bytes/sec<br>
  downlink serial rate: 2562 Bytes/sec<br>
  receiver sensitivity: -108 dBm
- 19 Hz Mode<br>
  frequency bands: 2.4 GHz, 915/868 MHz, 433 MHz/70 cm (SX1280, SX1276, SX1262/LLCC68 chips)<br>
  uplink serial rate: 1207 Bytes/sec<br>
  downlink serial rate: 1547 Bytes/sec<br>
  receiver sensitivity: -112 dBm
- FLRC Mode (aka 111 Hz Mode)<br>
  frequency bands: 2.4 GHz (SX1280 chip)<br>
  uplink serial rate: 7111 Bytes/sec<br>
  downlink serial rate: 9111 Bytes/sec
- FSK Mode (50 Hz)<br>
  frequency bands: 915/868 MHz, 433 MHz/70 cm (SX1262/LLCC68 chips)<br>
  uplink serial rate: 3200 Bytes/sec<br>
  downlink serial rate: 4100 Bytes/sec

Further features:
- full diversity: mLRS transmitters and receivers which feature two Semtech Lora chips provide full diversity, for both receiving and transmitting. This really improves link quality in the far range, and allows advanced dual-antenna setups on the transmitter side.
- the receiver parameters can be set from the mLRS transmitter or radio; no need to mess with the receiver in any way.
- the transmitter and receiver parameters can be set via a Lua script, a CLI, or an OLED display. There is no need to recompile or reflash the firmware in order to change an option or parameter setting. 
- bind mode for binding "unknown" receivers to a transmitter.
- the mLRS system can be configured through the parameters for a wide range of applications and use cases. For a pictoral representation of some typical examples see [mLRS Setup examples](https://www.rcgroups.com/forums/showpost.php?p=48821735&postcount=332), and for more details [Documentation](https://github.com/olliw42/mLRS-docu).
- 10 model configurations stored in the mLRS transmitter, selected by "Receiver" number in OpenTx/EdgeTx radios.  
- support of CRSF and ArduPilot Passthrough protocol; enables using the Yaapu Telemetry app on standard radios (out of the box, no need for extra dongles anymore!).
- support for buzzer, OLED display & five-way button, serial2. 
- support of ESP32 modules for wireless connection to a ground control station.
- optimizations for ArduPilot and PX4 systems.
- advanced features like innovative flow control, "except" and "ortho" features, MavlinkX for reduced packet loss and data compression. Call them mLRS technology.
- support of plenty platforms: STM32F103, STM32G4, STM32L4, STM32WLE5, Wio-E5, E28, E22, E77, SX1280, SX1262, SX1276, LLCC68.

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

For the 2.4 GHz band, the available range test reports consistently exceed the above estimated ranges (e.g., [8.3 km were reported](https://www.rcgroups.com/forums/showpost.php?p=50964339&postcount=1721) for 2.4 GHz, 50 Hz, 9 dBm (8 mW), which translates to 29 km at 100 mW). For the other frequency bands less information is available. Note that mLRS supports full diversity, which when enabled has been found to significantly improve performance at lower link budget, i.e., allow to operate at larger ranges.

The FLRC and FSK modes are not intended for long range.

## Hardware ##

Hardware is still a problem. One might be tempted to think that the recent commercial ExpressLRS hardware should be good platforms, but this is unfortuantely not so. The ESP's they use simply do not offer the peripherals which are desired for mLRS transmitters, and STM32's were hence chosen as main platform. However, this is not a decission against ESP32, to the contrary: If anyone wants to add ESP32 support for mLRS then please join.

The code currently supports:
- Flysky FRM303 transmitter module (2.4 GHz)
- Frsky R9M and R9M Lite Pro transmitter modules and R9 MX, R9 MM and R9 Mini receivers (868/915 MHz)
- SeeedStudio Wio-E5 Mini and Grove Wio-E5 boards (868/915 MHz, 433 MHz/70 cm)
- EByte E77 MBL board (868/915 MHz, 433 MHz/70 cm)
- several DIY boards you can find in https://github.com/olliw42/mLRS-hardware

In the 915/868 MHz range, the Frsky R9M & R9 MX system provides a simple and readily available entry into mLRS. In this sense it is the best option available currently. Its big disadvantage is however that the receiver's transmission power is quite low and telemetry range thus relatively short. This can be mitigated by using the R9M Lite Pro (or R9M) transmitter module as receiver, which is supported by mLRS. 

The SeeedStudio Wio-E5 boards and the EByte E77-MBL board are also readily available, and hence excellent options too to enter mLRS. The "easy-to-solder" module, which uses an Ebyte E77 module, is a simple DIY option for building a mLRS receiver (it can also be used to build a mLRS transmitter). These boards are all based on the STM32WL5E chip and thus provide all the advantages of the SX1262, like the 31 Hz mode. Their maximum power is 22 dBm, and they can be used in the 915/868 MHz and 433 MHz/70 cm frequency ranges.

In the 2.4 GHz range, the Flysky FRM303 transmitter module is a great and readily available, albeit expensive, option. mLRS supports using it as transmitter as well as receiver. Concerning receivers, the DIY options are however probably the way to go. The DIY options also offer transmitters, including the most capable mLRS transmitters available.

Don't hesitate to join the discussion thread at rcgroups or the discord channel for more details.

## Firmware: Flashing ##

Ready-to-flash firmware can be found in the "firmware" folder. All you need to do is to flash the .hex file appropriate for your board into the device (it is not required to install the software for compiling as described in the next chapter). The mLRS transmitter can then be configured to your needs via the CLI, the mLRS Configuration Lua script, or the OLED display if available. The mLRS receiver is configured by first binding it to the transmitter, and then configuring it through the transmitter, exactly like the transmitter is configured.

## Software: Installation Bits and Bops ##

This is a STM32CubeIDE project. I don't have much experience with this framework, and it seems it is not ideal for shared projects. This procedure should work:

Let's assume that the project should be located in the folder C:/Me/Documents/Github/mlrs.
 
1. Clone and setup the project files
- open a command line processor
- cd into `C:/Me/Documents/Github` (not C:/Me/Documents/Github/mlrs !)
- `git clone https://github.com/olliw42/mLRS.git mlrs`
- `cd mlrs`
- run `run_setup.py`. This does two steps: initializes submodules, and generates mavlink library files.

For cloning you of course can use any other tool you like, but ensure that the submodules are also retrieved (git submodule --init --recursive).

2. STM32CubeIDE
- download and install STM32CubeIDE
- start STM32CubeIDE
- in Launcher select Workspace by hitting [Browse...] button, and browse to `C:/Me/Documents/Github/mlrs/mLRS`. Hit [Launch] button. ***Note***: it is not C:/Me/Documents/Github/mlrs but C:/Me/Documents/Github/mlrs/mLRS! If you proceed with the wrong path then there will be a compile error "undefined reference to main_main()"!
- in the IDE's top bar go to `File->Open Projects from File System`
- in the Importer select Import source by hitting [Directory...] button, and browse to the desired project. E.g. select `C:/Me/Documents/Github/mlrs/mLRS/rx-diy-board01-f103cb`. Hit [Finish] button.
- change from Debug to Release configuration: Go to the 'hammer' icon in the top icon bar, click on the down arrow right to it, and select `Release`. ***Note***: if you don't do that then there will be a compile error "undefined reference to main_main()"!
- open the file `mlrs-rx.cpp` or `mlrs-tx.cpp` into the editor
- compiling should work now: Go to the green 'right-pointing triangle' icon in the top icon bar and click it
- Repeat the last five steps for each board you are interested in

<img src="https://user-images.githubusercontent.com/6089567/154903396-25f62bf6-573a-4b80-9720-a0ad4a21f291.jpg" width="480">

The STM32CubeIDE has its weirdness, so you may have to get used to it. 

In case of issues with this procedure, don't hesitate to join the discussion thread at rcgroups or the discord channel, or submit an issue in the github repository.

#### Dependencies ####

You need to have git and Python3 installed (depending on the Python3 distribution you may need to install further libraries).

## Further Documentation ##

You find many more information here:

[mLRS Documentation](https://github.com/olliw42/mLRS-docu/blob/master/README.md)
