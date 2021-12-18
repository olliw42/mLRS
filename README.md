# mLRS #

This is the mLRS project.

The goal is an open source 2.4 GHz LORA-based high-performance long-range radio link, which provides fully transparent bidirectional serial connection combined with full remote control.

The 'm' in the project name alludes to 'Mavlink', as it will have features which optimizes performance for Mavlink systems. However, it always will also provide a transparent serial link and hence will be of wider use and by no means be limited to Mavlink systems only. The 'LR' in the project name alludes to 'long range', which however should not be understood in terms of an absolute range, like 50 km or so, but - of course - as the best possible range under the given conditions. Physical laws simply say that the higher the data rate the shorter the range. So, mLRS cannot compete range-wise with systems which achieve their range by reducing data rate to the minimal, taking according compromises. The goal of mLRS is to achieve a high range under the condition of a relatively high data rate. 

Typical specs could be 14 RC channels at "full" resolution (11 bit for channels 1-4, 8 bit for channels 5-14) with 50 Hz update rate, and serial data rates of about 3-5 kB/s.

Many LRS or radio links with telemetry exist, among them open source projects such as SiK radios, OpenLRS, ExpressLRS, but also befinitiv wifibroadcast based projects like OpenHD or Ruby, closed source hobbyist projects such as UltimateLRS, QczekLRS, as well as commercial systems such as DragonLink, RFD900, Dronee Zoon, Siyi, but also TBS Crossfire and alike.

However, while all these systems are truely excellent and achieve their goals, and some of them are indeed close to what the project aims at, none of them check all points, like 
- relatively cheap
- 2.4 GHz
- full-duplex with sufficient data rate
- open source
- rich features for Mavlink systems

Hence this project.

## Disclaimer ##

You of course use the project fully at your own risk.

## Project Status ##

The project is work in progress, and there is still a long mile to go before it could be called mature or reliable.

It is "working" in the sense that it offers a bidirectional serial link with RC data, and as such provides the basic framework. It also integrates with the MAVLink for OpenTx project. But as said, it is far from really usable.

## Installation Bits and Bops ##

This is a STM32CubeIde project. I don't have yet much experience with this framework, so I can't say much reliable, but this may work:
- download and install latest STM32CubeIde
- clone this repository; ensure that the submodules are also retrieved (if not run git submodule --init --recursive)
- open Stm32CubeIde and go to 'File'->'New'->'STM32 Project from existing .ioc' and browse to one of the .ioc files. IMPORTANT: before hitting 'Finish' check the 'C++' box.
- repeat this for all .ioc you want to open
- copy from the 'st-hal' folder the content of the respective 'STM32FXxx_HAL_Driver' folder to the equally named folder in the 'Driver' folder
- run 'fmav_generate_c_library.py' in 'mLRS/Common/mavlink'

Not very convennient yet, but this will improve with time :)

## Hardware ##

Hardware is quite a problem currently. One might be tempted to think that all the recent ExpressLRS hardware should be good platforms, but this is unfortuantely not so. The code so far is for the Siyi FM30 system (early version only, i.e., the version with the STM32 chips); the TX module needs few small hardware modifications.