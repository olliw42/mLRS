# mLRS Documentation: SeeedStudio Wio-E5 Boards #

([back to main page](../README.md))

The SeeedStudio [Wio-E5 module](https://wiki.seeedstudio.com/LoRa-E5_STM32WLE5JC_Module) is a highly attractive module for building mLRS equipment. SeeedStudio provides a number of boards which are based on this module, and which are quite interesting hardware for mLRS. However, these boards also provide some inconveniences since their pins are not ready-made for the purposes of mLRS. So, some tweaking and (easy) soldering is required.


## SeeedStudio Wio-E5 mini dev Board ##

https://wiki.seeedstudio.com/LoRa_E5_mini/

Connections (name in respect to board print-ons):

- serial: Tx2,Rx2
- in: Rx1
- debug: D0
- com/cli: Tx,Rx and on-board USB plug
- led green: A4 (solder a led with resistor to GND)
- led red: on-board
- button: on-board (BOOT button)

## SeeedStudio Grove Wio-E5 Board ##

https://wiki.seeedstudio.com/Grove_LoRa_E5_New_Version/

Connections (name in respect to board print-ons):

- serial: Tx,Rx on connector, and solder pads on bottom
- out: none
- debug: none
- led green: none
- led red: on-board (solder jumper on bottom needs to be closed)
- button: solder button to BOOT solder pad on the bottom and GND
