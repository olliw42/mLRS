# mLRS Documentation: SeeedStudio Wio-E5 Boards #

([back to main page](../README.md))

The SeeedStudio [Wio-E5 module](https://wiki.seeedstudio.com/LoRa-E5_STM32WLE5JC_Module) is a highly attractive module for building mLRS equipment. SeeedStudio provides a number of boards which are based on this module, and which are quite interesting hardware for mLRS. However, these boards also provide some inconveniences since their pins are not ready-made for the purposes of mLRS. So, some tweaking and (easy) soldering is required.


## SeeedStudio Wio-E5 mini dev Board as Tx Module ##

https://wiki.seeedstudio.com/LoRa_E5_mini/

### As Tx Module ###

Connections (name in respect to board print-ons):

- serial: Tx2,Rx2
- in: Rx1
- com/cli: Tx,Rx and on-board USB plug
- debug: D0
- led green: A4 (solder a green LED with resistor > 300 Ohms to GND)
- led red: on-board
- button: on-board (BOOT button)

### As Rx Module ###

Connections (name in respect to board print-ons):

- serial: Tx2,Rx2
- out: Tx1
- debug: Tx, and on-board USB plug
- led green: A4 (solder a green LED with resistor > 300 Ohms to GND)
- led red: on-board
- button: on-board (BOOT button)

## SeeedStudio Grove Wio-E5 Board as Rx Module ##

https://wiki.seeedstudio.com/Grove_LoRa_E5_New_Version/

### As Tx Module ###

not recommended

### As Rx Module ###

Connections (name in respect to board print-ons):

- serial: Tx,Rx on connector, and solder pads on bottom
- out: none
- debug: none
- led green: none
- led red: on-board (solder jumper on the bottom of the board needs to be closed)
- button: BOOT solder pad (solder a button between the BOOT pad and GND)

Note: There is no convenient way to connect a green LED, and you thus won't get the information conveyed by it (like connection). It is possible to work around this but it would require some more sophisticated solder work.
