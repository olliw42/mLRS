# mLRS Documentation: Frsky R9M, R9MX #

([back to main page](../README.md))

The Frsky R9M transmitter module and R9MX receiver are commercially available and hence interesting hardware for mLRS. However, the R9 system also provides some hurdles in that flashing is currently a bit of a hack and in that the R9M hardware is not entirely fit for the purpose. Hence this page.


## R9M Tx Module ##

TODO: serial port limitations

TODO: dip switches


## Flashing ##

ExpressLRS has figured out a convenient and easy way, which is unfortunately not available for mLRS (someone needs to figure it out). For flashing mLRS only the grass-route DIY procedure using a ST-Link programmer is currently possible.

Note: Fashing mLRS with the ST-Link is a non-reversible operation, i.e., it is not possible to revert back to the original Frsky firmware. You can still switch to ExpressLRS.

Comment: mLRS also supports the R9MM receiver. However, flashing the R9MM with ST-Link is extremely tedious and really requires top soldering skills, as one needs to connect to four tiny solder pads. You really should consider using the R9MX receiver instead.

In principle, the procedure goes eaxtly as already described in the ExpressLRS docs:
- R9M module: https://www.expresslrs.org/1.0/quick-start/tx-r9m/#flashing-using-stlink
- R9MX receiver: https://www.expresslrs.org/1.0/quick-start/rx-stlink/

In these docs it is suggested to download and use the "ST-LINK Utility" software. This software is pretty outdated (NRND = not recommended for new designs), and the new recommended tool is "STM32CubeProgrammer". However, there are catches, at least in my experience:
- ST-LINK Utility appears to not work with newer ST-Link programmers.
- STM32CubeProgrammer/STM32CubeIDE is quite nasty with which ST-Link programmer is used. It seems ST really wants their tools to only work well with "original" or legit ST-Link programmers. I especially had significant issues with getting the cheap and widely available 8$ STLinkV2 usb-stick-like clones to work with STM32CubeProgrammer/STM32CubeIDE, and they then never worked reliably or to my satisfaction.
- It seems it can happen that ST-LINK Utility and STM32CubeProgrammer/STM32CubeIDE do not like each other.
- The STM32CubeProgrammer runs default at high SWD frequecy like 4000kHz on cheapo STLinkV2 clones. Setting the frequency manually to 480kHz and short wires to the device make it work more reliable.

So, if you use or want to use one of these 8$ STLinkV2 usb-stick-like clones, you probably want to install ST-LINK Utility and see how that works for you. You will then (likely) not be able to flash from STM32CubeIDE, but have to go via the ST-LINK Utility.

If anyone has deeper/better insight into the STM32CubeProgrammer/STM32CubeIDE vs ST-LINK Utility and cheap STLinkV2 programmer clones issues, please help :)

If you wonder what STLink programmer I am using: Every NUCLEO board comes also with a STLink programmer... and these board are relatively cheap too, about 15Eur.

