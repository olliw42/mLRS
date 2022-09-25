# mLRS Documentation: Flashing Frsky R9M, R9MX #

([back to main page](../README.md))

The Frsky R9M transmitter module and R9MX receiver are commercially available and hence interesting hardware for mLRS. However, flashing them is quite a hack currently.

ExpressLRS has figured out a convenient and easy way, which unfortunately is not available for mLRS (someone needs to figure it out). For flashing mLRS only the grass-route DIY procedure using a ST-Link programmer is currently possible.

Note: Fashing mLRS with the ST-Link is a non-reversible operation, i.e., it is not possible to revert back to the original Frsky firmware.

Comment: mLRS also supports the R9MM receiver. However, flashing it with ST-Link is extremely tedious and really requires top soldering skills, as one needs to connect to four tiny solder pads. You really should consider using the R9MX receiver instead.

In principle, the procedure goes eaxtly as already described in the ExpressLRS docs:
- R9M module: https://www.expresslrs.org/1.0/quick-start/tx-r9m/#flashing-using-stlink
- R9MX receiver: https://www.expresslrs.org/1.0/quick-start/rx-stlink/

In these docs it is suggested to download and use the "ST-LINK Utility" software. This software is pretty outdated (NRND = not recommended for new designs), and the new recommended tool is "STM32CubeProgrammer". However, there are significant catches, at least in my experience:
- It seems that one cannot use both ST-LINK Utility and STM32CubeProgrammer/STM32CubeIDE. At least for me installing STM32CubeProgrammer and/or STM32Cube IDE made ST-LINK Utility not work anymore.
- STM32CubeProgrammer/STM32CubeIDE is quite nasty with which ST-Link programmer is used. It seems ST really wants their tools to only work well with "original" ST-Link programmers. I especially had significant issues with getting the cheap and widely available 8$ ST-Link usb-stick-like clones to work with STM32CubeProgrammer/STM32CubeIDE, and they then never worked reliably.

So, unfortunately, it seems that one needs to decide, depending on which ST-Link programmer one is using. My suggestion would be to install ST-LINK Utility and then to see how that works for you.

If anyone has deeper/better insight into the STM32CubeProgrammer/STM32CubeIDE vs ST-LINK Utility and cheap ST-Link programmer clones issues, please help out :)

