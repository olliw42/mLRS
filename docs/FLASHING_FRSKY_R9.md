# mLRS Documentation: Flashing Frsky R9M, R9MX #

([back to main page](../README.md))

The Frsky R9M transmitter module and R9MX receiver are commercially available and hence interesting hardware for mLRS. However, flashing them is quite a hack currently.

ExpressLRS has figured out a convenient and easy way, which unfortunately is not available for mLRS (someone needs to figure it out). For flashing mLRS only the grass-route DIY procedure using a ST-Link programmer is currently possible.

Note: Fashing mLRS with the ST-Link is a non-reversible operation, i.e., it is not possible to revert back to the original Frsky firmware.

Comment: Also the R9MM receiver is supported. However, flashing it with the ST-Link is extremely tedious and really requires top soldering skils, as one needs to connect to four tiny solder pads. You really should consider using the R9MX receiver instead.

In principle, the procedure goes eaxtly as already described in the ExpressLRS docs:
- R9M module: https://www.expresslrs.org/1.0/quick-start/tx-r9m/#flashing-using-stlink
- R9MX receiver: https://www.expresslrs.org/1.0/quick-start/rx-stlink/

