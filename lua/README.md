# mLRS - Lua Scripts #

This folder contains various Lua scripts for EdgeTx, OpenTx or FrSky/ETHOS radios.

## mLRS Configuration Script

Lua scripts which may be used to change the Tx module and connected receiver settings, when the Tx module is installed in an EdgeTx, OpenTx or FrSky/ETHOS radio.

### EdgeTx, OpenTx Radios

[mLRS.lua](mLRS.lua) is for radios with color screens

[mLRS-bw.lua](mLRS-bw.lua) is for radios with black and white screens

[mLRS-bw-luac.lua](mLRS-bw-luac.lua) is for some radios with black and white screens which report "not enough memory" with the mLRS-bw.lua file.

Please see the [documentation page](https://github.com/olliw42/mLRS-docu/blob/master/docs/LUA.md#mlrs-documentation-mlrs-lua-script) for install and usage details

> [!NOTE] 
> If on a black&white EdgeTx/OpenTx radio you receive a "not enough memory" error, please try the mLRS-bw-luac.lua file. You can also try reducing the file size by running the code through a Lua minifier tool such as this one [here](https://mothereff.in/lua-minifier), and install the minified version as a workaround. If neither solution works, please report this to the mLRS developers.

### FrSky/ETHOS Radios

The Lua script files for ETHOS radios are found in the folder named "Ethos". In that folder you also find information on how to install and use it.

## mLRS Statistics Widget

The file main.lua in the folder \StatisticsLuaScripts\EdgeTx\mLRSStatsW is the statistics widget for EdgeTx color radios. Copy it to the \WIDGETS\mLRSStatsW folder on the SD card.

The file main.lua in the folder \StatisticsLuaScripts\Ethos\mLRSStatsW is the same widget for ETHOS (FrSky) radios. Copy main.lua to SD:\scripts\mLRSStatsW\, restart the radio and add the "mLRS Statistics" widget to a screen. A full screen layout gives the detailed view, a small widget shows a compact view. Tested on a FrSky X20S (800x480) with Ethos 1.6.6; the layout scales to other screen sizes.

These widgets require mLRS Tx module firmware v1.4.04 or later.




