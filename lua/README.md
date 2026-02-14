# mLRS - Lua #

This folder contains Lua scripts which may be used to change the Tx module and connected receiver settings when the Tx module is installed in an EdgeTx, OpenTx or Ethos (FrSky) radio.

## EdgeTx, OpenTx Radios

[mLRS.lua](mLRS.lua) is for radios with color screens

[mLRS-bw.lua](mLRS-bw.lua) is for radios with black and white screens

[mLRS-bw-luac.lua](mLRS-bw-luac.lua) is for some radios with black and white screens which report "not enough memory" with the mLRS-bw.lua file.

Please see the [documentation page](https://github.com/olliw42/mLRS-docu/blob/master/docs/LUA.md#mlrs-documentation-mlrs-lua-script) for install and usage details

> [!NOTE] 
> If on a bw EdgeTx/OpenTx radio you receive a "not enough memory" error, please try the mLRS-bw-luac.lua file. You can also try reducing the file size by running the code through a Lua minifier tool such as this one [here](https://mothereff.in/lua-minifier), and install the minified version as a workaround. If neither solution works, please report this to the mLRS developers.

## Ethos (FrSky) Radios

The Lua script files for Ethos radios are found in the folder named "Ethos". In that folder you also find information on how to install and use it.




