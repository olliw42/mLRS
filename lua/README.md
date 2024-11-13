# mLRS - lua #

This folder contains lua scripts which may be used to change the Tx module and connected receiver settings when the Tx module is installed in a OpenTx or EdgeTx radio.

[mLRS-bw.lua](mLRS-bw.lua) is for radios with black and white screens

[mLRS.lua](mLRS.lua) is for radios with color screens

Please see the [documentation page](https://github.com/olliw42/mLRS-docu/blob/master/docs/LUA.md#mlrs-documentation-mlrs-lua-script) for install and usage details

Note:  If you receive a "not enough memory" error, you can try reducing the file size by running the code through a lua minifier tool such as the one [here](https://mothereff.in/lua-minifier) and install the minified version as a workaround.  Please also report this to the mLRS developers.
