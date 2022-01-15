//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
// OlliW @ www.olliw.eu
//*******************************************************
// TODO
//*******************************************************
/*


- get a clean HAL concept

- get a clear mbridge and in concept, currently quite ugly as uart.h conflicts

- handle allowed and not possible parameter combinations properly

- do channel order handling on transmitter side, not receiver side !?
  or do channel order on transmitter such that it is AETR on the air, and on receiver to what is desired
  1 has an advantage if all params are known on transmitter side
  2 is more canonical has also advantage with failsafe, as T is known !!!

- when we eventually do OTA update of the rx, the rx firmware must be universal in the sense that it supports all possible
  rx boards. Gladly, rx modules are relatively simple. But: Craft a concept which hopefully will last for a long time.
  It won't work for different hardware platforms!?
  Ultimately I think it should be all G4 (and maybe ESP32)
*/




