//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
// OlliW @ www.olliw.eu
//*******************************************************
// TODO
//*******************************************************
/*

ISSUES:
- why is otx not getting the tx & rx link stats telemetry?
- what do the crsf values exactly mean? should rssi be in -db or +db??
- convert power
- redo crsf & mbridge such that they can use the very same interface

- time for enabling/disabling PA??

TODO:

- option to select serial or serial with mavlink parsing, allow e.g. radio_status only in latter mode

- rate management by radio_status txbuf
  note: rts/cts is not a substitute as this wouldn't help in router situations

- support high_latency2

- get a clean HAL concept

- get a clear mbridge and in concept, currently quite ugly as uart.h conflicts

- handle allowed and not possible parameter combinations properly

- when we eventually do OTA update of the rx, the rx firmware must be universal in the sense that it supports all possible
  rx boards. Gladly, rx modules are relatively simple. But: Craft a concept which hopefully will last for a long time.
  It won't work for different hardware platforms!?
  Ultimately I think it should be all G4 (and maybe ESP32)

- idea: switch to a longer-range mode when not connected, would also allow rc data to be transmitted further out
*/




