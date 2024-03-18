//*******************************************************
// Copyright (c) OlliW, OlliW42, www.olliw.eu
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// my stripped down UART standard library
// only TX, no RX, no halfduplex, no wait and tmo rx functions, no convenience functions, quite a number more strips
//*******************************************************

void uartc_init(void)
{
  Serial1.begin(115200);
}

uint16_t uartc_putc(char c)
{
  Serial1.write(c);
  return 1;
}
