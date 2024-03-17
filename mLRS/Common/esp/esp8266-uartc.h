void uartc_init(void)
{
  Serial.begin(115200);
}

uint16_t uartc_putc(char c)
{
  Serial.write(c);
  return 1;
}
