//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
// OlliW @ www.olliw.eu
//*******************************************************
// SX Serial Interface
//********************************************************
#ifndef SX_SERIAL_INTERFACE_RX_H
#define SX_SERIAL_INTERFACE_TX_H
#pragma once



class tRxSxSerial : public tSerialBase
{
  public:
      void Init(void)
      {
          tSerialBase::Init();
      }

      virtual bool available(void)
      {
          if (SERIAL_LINK_MODE_IS_MAVLINK(Setup.Rx.SerialLinkMode)) {
              return mavlink.available(); // get from serial via mavlink parser
          }
          return serial.available(); // get from serial
      }

      virtual char getc(void)
      {
          if (SERIAL_LINK_MODE_IS_MAVLINK(Setup.Rx.SerialLinkMode)) {
              return mavlink.getc(); // get from serial via mavlink parser
          }
          return serial.getc(); // get from serial
      }

      virtual void flush(void)
      {
          mavlink.flush(); // we don't distinguish here, can't harm to always flush mavlink handler
          serial.flush();
      }

      virtual void putc(char c)
      {
          if (SERIAL_LINK_MODE_IS_MAVLINK(Setup.Rx.SerialLinkMode)) {
              mavlink.putc(c); // send to serial via mavlink parser
          } else {
              serial.putc(c); // send to serial
          }
      }
};


#endif // SX_SERIAL_INTERFACE_RX_H
