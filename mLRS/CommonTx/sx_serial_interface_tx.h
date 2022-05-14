//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
// OlliW @ www.olliw.eu
//*******************************************************
// SX Serial Interface
//********************************************************
#ifndef SX_SERIAL_INTERFACE_TX_H
#define SX_SERIAL_INTERFACE_TX_H
#pragma once


tSerialBase* serialport; // currently still needed by MavlinkBase class


class tTxSxSerial : public tSerialBase
{
  public:
      void Init(tSerialBase* _serial, tSerialBase* _mbridge, tSerialBase* _serial2)
      {
          tSerialBase::Init();

          switch (Setup.Tx.SerialDestination) {
          case SERIAL_DESTINATION_SERIAL_PORT:
              serialport = _serial; //&serial;
              break;
          case SERIAL_DESTINATION_MBRDIGE:
              serialport = _mbridge; //&mbridge;
              break;
          default:
              serialport = nullptr;
          }
      }

      bool IsEnabled(void) { return (serialport != nullptr); }

      virtual bool available(void)
      {
          if (!serialport) return false;

          if (Setup.Tx.SerialLinkMode == SERIAL_LINK_MODE_MAVLINK) return mavlink.available(); // get from serial via mavlink parser
          return serialport->available(); // get from serial
      }

      virtual char getc(void)
      {
          if (!serialport) return '\0';

          if (Setup.Tx.SerialLinkMode == SERIAL_LINK_MODE_MAVLINK) return mavlink.getc(); // get from serial via mavlink parser
          return serialport->getc(); // get from serial
      }

      virtual void flush(void)
      {
          if (!serialport) return;

          mavlink.flush(); // we don't distinguish here, can't harm to always flush mavlink handler
          serialport->flush();
      }

      virtual void putc(char c)
      {
          if (!serialport) return;

          if (Setup.Tx.SerialLinkMode == SERIAL_LINK_MODE_MAVLINK) { // this has to go via the parser
              mavlink.putc(c);
          } else {
            serialport->putc(c);
          }
      }
};


#endif // SX_SERIAL_INTERFACE_TX_H
