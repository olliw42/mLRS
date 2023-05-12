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


tSerialBase* serialport = nullptr; // currently still needed by MavlinkBase class


class tTxSxSerial : public tSerialBase
{
  public:
    void Init(tSerialBase* _serial, tSerialBase* _mbridge, tSerialBase* _serial2)
    {
        tSerialBase::Init();

        switch (Setup.Tx.SerialDestination) {
        case SERIAL_DESTINATION_SERIAL:
            serialport = _serial;
            break;
        case SERIAL_DESTINATION_MBRDIGE:
            serialport = _mbridge;
            break;
        case SERIAL_DESTINATION_SERIAL2:
            serialport = _serial2;
            break;
        default:
            while (1) {} // must not happen
        }
    }

    bool IsEnabled(void)
    {
        return true;
    }

    virtual bool available(void)
    {
        if (!connected_and_rx_setup_available()) return 0;
        if (SERIAL_LINK_MODE_IS_MAVLINK(Setup.Rx.SerialLinkMode)) {
            return mavlink.available(); // get from serial via mavlink parser
        }
        return serialport->available(); // get from serial
    }

    virtual char getc(void)
    {
        if (!connected_and_rx_setup_available()) return 0;
        if (SERIAL_LINK_MODE_IS_MAVLINK(Setup.Rx.SerialLinkMode)) {
            return mavlink.getc(); // get from serial via mavlink parser
        }
        return serialport->getc(); // get from serial
    }

    virtual void flush(void)
    {
        mavlink.flush(); // we don't distinguish here, can't harm to always flush mavlink handler
        serialport->flush();
    }

    virtual void putc(char c)
    {
        if (!connected_and_rx_setup_available()) return;
        if (SERIAL_LINK_MODE_IS_MAVLINK(Setup.Rx.SerialLinkMode)) { // this has to go via the parser
            mavlink.putc(c);
            return;
        }
        serialport->putc(c);
    }
};


#endif // SX_SERIAL_INTERFACE_TX_H
