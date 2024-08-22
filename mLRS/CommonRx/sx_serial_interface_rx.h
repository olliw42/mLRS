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

    bool available(void) override
    {
        if (SERIAL_LINK_MODE_IS_MAVLINK(Setup.Rx.SerialLinkMode)) {
            return mavlink.available(); // get from serial via MAVLink parser
        }
        if (SERIAL_LINK_MODE_IS_MSP(Setup.Rx.SerialLinkMode)) {
            return msp.available(); // get from serial via MSP parser
        }
        return serial.available(); // get from serial
    }

    char getc(void) override
    {
        if (SERIAL_LINK_MODE_IS_MAVLINK(Setup.Rx.SerialLinkMode)) {
            return mavlink.getc(); // get from serial via MAVLink parser
        }
        if (SERIAL_LINK_MODE_IS_MSP(Setup.Rx.SerialLinkMode)) {
            return msp.getc(); // get from serial via MSP parser
        }
        return serial.getc(); // get from serial
    }

    void putbuf(uint8_t* buf, uint16_t len) override
    {
        if (SERIAL_LINK_MODE_IS_MAVLINK(Setup.Rx.SerialLinkMode)) {
            for (uint16_t i = 0; i < len; i++) mavlink.putc(buf[i]); // send to serial via MAVLink parser
            return;
        }
        if (SERIAL_LINK_MODE_IS_MSP(Setup.Rx.SerialLinkMode)) {
            for (uint16_t i = 0; i < len; i++) msp.putc(buf[i]); // send to serial via MSP parser
            return;
        }
        serial.putbuf(buf, len); // send to serial
    }

    void flush(void) override
    {
        mavlink.flush(); // we don't distinguish here, can't harm to always flush MAVLink handler
        msp.flush(); // we don't distinguish here, can't harm to always flush MSP handler
        serial.flush();
    }
};


#endif // SX_SERIAL_INTERFACE_RX_H
