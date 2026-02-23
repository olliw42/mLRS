//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
// OlliW @ www.olliw.eu
//*******************************************************
// INFO
//********************************************************
#ifndef INFO_H
#define INFO_H
#pragma once


//-------------------------------------------------------
// Info Class
//-------------------------------------------------------

class tTxInfo
{
  public:
    void Init(void)
    {
        wireless.device_id = 0; // unknown
        wireless.device_name[0] = '\0'; // unknown
    }

    bool WirelessDeviceName_cli(char* const s)
    {
        if (wireless.device_name[0]) {
            strncpy(s, wireless.device_name, sizeof(wireless.device_name)-1);
            return true;
        }
        return false;
    }

    bool WirelessDeviceName_disp(char* const s)
    {
        if (wireless.device_name[0]) {
            // esp bridge: mLRS-xxxx AP UDP
            // HC04: Matek-mLRS-xxxxx-BT
            if (!strncmp(wireless.device_name, "Matek-", 6)) {
                strcpy(s, wireless.device_name + 6); // strip off "Matek-"
                s[10] = '\0'; // strip off "-BT"
            } else {
                strcpy(s, wireless.device_name);
                s[9] = '\0';  // strip off " AP UDP" or whatever comes, note: strncpy() is not reliable
dbg.puts(s);
            }
            return true;
        }
        return false;
    }

    struct {
        uint16_t device_id; // device_id as used in SSID
        char device_name[48]; // SSID
    } wireless;
};


#endif // INFO_H
