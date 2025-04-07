//*******************************************************
// mLRS Wireless Bridge for ESP32
// Copyright (c) www.olliw.eu, OlliW, OlliW42
// License: GPL v3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// AT Mode
//*******************************************************


// forward declarations
void serialFlushRx(void);


typedef enum {
    AT_NAME_QUERY = 0,
    AT_RESTART,
    AT_BAUD_QUERY,
    AT_BAUD_9600,
    AT_BAUD_19200,
    AT_BAUD_38400,
    AT_BAUD_57600,
    AT_BAUD_115200,
    AT_BAUD_230400,
    AT_WIFICHANNEL_QUERY,
    AT_WIFICHANNEL_1,
    AT_WIFICHANNEL_6,
    AT_WIFICHANNEL_11,
    AT_WIFICHANNEL_13,
    AT_WIFIPOWER_QUERY,
    AT_WIFIPOWER_0_LOW,
    AT_WIFIPOWER_1_MED,
    AT_WIFIPOWER_2_MAX,
    AT_PROTOCOL_QUERY,
    AT_PROTOCOL_0_TCP,
    AT_PROTOCOL_1_UDP,
    AT_PROTOCOL_3_BT,
    AT_CMDS_NUM,
} AT_NAME_ENUM;

const char* at_cmds[AT_CMDS_NUM] = {
     "AT+NAME=?",
     "AT+RESTART",
     "AT+BAUD=?",
     "AT+BAUD=9600",
     "AT+BAUD=19200",
     "AT+BAUD=38400",
     "AT+BAUD=57600",
     "AT+BAUD=115200",
     "AT+BAUD=230400",
     "AT+WIFICHANNEL=?",
     "AT+WIFICHANNEL=01", // required to avoid it been taken for 11 or 13
     "AT+WIFICHANNEL=06",
     "AT+WIFICHANNEL=11",
     "AT+WIFICHANNEL=13",
     "AT+WIFIPOWER=?",
     "AT+WIFIPOWER=0",
     "AT+WIFIPOWER=1",
     "AT+WIFIPOWER=2",
     "AT+PROTOCOL=?",
     "AT+PROTOCOL=0",
     "AT+PROTOCOL=1",
     "AT+PROTOCOL=3",
};


class AtMode {
  public:
    void Init(uint8_t _gpio_pin);
    bool Do(void);

  private:
    uint8_t gpio0_pin;
    bool gpio_is_low;
    unsigned long startup_tmo_ms;
    uint8_t at_pos;
    uint8_t at_buf[128];
    
    bool restart_needed;

    void restart(void);
};


void AtMode::Init(uint8_t _gpio_pin) 
{
    gpio0_pin = _gpio_pin;
    pinMode(gpio0_pin, INPUT);

    at_pos = 0;
    gpio_is_low = false;
    restart_needed = false;

#ifndef ESP8266
    // https://github.com/espressif/esp-idf/issues/12473
    esp_log_level_set("wifi", ESP_LOG_NONE);
    // ?? esp_wifi_nan_stop();
#endif

    startup_tmo_ms = 750 + millis(); // stay in AT loop for at least 750 ms, 0 = startup timeout ended
}


bool AtMode::Do(void) 
{ 
    // handle toggles in GPIO0
    if (!gpio_is_low) {
        if (digitalRead(GPIO0_IO) == LOW) { // toggled LOW
            gpio_is_low = true;
            at_pos = 0;
            restart_needed = false;
            serialFlushRx();
        }
    } else {
        if (digitalRead(GPIO0_IO) == HIGH) { // toggled HIGH
            gpio_is_low = false;
            startup_tmo_ms = 0; // also declare end of startup timeout phase
            serialFlushRx();
        }
    }

    // handle startup timeout
    if (startup_tmo_ms) {
        if (millis() > startup_tmo_ms) startup_tmo_ms = 0;
    } 

    // when not in startup phase and not GPIO0 low, do to WiFi loop 
    if (!startup_tmo_ms && !gpio_is_low) return false;

    int avail = SERIAL.available();
    if (avail > 0) {
        char c;
        SERIAL.read(&c, 1);

        if (at_pos > 96) at_pos = 0; // argh ...

        at_buf[at_pos++] = c;
        at_buf[at_pos] = '\0'; // to make it a str
        bool possible_match = false;
        for (int i = 0; i < AT_CMDS_NUM; i++) {
            if (strncmp((char*)at_buf, at_cmds[i], at_pos) != 0) continue; // not even a possible match
            possible_match = true;
            if (strcmp((char*)at_buf, at_cmds[i]) == 0) { // full match
                if (i == AT_NAME_QUERY || i == AT_BAUD_QUERY || i == AT_WIFICHANNEL_QUERY || 
                    i == AT_WIFIPOWER_QUERY || i == AT_PROTOCOL_QUERY) {
                    at_buf[0] = 'O';
                    at_buf[1] = 'K';
                    SERIAL.write(at_buf, at_pos - 1); // don't send the '?'
                    switch (i) {
                        case AT_NAME_QUERY:
                            SERIAL.write("mLRS-Wireless-Bridge-");
                            SERIAL.write(VERSION_STR);
                            break;
                        case AT_BAUD_QUERY: SERIAL.print(g_baudrate); break;
                        case AT_WIFICHANNEL_QUERY: SERIAL.print(g_wifichannel); break;
                        case AT_WIFIPOWER_QUERY: SERIAL.print(g_wifipower); break;
                        case AT_PROTOCOL_QUERY: SERIAL.print(g_protocol); break;
                    }
                    SERIAL.write("\r\n");
                } else
                if (i == AT_RESTART) {
                    // for some reasons I don't understand it "often" gives a wifi:NAN WiFi stop error
                    //if (1) {
                    if (restart_needed) {
                        at_buf[0] = 'O';
                        at_buf[1] = 'K';
                        SERIAL.write(at_buf, at_pos);
                        SERIAL.write("\r\n");
                        delay(100);
                        restart();
                    } else {
                        SERIAL.write("KO\r\n"); // cmd recognized, but not executed
                    }
                } else
                if (i >= AT_BAUD_9600 && i <= AT_BAUD_230400) {
                    at_buf[0] = 'O';
                    at_buf[1] = 'K';
                    SERIAL.write(at_buf, at_pos);
                    SERIAL.write("\r\n");
                    int new_baudrate = atoi((char*)(at_buf + 8)); // AT+BAUD=9600
                    if (new_baudrate != g_baudrate) {
                        preferences.putInt(G_BAUDRATE_STR, new_baudrate);
                        restart_needed = true;
                    }
                } else
                if (i >= AT_WIFICHANNEL_1 && i <= AT_WIFICHANNEL_13) {
                    at_buf[0] = 'O';
                    at_buf[1] = 'K';
                    SERIAL.write(at_buf, at_pos);
                    SERIAL.write("\r\n");
                    int new_wifichannel = atoi((char*)(at_buf + 15)); // AT+WIFICHANNEL=1
                    if (new_wifichannel != g_wifichannel) {
                        preferences.putInt(G_WIFICHANNEL_STR, new_wifichannel);
                        restart_needed = true;
                    }
                } else
                if (i >= AT_WIFIPOWER_0_LOW && i <= AT_WIFIPOWER_2_MAX) {
                    at_buf[0] = 'O';
                    at_buf[1] = 'K';
                    SERIAL.write(at_buf, at_pos);
                    SERIAL.write("\r\n");
                    int new_wifipower = atoi((char*)(at_buf + 13)); // AT+WIFIPOWER=1
                    if (new_wifipower != g_wifipower) {
                        preferences.putInt(G_WIFIPOWER_STR, new_wifipower);
                        restart_needed = true;
                    }
                } else
                if (i == AT_PROTOCOL_0_TCP || i == AT_PROTOCOL_1_UDP || i == AT_PROTOCOL_3_BT) {
                    at_buf[0] = 'O';
                    at_buf[1] = 'K';
                    SERIAL.write(at_buf, at_pos);
                    SERIAL.write("\r\n");
                    int new_protocol = atoi((char*)(at_buf + 12)); // AT+PROTOCOL=1
                    if (new_protocol != g_protocol) {
                        preferences.putInt(G_PROTOCOL_STR, new_protocol);
                        restart_needed = true;
                    }
                }
                at_pos = 0;
                break;
            }
        }
        if (!possible_match) { // no possible match found, so reset
            at_pos = 0;
        }

    } // avail

    return true;
}


void AtMode::restart(void)
{
    preferences.end();    
    ESP.restart();
}



