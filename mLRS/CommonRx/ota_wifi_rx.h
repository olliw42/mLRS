//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// OTA Wi-Fi Handler for ESP8266/ESP8285/ESP32 - WebUI
//*******************************************************
#ifndef OTA_WIFI_RX_H
#define OTA_WIFI_RX_H
#pragma once


#if defined ESP8266 || defined ESP32

#ifdef ESP8266
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <Updater.h>
#else
#include <WiFi.h>
#include <WebServer.h>
#include <Update.h>
#endif


#define OTA_WIFI_SSID              "mLRS_RX"
#define OTA_WIFI_PASSWORD          "mlrs1234"
#define OTA_WIFI_TIMEOUT_MS        300000  // 5 min wifi mode timeout
#define OTA_WIFI_AUTO_TIMEOUT_MS   60000   // 60s auto-enter timeout

// minimal HTML page for firmware upload
const char OTA_HTML_PAGE[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html><head><title>mLRS</title>
<meta name="viewport" content="width=device-width,initial-scale=1">
<style>body{font-family:sans-serif;text-align:center;padding:20px;background:#051923;color:#eee}
h1{color:#00d4ff;margin-bottom:20px;font-size:1.6em}
.dev-info{color:#eee;font-size:1.4em;font-weight:bold;margin:10px 0}
.box{background:#003554;padding:20px;border-radius:10px;max-width:400px;margin:auto}
input[type=file]{margin:30px auto;display:block}input[type=submit]{background:#00d4ff;color:#000;padding:10px 30px;border:none;border-radius:5px;cursor:pointer;font-weight:bold;font-size:1.1em}
#prog{width:100%;height:20px;margin-top:15px;display:none}</style></head>
<body><div class="box"><h1>mLRS Rx Firmware Update</h1>
<div class="dev-info">Device Name: %DEVICE%</div><div class="dev-info">Firmware Version: %VERSION%</div>
<form method="POST" action="/update" enctype="multipart/form-data" id="f">
<input type="file" name="fw" accept=".bin" required><br><input type="submit" value="Upload">
<progress id="prog" value="0" max="100"></progress></form><p id="s"></p></div>
<script>
var f=document.getElementById('f');
f.onsubmit=function(e){
e.preventDefault();var p=document.getElementById('prog'),s=document.getElementById('s');
p.style.display='block';var x=new XMLHttpRequest();x.open('POST','/update',true);
x.upload.onprogress=function(e){if(e.lengthComputable)p.value=(e.loaded/e.total)*100;};
x.onload=function(){if(x.status==200){s.innerHTML='<h1>Success!</h1>'+x.responseText+'<p>Rebooting... AP will disconnect.</p>';f.style.display='none';}else{s.innerHTML='<h1>Error!</h1>'+x.responseText;}};
x.send(new FormData(f));};
</script></body></html>
)rawliteral";


// note: static here is intentional, this header is included only once
#ifdef ESP8266
static ESP8266WebServer _ota_server(80);
#elif defined ESP32
static WebServer _ota_server(80);
#endif


class tRxOtaWifi
{
  public:
    void Init(void);
    void Do(void);
    bool IsActive(void) { return _wifi_active; }

    // call from main loop - returns true if should enter wifi mode
    bool CheckAutoTimeout(bool connected_once, bool in_bind);

    // call to enter wifi mode (call after setting sx to idle)
    void Enter(void);

  private:
    bool _wifi_active;
    bool _ota_in_progress;
    uint32_t _wifi_enter_time;
    uint32_t _auto_timeout_start;
    uint32_t _led_next_ms;
    String _update_error;
    uint8_t _led_state;

    void _handleRoot(void);
    void _handleUpdate(void);
    void _handleUpdateUpload(void);
    void _doLed(void);
};


void tRxOtaWifi::Init(void)
{
    _wifi_active = false;
    _ota_in_progress = false;
    _led_state = 0;
    _led_next_ms = 0;
    _auto_timeout_start = millis();
}


bool tRxOtaWifi::CheckAutoTimeout(bool connected_once, bool in_bind)
{
    // only auto-enter if never connected and not in bind mode
    if (!connected_once && !in_bind) {
        if ((millis() - _auto_timeout_start) > OTA_WIFI_AUTO_TIMEOUT_MS) {
            return true;
        }
    }
    return false;
}


void tRxOtaWifi::_handleRoot(void)
{
    _ota_server.setContentLength(CONTENT_LENGTH_UNKNOWN);
    _ota_server.send(200, "text/html", "");

    String s;
#ifdef ESP8266
    s = FPSTR(OTA_HTML_PAGE);
#else
    s = String(OTA_HTML_PAGE);
#endif

    s.replace("%DEVICE%", DEVICE_NAME);
    s.replace("%VERSION%", VERSIONONLYSTR);

    _ota_server.sendContent(s);
    _ota_server.sendContent(""); // end of stream
}


void tRxOtaWifi::_handleUpdate(void)
{
    _ota_server.sendHeader("Connection", "close");
    if (Update.hasError()) {
        String msg = "Update Failed: " + _update_error;
        _ota_server.send(500, "text/plain", msg);
        _ota_in_progress = false;
    } else {
        _ota_server.send(200, "text/plain", "Update OK! Rebooting...");
        delay(500);
        ESP.restart();
    }
}


void tRxOtaWifi::_handleUpdateUpload(void)
{
    HTTPUpload& upload = _ota_server.upload();

    if (upload.status == UPLOAD_FILE_START) {
        _ota_in_progress = true;
        _update_error = "";
        uint32_t maxSketchSpace = (ESP.getFreeSketchSpace() - 0x1000) & 0xFFFFF000;
        if (!Update.begin(maxSketchSpace)) {
            _update_error = "not enough space";
            _ota_in_progress = false;
        }
    } else if (upload.status == UPLOAD_FILE_WRITE) {
        if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
            _update_error = "write failed";
            _ota_in_progress = false;
        }
    } else if (upload.status == UPLOAD_FILE_END) {
        if (!Update.end(true)) {
            _update_error = "finalize failed";
            _ota_in_progress = false;
        }
    }
    yield();
}


void tRxOtaWifi::Enter(void)
{
    WiFi.mode(WIFI_AP);
    WiFi.softAP(OTA_WIFI_SSID, OTA_WIFI_PASSWORD);

    _ota_server.on("/", [this]() { _handleRoot(); });
    _ota_server.on("/update", HTTP_POST,
        [this]() { _handleUpdate(); },
        [this]() { _handleUpdateUpload(); }
    );
    _ota_server.begin();

    _wifi_active = true;
    _wifi_enter_time = millis();
    _led_state = 0;
    _led_next_ms = millis();
}


void tRxOtaWifi::_doLed(void)
{
    if (millis() < _led_next_ms) return;

    if (_ota_in_progress) {
        led_red_on();
        _led_next_ms = millis() + 100;
    } else {
        // double-blink pattern
        switch (_led_state) {
        case 0: led_red_on();  _led_next_ms = millis() + 100; break;
        case 1: led_red_off(); _led_next_ms = millis() + 100; break;
        case 2: led_red_on();  _led_next_ms = millis() + 100; break;
        case 3: led_red_off(); _led_next_ms = millis() + 600; break;
        }
        _led_state = (_led_state + 1) % 4;
    }
}


void tRxOtaWifi::Do(void)
{
    if (!_wifi_active) return;

    _ota_server.handleClient();
    _doLed();

    // timeout - exit wifi mode if no upload started
    if (!_ota_in_progress && (millis() - _wifi_enter_time) > OTA_WIFI_TIMEOUT_MS) {
        WiFi.mode(WIFI_OFF);
        _wifi_active = false;
        ESP.restart();
    }
}


#endif // ESP8266 || ESP32

#endif // OTA_WIFI_RX_H
