//*******************************************************
// mLRS ESP-NOW
// Copyright (c) www.olliw.eu, OlliW, OlliW42
// License: GPL v3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// Sniffer Mode — promiscuous capture + MAC auto-detection
//*******************************************************
// 7. Mar. 2026
//*******************************************************
#ifndef ESPNOW_SNIFFER_H
#define ESPNOW_SNIFFER_H


//-------------------------------------------------------
// ESP-NOW frame layout constants (from promiscuous capture)
//-------------------------------------------------------
//
// offset  field
// ------  -----
// 0-1     frame control (type=0, subtype=0x0D for action)
// 2-3     duration
// 4-9     address 1 (destination MAC)
// 10-15   address 2 (source MAC)
// 16-21   address 3 (BSSID)
// 22-23   sequence control
// 24      category code (0x7F = vendor-specific)
// 25-27   OUI (0x18, 0xFE, 0x34 = Espressif)
// 28-31   random values (4 bytes)
// 32      element ID (0xDD = 221)
// 33      length
// 34-36   OUI again (0x18, 0xFE, 0x34)
// 37      type (4 = ESP-NOW)
// 38      version
// 39+     ESP-NOW payload (the serial data bytes)
//

#define ESPNOW_FRAME_MIN_LEN       40  // minimum valid frame length
#define ESPNOW_MAX_PAYLOAD_LEN     250 // espnow v1 max payload
#define ESPNOW_MAC_HEADER_LEN      24
#define ESPNOW_ADDR1_OFFSET        4   // destination MAC
#define ESPNOW_ADDR2_OFFSET        10  // source MAC
#define ESPNOW_CATEGORY_OFFSET     24
#define ESPNOW_OUI_OFFSET          25
#define ESPNOW_ELEMID_OFFSET       32
#define ESPNOW_ELEMLEN_OFFSET      33
#define ESPNOW_TYPE_OFFSET         37
#define ESPNOW_PAYLOAD_OFFSET      39

const uint8_t IRAM_ATTR espressif_oui[3] = { 0x18, 0xFE, 0x34 };


//-------------------------------------------------------
// IRAM-safe helpers
//-------------------------------------------------------

// iram-safe MAC compare — avoids memcmp which may not be in IRAM
bool IRAM_ATTR mac_equal(const uint8_t* a, const uint8_t* b)
{
    return a[0] == b[0] && a[1] == b[1] && a[2] == b[2] &&
           a[3] == b[3] && a[4] == b[4] && a[5] == b[5];
}

// iram-safe MAC copy — avoids memcpy which may not be in IRAM
void IRAM_ATTR mac_copy(uint8_t* dst, const uint8_t* src)
{
    dst[0] = src[0]; dst[1] = src[1]; dst[2] = src[2];
    dst[3] = src[3]; dst[4] = src[4]; dst[5] = src[5];
}

// iram-safe 3-byte OUI compare
bool IRAM_ATTR oui_equal(const uint8_t* a, const uint8_t* b)
{
    return a[0] == b[0] && a[1] == b[1] && a[2] == b[2];
}


//-------------------------------------------------------
// MAC auto-detection state machine
//-------------------------------------------------------

typedef enum {
    DETECT_COLLECTING = 0,
    DETECT_DECIDED,
} detect_phase_t;

volatile detect_phase_t detect_phase;

// track up to 2 unique source MACs
uint8_t detected_macs[2][6];
volatile uint32_t detected_counts[2];
volatile int detected_count;
volatile unsigned long detect_second_mac_ms;

uint8_t bridge_mac[6];
volatile bool bridge_mac_known;

// signal that we've received at least one espnow frame (for channel scan)
volatile bool espnow_frame_received;

void detect_init(void)
{
#ifdef BRIDGE_MAC
    // hardcoded MAC, skip straight to decided
    uint8_t mac[] = BRIDGE_MAC;
    memcpy(bridge_mac, mac, 6);
    bridge_mac_known = true;
    detect_phase = DETECT_DECIDED;
#else
    detect_phase = DETECT_COLLECTING;
    detected_count = 0;
    detected_counts[0] = 0;
    detected_counts[1] = 0;
    detect_second_mac_ms = 0;
    bridge_mac_known = false;
#endif
    espnow_frame_received = false;
}


//-------------------------------------------------------
// promiscuous callback
//-------------------------------------------------------

void IRAM_ATTR promisc_recv_cb(void* buf, wifi_promiscuous_pkt_type_t type)
{
    // only care about management frames
    if (type != WIFI_PKT_MGMT) return;

    const wifi_promiscuous_pkt_t* pkt = (const wifi_promiscuous_pkt_t*)buf;
    const uint8_t* frame = pkt->payload;
    int frame_len = pkt->rx_ctrl.sig_len;

    // sanity: minimum length for an espnow action frame
    if (frame_len < ESPNOW_FRAME_MIN_LEN) return;

    // check frame control: type=0 (management), subtype=0x0D (action)
    // frame control byte 0: protocol(2) | type(2) | subtype(4)
    // management = type 0, action = subtype 13 => byte0 = 0xD0
    if (frame[0] != 0xD0) return;

    // check category = vendor-specific (0x7F)
    if (frame[ESPNOW_CATEGORY_OFFSET] != 0x7F) return;

    // check espressif OUI
    if (!oui_equal(&frame[ESPNOW_OUI_OFFSET], espressif_oui)) return;

    // check element ID = 0xDD (vendor-specific) and type = 4 (ESP-NOW)
    if (frame[ESPNOW_ELEMID_OFFSET] != 0xDD) return;
    if (frame[ESPNOW_TYPE_OFFSET] != 0x04) return;

    // extract payload length from the element length field
    // element length covers: OUI(3) + type(1) + version(1) + payload
    int elem_len = frame[ESPNOW_ELEMLEN_OFFSET];
    int payload_len = elem_len - 5; // subtract OUI + type + version
    if (payload_len <= 0) return;
    if (payload_len > ESPNOW_MAX_PAYLOAD_LEN) return; // sanity cap
    if (ESPNOW_PAYLOAD_OFFSET + payload_len > frame_len) return;

    const uint8_t* src_mac = &frame[ESPNOW_ADDR2_OFFSET];
    const uint8_t* payload = &frame[ESPNOW_PAYLOAD_OFFSET];

    // signal that we've heard an espnow frame (for channel scan)
    espnow_frame_received = true;

    // mac auto-detection / filtering
    if (detect_phase == DETECT_DECIDED) {
        // only forward frames from the bridge MAC
        if (!mac_equal(src_mac, bridge_mac)) return;
        rxbuf_push(payload, payload_len);
    } else {
        // collecting phase: track MACs and count packets
        int mac_idx = -1;
        for (int i = 0; i < detected_count; i++) {
            if (mac_equal(src_mac, detected_macs[i])) {
                mac_idx = i;
                break;
            }
        }
        if (mac_idx < 0 && detected_count < 2) {
            // new MAC discovered
            mac_idx = detected_count;
            mac_copy(detected_macs[mac_idx], src_mac);
            detected_counts[mac_idx] = 0;
            detected_count++;
            if (detected_count == 2) {
                detect_second_mac_ms = millis();
            }
        }
        if (mac_idx >= 0) {
            detected_counts[mac_idx]++;
        }

        // buffer all data during collection so nothing is lost
        rxbuf_push(payload, payload_len);
    }
}


//-------------------------------------------------------
// scan hooks (called by scan_for_traffic in .ino)
//-------------------------------------------------------

void scan_reset(void)
{
    espnow_frame_received = false;
}

bool scan_check(void)
{
    return espnow_frame_received;
}


//-------------------------------------------------------
// check if auto-detection is ready to decide
//-------------------------------------------------------

void detect_check(void)
{
    if (detect_phase != DETECT_COLLECTING) return;
    if (detected_count == 0) return;

    uint32_t total = detected_counts[0] + (detected_count > 1 ? detected_counts[1] : 0);
    unsigned long elapsed = (detected_count > 1) ? (millis() - detect_second_mac_ms) : 0;

    // decide once we have 50 total packets or 2 seconds since seeing second MAC
    if (total >= 50 || elapsed >= 2000) {
        // higher count = bridge (telemetry downlink is continuous)
        int bridge_idx = 0;
        if (detected_count > 1 && detected_counts[1] > detected_counts[0]) {
            bridge_idx = 1;
        }
        memcpy(bridge_mac, detected_macs[bridge_idx], 6);
        bridge_mac_known = true;
        // set phase AFTER MAC is fully written to avoid ISR reading partial MAC
        detect_phase = DETECT_DECIDED;
    }
}


//-------------------------------------------------------
// wifi setup (sniffer — promiscuous mode specific)
//-------------------------------------------------------

void setup_wifi_mode(void)
{
    // enable promiscuous mode, filter for management frames only
    wifi_promiscuous_filter_t filt = { .filter_mask = WIFI_PROMIS_FILTER_MASK_MGMT };
    esp_wifi_set_promiscuous_filter(&filt);
    esp_wifi_set_promiscuous_rx_cb(promisc_recv_cb);
    esp_wifi_set_promiscuous(true);

    // scan channels until we find espnow traffic
    scan_for_traffic();
}


#endif // ESPNOW_SNIFFER_H
