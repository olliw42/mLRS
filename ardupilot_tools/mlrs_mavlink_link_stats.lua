----------------------------------------------------------------------
-- Copyright (c) MLRS project
-- GPL3
-- https://www.gnu.org/licenses/gpl-3.0.de.html
----------------------------------------------------------------------
-- Lua script for ArduPilot
-- Logging of link statistics data send via MAVLink
----------------------------------------------------------------------
-- contributed by twistedwings
----------------------------------------------------------------------
-- Version 2026-09-13.00
----------------------------------------------------------------------
-- To install the script:
--   - set SCR_ENABLE = 1
--   - put the script in APM/SCRIPTS/ on the microSD of the flight controller
--   - restart the flight controller
-- Required mLRS receiver settings:
--   - 'Rx Ser Link Mode' = 'mavlink' or 'mavlinkX' ('mavlinkX' should be prefered)
--   - 'Rx Snd RcChannel' = 'rc channels' (NOT 'rc override'!)
-- Works with mLRS v1.3.04 and later
--
-- Script creates two ArduPilot parameters:
--   - MLRS_SCR_ENABLE: allows to enable/disable logging, 0: stops logging, >= 1: is logging (default is 1)
--   - MLRS_SCR_DBG:    allows to set the debug level, 0: disabled. 1: level 1, 2: level2, 3: all (default is 0)


----------------------------------------------------------------------
--
----------------------------------------------------------------------

local debugEnabled = 0 -- 0: disabled. 1: level 1, 2: level2, 3: all

local PARAM_KEY = 53 -- hopefully no other script is using it; script asserts if unavailable


-- MAVLink messages metadata

local RADIO_LINK_SYSTEM_ID = 51
local MAV_COMP_ID_TELEMETRY_RADIO = 68

local TUNNEL_MSG_ID = 385
local MLRS_TUNNEL_PAYLOAD_TYPE_RADIO_LINK_STATS = 208
local MLRS_TUNNEL_PAYLOAD_TYPE_RADIO_LINK_INFORMATION = 209

local MLRS_RADIO_LINK_STATS_MSG_ID = 60045
local MLRS_RADIO_LINK_INFORMATION_MSG_ID = 60046


-- Initialize MAVLink via Lua script
-- mavlink:init(msg_queue_length, num_rx_msgid)
-- 1st number determines size of buffer, 2nd number determines number of IDs which can be registered
-- Note: it is important to use a msg_queue_length >= 4

mavlink.init(4, 10)
mavlink.register_rx_msgid(TUNNEL_MSG_ID)
mavlink.register_rx_msgid(MLRS_RADIO_LINK_STATS_MSG_ID)
mavlink.register_rx_msgid(MLRS_RADIO_LINK_INFORMATION_MSG_ID)


----------------------------------------------------------------------
-- ArduPilot version & parameters
----------------------------------------------------------------------

local ap_fwversion = FWVersion:major() * 10000 + FWVersion:minor() * 100 + FWVersion:patch()


assert(param:add_table(PARAM_KEY, "MLRS_", 16), "mLRS SCRIPT: could not create parameter table")
assert(param:add_param(PARAM_KEY, 1, "SCR_ENABLE", 1)) -- enabled per default
assert(param:add_param(PARAM_KEY, 2, "SCR_DBG", 0)) -- no debugging per default


local param_enable = Parameter("MLRS_SCR_ENABLE")
local param_debug = Parameter("MLRS_SCR_DBG")


----------------------------------------------------------------------
-- Debug handling
----------------------------------------------------------------------

local debugLevel = debugEnabled

local function do_param_debug()
    if debugEnabled == 0 then
        debugLevel = param_debug:get()
    end
end

do_param_debug()


----------------------------------------------------------------------
-- Message decoder and handler functions
----------------------------------------------------------------------

local function decode_header(msg)
    -- build up a map of the header
    local header = {}

    -- mavlink.receive_chan() returns as first result a mavlink_message_t structure
    -- which holds the message crc in the first two bytes, the magic STX and rest of the message are following
    -- the STX is thus found in the third byte
    local pos = 3

    -- magic packet start marker, can be used to identify the MAVLink version
    header.stx, pos = string.unpack("<B", msg, pos)

    if header.stx ~= 0xFD then -- we only do MAVLink 2, and we only need MAVLink 2
        header.msgid = nil -- we use this to detect a false decode
    end

    -- strip the payload length
    header.len, pos = string.unpack("<B", msg, pos)
    -- strip the incompat/compat flags
    header.incompat_flags, header.compat_flags, pos = string.unpack("<BB", msg, pos)
    -- fetch seq/sysid/compid
    header.seq, header.sysid, header.compid, pos = string.unpack("<BBB", msg, pos)
    -- fetch the message id
    header.msgid, pos = string.unpack("<I3", msg, pos)

    return header, pos
end


-- Note on logging:
-- care must be taken when selecting a name, must be less than 5 characters and not clash with an existing log type
-- format characters specify the type of variable to be logged, see AP_Logger/README.md
-- https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Logger
-- not all format types are supported by scripting, only: i, L, e, f, n, M, B, I, E, and N
-- Lua automatically adds a timestamp in micro seconds

-- For comparison, log structure of (older) BetaPilot
-- { LOG_RADIO_LINK_STATS_MSG_RX, sizeof(log_RadioLinkStatsRx), \
--   "RDRX", "QBBBbBbBB", "TimeUS,rxLQrc,rxLQser,rxRssi1,rxSnr1,rxRssi2,rxSnr2,rxRAn,rxTAn", "s%%------", "F--------", true }, \
-- { LOG_RADIO_LINK_STATS_MSG_TX, sizeof(log_RadioLinkStatsTx), \
--   "RDTX", "QBBbBbBBB", "TimeUS,txLQser,txRssi1,txSnr1,txRssi2,txSnr2,txRAn,txTAn,flags", "s%-------", "F--------", true }, \

-- Note on pos:
-- A TUNNEL meassge contains the same data as a MLRS_RADIO_xxx message, but at a differnt pos in the payload.
-- Hence, passing through the pos allows the mlrs radio handlers to be used for both.

local function handle_mlrs_radio_link_stats(msg, pos)
    -- process the message payload
    local flags, pos = string.unpack("<I2", msg, pos)

    local target_system, pos = string.unpack("<B", msg, pos)
    local target_component, pos = string.unpack("<B", msg, pos)

    local rx_LQ_rc, pos = string.unpack("<B", msg, pos) -- uint8_t, 1 .. 100, 0 = disconnected
    local rx_LQ_ser, pos = string.unpack("<B", msg, pos) -- uint8_t, 1 .. 100, 0 = disconnected
    local rx_rssi1, pos = string.unpack("<B", msg, pos) -- uint8_t, 1 .. 253 = -rssi, 254 = disconnected, 255 if antenna 1 not used
    local rx_snr1, pos = string.unpack("<b", msg, pos) -- int8_t, 127 if antenna1 not used

    local tx_LQ_ser, pos = string.unpack("<B", msg, pos) -- uint8_t, 1 .. 100, 0 = disconnected
    local tx_rssi1, pos = string.unpack("<B", msg, pos) -- uint8_t, 1 .. 253 = -rssi, 254 = disconnected, 255 = unknown
    local tx_snr1, pos = string.unpack("<b", msg, pos) -- int8_t, always 127 for unknown
    local rx_rssi2, pos = string.unpack("<B", msg, pos) -- uint8_t, 255 if antenna 2 not used
    local rx_snr2, pos = string.unpack("<b", msg, pos) -- int8_t, 127 if antenna 2 not used
    local tx_rssi2, pos = string.unpack("<B", msg, pos) -- uint8_t, always 255 for unknown
    local tx_snr2, pos = string.unpack("<b", msg, pos) -- int8_t, always 127 for unknown

    local frequency1, pos = string.unpack("<f", msg, pos) -- float, frequency on RF band 1 in Hz
    local frequency2, pos = string.unpack("<f", msg, pos) -- float, frequency on RF band 2 in Hz, 0 for single band receivers

    if debugLevel >= 2 then
        gcs:send_text(6, "mLRS RADIO LINK STATS")
    end

    -- logging
	local unit23 = ap_fwversion >= 40702 and 'RRRRz' or '----z'
	
    logger:write('MLR1',
        'rx_lq_rc,rx_lq_ser,tx_lq_ser,flags',
        'BBBI',
        '%%%-',
        '0000',
        rx_LQ_rc, rx_LQ_ser, tx_LQ_ser, flags)

    logger:write('MLR2',
        'rx_rssi1,rx_snr1,tx_rssi1,tx_snr1,f1',
        'iiiif',
        unit23,
        '00000',
        -rx_rssi1, rx_snr1, -tx_rssi1, tx_snr1, frequency1)
    logger:write('MLR3',
        'rx_rssi2,rx_snr2,tx_rssi2,tx_snr2,f2',
        'iiiif',
        unit23,
        '00000',
        -rx_rssi2, rx_snr2, -tx_rssi2, tx_snr2, frequency2)
end


local function handle_mlrs_radio_link_information(msg, pos)
    -- process the message payload
    local tx_frame_rate, pos = string.unpack("<I2", msg, pos) -- uint16_t, frame rate in Hz
    local rx_frame_rate, pos = string.unpack("<I2", msg, pos) -- uint16_t, frame rate in Hz, always equal to tx frame rate
    local tx_ser_data_rate, pos = string.unpack("<I2", msg, pos) -- uint16_t, maximum possible upstream data rate in Bytes/sec
    local rx_ser_data_rate, pos = string.unpack("<I2", msg, pos) -- uint16_t, maximum possible downstream data rate in Bytes/sec
    local target_system, pos = string.unpack("<B", msg, pos)
    local target_component, pos = string.unpack("<B", msg, pos)
    local type_, pos = string.unpack("<B", msg, pos) -- uint8_t, always 6 for MLRS_RADIO_LINK_TYPE_MLRS
    local mode, pos = string.unpack("<B", msg, pos) -- uint8_t, mLRS mode
    local tx_power, pos = string.unpack("<b", msg, pos) -- int8_t, always 127 for unknown
    local rx_power, pos = string.unpack("<b", msg, pos) -- int8_t, receiver power in dBm
    local mode_str, pos = string.unpack("<c6", msg, pos) -- char[6], mLRS mode as string
    local band_str, pos = string.unpack("<c6", msg, pos) -- char[6], RF band
    local tx_receive_sensitivity, pos = string.unpack("<B", msg, pos) -- uint8_t, negative sensitivity in dBm
    local rx_receive_sensitivity, pos = string.unpack("<B", msg, pos) -- uint8_t, always equal to tx receive sensitivity

    if debugLevel >= 2 then
        gcs:send_text(6, "mLRS RADIO LINK INFO")
    end

    -- 50 0 50 0 128 12 4 16 0 0 6 0 127 13 53 48 72 122 0 0 50 46 52 71 0 0 151 151

	local unit4 = ap_fwversion >= 40702 and "z-RR" or 'z---'
	local unit5 = ap_fwversion >= 40702 and "BBRR" or 'BB--'

    logger:write('MLR4',
        'fr_rate,mode,tx_pwr,rx_pwr',
        'IBii',
        unit4,
        '0000',
        tx_frame_rate, mode, tx_power, rx_power)
    logger:write('MLR5',
        'tx_ser_rate,rx_ser_rate,tx_sen,rx_sen',
        'IIii',
        unit5,
        '0000',
        tx_ser_data_rate, rx_ser_data_rate, -tx_receive_sensitivity, -rx_receive_sensitivity)
end


local function handle_tunnel(msg, pos)
    -- process the message payload
    local payload_type, pos = string.unpack("<I2", msg, pos)
    local target_system, pos = string.unpack("<B", msg, pos)
    local target_component, pos = string.unpack("<B", msg, pos)
    local payload_length, pos = string.unpack("<B", msg, pos)

    if debugLevel >= 2 then
        gcs:send_text(6, string.format("mLRS TUNNEL payload_type %d", payload_type))
    end

    if payload_type == MLRS_TUNNEL_PAYLOAD_TYPE_RADIO_LINK_STATS then
        handle_mlrs_radio_link_stats(msg, pos)
    elseif payload_type == MLRS_TUNNEL_PAYLOAD_TYPE_RADIO_LINK_INFORMATION then
        handle_mlrs_radio_link_information(msg, pos)
    end
end


----------------------------------------------------------------------
-- Main loop
----------------------------------------------------------------------

local tlast_5sec = 0
local msg_count = 0
local msg_stats_count = 0
local msg_info_count = 0
local msg_tunnel_count = 0


local function handle_msg(header, msg, pos)
    if debugLevel >= 3 then
        gcs:send_text(6, string.format("Received msg %d from %d %d", header.msgid, header.sysid, header.compid))
    end

    msg_count = msg_count + 1

    if header.msgid == MLRS_RADIO_LINK_STATS_MSG_ID then
        msg_stats_count = msg_stats_count + 1
        handle_mlrs_radio_link_stats(msg, pos)
    elseif header.msgid == MLRS_RADIO_LINK_INFORMATION_MSG_ID then
        msg_info_count = msg_info_count + 1
        handle_mlrs_radio_link_information(msg, pos)
    elseif header.msgid == TUNNEL_MSG_ID then
        msg_tunnel_count = msg_tunnel_count + 1
        handle_tunnel(msg, pos)
    end
end


local function update()
    if param_enable:get() == 0 then
        return update, 5000 -- try again in 5 seconds
    end

    -- runtime: mean 204 us, max 2318 us ??

    while true do
        local msg = mavlink.receive_chan() -- only returns for registered messages, so no need to worry much about flooding
        if msg == nil then
            break
        end
        local header, pos = decode_header(msg)
        if header ~= nil and header.msgid ~= nil then
            if header.sysid == RADIO_LINK_SYSTEM_ID and header.compid == MAV_COMP_ID_TELEMETRY_RADIO then -- is from our mLRS receiver
                handle_msg(header, msg, pos)
            end
        end
    end

    local tnow = millis()
    if tnow - tlast_5sec >= 5000 then
        tlast_5sec = tnow
        do_param_debug()
        if debugLevel > 0 then
            gcs:send_text(6, string.format("mLRS msg cnt: %d (%d %d %d)", msg_count, msg_stats_count, msg_info_count, msg_tunnel_count))
        end
    end

    return update, 10 -- run at 100 Hz, it is important to write update and not update()
end


return update()
