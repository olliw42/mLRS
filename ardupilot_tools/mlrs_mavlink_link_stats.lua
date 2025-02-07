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
-- Version 2025-02-07.01
----------------------------------------------------------------------
-- To install the script:
-- 	 - set SCR_ENABLE = 1
--   - put the script in APM/SCRIPTS/ on the microSD of the flight controller
--   - restart the flightcontroller
-- Configuration of mLRS receiver:
--   - 'Rx Ser Link Mode' = 'mavlink' or 'mavlinkX' ('mavlinkX' should be prefered)
--   - 'Rx Snd RcChannel' = 'rc channels' (NOT 'rc override'!)
-- works with mLRS v1.3.04 and later


-- TUNNEL message metadata

local TUNNEL_MSG_ID = 385

local RADIO_LINK_SYSTEM_ID = 51
local MAV_COMP_ID_TELEMETRY_RADIO = 68

local MLRS_TUNNEL_PAYLOAD_TYPE_RADIO_LINK_STATS = 208
local MLRS_TUNNEL_PAYLOAD_TYPE_RADIO_LINK_INFORMATION = 209


-- Initialize MAVLink via Lua script
-- mavlink:init(msg_queue_length, num_rx_msgid)
-- 1st number determines size of buffer, 2nd number determines number of IDs which can be registered
-- Note: it's important to use a msg_queue_length >= 2

mavlink.init(2, 10)
mavlink.register_rx_msgid(TUNNEL_MSG_ID)


----------------------------------------------------------------------
-- Decoder and handler functions
----------------------------------------------------------------------

function decode_header(msg)
    -- build up a map of the header
    local header = {}

    local pos = 3 -- TODO: why that?

    -- Magic packet start marker, can be used to identify the MAVLink version
    header.stx, pos = string.unpack("<B", msg, pos)
    if (header.stx == 0xFE) then -- mavlink 1
        header.protocol_version = 1
    elseif (header.stx == 0XFD) then --mavlink 2
        header.protocol_version = 2
    else
        -- arg, something is bad
        -- error("Invalid magic byte")
        header.protocol_version = -1
    end

    if header.protocol_version == 2 then -- we only do mavlink 2, and we only need mavlink 2 since TUNNEL ID > 256
        -- strip the payload length
        header.len, pos = string.unpack("<B", msg, pos)

        -- strip the incompat/compat flags
        header.incompat_flags, header.compat_flags, pos = string.unpack("<BB", msg, pos)

        -- fetch seq/sysid/compid
        header.seq, header.sysid, header.compid, pos = string.unpack("<BBB", msg, pos)

        -- fetch the message id
        header.msgid, pos = string.unpack("<I3", msg, pos)
	else
		header.msgid = -1 -- we use this to detect a false decode
    end

    return header, pos
end


-- Note on logging:
-- care must be taken when selecting a name, must be less than 5 characters and not clash with an existing log type
-- format characters specify the type of variable to be logged, see AP_Logger/README.md
-- https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Logger
-- not all format types are supported by scripting, only: i, L, e, f, n, M, B, I, E, and N
-- Lua automatically adds a timestamp in micro seconds

function handle_radio_link_stats(msg, pos)
    -- process the mavlink payload
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

    -- MP: MAV_RX_RSSI1
    --gcs:send_named_float("RX_RSSI1", -rx_rssi1)

    -- logging
    logger:write('MLR1',
        'rx_lq_rc,rx_lq_ser,tx_lq_ser,flags',
        'BBBi',
        rx_LQ_rc, rx_LQ_ser, tx_LQ_ser, flags)
    logger:write('MLR2',
        'rx_rssi1,rx_snr1,tx_rssi1,tx_snr1,f1',
        'iiiif',
        -rx_rssi1, rx_snr1, -tx_rssi1, tx_snr1, frequency1)
    logger:write('MLR3',
        'rx_rssi2,rx_snr2,tx_rssi2,tx_snr2,f2',
        'iiiif',
        -rx_rssi2, rx_snr2, -tx_rssi2, tx_snr2, frequency2)
end


function handle_radio_link_information(msg, pos)
    -- process the mavlink payload
    local tx_frame_rate, pos = string.unpack("<I2", msg, pos) -- uint16_t, frame rate in Hz
    local rx_frame_rate, pos = string.unpack("<I2", msg, pos) -- uint16_t, frame rate in Hz, always equal to tx frame rate
    local tx_ser_data_rate, pos = string.unpack("<I2", msg, pos) -- uint16_t, maximum possible upstream data rate in Bytes/sec
    local rx_ser_data_rate, pos = string.unpack("<I2", msg, pos) -- uint16_t, maximum possible downstream data rate in Bytes/sec
    local target_system, pos = string.unpack("<B", msg, pos)
    local target_component, pos = string.unpack("<B", msg, pos)
    local type_, pos = string.unpack("<B", msg, pos) -- uint8_t, always 6 for MLRS_RADIO_LINK_TYPE_MLRS
    local mode, pos = string.unpack("<B", msg, pos) -- uint8_t, mLRS mode
    local tx_power, pos = string.unpack("<B", msg, pos) -- int8_t, always 127 for unknown
    local rx_power, pos = string.unpack("<B", msg, pos) -- int8_t, receiver power in dBm
    local mode_str, pos = string.unpack("<c6", msg, pos) -- char[6], mLRS mode as string
    local band_str, pos = string.unpack("<c6", msg, pos) -- char[6], RF band
    local tx_receive_sensitivity, pos = string.unpack("<B", msg, pos) -- uint8_t, negative sensitivity in dBm
    local rx_receive_sensitivity, pos = string.unpack("<B", msg, pos) -- uint8_t, always equal to tx receive sensitivity

    -- 50 0 50 0 128 12 4 16 0 0 6 0 127 13 53 48 72 122 0 0 50 46 52 71 0 0 151 151

    logger:write('MLR4',
        'fr_rate,mode,tx_pwr,rx_pwr',
        'IBBB',
        tx_frame_rate, mode, tx_power, rx_power)
    logger:write('MLR5',
        'tx_ser_rate,rx_ser_rate,tx_sen,rx_sen',
        'IIii',
        tx_ser_data_rate, rx_ser_data_rate, -tx_receive_sensitivity, -rx_receive_sensitivity)
end


function handle_tunnel(msg, pos)
    -- process the mavlink payload
    local payload_type, pos = string.unpack("<I2", msg, pos)
    local target_system, pos = string.unpack("<B", msg, pos)
    local target_component, pos = string.unpack("<B", msg, pos)
    local payload_length, pos = string.unpack("<B", msg, pos)
	
    --gcs:send_text(6, string.format("handle_tunnel payload_type %d", payload_type))
	
    if payload_type == MLRS_TUNNEL_PAYLOAD_TYPE_RADIO_LINK_STATS then
        handle_radio_link_stats(msg, pos)
    elseif payload_type == MLRS_TUNNEL_PAYLOAD_TYPE_RADIO_LINK_INFORMATION then
        handle_radio_link_information(msg, pos)
    end
end


----------------------------------------------------------------------
-- Main loop
----------------------------------------------------------------------

function update()
    -- runtime: mean 204 us, max 2318 us
    
	local msg = mavlink.receive_chan()
    
    if msg ~= nil then
        local header, pos = decode_header(msg)
        if header.msgid == TUNNEL_MSG_ID then
            --gcs:send_text(6, string.format("Received TUNNEL from %d %d", header.sysid, header.compid))
            if header.sysid == RADIO_LINK_SYSTEM_ID and header.compid == MAV_COMP_ID_TELEMETRY_RADIO then
                handle_tunnel(msg, pos)
            end
        end
		
        return update() -- give it an immediate second try
    end

    return update(), 10 -- run at 100 Hz
end


return update()
