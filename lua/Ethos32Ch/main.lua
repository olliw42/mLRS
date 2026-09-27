----------------------------------------------------------------------
-- Copyright (c) OlliW @ www.olliw.eu
-- GPL3
-- https://www.gnu.org/licenses/gpl-3.0.de.html
----------------------------------------------------------------------
-- mLRS 32 Channels Lua Task Script for Ethos
----------------------------------------------------------------------
-- copy main.lua to SD:\scripts\mlrs32\ on the Ethos radio SD card, restart radio
-- runs as background task, sends channels 17-32 as CRSF 0x17 frame
-- only active when the Tx module identifies as mLRS via CRSF device ping


local VERSION = {
    script = '2026-09-27.00', -- add a '.01' if needed for the day
}

local FIRST_CH = 16 -- 0-based, set to 0 to mirror channels 1 - 16 for testing

local SEND_PERIOD_S = 0.1 -- 10 Hz
local PING_PERIOD_S = 1.0 -- while not detected
local REPING_PERIOD_S = 5.0 -- while detected
local DETECT_TIMEOUT_S = 12.0 -- drop detection if no device info reply


----------------------------------------------------------------------
-- CRSF Helper
----------------------------------------------------------------------

local CRSF_FRAME_ID_PING_DEVICES = 0x28
local CRSF_FRAME_ID_DEVICE_INFO = 0x29
local CRSF_ADDRESS_TRANSMITTER_MODULE = 0xEE
local CRSF_ADDRESS_RADIO = 0xEA

local sensor = nil


local function getSensor()
    if sensor == nil and crsf ~= nil and crsf.getSensor ~= nil then
        sensor = crsf.getSensor()
    end
    return sensor
end


local function sendPing()
    local s = getSensor()
    if s == nil then return false end
    -- address Tx module, not broadcast, so ELRS doesn't forward the ping over the air
    return s:pushFrame(CRSF_FRAME_ID_PING_DEVICES, { CRSF_ADDRESS_TRANSMITTER_MODULE, CRSF_ADDRESS_RADIO })
end


-- device info: [dest, origin,] name\0, serial(4), hw id(4), fw id(4), ...
-- mLRS sets serial to the chars 'mLRS', so look for it right after the name's terminator
local function isMlrsDeviceInfo(data)
    if data == nil then return false end
    for i = 1, #data - 4 do
        if data[i] == 0 and data[i+1] == 0x6D and data[i+2] == 0x4C and
           data[i+3] == 0x52 and data[i+4] == 0x53 then
            return true
        end
    end
    return false
end


----------------------------------------------------------------------
-- 0x17 Helper
----------------------------------------------------------------------
-- command = 0x17
-- data[1] = CRSF 0x17 configuration byte
-- data[2..23] = packed channel data

-- range is -1024 .. 0 .. +1024 for -100% .. +100%
-- ArduPilot 0x17, 11 bit: pwm = x / 2 + 988

local channelSources = {}


local function getChannelValue(ch)
    local src = channelSources[ch]
    if src == nil then
        src = system.getSource({ category = CATEGORY_CHANNEL, member = ch })
        if src == nil then return nil end
        channelSources[ch] = src
    end
    return src:value()
end


local function outputToCrsf(value)
    if value == nil then return 1024 end
    local v = math.floor(value + 1024.5) -- Ethos may return floats, bit ops need integers
    if v < 0 then return 0 end
    if v > 2047 then return 2047 end
    return v
end


local function sendChannels0x17()
    local s = getSensor()
    if s == nil then return false end

    local data = {}
    local pos = 1

    data[pos] = 0x20 + 16 -- 11 bit, 16 channel start = 0x20 + 0x10 = 0x30
    pos = pos + 1

    local bitBuffer = 0
    local bitCount = 0
    for ch = FIRST_CH, FIRST_CH + 15 do
        local value = outputToCrsf(getChannelValue(ch))
        bitBuffer = bitBuffer | (value << bitCount)
        bitCount = bitCount + 11
        while bitCount >= 8 do
            data[pos] = bitBuffer & 0xFF
            pos = pos + 1
            bitBuffer = bitBuffer >> 8
            bitCount = bitCount - 8
        end
    end

    return s:pushFrame(0x17, data)
end


----------------------------------------------------------------------
-- Script Ethos Task Interface
----------------------------------------------------------------------

local detected = false
local other_module = false
local tlast_detect = 0
local tlast_ping = 0
local tlast_send = 0


local function handleFrames()
    local s = getSensor()
    if s == nil then return end
    for _ = 1, 8 do
        local cmd, data = s:popFrame(CRSF_FRAME_ID_DEVICE_INFO)
        if cmd == nil then break end
        if cmd == CRSF_FRAME_ID_DEVICE_INFO then
            if isMlrsDeviceInfo(data) then
                if not detected then print("mLRS32: mLRS Tx detected") end
                detected = true
                other_module = false
                tlast_detect = os.clock()
            else
                if not other_module then print("mLRS32: non-mLRS Tx, stopping") end
                other_module = true
            end
        end
    end
end


local function wakeup()
    if other_module then return end -- a non-mLRS module answered, stay silent until restart

    local tnow = os.clock()

    handleFrames()
    if other_module then return end

    if detected and tnow - tlast_detect > DETECT_TIMEOUT_S then
        print("mLRS32: mLRS Tx lost")
        detected = false
    end

    local ping_period = detected and REPING_PERIOD_S or PING_PERIOD_S
    if tnow - tlast_ping >= ping_period then
        tlast_ping = tnow
        sendPing()
    end

    if detected and tnow - tlast_send >= SEND_PERIOD_S then
        tlast_send = tnow
        sendChannels0x17()
    end
end


local function init()
    system.registerTask({ name = "mLRS 32Ch", key = "mlrs32", wakeup = wakeup })
end


return { init = init }
