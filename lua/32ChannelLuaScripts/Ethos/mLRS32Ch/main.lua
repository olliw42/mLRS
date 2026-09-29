----------------------------------------------------------------------
-- Copyright (c) mLRS Project
-- GPL3
-- https://www.gnu.org/licenses/gpl-3.0.de.html
----------------------------------------------------------------------
-- mLRS 32 Channels Lua Task Script for Ethos
----------------------------------------------------------------------
-- copy main.lua to SD:\scripts\mlrs32\ on the Ethos radio SD card, restart radio
-- then enable the task per model: Model setup -> Lua -> Lua tasks -> "mLRS 32Ch"
-- runs as background task, sends channels 17-32 as CRSF 0x17 frame


local VERSION = {
    script = '2026-09-27.00', -- add a '.01' if needed for the day
}

local FIRST_CH = 16 -- 0-based, set to 0 to mirror channels 1 - 16 for testing

local SEND_PERIOD_S = 0.1 -- 10 Hz


----------------------------------------------------------------------
-- CRSF Helper
----------------------------------------------------------------------

local sensor = nil


local function getSensor()
    if sensor == nil and crsf ~= nil and crsf.getSensor ~= nil then
        sensor = crsf.getSensor()
    end
    return sensor
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

local tlast_send = 0


local function wakeup()
    local tnow = os.clock()
    if tnow - tlast_send >= SEND_PERIOD_S then
        tlast_send = tnow
        sendChannels0x17()
    end
end


local function init()
    system.registerTask({ name = "mLRS 32Ch", key = "mlrs32", wakeup = wakeup })
end


return { init = init }
