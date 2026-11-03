----------------------------------------------------------------------
-- Copyright (c) mLRS Project
-- GPL3
-- https://www.gnu.org/licenses/gpl-3.0.de.html
----------------------------------------------------------------------
-- mLRS 32 Channels Lua Mixes Script
----------------------------------------------------------------------
-- copy script to the SCRIPTS\MIXES folder on the EdgeTx SD card
-- file name must not exceed 6 chars (excluding .lua)


local VERSION = {
    script = '2026-10-06.00', -- add a '.01' if needed for the day
}


-- entry format: { name, VALUE, min, max, default }; name is only the label shown in the radio UI
-- values are passed to run() as arguments by position, not by name
local input = {
    { "Enable", VALUE, 0, 1, 1 },
}

local output = {}


----------------------------------------------------------------------
-- 0x17 Helper
----------------------------------------------------------------------
-- command = 0x17
-- data[1] = CRSF 0x17 configuration byte
-- data[2..23] = packed channel data

-- range is -1024 .. 0 .. +1024 for -100% .. +100%
-- ArduPilot 0x17, 11 bit: pwm = x / 2 + 988
-- => x =    0 for pwm = 988
--    x = 1024 for pwm = 1500
--    x = 2047 for pwm = 2011
-- this is rather a range of +- 101%, we ignore this

local function outputToCrsf(value)
    if value == nil then return 1024 end
    local v = value + 1024
    if v < 0 then return 0 end
    if v > 2047 then return 2047 end
    return v
end


local function sendChannels0x17()
    local data = {}
    local pos = 1

    data[pos] = 0x20 + 16 -- 11 bit, 16 channel start = 0x20 + 0x10 = 0x30
    pos = pos + 1

    local bitBuffer = 0
    local bitCount = 0
    for ch = 16, 31 do
        local value = outputToCrsf(getOutputValue(ch))
        bitBuffer = bitBuffer | (value << bitCount)
        bitCount = bitCount + 11
        while bitCount >= 8 do
            data[pos] = bitBuffer & 0xFF
            pos = pos + 1
            bitBuffer = bitBuffer >> 8
            bitCount = bitCount - 8
        end
    end
    -- with 16 x 11 bits this isn't actually needed, 176 bits divides exactly into 22 bytes
    if bitCount > 0 then
        data[pos] = bitBuffer & 0xFF
    end

    return crossfireTelemetryPush(0x17, data)
end


----------------------------------------------------------------------
-- Script EdgeTx Interface
----------------------------------------------------------------------

local tlast_10ms = 0


local function init()
    tlast_10ms = 0
end


local function run(enabled)
    if enabled == 0 then return end

    local tnow_10ms = getTime()

    if tnow_10ms - tlast_10ms >= 20 then -- 5 Hz
        tlast_10ms = tnow_10ms
        sendChannels0x17()
    end
end


return {
    input = input,
    output = output,
    init = init,
    run = run,
}
