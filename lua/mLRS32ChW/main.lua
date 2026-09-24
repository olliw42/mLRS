local widgetName = "mLRS 32Ch Widget"
----------------------------------------------------------------------
-- Copyright (c) OlliW @ www.olliw.eu
-- GPL3
-- https://www.gnu.org/licenses/gpl-3.0.de.html
----------------------------------------------------------------------
-- mLRS 32 Channels Lua Widget
----------------------------------------------------------------------
-- copy script to SCRIPTS\WIDGETS\mLRS32Ch folder on EdgeTx SD card


local VERSION = {
    script = '2026-09-24.00', -- add a '.01' if needed for the day
}


local options = {
    { "Color", COLOR, lcd.RGB(255, 255, 255) },
    { "Enabled", BOOL, 1 },
}


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
--    for ch = 16, 31 do 
-- momentarily, for testing, we simply mirror channels 1 - 16
    for ch = 0, 15 do
        value = outputToCrsf(getOutputValue(ch))
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


local function sendChannels0x16() -- just for testing
    local data = {}
    local pos = 1
    
    local bitBuffer = 0
    local bitCount = 0
    for ch = 0, 15 do
        value = outputToCrsf(getOutputValue(ch))
        bitBuffer = bitBuffer | (value << bitCount)
        bitCount = bitCount + 11
        while bitCount >= 8 do
            data[pos] = bitBuffer & 0xFF
            pos = pos + 1
            bitBuffer = bitBuffer >> 8
            bitCount = bitCount - 8
        end
    end
    
    data[pos] = 0
    pos = pos + 1
    
    bitBuffer = 0
    bitCount = 0
--    for ch = 16, 31 do 
-- momentarily, for testing, we simply mirror channels 1 - 16
    for ch = 0, 15 do
        value = outputToCrsf(getOutputValue(ch))
        bitBuffer = bitBuffer | (value << bitCount)
        bitCount = bitCount + 11
        while bitCount >= 8 do
            data[pos] = bitBuffer & 0xFF
            pos = pos + 1
            bitBuffer = bitBuffer >> 8
            bitCount = bitCount - 8
        end
    end
    
    return crossfireTelemetryPush(0x16, data)
end


----------------------------------------------------------------------
-- Script EdgeTx Interface
----------------------------------------------------------------------

local function create(zone, options)
    local widget = { zone = zone, options = options }
    
    widget.tlast_10ms = 0
    
    return widget
end


local function update(widget, options)
    widget.options = options
end


local function background(widget)
    if widget.options.Enabled == 0 then return end
  
    local tnow_10ms = getTime()

    if tnow_10ms - widget.tlast_10ms >= 20 then
        widget.tlast_10ms = tnow_10ms
        sendChannels0x17()
        --sendChannels0x16() -- just for testing
    end
end


local function refresh(widget, event, touchState)
    lcd.drawText(0, 0, "32Ch", widget.options.Color)
    
    if widget.options.Enabled == 0 then 
        lcd.drawText(0, 20, "off", widget.options.Color)
        return 
    end
    lcd.drawNumber(0, 20, outputToCrsf(getOutputValue(0)), widget.options.Color) -- to just show something moving
  
    background(widget)
  
    local zone = widget.zone
    if zone.w ~= LCD_W or zone.h ~= LCD_H then return end -- skip out if not full size widget

    local columns = 4
    local rows = 8
    local colWidth = math.floor(zone.w / columns)
    local rowHeight = 30
    local flags = SMLSIZE

    for ch = 0, 15 do
        local col = math.floor(ch / rows)
        local row = ch % rows
        local x = zone.x + col * colWidth + 100
        local y = zone.y + row * rowHeight

        local value = outputToCrsf(getOutputValue(ch))

        lcd.drawText(x, y, string.format("%02d", ch+1), flags)
        lcd.drawText(x + 22, y, string.format("%5d", value or 0), flags)
    end
end


return {
    name = widgetName,
    options = options,
    create = create,
    update = update,
    background = background,
    refresh = refresh,
}
