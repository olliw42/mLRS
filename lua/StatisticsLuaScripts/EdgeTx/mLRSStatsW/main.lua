local widgetName = "mLRS Statistics Widget"
----------------------------------------------------------------------
-- Copyright (c) OlliW @ www.olliw.eu
-- GPL3
-- https://www.gnu.org/licenses/gpl-3.0.de.html
----------------------------------------------------------------------
-- copy script to the WIDGETS\mLRSStatsW folder on the EdgeTx SD card
-- version 2026-10-17.00


local options = {
    { "Color", COLOR, lcd.RGB(255, 255, 255) },
}


local ui = {}
ui.COLOR_WHITE = lcd.RGB(0xFF, 0xFF, 0xFF)
ui.COLOR_BLACK = lcd.RGB(0x00, 0x00, 0x00)
ui.COLOR_LIGHTGREY = lcd.RGB(0xB0, 0xB0, 0xB0)
ui.COLOR_GREEN = lcd.RGB(25, 150, 50)
ui.COLOR_RED = lcd.RGB(0xE5, 0x20, 0x1E)
ui.COLOR_YELLOW = lcd.RGB(0xFF, 0xD0, 0x00)
ui.COLOR_BACKGROUND = lcd.RGB(0x08, 0x54, 0x88)
ui.COLOR_SKY = lcd.RGB(135, 206, 235)
ui.COLOR_EARTH = lcd.RGB(107, 142, 35)


----------------------------------------------------------------------
-- CRSF Mb statistics decoder
----------------------------------------------------------------------
-- frame: 0xEA, len, 0x82, 0x65, 17 bytes, crc

local mbStats = nil
local mbRssiList = nil
local mbRssiListCountLast = 0


local function decodeCrsfMbStatistics(command, packet)
    if command ~= 0x82 or #packet < 18 or packet[1] ~= 0x65 then
        return nil
    end

    if mbStats == nil then mbStats = {} end

    local b = packet[2]
    mbStats.connected    = b & 0x01
    mbStats.binding      = (b >> 1) & 0x01
    mbStats.dualband     = (b >> 2) & 0x01
    mbStats.rx_available = (b >> 3) & 0x01
    mbStats.privacy      = (b >> 4) & 0x03

    b = packet[3]
    mbStats.rx_actual_diversity = b & 0x0F
    mbStats.tx_actual_diversity = (b >> 4) & 0x0F

    b = packet[4]
    mbStats.receive_antenna           = b & 0x01
    mbStats.transmit_antenna          = (b >> 1) & 0x01
    mbStats.receiver_receive_antenna  = (b >> 2) & 0x01
    mbStats.receiver_transmit_antenna = (b >> 3) & 0x01

    mbStats.rssi1 = packet[5]
    if mbStats.rssi1 >= 128 then mbStats.rssi1 = mbStats.rssi1 - 256 end
    mbStats.rssi2 = packet[6]
    if mbStats.rssi2 >= 128 then mbStats.rssi2 = mbStats.rssi2 - 256 end
    mbStats.receiver_rssi = packet[7]
    if mbStats.receiver_rssi >= 128 then mbStats.receiver_rssi = mbStats.receiver_rssi - 256 end

    mbStats.LQ_serial = packet[8]
    mbStats.receiver_LQ_rc = packet[9]
    mbStats.receiver_LQ_serial = packet[10]

    b = packet[14] * 256*256*256 + packet[13] * 256*256 + packet[12] * 256 + packet[11]
    mbStats.bytes_transmitted = b & 0x3FFF
    mbStats.bytes_received    = (b >> 14) & 0x3FFF

    b = packet[18] * 256*256*256 + packet[17] * 256*256 + packet[16] * 256 + packet[15]
    mbStats.fhss1_curr_i = b & 0x1F
    mbStats.fhss1_cnt    = (b >> 5) & 0x1F
    mbStats.fhss2_curr_i = (b >> 10) & 0x1F
    mbStats.fhss2_cnt    = (b >> 15) & 0x1F

    if mbRssiList == nil or mbStats.fhss1_cnt ~= mbRssiListCountLast then
        mbRssiList = {}
        for i = 0, mbStats.fhss1_cnt - 1 do mbRssiList[i] = { rssi1 = 0, rssi2 = 0 } end
        mbRssiListCountLast = mbStats.fhss1_cnt
    end
    mbRssiList[mbStats.fhss1_curr_i] = { rssi1 = mbStats.rssi1, rssi2 = mbStats.rssi2 }

    return true
end


----------------------------------------------------------------------
-- Drawing
----------------------------------------------------------------------
-- view is laid out for 480x272 screen and scaled to the widget

local sx, sy = 1, 1
local function X(x) return math.floor(x * sx + 0.5) end
local function Y(y) return math.floor(y * sy + 0.5) end

local function drawText(x, y, txt, color)
    lcd.drawText(X(x), Y(y), txt, color)
end

local function drawNumber(x, y, val, color)
    lcd.drawNumber(X(x), Y(y), val, color)
end

local function drawRectangle(x, y, w, h, flag)
    lcd.drawRectangle(X(x), Y(y), X(w), Y(h), flag)
end

local function drawLine(x1, y1, x2, y2, flag, color)
    lcd.drawLine(X(x1), Y(y1), X(x2), Y(y2), flag, color)
end

local function drawFilledTriangle(x1, y1, x2, y2, x3, y3, flag)
    lcd.drawFilledTriangle(X(x1), Y(y1), X(x2), Y(y2), X(x3), Y(y3), flag)
end

local function drawFilledRectangle(x, y, w, h, flag)
    lcd.drawFilledRectangle(X(x), Y(y), X(w), Y(h), flag)
end

local function drawCircle(x, y, r, color)
    lcd.drawCircle(X(x), Y(y), X(r), color) -- TODO: better r
end


local function drawCompact(widget, w, h)
    local color = widget.options.Color
    if mbStats == nil then
        lcd.drawText(0, 0, "waits...", color)
        return
    end
    lcd.drawText(w - 2, 0, CHAR_TELEMETRY..mbStats.privacy, color + RIGHT + SMLSIZE)
    if mbStats.connected < 1 then
        lcd.drawText(0, 0, "!", color)
    else
        lcd.drawText(0, 0, ">", color)
        lcd.drawNumber(10, 0, mbStats.bytes_transmitted, color)
        lcd.drawText(0, 20, "<", color)
        lcd.drawNumber(10, 20, mbStats.bytes_received, color)
    end
end


local function drawFull(widget, w, h)
    if w == 800 and h == 480 then
        sx, sy = 1.6, 1.6
    end

    lcd.resetBacklightTimeout()
    lcd.clear(ui.COLOR_BACKGROUND)

    lcd.setColor(CUSTOM_COLOR, ui.COLOR_WHITE)
    drawText(50, 1, "mLRS Statistics", CUSTOM_COLOR)
    if mbStats == nil then
        drawText(100, 20, "Waiting for packet...", CUSTOM_COLOR)
        return
    end

    local x, y

    x = 255
    y = 25
    lcd.setColor(CUSTOM_COLOR, ui.COLOR_WHITE)
    drawText(x-72, y, "rx LQ", CUSTOM_COLOR)
    drawNumber(x, y-6, mbStats.receiver_LQ_rc, CUSTOM_COLOR+MIDSIZE+CENTER)

    y = 90
    lcd.setColor(CUSTOM_COLOR, ui.COLOR_RED)
    drawRectangle(60, y, 100, 50, CUSTOM_COLOR+SOLID)
    drawRectangle(61, y+1, 98, 48, CUSTOM_COLOR+SOLID)
    drawRectangle(300, y, 100, 50, CUSTOM_COLOR+SOLID)
    drawRectangle(301, y+1, 98, 48, CUSTOM_COLOR+SOLID)
    drawLine(177, y+25-10, 280, y+25-10, SOLID, CUSTOM_COLOR)
    drawLine(177, y+25-9, 280, y+25-9, SOLID, CUSTOM_COLOR)
    drawLine(177, y+25-11, 280, y+25-11, SOLID, CUSTOM_COLOR)
    drawLine(180, y+25+10, 283, y+25+10, SOLID, CUSTOM_COLOR)
    drawLine(180, y+25+11, 283, y+25+11, SOLID, CUSTOM_COLOR)
    drawLine(180, y+25+9, 283, y+25+9, SOLID, CUSTOM_COLOR)
    drawFilledTriangle(280+5, y+25-10, 280, y+25-10-5, 280, y+25-10+5, CUSTOM_COLOR+SOLID)
    drawFilledTriangle(180-5, y+25+10, 180, y+25+10-5, 180, y+25+10+5, CUSTOM_COLOR+SOLID)

    lcd.setColor(CUSTOM_COLOR, ui.COLOR_WHITE)

    x = 60
    y = y - 45
    drawText(x-10, y, "tx rssi", CUSTOM_COLOR)
    local rssi = (mbStats.receive_antenna == 0) and mbStats.rssi1 or mbStats.rssi2 -- that's Lua for (cond) ? a1 : a2
    drawNumber(x+70, y, rssi, CUSTOM_COLOR)
    drawText(x-10, y+20, "tx LQ ser", CUSTOM_COLOR)
    drawNumber(x+70, y+20, mbStats.LQ_serial, CUSTOM_COLOR)

    local s = (mbStats.transmit_antenna == 0) and "a1" or "a2"
    drawText(x+75, y+50, s, CUSTOM_COLOR)
    s = (mbStats.receive_antenna == 0) and "a1" or "a2"
    drawText(x+75, y+70, s, CUSTOM_COLOR)

    x = 300
    drawText(x, y, "rx rssi", CUSTOM_COLOR)
    drawNumber(x+80, y, mbStats.receiver_rssi, CUSTOM_COLOR)
    drawText(x, y+20, "rx LQ ser", CUSTOM_COLOR)
    drawNumber(x+80, y+20, mbStats.receiver_LQ_serial, CUSTOM_COLOR)

    s = (mbStats.receiver_receive_antenna == 0) and "a1" or "a2"
    drawText(x+5, y+50, s, CUSTOM_COLOR)
    s = (mbStats.receiver_transmit_antenna == 0) and "a1" or "a2"
    drawText(x+5, y+70, s, CUSTOM_COLOR)

    x = 190
    y = y + 35
    drawText(x, y, "Bps", CUSTOM_COLOR)
    drawNumber(x+46, y, mbStats.bytes_transmitted, CUSTOM_COLOR)
    y = y + 50
    drawText(x, y, "Bps", CUSTOM_COLOR)
    drawNumber(x+46, y, mbStats.bytes_received, CUSTOM_COLOR)

    -- privacy key symbol
    x = 225
    y = 114
    if mbStats.privacy > 0 then
        drawCircle(x, y, 5, CUSTOM_COLOR)
        local xend = x + 15 + mbStats.privacy * 3 -- 1: 17, 2: 20, 3: 23
        drawLine(x+4, y, xend, y, SOLID, CUSTOM_COLOR)
        for i = 1, mbStats.privacy do
            drawLine(xend, y, xend, y-5, SOLID, CUSTOM_COLOR)
            xend = xend - 5
        end
    end

    -- RSSI spectrum graph
    x = 70
    y = 260
    lcd.setColor(CUSTOM_COLOR, ui.COLOR_LIGHTGREY)
    drawLine(x - 10, y, x + 24*15 + 10, y, SOLID, CUSTOM_COLOR)
    drawLine(x - 10, y - (110 - 90), x + 24*15 + 10, y - (110 - 90), SOLID, CUSTOM_COLOR)
    drawLine(x - 10, y - (110 - 50), x + 24*15 + 10, y - (110 - 50), SOLID, CUSTOM_COLOR)
    drawText(x-15, y-9, "-110", CUSTOM_COLOR+RIGHT+SMLSIZE)
    drawText(x-15, y-9-20, "-90", CUSTOM_COLOR+RIGHT+SMLSIZE)
    drawText(x-15, y-9-60, "-50", CUSTOM_COLOR+RIGHT+SMLSIZE)

    if mbRssiList == nil then return end

    lcd.setColor(CUSTOM_COLOR, ui.COLOR_YELLOW)
    for i = 0, mbStats.fhss1_cnt - 1 do
        local r = mbRssiList[i].rssi1
        if r > 120 then
            drawFilledRectangle(x + i*15 - 1, y - 1, 3, 5, CUSTOM_COLOR+SOLID)
        else
            if r > -40 then r = -40 end
            r = 110 + r
            drawLine(x + i*15, y, x + i*15, y - r, SOLID, CUSTOM_COLOR)
        end
    end

    lcd.setColor(CUSTOM_COLOR, ui.COLOR_RED)
    for i = 0, mbStats.fhss1_cnt - 1 do
        local r = mbRssiList[i].rssi2
        if r > 120 then
            drawFilledRectangle(x+3 + i*15 - 1, y - 1, 3, 5, CUSTOM_COLOR+SOLID)
        else
            if r > -40 then r = -40 end
            r = 110 + r
            drawLine(x+3 + i*15, y, x+3 + i*15, y - r, SOLID, CUSTOM_COLOR)
        end
    end
end


----------------------------------------------------------------------
-- Widget
----------------------------------------------------------------------

local function create(zone, options)
    if model.getModule(0).Type ~= 5 and model.getModule(1).Type ~= 5 then
        error("CRSF not enabled!")
    end
    local widget = { zone = zone, options = options }
    return widget
end


local function update(widget, options)
    widget.options = options
end


local function background(widget)
end


local function refresh(widget, event, touchState)
    while true do
        local command, packet = crossfireTelemetryPop()
        if command == nil or packet == nil then break end
        decodeCrsfMbStatistics(command, packet)
    end

    local zone = widget.zone
    if zone.w ~= LCD_W or zone.h ~= LCD_H then -- is not full size widget
        drawCompact(widget, zone.w, zone.h)
    else
        drawFull(widget, zone.w, zone.h)
    end
end


return {
    name = widgetName,
    options = options,
    create = create,
    update = update,
    refresh = refresh,
    background = background
}
