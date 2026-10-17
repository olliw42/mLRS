local widgetName = "mLRS Statistics"
----------------------------------------------------------------------
-- Copyright (c) mLRS project
-- GPL3
-- https://www.gnu.org/licenses/gpl-3.0.de.html
----------------------------------------------------------------------
-- contributed by erwinquilloy
----------------------------------------------------------------------
-- Ethos port of EdgeTx\mLRSStatsW\main.lua (EdgeTx widget)
-- copy main.lua to SD:\scripts\mLRSStatsW\ on the Ethos radio SD card
-- restart radio
-- then add the "mLRS Statistics" widget to a screen; use a full screen layout for the
-- detailed view, a small widget shows the compact view
-- requires mLRS Tx module firmware v1.4.04 or later (CRSF 0x82/0x65 Mb statistics frame)
-- version 2026-10-17.00


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


local STALE_10ms = 150 -- no stats frame for this long -> show waiting again
local PAINT_PERIOD_10ms = 10


----------------------------------------------------------------------
-- CRSF Mb statistics decoder
----------------------------------------------------------------------
-- frame: 0xEA, len, 0x82, 0x65, 17 bytes, crc
-- Ethos popFrame() returns data[1] = first payload byte (0x65), like EdgeTx

local mbStats = nil
local mbStatsTLast_10ms = 0
local mbRssiList = nil
local mbRssiListCountLast = 0


local function tnow_10ms() return math.floor(os.clock() * 100) end


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

    mbStatsTLast_10ms = tnow_10ms()
    return true
end


----------------------------------------------------------------------
-- Drawing
----------------------------------------------------------------------
-- the detailed view is laid out on the EdgeTx 480x272 grid and scaled to the widget

local sx, sy = 1, 1
local function X(x) return math.floor(x * sx + 0.5) end
local function Y(y) return math.floor(y * sy + 0.5) end

local FONT_SMALL, FONT_NORMAL, FONT_BIG = FONT_XS, FONT_S, FONT_L

local function drawText(x, y, s, font, flags)
    lcd.font(font or FONT_NORMAL)
    lcd.drawText(X(x), Y(y), tostring(s), flags or TEXT_LEFT)
end

local function drawRectangle(x, y, w, h)
    lcd.drawRectangle(X(x), Y(y), X(w), Y(h))
end

local function drawLine(x1, y1, x2, y2)
    lcd.drawLine(X(x1), Y(y1), X(x2), Y(y2))
end

local function drawFilledTriangle(x1, y1, x2, y2, x3, y3)
    lcd.drawFilledTriangle(X(x1), Y(y1), X(x2), Y(y2), X(x3), Y(y3))
end

local function drawFilledRectangle(x, y, w, h)
    lcd.drawFilledRectangle(X(x), Y(y), X(w), Y(h))
end

local function drawCircle(x, y, r)
    lcd.drawCircle(X(x), Y(y), X(r))
end


local function drawCompact(widget, w, h)
    lcd.color(widget.color)
    lcd.font(FONT_S)
    if mbStats == nil then
        lcd.drawText(2, 2, "waits...")
        return
    end
    lcd.drawText(w - 2, 2, "P"..mbStats.privacy, TEXT_RIGHT)
    if mbStats.connected < 1 then
        lcd.drawText(2, 2, "!")
    else
        local _, th = lcd.getTextSize("0")
        lcd.drawText(2, 2, "> "..mbStats.bytes_transmitted)
        lcd.drawText(2, 2 + th, "< "..mbStats.bytes_received)
    end
end


local function drawFull(widget, w, h)
    -- this piece must come here
    sx, sy = w / 480, h / 272
    if sy >= 1.4 then
        FONT_SMALL, FONT_NORMAL, FONT_BIG = FONT_S, FONT_STD, FONT_XL
    else
        FONT_SMALL, FONT_NORMAL, FONT_BIG = FONT_XS, FONT_S, FONT_L
    end

    lcd.color(ui.COLOR_BACKGROUND)
    lcd.drawFilledRectangle(0, 0, w, h)

    lcd.color(ui.COLOR_WHITE)
    drawText(50, 1, "mLRS Statistics")
    if mbStats == nil then
        drawText(100, 20, "Waiting for packet...")
        return
    end

    local x, y

    x = 255
    y = 25
    drawText(x-72, y, "rx LQ")
    drawText(x, y-6, mbStats.receiver_LQ_rc, FONT_BIG, TEXT_CENTERED)

    y = 90
    lcd.color(ui.COLOR_RED)
    drawRectangle(60, y, 100, 50)
    drawRectangle(61, y+1, 98, 48)
    drawRectangle(300, y, 100, 50)
    drawRectangle(301, y+1, 98, 48)
    drawLine(177, y+25-10, 280, y+25-10)
    drawLine(177, y+25-9, 280, y+25-9)
    drawLine(177, y+25-11, 280, y+25-11)
    drawLine(180, y+25+10, 283, y+25+10)
    drawLine(180, y+25+11, 283, y+25+11)
    drawLine(180, y+25+9, 283, y+25+9)
    drawFilledTriangle(280+5, y+25-10, 280, y+25-10-5, 280, y+25-10+5)
    drawFilledTriangle(180-5, y+25+10, 180, y+25+10-5, 180, y+25+10+5)

    lcd.color(ui.COLOR_WHITE)

    x = 60
    y = y - 45
    drawText(x-10, y, "tx rssi")
    local rssi = (mbStats.receive_antenna == 0) and mbStats.rssi1 or mbStats.rssi2 -- that's Lua for (cond) ? a1 : a2
    drawText(x+70, y, rssi)
    drawText(x-10, y+20, "tx LQ ser")
    drawText(x+70, y+20, mbStats.LQ_serial)

    local s = (mbStats.transmit_antenna == 0) and "a1" or "a2"
    drawText(x+75, y+50, s)
    s = (mbStats.receive_antenna == 0) and "a1" or "a2"
    drawText(x+75, y+70, s)

    x = 300
    drawText(x, y, "rx rssi")
    drawText(x+80, y, mbStats.receiver_rssi)
    drawText(x, y+20, "rx LQ ser")
    drawText(x+80, y+20, mbStats.receiver_LQ_serial)

    s = (mbStats.receiver_receive_antenna == 0) and "a1" or "a2"
    drawText(x+5, y+50, s)
    s = (mbStats.receiver_transmit_antenna == 0) and "a1" or "a2"
    drawText(x+5, y+70, s)

    x = 190
    y = y + 35
    drawText(x, y, "Bps")
    drawText(x+46, y, mbStats.bytes_transmitted)
    y = y + 50
    drawText(x, y, "Bps")
    drawText(x+46, y, mbStats.bytes_received)

    -- privacy key symbol
    x = 225
    y = 114
    if mbStats.privacy > 0 then
        drawCircle(x, y, 5)
        local xend = x + 15 + mbStats.privacy * 3 -- 1: 17, 2: 20, 3: 23
        drawLine(x+4, y, xend, y)
        for i = 1, mbStats.privacy do
            drawLine(xend, y, xend, y-5)
            xend = xend - 5
        end
    end

    -- RSSI spectrum graph
    x = 70
    y = 260
    lcd.color(ui.COLOR_LIGHTGREY)
    drawLine(x - 10, y, x + 24*15 + 10, y)
    drawLine(x - 10, y - (110 - 90), x + 24*15 + 10, y - (110 - 90))
    drawLine(x - 10, y - (110 - 50), x + 24*15 + 10, y - (110 - 50))
    drawText(x-15, y-9, "-110", FONT_SMALL, TEXT_RIGHT)
    drawText(x-15, y-9-20, "-90", FONT_SMALL, TEXT_RIGHT)
    drawText(x-15, y-9-60, "-50", FONT_SMALL, TEXT_RIGHT)

    if mbRssiList == nil then return end

    lcd.color(ui.COLOR_YELLOW)
    for i = 0, mbStats.fhss1_cnt - 1 do
        local r = mbRssiList[i].rssi1
        if r > 120 then
            drawFilledRectangle(x + i*15 - 1, y - 1, 3, 5)
        else
            if r > -40 then r = -40 end
            r = 110 + r
            drawLine(x + i*15, y, x + i*15, y - r)
        end
    end

    lcd.color(ui.COLOR_RED)
    for i = 0, mbStats.fhss1_cnt - 1 do
        local r = mbRssiList[i].rssi2
        if r > 120 then
            drawFilledRectangle(x+3 + i*15 - 1, y - 1, 3, 5)
        else
            if r > -40 then r = -40 end
            r = 110 + r
            drawLine(x+3 + i*15, y, x+3 + i*15, y - r)
        end
    end
end


----------------------------------------------------------------------
-- Widget
----------------------------------------------------------------------

local function create()
    return { color = ui.COLOR_WHITE, sensor = nil, lastPaint_10ms = 0, dirty = true, stale = true }
end


local function wakeup(widget)
    if widget.sensor == nil then widget.sensor = crsf.getSensor() end

    while true do
        local command, packet = widget.sensor:popFrame(0x82)
        if command == nil or packet == nil then break end
        if decodeCrsfMbStatistics(command, packet) then widget.dirty = true end
    end

    local stale = (mbStats == nil) or (tnow_10ms() - mbStatsTLast_10ms > STALE_10ms)
    if stale and mbStats ~= nil then
        mbStats = nil
        mbRssiList = nil
    end
    if stale ~= widget.stale then
        widget.stale = stale
        widget.dirty = true
    end

    if widget.dirty and (tnow_10ms() - widget.lastPaint_10ms >= PAINT_PERIOD_10ms) then
        widget.dirty = false
        widget.lastPaint_10ms = tnow_10ms()
        lcd.invalidate()
    end
end


local function paint(widget)
    local w, h = lcd.getWindowSize()
    if w >= 400 and h >= 200 then
        drawFull(widget, w, h)
    else
        drawCompact(widget, w, h)
    end
end


local function configure(widget)
    local line = form.addLine("Color (compact view)")
    form.addColorField(line, nil,
        function() return widget.color end,
        function(value) widget.color = value end
        )
end


local function read(widget)
    local c = storage.read("color")
    if c ~= nil then widget.color = c end
end


local function write(widget)
    storage.write("color", widget.color)
end


local function init()
    system.registerWidget({
        key = "mlrsst",
        name = widgetName,
        create = create,
        wakeup = wakeup,
        paint = paint,
        configure = configure,
        read = read,
        write = write,
    })
end

return {
    init = init
}
