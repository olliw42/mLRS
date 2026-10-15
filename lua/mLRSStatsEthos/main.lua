local widgetName = "mLRS Statistics"
----------------------------------------------------------------------
-- Copyright (c) OlliW @ www.olliw.eu
-- GPL3
-- https://www.gnu.org/licenses/gpl-3.0.de.html
----------------------------------------------------------------------
-- Ethos port of lua/mLRSStatsW/main.lua (EdgeTx widget, version 2026-10-10.00)
-- copy main.lua to SD:\scripts\mlrsstats\ on the Ethos radio SD card, restart radio
-- then add the "mLRS Statistics" widget to a screen; use a full screen layout for the
-- detailed view, a small widget shows the compact view
-- requires mLRS Tx module firmware v1.4.04 or later (CRSF 0x82/0x65 Mb statistics frame)
-- version 2026-10-14.00


local COLOR_WHITE = lcd.RGB(0xFF, 0xFF, 0xFF)
local COLOR_LIGHTGREY = lcd.RGB(0xB0, 0xB0, 0xB0)
local COLOR_RED = lcd.RGB(0xE5, 0x20, 0x1E)
local COLOR_YELLOW = lcd.RGB(0xFF, 0xD0, 0x00)
local COLOR_BACKGROUND = lcd.RGB(0x08, 0x54, 0x88)

local STALE_S = 1.5 -- no stats frame for this long -> show waiting again
local PAINT_PERIOD_S = 0.1


----------------------------------------------------------------------
-- CRSF Mb statistics decoder
----------------------------------------------------------------------
-- frame: 0xEA, len, 0x82, 0x65, 17 bytes, crc
-- Ethos popFrame() returns data[1] = first payload byte (0x65), like EdgeTx

local mbStats = nil
local mbStatsAt = 0
local mbRssiList = nil
local mbRssiListCountLast = 0

local function now() return os.clock() end

local function s8(v)
    if v >= 128 then return v - 256 end
    return v
end


local function decodeCrsfMbStatistics(command, packet)
    if command ~= 0x82 or packet == nil or #packet < 18 or packet[1] ~= 0x65 then
        return false
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

    mbStats.rssi1 = s8(packet[5])
    mbStats.rssi2 = s8(packet[6])
    mbStats.receiver_rssi = s8(packet[7])

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
    if mbStats.fhss1_curr_i < mbStats.fhss1_cnt then
        mbRssiList[mbStats.fhss1_curr_i] = { rssi1 = mbStats.rssi1, rssi2 = mbStats.rssi2 }
    end

    mbStatsAt = now()
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

local function text(x, y, s, font, flags)
    lcd.font(font or FONT_NORMAL)
    lcd.drawText(X(x), Y(y), tostring(s), flags or TEXT_LEFT)
end

local function line(x1, y1, x2, y2)
    lcd.drawLine(X(x1), Y(y1), X(x2), Y(y2))
end

local function thickHLine(x1, x2, y)
    -- 3 px thick on the EdgeTx grid, scaled
    local yc, t = Y(y), math.max(1, math.floor(sy + 0.5))
    for d = -t, t do lcd.drawLine(X(x1), yc + d, X(x2), yc + d) end
end

local function triangle(x1, y1, x2, y2, x3, y3)
    if lcd.drawFilledTriangle then
        lcd.drawFilledTriangle(X(x1), Y(y1), X(x2), Y(y2), X(x3), Y(y3))
    else
        line(x1, y1, x2, y2); line(x2, y2, x3, y3); line(x3, y3, x1, y1)
    end
end

local function box(x, y, w, h)
    lcd.drawRectangle(X(x), Y(y), X(w), Y(h))
    lcd.drawRectangle(X(x)+1, Y(y)+1, X(w)-2, Y(h)-2)
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
        return
    end
    local _, th = lcd.getTextSize("0")
    lcd.drawText(2, 2, "> "..mbStats.bytes_transmitted)
    lcd.drawText(2, 2 + th, "< "..mbStats.bytes_received)
    if h >= 3 * th + 4 then
        lcd.drawText(2, 2 + 2*th, "LQ "..mbStats.receiver_LQ_rc)
    end
end


local function drawFull(widget, w, h)
    sx, sy = w / 480, h / 272
    if sy >= 1.4 then
        FONT_SMALL, FONT_NORMAL, FONT_BIG = FONT_S, FONT_STD, FONT_XL
    else
        FONT_SMALL, FONT_NORMAL, FONT_BIG = FONT_XS, FONT_S, FONT_L
    end

    lcd.color(COLOR_BACKGROUND)
    lcd.drawFilledRectangle(0, 0, w, h)

    lcd.color(COLOR_WHITE)
    text(50, 1, "mLRS Statistics")
    if mbStats == nil then
        text(100, 20, "Waiting for packet...")
        return
    end

    local x, y

    x = 255
    y = 25
    text(x-72, y, "rx LQ")
    text(x, y-6, mbStats.receiver_LQ_rc, FONT_BIG, TEXT_CENTERED)

    y = 90
    lcd.color(COLOR_RED)
    box(60, y, 100, 50)
    box(300, y, 100, 50)
    thickHLine(177, 280, y+25-10)
    thickHLine(180, 283, y+25+10)
    triangle(280+5, y+25-10, 280, y+25-10-5, 280, y+25-10+5)
    triangle(180-5, y+25+10, 180, y+25+10-5, 180, y+25+10+5)

    lcd.color(COLOR_WHITE)

    x = 60
    y = y - 45
    text(x-10, y, "tx rssi")
    local rssi = (mbStats.receive_antenna == 0) and mbStats.rssi1 or mbStats.rssi2
    text(x+70, y, rssi)
    text(x-10, y+20, "tx LQ ser")
    text(x+70, y+20, mbStats.LQ_serial)

    text(x+75, y+50, (mbStats.transmit_antenna == 0) and "a1" or "a2")
    text(x+75, y+70, (mbStats.receive_antenna == 0) and "a1" or "a2")

    x = 300
    text(x, y, "rx rssi")
    text(x+80, y, mbStats.receiver_rssi)
    text(x, y+20, "rx LQ ser")
    text(x+80, y+20, mbStats.receiver_LQ_serial)

    text(x+5, y+50, (mbStats.receiver_receive_antenna == 0) and "a1" or "a2")
    text(x+5, y+70, (mbStats.receiver_transmit_antenna == 0) and "a1" or "a2")

    x = 190
    y = y + 35
    text(x, y, "Bps")
    text(x+46, y, mbStats.bytes_transmitted)
    y = y + 50
    text(x, y, "Bps")
    text(x+46, y, mbStats.bytes_received)

    -- privacy key symbol
    x = 225
    y = 114
    if mbStats.privacy > 0 then
        lcd.drawCircle(X(x), Y(y), X(5))
        local xend = x + 15 + mbStats.privacy * 3 -- 1: 17, 2: 20, 3: 23
        line(x+4, y, xend, y)
        for i = 1, mbStats.privacy do
            line(xend, y, xend, y-5)
            xend = xend - 5
        end
    end

    -- RSSI spectrum graph

    x = 70
    y = 260
    lcd.color(COLOR_LIGHTGREY)
    line(x - 10, y, x + 24*15 + 10, y)
    line(x - 10, y - (110 - 90), x + 24*15 + 10, y - (110 - 90))
    line(x - 10, y - (110 - 50), x + 24*15 + 10, y - (110 - 50))
    text(x-15, y-9, "-110", FONT_SMALL, TEXT_RIGHT)
    text(x-15, y-9-20, "-90", FONT_SMALL, TEXT_RIGHT)
    text(x-15, y-9-60, "-50", FONT_SMALL, TEXT_RIGHT)

    if mbRssiList == nil then return end

    local barw = math.max(1, math.floor(sx + 0.5))

    for ant = 1, 2 do
        lcd.color(ant == 1 and COLOR_YELLOW or COLOR_RED)
        local xo = (ant == 1) and 0 or 3
        for i = 0, mbStats.fhss1_cnt - 1 do
            local e = mbRssiList[i]
            local r = e and ((ant == 1) and e.rssi1 or e.rssi2) or 0
            local xi = x + xo + i*15
            if r > 120 then -- invalid marker
                lcd.drawFilledRectangle(X(xi - 1), Y(y - 1), X(3), Y(5))
            else
                if r > -40 then r = -40 end
                r = 110 + r
                if r > 0 then
                    lcd.drawFilledRectangle(X(xi) - barw // 2, Y(y - r), barw, Y(y) - Y(y - r))
                end
            end
        end
    end
end


----------------------------------------------------------------------
-- Widget
----------------------------------------------------------------------

local function create()
    return { color = COLOR_WHITE, sensor = nil, lastPaint = 0, dirty = true, stale = true }
end


local function wakeup(widget)
    if widget.sensor == nil and crsf ~= nil and crsf.getSensor ~= nil then
        widget.sensor = crsf.getSensor()
    end

    if widget.sensor ~= nil then
        for _ = 1, 32 do
            local command, packet = widget.sensor:popFrame(0x82)
            if command == nil then break end
            if decodeCrsfMbStatistics(command, packet) then widget.dirty = true end
        end
    end

    local stale = (mbStats == nil) or (now() - mbStatsAt > STALE_S)
    if stale and mbStats ~= nil then
        mbStats = nil
        mbRssiList = nil
    end
    if stale ~= widget.stale then
        widget.stale = stale
        widget.dirty = true
    end

    if widget.dirty and (now() - widget.lastPaint >= PAINT_PERIOD_S) then
        widget.dirty = false
        widget.lastPaint = now()
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
        function(value) widget.color = value end)
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

return { init = init }
