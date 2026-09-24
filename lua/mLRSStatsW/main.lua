local widgetName = "mLRS Statistics Widget"
----------------------------------------------------------------------
-- Copyright (c) OlliW @ www.olliw.eu
-- GPL3
-- https://www.gnu.org/licenses/gpl-3.0.de.html
----------------------------------------------------------------------

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
----------------------------------------------------------------------

local mbStats = nil    
local mbRssiList = nil


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

    mbStats.rssi1 = packet[5];
    if mbStats.rssi1 >= 128 then mbStats.rssi1 = mbStats.rssi1 - 256 end
    mbStats.rssi2 = packet[6];
    if mbStats.rssi2 >= 128 then mbStats.rssi2 = mbStats.rssi2 - 256 end
    mbStats.receiver_rssi = packet[7];
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
    
    if mbRssiList == nil then
        mbRssiList = {}
        for i = 0, mbStats.fhss1_cnt - 1 do mbRssiList[i] = { rssi1 = 0, rssi2 = 0 } end
    end    
    mbRssiList[mbStats.fhss1_curr_i] = { rssi1 = mbStats.rssi1, rssi2 = mbStats.rssi2 }
end


local function drawIt(widget, event)
    local zone = widget.zone
    if zone.w ~= LCD_W or zone.h ~= LCD_H then -- is not full size widget 
        if mbStats == nil then
            lcd.drawText(0, 0, "waits...", widget.options.Color)
            return
        end
        if mbStats.connected < 1 then 
            lcd.drawText(0, 0, "!", widget.options.Color)
        else
            --lcd.drawText(zone.w-5, 0, CHAR_TELEMETRY, widget.options.Color + RIGHT)
            --lcd.drawText(zone.w-5, 2, "*", widget.options.Color + RIGHT)
            --lcd.drawText(zone.w-2, 0, "*"..mbStats.privacy, widget.options.Color + RIGHT)
            --if mbStats.privacy > 0 then
            --    lcd.drawText(zone.w-4, 0, "p"..mbStats.privacy, widget.options.Color + RIGHT)
            --end   
            lcd.drawText(zone.w-2, 0, CHAR_TELEMETRY..mbStats.privacy, widget.options.Color + RIGHT + SMLSIZE)
            lcd.drawText(0, 0, ">", widget.options.Color)        
            lcd.drawNumber(10, 0, mbStats.bytes_transmitted, widget.options.Color)
            lcd.drawText(0, 20, "<", widget.options.Color)        
            lcd.drawNumber(10, 20, mbStats.bytes_received, widget.options.Color)
        end    
        return
    end
  
    lcd.clear(ui.COLOR_BACKGROUND)
  
    lcd.setColor(CUSTOM_COLOR, ui.COLOR_WHITE)
    lcd.drawText(50, 1, "mLRS Statistics", CUSTOM_COLOR)
    if mbStats == nil then
        lcd.drawText(100, 20, "Waiting for packet...", CUSTOM_COLOR)
        return
    end
    
    local x, y;
    
    x = 255;
    y = 25;
    lcd.setColor(CUSTOM_COLOR, ui.COLOR_WHITE)
    lcd.drawText(x-72, y, "rx LQ", CUSTOM_COLOR)  
    lcd.drawNumber(x, y-6, mbStats.receiver_LQ_rc, CUSTOM_COLOR+MIDSIZE+CENTER)
    
    y = 90
    lcd.setColor(CUSTOM_COLOR, ui.COLOR_RED)
    lcd.drawRectangle(60, y, 100, 50, CUSTOM_COLOR+SOLID)    
    lcd.drawRectangle(61, y+1, 98, 48, CUSTOM_COLOR+SOLID)    
    lcd.drawRectangle(300, y, 100, 50, CUSTOM_COLOR+SOLID)    
    lcd.drawRectangle(301, y+1, 98, 48, CUSTOM_COLOR+SOLID)    
    lcd.drawLine(177, y+25-10, 280, y+25-10, SOLID, CUSTOM_COLOR)
    lcd.drawLine(177, y+25-9, 280, y+25-9, SOLID, CUSTOM_COLOR)
    lcd.drawLine(177, y+25-11, 280, y+25-11, SOLID, CUSTOM_COLOR)
    lcd.drawLine(180, y+25+10, 283, y+25+10, SOLID, CUSTOM_COLOR)
    lcd.drawLine(180, y+25+11, 283, y+25+11, SOLID, CUSTOM_COLOR)
    lcd.drawLine(180, y+25+9, 283, y+25+9, SOLID, CUSTOM_COLOR)
    lcd.drawFilledTriangle(280+5, y+25-10, 280,y+25-10-5, 280,y+25-10+5, CUSTOM_COLOR+SOLID)
    lcd.drawFilledTriangle(180-5, y+25+10, 180,y+25+10-5, 180,y+25+10+5, CUSTOM_COLOR+SOLID)
    
    lcd.setColor(CUSTOM_COLOR, ui.COLOR_WHITE)
    
    x = 60;
    y = y - 45;
    lcd.drawText(x-10, y, "tx rssi", CUSTOM_COLOR)
    local rssi = (mbStats.receive_antenna == 0) and mbStats.rssi1 or mbStats.rssi2 -- that's Lua for (cond) ? a1 : a2
    lcd.drawNumber(x+70, y, rssi, CUSTOM_COLOR)
    lcd.drawText(x-10, y+20, "tx LQ ser", CUSTOM_COLOR)  
    lcd.drawNumber(x+70, y+20, mbStats.LQ_serial, CUSTOM_COLOR)
    
    local s = (mbStats.transmit_antenna == 0) and "a1" or "a2"
    lcd.drawText(x+75, y+50, s, CUSTOM_COLOR)  
    s = (mbStats.receive_antenna == 0) and "a1" or "a2"
    lcd.drawText(x+75, y+70, s, CUSTOM_COLOR)  
    
    x = 300;
    lcd.drawText(x, y, "rx rssi", CUSTOM_COLOR)  
    lcd.drawNumber(x+80, y, mbStats.receiver_rssi, CUSTOM_COLOR)
    lcd.drawText(x, y+20, "rx LQ ser", CUSTOM_COLOR)  
    lcd.drawNumber(x+80, y+20, mbStats.receiver_LQ_serial, CUSTOM_COLOR)
    
    s = (mbStats.receiver_receive_antenna == 0) and "a1" or "a2"
    lcd.drawText(x+5, y+50, s, CUSTOM_COLOR)  
    s = (mbStats.receiver_transmit_antenna == 0) and "a1" or "a2"
    lcd.drawText(x+5, y+70, s, CUSTOM_COLOR)  
   
    x = 190;
    y = y + 35;
    lcd.drawText(x, y, "Bps", CUSTOM_COLOR)  
    lcd.drawNumber(x+46, y, mbStats.bytes_transmitted, CUSTOM_COLOR)
    y = y + 50;
    lcd.drawText(x, y, "Bps", CUSTOM_COLOR)  
    lcd.drawNumber(x+46, y, mbStats.bytes_received, CUSTOM_COLOR)
    
    x = 225;
    y = 114;
    if mbStats.privacy > 0 then
        lcd.drawCircle(x, y, 5, CUSTOM_COLOR)
        local xend = x + 15 + mbStats.privacy * 3 -- 1: 17, 2: 20, 3: 23
        lcd.drawLine(x+4, y, xend, y, SOLID, CUSTOM_COLOR)
        lcd.drawLine(xend, y, xend, y-5, SOLID, CUSTOM_COLOR)
        for i = 1, mbStats.privacy do
            lcd.drawLine(xend, y, xend, y-5, SOLID, CUSTOM_COLOR)
            xend = xend - 5
        end    
    end    
    
    -- RSSI spektrum graph
    
    x = 70
    y = 260
    lcd.setColor(CUSTOM_COLOR, ui.COLOR_LIGHTGREY)
    lcd.drawLine(x - 10, y, x + 23*15 + 10, y, SOLID, CUSTOM_COLOR)
    lcd.drawLine(x - 10, y - (110 - 90), x + 23*15 + 10, y - (110 - 90), SOLID, CUSTOM_COLOR)
    lcd.drawLine(x - 10, y - (110 - 50), x + 23*15 + 10, y - (110 - 50), SOLID, CUSTOM_COLOR)
    lcd.drawText(x-35, y-10, "-110", CUSTOM_COLOR+SMLSIZE)
    lcd.drawText(x-35, y-10-20, "-90", CUSTOM_COLOR+SMLSIZE)
    lcd.drawText(x-35, y-10-60, "-50", CUSTOM_COLOR+SMLSIZE)
    
    if mbRssiList == nil then return end
    
    lcd.setColor(CUSTOM_COLOR, ui.COLOR_YELLOW)
    for i = 0, mbStats.fhss1_cnt - 1 do
      local r = mbRssiList[i].rssi1
      if r > 120 then 
        lcd.drawFilledRectangle(x + i*15 - 1, y - 1, 3, 5, CUSTOM_COLOR+SOLID)
      else
        if r > -40 then r = -40 end
        r = 110 + r 
        lcd.drawLine(x + i*15, y, x + i*15, y - r, SOLID, CUSTOM_COLOR)
      end  
    end 
    
    lcd.setColor(CUSTOM_COLOR, ui.COLOR_RED)
    for i = 0, mbStats.fhss1_cnt - 1 do
      local r = mbRssiList[i].rssi2
      if r > 120 then 
        lcd.drawFilledRectangle(x+3 + i*15 - 1, y - 1, 3, 5, CUSTOM_COLOR+SOLID)
      else 
        if r > -40 then r = -40 end
        r = 110 + r 
        lcd.drawLine(x+3 + i*15, y, x+3 + i*15, y - r, SOLID, CUSTOM_COLOR)
      end  
    end
end


----------------------------------------------------------------------
-- Initialization
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
    local command, packet = nil, nil
    while true do
        local c, p = crossfireTelemetryPop()
        if c == nil or p == nil then break end
        command = c
        packet = p
    end
    if command ~= nil and packet ~= nil then
        decodeCrsfMbStatistics(command, packet)
    end

    drawIt(widget, event)
end


return {
    name = widgetName,
    options = options,
    create = create,
    update = update,
    refresh = refresh,
    background = background
}
