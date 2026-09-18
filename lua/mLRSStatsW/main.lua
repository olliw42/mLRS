local widgetName = "mLRS Statistics"
----------------------------------------------------------------------
-- Copyright (c) OlliW @ www.olliw.eu
-- GPL3
-- https://www.gnu.org/licenses/gpl-3.0.de.html
----------------------------------------------------------------------

local options = {
  -- No user-configurable options yet.
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

local mb_stats = nil    

local mbRssiList = nil


local function decodeCrsfMbStatistics(command, packet)
    if command ~= 0x82 or #packet < 18 or packet[1] ~= 0x65 then
        return nil
    end

    if mb_stats == nil then mb_stats = {} end     

    local b = packet[2]
    mb_stats.connected    = b & 0x01
    mb_stats.binding      = (b >> 1) & 0x01
    mb_stats.dualband     = (b >> 2) & 0x01
    mb_stats.rx_available = (b >> 3) & 0x01

    b = packet[3]
    mb_stats.rx_actual_diversity = b & 0x0F
    mb_stats.tx_actual_diversity = (b >> 4) & 0x0F

    local b = packet[4]
    mb_stats.receive_antenna           = b & 0x01
    mb_stats.transmit_antenna          = (b >> 1) & 0x01
    mb_stats.receiver_receive_antenna  = (b >> 2) & 0x01
    mb_stats.receiver_transmit_antenna = (b >> 3) & 0x01

    mb_stats.rssi1 = packet[5];
    if mb_stats.rssi1 >= 128 then mb_stats.rssi1 = mb_stats.rssi1 - 256 end
    mb_stats.rssi2 = packet[6];
    if mb_stats.rssi2 >= 128 then mb_stats.rssi1 = mb_stats.rssi2 - 256 end
    mb_stats.receiver_rssi = packet[7];
    if mb_stats.receiver_rssi >= 128 then mb_stats.receiver_rssi = mb_stats.receiver_rssi - 256 end

    mb_stats.LQ_serial = packet[8]
    mb_stats.receiver_LQ_rc = packet[9]
    mb_stats.receiver_LQ_serial = packet[10]

    mb_stats.bytes_transmitted = packet[12] * 256 + packet[11]
    mb_stats.bytes_received = packet[14] * 256 + packet[13]

    mb_stats.fhss1_curr_i = packet[15]
    mb_stats.fhss1_cnt    = packet[16]
    mb_stats.fhss2_curr_i = packet[17]
    mb_stats.fhss2_cnt    = packet[18]
    
    if mbRssiList == nil then
        mbRssiList = {}
        for i = 0, mb_stats.fhss1_cnt - 1 do mbRssiList[i] = { rssi1 = 0, rssi2 = 0 } end
    end    
    mbRssiList[mb_stats.fhss1_curr_i] = { rssi1 = mb_stats.rssi1, rssi2 = mb_stats.rssi2 }
end


local function drawIt(event)
    lcd.setColor(CUSTOM_COLOR, ui.COLOR_WHITE)
    lcd.drawText(50, 1, "mLRS Statistics", CUSTOM_COLOR)
    if mb_stats == nil then
        lcd.drawText(100, 20, "Waiting for packet...", CUSTOM_COLOR)
        return
    end
    
    local x, y;
    
    x = 255;
    y = 25;
    lcd.setColor(CUSTOM_COLOR, ui.COLOR_WHITE)
    lcd.drawText(x-72, y, "rx LQ", CUSTOM_COLOR)  
    lcd.drawNumber(x, y-6, mb_stats.receiver_LQ_rc, CUSTOM_COLOR+MIDSIZE+CENTER)
    
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
    if mb_stats.receive_antenna == 0 then
        lcd.drawNumber(x+70, y, mb_stats.rssi1, CUSTOM_COLOR)
    else
        lcd.drawNumber(x+70, y, mb_stats.rssi2, CUSTOM_COLOR)
    end
    lcd.drawText(x-10, y+20, "tx LQ ser", CUSTOM_COLOR)  
    lcd.drawNumber(x+70, y+20, mb_stats.LQ_serial, CUSTOM_COLOR)
    
    if mb_stats.transmit_antenna == 0 then
        lcd.drawText(x+75, y+50, "a1", CUSTOM_COLOR)  
    else    
        lcd.drawText(x+75, y+50, "a2", CUSTOM_COLOR)  
    end    
    if mb_stats.receive_antenna == 0 then
        lcd.drawText(x+75, y+70, "a1", CUSTOM_COLOR)  
    else    
        lcd.drawText(x+75, y+70, "a2", CUSTOM_COLOR)  
    end    
    
    x = 300;
    lcd.drawText(x, y, "rx rssi", CUSTOM_COLOR)  
    lcd.drawNumber(x+80, y, mb_stats.receiver_rssi, CUSTOM_COLOR)
    lcd.drawText(x, y+20, "rx LQ ser", CUSTOM_COLOR)  
    lcd.drawNumber(x+80, y+20, mb_stats.receiver_LQ_serial, CUSTOM_COLOR)
    
    if mb_stats.receiver_receive_antenna == 0 then
        lcd.drawText(x+5, y+50, "a1", CUSTOM_COLOR)  
    else    
        lcd.drawText(x+5, y+50, "a2", CUSTOM_COLOR)  
    end    
    if mb_stats.receiver_transmit_antenna == 0 then
        lcd.drawText(x+5, y+70, "a1", CUSTOM_COLOR)  
    else    
        lcd.drawText(x+5, y+70, "a2", CUSTOM_COLOR)  
    end    
   
    x = 190;
    y = y + 35;
    lcd.drawText(x, y, "Bps", CUSTOM_COLOR)  
    lcd.drawNumber(x+46, y, mb_stats.bytes_transmitted, CUSTOM_COLOR)
    y = y + 50;
    lcd.drawText(x, y, "Bps", CUSTOM_COLOR)  
    lcd.drawNumber(x+46, y, mb_stats.bytes_received, CUSTOM_COLOR)
    
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
    for i = 0, mb_stats.fhss1_cnt - 1 do
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
    for i = 0, mb_stats.fhss1_cnt - 1 do
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
    while true do
        local command, packet = crossfireTelemetryPop()
        if command == nil or packet == nil then
            break
        end
        
        decodeCrsfMbStatistics(command, packet)
    end

    lcd.setColor(CUSTOM_COLOR, ui.COLOR_BACKGROUND)
    lcd.clear(CUSTOM_COLOR)
    drawIt(event)
end


return {
    name = widgetName,
    options = options,
    create = create,
    update = update,
    refresh = refresh,
    background = background
}
