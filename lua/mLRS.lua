----------------------------------------------------------------------
-- Copyright (c) MLRS project
-- GPL3
-- https://www.gnu.org/licenses/gpl-3.0.de.html
-- OlliW @ www.olliw.eu
----------------------------------------------------------------------
-- Lua TOOLS script
----------------------------------------------------------------------
-- copy script to SCRIPTS\TOOLS folder on OpenTx SD card


local t_last = 0
local DEVICE_ITEM_TX = nil
local DEVICE_ITEM_RX = nil
local DEVICE_PARAM_LIST = nil
local DEVICE_PARAM_LIST_complete = false
local DEVICE_RX_connected = false
local DEVICE_INFO = nil


local function clearParams()
    DEVICE_ITEM_TX = nil
    DEVICE_ITEM_RX = nil
    DEVICE_PARAM_LIST = nil
    DEVICE_PARAM_LIST_complete = false
    DEVICE_RX_connected = false
    DEVICE_INFO = nil
end


----------------------------------------------------------------------
-- helper to convert command payloads
----------------------------------------------------------------------

local function mb_to_string(payload,pos,len)
    local str = ""
    for i = 0,len-1 do 
        if payload[pos+i] == 0 then break end
        str = str .. string.char(payload[pos+i]) 
    end
    return str
end    

local function mb_to_u8(payload,pos)
    return payload[pos]
end    

local function mb_to_i8(payload,pos)
    local v = payload[pos+0]
    if v >= 128 then v = v - 256 end
    return v
end    

local function mb_to_u16(payload,pos)
    return payload[pos+0] + payload[pos+1]*256
end    

local function mb_to_i16(payload,pos)
    local v = payload[pos+0] + payload[pos+1]*256
    if v >= 32768 then v = v - 65536 end
    return v
end    

local function mb_to_u32(payload,pos)
    return payload[pos+0] + payload[pos+1]*256 + payload[pos+2]*256*256 + payload[pos+2]*256*256*256
end    

local function mb_to_value(payload,pos,typ)
    if typ == mbridge.PARAM_TYPE_UINT8 then -- UINT8
        return mb_to_u8(payload,pos)
    elseif typ == mbridge.PARAM_TYPE_INT8 then -- INT8
        return mb_to_i8(payload,pos)
    elseif typ == mbridge.PARAM_TYPE_UINT16 then -- UINT16
        return mb_to_u16(payload,pos)
    elseif typ == mbridge.PARAM_TYPE_INT16 then -- INT16
        return mb_to_i16(payload,pos)
    elseif typ == mbridge.PARAM_TYPE_LIST then -- LIST
        return payload[pos+0]
    end
    return 0
end    

local function mb_to_value_or_str6(payload,pos,typ)
    if typ == 5 then --mbridge.PARAM_TYPE_STR6 then
        return mb_to_string(payload,pos,6)
    else
        return mb_to_value(payload,pos,typ)
    end
end    

local function mb_to_options(payload,pos,len)
    local str = ""
    for i = 0,len-1 do 
        if payload[pos+i] == 0 then break end
        str = str .. string.char(payload[pos+i]) 
    end
    str = str .. ","
    opt = {};
    for s in string.gmatch(str, "([^,]+)") do
        table.insert(opt, s)
    end
    return opt
end    

local function mb_to_firmwareversions_string(u32)
    local v1,v2,v3,v4
    v1 = math.floor(u32 / 1000000)
    u32 = u32 - v1 * 1000000
    v2 = math.floor(u32 / 10000)
    u32 = u32 - v2 * 10000
    v3 = math.floor(u32 / 100)
    u32 = u32 - v3 * 100
    v4 = u32
    return string.format("v%d.%02d.%02d", v2, v3, v4)
end

local function mb_to_u8_bits(payload,pos,bitpos,bitmask)
    local v = payload[pos]
    v = bit32.rshift(v, bitpos)
    v = bit32.band(v, bitmask)
    return v    
end    

local diversity_list = {}
diversity_list[0] = "enabled"
diversity_list[1] = "antenna1"
diversity_list[2] = "antenna2"

local freq_band_list = {}
freq_band_list[0] = "2.4 GHz"
freq_band_list[1] = "915 MHz FCC"
freq_band_list[2] = "868 MHz"


----------------------------------------------------------------------
-- looper to send and read command frames
----------------------------------------------------------------------

local function doParamLoop()
    -- trigger getting device items and param items
    local t_10ms = getTime()
    if t_10ms - t_last > 10 then
      t_last = t_10ms
      if DEVICE_ITEM_TX == nil then
          mbridge.cmdPush(mbridge.CMD_DEVICE_REQUEST_ITEM, {})
      elseif DEVICE_INFO == nil then    
          if DEVICE_ITEM_RX ~= nil then
              DEVICE_INFO_LIST = {}
              mbridge.cmdPush(mbridge.CMD_REQUEST_CMD, {mbridge.CMD_INFO})
          end    
      elseif DEVICE_PARAM_LIST == nil then
          if DEVICE_INFO ~= nil then
              DEVICE_PARAM_LIST = {}
              mbridge.cmdPush(mbridge.CMD_PARAM_REQUEST_LIST, {})
          end    
      end  
    end    
  
    -- handle received commands
    local cmd = mbridge.cmdPop()
    if cmd ~= nil then
      if cmd.cmd == mbridge.CMD_DEVICE_ITEM_TX then 
          -- MBRIDGE_CMD_DEVICE_ITEM_TX
          DEVICE_ITEM_TX = cmd
          DEVICE_ITEM_TX.version = mb_to_u32(cmd.payload, 0)
          DEVICE_ITEM_TX.name = mb_to_string(cmd.payload, 4, 20)
          DEVICE_ITEM_TX.version_str = mb_to_firmwareversions_string(DEVICE_ITEM_TX.version)
      elseif cmd.cmd == mbridge.CMD_DEVICE_ITEM_RX then 
          -- MBRIDGE_CMD_DEVICE_ITEM_RX
          DEVICE_ITEM_RX = cmd
          DEVICE_ITEM_RX.version = mb_to_u32(cmd.payload, 0)
          DEVICE_ITEM_RX.name = mb_to_string(cmd.payload, 4, 20)
          DEVICE_ITEM_RX.version_str = mb_to_firmwareversions_string(DEVICE_ITEM_RX.version)
          if cmd.payload[4] > 0 and cmd.payload[5] > 0 then -- checking c[4] would be sufficient, but hey
              DEVICE_RX_connected = true
          end    
      elseif cmd.cmd == mbridge.CMD_INFO then 
          -- MBRIDGE_CMD_INFO
          DEVICE_INFO = cmd
          DEVICE_INFO.receiver_sensitivity = mb_to_i16(cmd.payload,0)
          DEVICE_INFO.frequency_band = mb_to_u8(cmd.payload,2)
          DEVICE_INFO.tx_power_dbm = mb_to_i8(cmd.payload,3)
          DEVICE_INFO.tx_diversity = mb_to_u8_bits(cmd.payload,5,1,0x3)
          DEVICE_INFO.rx_power_dbm = mb_to_i8(cmd.payload,4)
          DEVICE_INFO.rx_available = mb_to_u8_bits(cmd.payload,5,0,0x1)
          DEVICE_INFO.rx_diversity = mb_to_u8_bits(cmd.payload,5,3,0x3)
      elseif cmd.cmd == mbridge.CMD_PARAM_ITEM then 
          -- MBRIDGE_CMD_PARAM_ITEM
          local index = cmd.payload[0]
          if index < 128 then
              DEVICE_PARAM_LIST[index] = cmd
              DEVICE_PARAM_LIST[index].typ = mb_to_u8(cmd.payload, 1)
              DEVICE_PARAM_LIST[index].name = mb_to_string(cmd.payload, 2, 16)
              DEVICE_PARAM_LIST[index].value = mb_to_value_or_str6(cmd.payload, 18, DEVICE_PARAM_LIST[index].typ)
              DEVICE_PARAM_LIST[index].min = 0
              DEVICE_PARAM_LIST[index].max = 0
              DEVICE_PARAM_LIST[index].unit = ""
              DEVICE_PARAM_LIST[index].options = {}
              DEVICE_PARAM_LIST[index].allowed_mask = 65536
          elseif index == 255 then
            DEVICE_PARAM_LIST_complete = true
          end  
      elseif cmd.cmd == mbridge.CMD_PARAM_ITEM2 then 
          -- MBRIDGE_CMD_PARAM_ITEM2
          local index = cmd.payload[0]
          if DEVICE_PARAM_LIST[index] ~= nil then
              if DEVICE_PARAM_LIST[index].typ < mbridge.PARAM_TYPE_LIST then
                  DEVICE_PARAM_LIST[index].min = mb_to_value(cmd.payload, 1, DEVICE_PARAM_LIST[index].typ)
                  DEVICE_PARAM_LIST[index].max = mb_to_value(cmd.payload, 3, DEVICE_PARAM_LIST[index].typ)
                  DEVICE_PARAM_LIST[index].unit = mb_to_string(cmd.payload, 7, 6)
              elseif DEVICE_PARAM_LIST[index].typ == mbridge.PARAM_TYPE_LIST then
                  DEVICE_PARAM_LIST[index].allowed_mask = mb_to_u16(cmd.payload, 1)
                  DEVICE_PARAM_LIST[index].options = mb_to_options(cmd.payload, 3, 21)
                  DEVICE_PARAM_LIST[index].item2payload = cmd.payload
                  DEVICE_PARAM_LIST[index].min = 0
                  DEVICE_PARAM_LIST[index].max = #DEVICE_PARAM_LIST[index].options - 1
              end  
          end -- anything else should not happen, but ???
      elseif cmd.cmd == mbridge.CMD_PARAM_ITEM3 then 
          -- MBRIDGE_CMD_PARAM_ITEM3
          local index = cmd.payload[0]
          if DEVICE_PARAM_LIST[index] ~= nil then
              if DEVICE_PARAM_LIST[index].typ == mbridge.PARAM_TYPE_LIST then
                  local s = DEVICE_PARAM_LIST[index].item2payload
                  for i=1,23 do s[23+i] = cmd.payload[i] end  
                  DEVICE_PARAM_LIST[index].options = mb_to_options(s, 3, 21+23)
                  DEVICE_PARAM_LIST[index].max = #DEVICE_PARAM_LIST[index].options - 1
              end  
          end -- anything else should not happen, but ???
      end
    end  
end    
    
    
----------------------------------------------------------------------
-- Edit stuff
----------------------------------------------------------------------

local page_nr = 0 -- 0: main, 1: edit Tx, 2: edit Rx

local cursor_idx = 2
local edit = false
local option_value = 0
    
local cursor_pidx = 0 -- parameter idx which corresponds to the current cursor_idx
local p_cnt = 0
    
    
local function cur_attr(idx)
    local attr = TEXT_COLOR
    if cursor_idx == idx and DEVICE_PARAM_LIST_complete then
        attr = attr + INVERS
        if edit then attr = attr + BLINK end    
    end    
    return attr
end    
    
    
local function param_value_inc(idx)
    local p = DEVICE_PARAM_LIST[idx]
    if p.typ < mbridge.PARAM_TYPE_LIST then
        p.value = p.value + 1
    elseif p.typ == mbridge.PARAM_TYPE_LIST then 
        local value = p.value
        while value <= p.max do 
            value = value + 1
            local m = bit32.lshift(1,value)
            if bit32.btest(m, p.allowed_mask) then p.value = value; break end
        end
    end
    if p.value > p.max then p.value = p.max end
    DEVICE_PARAM_LIST[idx].value = p.value
end    


local function param_value_dec(idx)
    local p = DEVICE_PARAM_LIST[idx]
    if p.typ < mbridge.PARAM_TYPE_LIST then
        p.value = p.value - 1
    elseif p.typ == mbridge.PARAM_TYPE_LIST then 
        local value = p.value
        while value >= p.min do 
            value = value - 1
            local m = bit32.lshift(1,value)
            if bit32.btest(m, p.allowed_mask) then p.value = value; break end
        end
    end
    if p.value < p.min then p.value = p.min end
    DEVICE_PARAM_LIST[idx].value = p.value
end
    
    
----------------------------------------------------------------------
-- Page Edit Tx/Rx
----------------------------------------------------------------------
    
local top_idx = 0
local page_N1 = 9
local page_N = 18
    
local function drawPageEdit(page_str)
    local x, y;
    
    y = 35
    lcd.drawText(5, y, page_str..":", TEXT_COLOR)  
    
    y = 60
    local dy = 21
    local y0 = y
    
    if cursor_idx < top_idx then top_idx = cursor_idx end
    if cursor_idx >= top_idx + page_N then top_idx = cursor_idx - page_N + 1 end
    
    local idx = 0
    for pidx = 2, #DEVICE_PARAM_LIST do      
      if DEVICE_PARAM_LIST[pidx] ~= nil and string.sub(DEVICE_PARAM_LIST[pidx].name,1,2) == page_str then
        local p = DEVICE_PARAM_LIST[pidx]
        local name = string.sub(p.name, 4)
           
        if idx >= top_idx and idx < top_idx + page_N then
            local shifted_idx = idx - top_idx 

            y = y0 + shifted_idx * dy
        
            local xofs = 0
            if shifted_idx >= page_N1 then y = y - page_N1*dy; xofs = 230 end
            
            lcd.drawText(10+xofs, y, name, TEXT_COLOR)
            if p.typ < mbridge.PARAM_TYPE_LIST then
                lcd.drawText(140+xofs, y, p.value.." "..p.unit, cur_attr(idx))  
            elseif p.typ == mbridge.PARAM_TYPE_LIST then
                lcd.drawText(140+xofs, y, p.options[p.value+1], cur_attr(idx))  
            end
        end
        
        if cursor_idx == idx then cursor_pidx = pidx end
           
        idx = idx + 1
      end  
    end
    
    p_cnt = idx
    
    local x_mid = LCD_W/2 - 5 
    if top_idx > 0 then
        local y_base = y0 - 4
        lcd.drawFilledTriangle(x_mid-6, y_base, x_mid+6, y_base, x_mid, y_base-6, TEXT_COLOR)
    end
    if p_cnt > top_idx + page_N then
        local y_base = y0 + page_N1*dy + 4
        lcd.drawFilledTriangle(x_mid-6, y_base, x_mid+6, y_base, x_mid, y_base+6, TEXT_COLOR)
    end      
end    


local function doPageEdit(event, page_str)
    lcd.drawFilledRectangle(0, 0, LCD_W, 30, TITLE_BGCOLOR)
    lcd.drawText(5, 5, "mLRS Configurator: Edit "..page_str, MENU_TITLE_COLOR)

    if event == EVT_VIRTUAL_EXIT and not edit then
        page_nr = 0
        cursor_idx = 2
        return
    end
     
    drawPageEdit(page_str) -- call before event handling, ensures that p_cnt, cursor_pidx are set

    if not edit then
        if event == EVT_VIRTUAL_EXIT then
        elseif event == EVT_VIRTUAL_ENTER then
            edit = true
        elseif event == EVT_VIRTUAL_NEXT then
            cursor_idx = cursor_idx + 1
            if cursor_idx >= p_cnt then cursor_idx = p_cnt - 1 end
        elseif event == EVT_VIRTUAL_PREV then
            cursor_idx = cursor_idx - 1
            if cursor_idx < 0 then cursor_idx = 0 end
        end
    else
        if event == EVT_VIRTUAL_EXIT then
            edit = false
        elseif event == EVT_VIRTUAL_ENTER then
            edit = false
        elseif event == EVT_VIRTUAL_NEXT then
            param_value_inc(cursor_pidx)
        elseif event == EVT_VIRTUAL_PREV then
            param_value_dec(cursor_pidx)
        end
    end  

end


----------------------------------------------------------------------
-- Page Main
----------------------------------------------------------------------

local function drawPageMain()
    local x, y;
    
    lcd.setColor(CUSTOM_COLOR, RED)
    
    y = 35
    lcd.drawText(5, y, "Tx:", TEXT_COLOR)  
    if DEVICE_ITEM_TX == nil then
      lcd.drawText(35, y, "---", TEXT_COLOR)  
    else
      lcd.drawText(35, y, DEVICE_ITEM_TX.name, TEXT_COLOR)  
      lcd.drawText(35, y+20, DEVICE_ITEM_TX.version_str, TEXT_COLOR)  
    end
    lcd.drawText(240, y, "Rx:", TEXT_COLOR)
    if not DEVICE_PARAM_LIST_complete then
    elseif not DEVICE_RX_connected then
      lcd.drawText(270, y, "not connected", TEXT_COLOR)  
    elseif DEVICE_ITEM_RX == nil then
      lcd.drawText(270, y, "---", TEXT_COLOR)  
    else
      lcd.drawText(270, y, DEVICE_ITEM_RX.name, TEXT_COLOR)  
      lcd.drawText(270, y+20, DEVICE_ITEM_RX.version_str, TEXT_COLOR)  
    end
 
    y = 90
    lcd.drawText(10, y, "Bind Phrase", TEXT_COLOR)
    if DEVICE_PARAM_LIST_complete then --DEVICE_PARAM_LIST ~= nil and DEVICE_PARAM_LIST[0] ~= nil then
        lcd.drawText(140, y, DEVICE_PARAM_LIST[0].value, cur_attr(0))  
    end    
    
    lcd.drawText(10, y + 21, "Mode", TEXT_COLOR)  
    if DEVICE_PARAM_LIST_complete then --DEVICE_PARAM_LIST ~= nil and DEVICE_PARAM_LIST[1] ~= nil then
        local p = DEVICE_PARAM_LIST[1]
        if p.options[p.value+1] ~= nil then
            lcd.drawText(140, y+20, p.options[p.value+1], cur_attr(1)) 
        end  
    end
  
    y = 145
    lcd.drawText(10, y, "Edit Tx", cur_attr(2))  
    lcd.drawText(10 + 80, y, "Edit Rx", cur_attr(3))  
    lcd.drawText(10 + 160, y, "Save", cur_attr(4))  
    lcd.drawText(10 + 240, y, "Reload", cur_attr(5))  
     
    -- show overview of some selected parameters
    y = 190
    lcd.setColor(CUSTOM_COLOR, GREY)
    lcd.drawFilledRectangle(0, y-6, LCD_W, 1, CUSTOM_COLOR)
    
    if not DEVICE_PARAM_LIST_complete then
        lcd.drawText(130, y+20, "parameters loading ...", TEXT_COLOR+BLINK+INVERS)  
        return
    end
    
    lcd.drawText(10, y, "Tx Power", TEXT_COLOR)  
    lcd.drawText(140, y, tostring(DEVICE_INFO.tx_power_dbm).." dBm", TEXT_COLOR)  
    lcd.drawText(10, y+20, "Tx Diversity", TEXT_COLOR) 
    
    if DEVICE_INFO.tx_diversity <= #diversity_list then
        lcd.drawText(140, y+20, diversity_list[DEVICE_INFO.tx_diversity], TEXT_COLOR)  
    else
        lcd.drawText(140, y+20, "?", TEXT_COLOR)  
    end    
    
    lcd.drawText(10+240, y, "Rx Power", TEXT_COLOR)
    lcd.drawText(10+240, y+20, "Rx Diversity", TEXT_COLOR)  
    if DEVICE_INFO.rx_power_dbm < 127 then
        lcd.drawText(140+240, y, tostring(DEVICE_INFO.rx_power_dbm).." dBm", TEXT_COLOR)  
        if DEVICE_INFO.rx_diversity <= #diversity_list then
          lcd.drawText(140+240, y+20, diversity_list[DEVICE_INFO.rx_diversity], TEXT_COLOR)  
        else  
          lcd.drawText(140+240, y+20, "?", TEXT_COLOR)  
        end  
    else  
        lcd.drawText(140+240, y, "?", TEXT_COLOR)  
        lcd.drawText(140+240, y+20, "?", TEXT_COLOR)  
    end    
    
    y = y + 2*20
    lcd.drawText(10, y, "Sensitivity", TEXT_COLOR)  
    lcd.drawText(140, y, tostring(DEVICE_INFO.receiver_sensitivity).." dBm", TEXT_COLOR)  
    
    lcd.drawText(10, y+20, "Freq. Band", TEXT_COLOR)
    if DEVICE_INFO.frequency_band <= #freq_band_list then
        lcd.drawText(140, y+20, freq_band_list[DEVICE_INFO.frequency_band], TEXT_COLOR)  
    else    
        lcd.drawText(140, y+20, "?", TEXT_COLOR)  
    end
end    


local function doPageMain(event)
    lcd.drawFilledRectangle(0, 0, LCD_W, 30, TITLE_BGCOLOR)
    lcd.drawText(5, 5, "mLRS Configurator: Main Page", MENU_TITLE_COLOR)
  
    if not edit then
        if event == EVT_VIRTUAL_EXIT then
        elseif event == EVT_VIRTUAL_ENTER and DEVICE_PARAM_LIST_complete then
            if cursor_idx == 2 then -- EditTX pressed
              page_nr = 1
              cursor_idx = 0
              top_idx = 0
              return
            elseif cursor_idx == 3 then -- EditRX pressed
              page_nr = 2
              cursor_idx = 0
              top_idx = 0
              return
            elseif cursor_idx == 5 then -- Reload pressed
              clearParams()
            else      
              edit = true
            end  
        elseif event == EVT_VIRTUAL_NEXT and DEVICE_PARAM_LIST_complete then
            cursor_idx = cursor_idx + 1
            if cursor_idx > 5 then cursor_idx = 5 end
          
            if cursor_idx == 3 and not DEVICE_RX_connected then cursor_idx = 4 end
            if cursor_idx == 4 then cursor_idx = 5 end --currently not allowed
          
        elseif event == EVT_VIRTUAL_PREV and DEVICE_PARAM_LIST_complete then
            cursor_idx = cursor_idx - 1
            if cursor_idx < 0 then cursor_idx = 0 end
          
            if cursor_idx == 4 then cursor_idx = 3 end --currently not allowed
            if cursor_idx == 3 and not DEVICE_RX_connected then cursor_idx = 2 end
          
        end
    else
        if event == EVT_VIRTUAL_EXIT then
            edit = false
        elseif event == EVT_VIRTUAL_ENTER then
            edit = false
        elseif event == EVT_VIRTUAL_NEXT then
            if cursor_idx == 1 then 
                param_value_inc(1)
            end    
        elseif event == EVT_VIRTUAL_PREV then
            if cursor_idx == 1 then 
                param_value_dec(1)
            end    
        end
    end  
  
    drawPageMain()
end


----------------------------------------------------------------------
----------------------------------------------------------------------

local function Do(event)
    lcd.clear()

    doParamLoop()
    
    if page_nr == 1 then
        doPageEdit(event,"Tx")
    elseif page_nr == 2 then
        doPageEdit(event,"Rx")
    else
        doPageMain(event)
    end
end


----------------------------------------------------------------------
-- Interface
----------------------------------------------------------------------

local function scriptInit()
end


local function scriptRun(event)
    if event == nil then
        error("Cannot be run as a model script!")
        return 2
    end  
    if mbridge == nil then
        error("mLRS not available!")
        return 2
    end
    if not edit and page_nr == 0 then
        if event == EVT_VIRTUAL_EXIT then
            return 2
        end    
    end  

    Do(event)
  
    return 0
end

return { init=scriptInit, run=scriptRun }
