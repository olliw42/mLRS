--local toolName = "TNS|mLRS Configurator|TNE"
----------------------------------------------------------------------
-- Copyright (c) MLRS project
-- GPL3
-- https://www.gnu.org/licenses/gpl-3.0.de.html
-- OlliW @ www.olliw.eu
----------------------------------------------------------------------
-- Lua TOOLS script
----------------------------------------------------------------------
-- copy script to SCRIPTS\TOOLS folder on OpenTx SD card
-- works with mLRS v0.1.13 and later, mOTX v33

local version = '2023-02-28.00'


-- experimental
local paramLoadDeadTime_10ms = 200 -- 150 was a bit too short
local disableParamLoadErrorWarnings = false


----------------------------------------------------------------------
-- it would be so nice to have
----------------------------------------------------------------------

local function drawFilledTriangle(x0, y0, x1, y1, x2, y2, flags)
    if lcd.drawFilledTriangle == nil then return end
    lcd.drawFilledTriangle(x0, y0, x1, y1, x2, y2, flags)
end    

local charSize = {}
charSize["a"] = 10
charSize["b"] = 10
charSize["c"] = 9
charSize["d"] = 10
charSize["e"] = 9
charSize["f"] = 7
charSize["g"] = 10
charSize["h"] = 10
charSize["i"] = 5
charSize["j"] = 6
charSize["k"] = 10
charSize["l"] = 5
charSize["m"] = 15
charSize["n"] = 10
charSize["o"] = 10
charSize["p"] = 10
charSize["q"] = 10
charSize["r"] = 7
charSize["s"] = 9
charSize["t"] = 6
charSize["u"] = 10
charSize["v"] = 9
charSize["w"] = 13
charSize["x"] = 9
charSize["y"] = 9
charSize["z"] = 9
charSize["0"] = 10
charSize["1"] = 10
charSize["2"] = 10
charSize["3"] = 10
charSize["4"] = 10
charSize["5"] = 10
charSize["6"] = 10
charSize["7"] = 10
charSize["8"] = 10
charSize["9"] = 10
charSize["_"] = 9
charSize["#"] = 11
charSize["-"] = 6
charSize["."] = 5

local function getCharWidth(c)
    if charSize[c] == nil then return 10 end
    return charSize[c]
end    


----------------------------------------------------------------------
-- MBridge CRSF emulation
----------------------------------------------------------------------

local isConnected = nil
local cmdPush = nil
local cmdPop = nil

local MBRIDGE_COMMANDPACKET_STX  = 0xA0
local MBRIDGE_COMMANDPACKET_MASK = 0xE0

local MBRIDGE_CMD_TX_LINK_STATS_LEN  = 22
local MBRIDGE_CMD_DEVICE_ITEM_LEN    = 24
local MBRIDGE_CMD_PARAM_ITEM_LEN     = 24
local MBRIDGE_CMD_REQUEST_CMD_LEN    = 18
local MBRIDGE_CMD_INFO_LEN           = 24
local MBRIDGE_CMD_PARAM_SET_LEN      = 7
local MBRIDGE_CMD_MODELID_SET_LEN    = 3

local MBRIDGE_PARAM_TYPE_UINT8       = 0
local MBRIDGE_PARAM_TYPE_INT8        = 1
local MBRIDGE_PARAM_TYPE_UINT16      = 2
local MBRIDGE_PARAM_TYPE_INT16       = 3
local MBRIDGE_PARAM_TYPE_LIST        = 4
local MBRIDGE_PARAM_TYPE_STR6        = 5

local MBRIDGE_CMD_TX_LINK_STATS      = 2
local MBRIDGE_CMD_REQUEST_INFO       = 3
local MBRIDGE_CMD_DEVICE_ITEM_TX     = 4
local MBRIDGE_CMD_DEVICE_ITEM_RX     = 5
local MBRIDGE_CMD_PARAM_REQUEST_LIST = 6
local MBRIDGE_CMD_PARAM_ITEM         = 7
local MBRIDGE_CMD_PARAM_ITEM2        = 8
local MBRIDGE_CMD_PARAM_ITEM3        = 9
local MBRIDGE_CMD_REQUEST_CMD        = 10
local MBRIDGE_CMD_INFO               = 11
local MBRIDGE_CMD_PARAM_SET          = 12
local MBRIDGE_CMD_PARAM_STORE        = 13
local MBRIDGE_CMD_BIND_START         = 14
local MBRIDGE_CMD_BIND_STOP          = 15
local MBRIDGE_CMD_MODELID_SET        = 16

local function mbridgeCmdLen(cmd)
    if cmd == MBRIDGE_CMD_TX_LINK_STATS then return MBRIDGE_CMD_TX_LINK_STATS_LEN; end
    if cmd == MBRIDGE_CMD_REQUEST_INFO then return 0; end
    if cmd == MBRIDGE_CMD_DEVICE_ITEM_TX then return MBRIDGE_CMD_DEVICE_ITEM_LEN; end
    if cmd == MBRIDGE_CMD_DEVICE_ITEM_RX then return MBRIDGE_CMD_DEVICE_ITEM_LEN; end
    if cmd == MBRIDGE_CMD_PARAM_REQUEST_LIST then return 0; end
    if cmd == MBRIDGE_CMD_PARAM_ITEM then return MBRIDGE_CMD_PARAM_ITEM_LEN; end
    if cmd == MBRIDGE_CMD_PARAM_ITEM2 then return MBRIDGE_CMD_PARAM_ITEM_LEN; end
    if cmd == MBRIDGE_CMD_PARAM_ITEM3 then return MBRIDGE_CMD_PARAM_ITEM_LEN; end
    if cmd == MBRIDGE_CMD_REQUEST_CMD then return MBRIDGE_CMD_REQUEST_CMD_LEN; end
    if cmd == MBRIDGE_CMD_INFO then return MBRIDGE_CMD_INFO_LEN; end
    if cmd == MBRIDGE_CMD_PARAM_SET then return MBRIDGE_CMD_PARAM_SET_LEN; end
    if cmd == MBRIDGE_CMD_PARAM_STORE then return 0; end
    if cmd == MBRIDGE_CMD_BIND_START then return 0; end
    if cmd == MBRIDGE_CMD_BIND_STOP then return 0; end
    if cmd == MBRIDGE_CMD_MODELID_SET then return MBRIDGE_CMD_MODELID_SET_LEN; end
    return 0;
end
  
local function crsfIsConnected()
    if getRSSI() ~= 0 then return true end
    return false
end
  
local function crsfCmdPush(cmd, payload)
    -- 'O', 'W', len/cmd, payload bytes
    local data = { 79, 87, cmd + MBRIDGE_COMMANDPACKET_STX }
    for i=1, mbridgeCmdLen(cmd) do data[#data + 1] = 0 end -- fill with zeros of correct length
    for i=1, #payload do data[3 + i] = payload[i] end -- fill in data
    -- crossfireTelemetryPush() extends it to
    -- 0xEE, len, 129, 'O', 'W', len/cmd, payload bytes, crc8
    return crossfireTelemetryPush(129, data)
end

local function crsfCmdPop()
    -- crossfireTelemetryPop() is invoked if
    -- address = RADIO_ADDRESS (0xEA) or UART_SYNC (0xC8)
    -- frame id != normal crsf telemetry sensor id
    -- 0xEE, len, 130, len/cmd, payload bytes, crc8
    local cmd, data = crossfireTelemetryPop()
    -- cmd = 130
    -- data = len/cmd, payload bytes
    if cmd == nil then return nil end
    if data == nil or data[1] == nil then return nil end -- Huston, we have a problem
    local command = data[1] - MBRIDGE_COMMANDPACKET_STX
    local res = {
        cmd = command,
        len = mbridgeCmdLen(command),
        payload = {}
    }
    for i=2, #data do res.payload[i-2] = data[i] end
    return res
end  


local function mbridgeIsConnected()
    local LStats = mbridge.getLinkStats()
    if LStats.LQ > 0 then return true end
    return false
end


local function setupBridge()
    if mbridge == nil or not mbridge.enabled() then
        isConnected = crsfIsConnected
        cmdPush = crsfCmdPush
        cmdPop = crsfCmdPop -- can return nil
    else  
        isConnected = mbridgeIsConnected
        cmdPush = mbridge.cmdPush
        cmdPop = mbridge.cmdPop -- can return nil
    end
end  


----------------------------------------------------------------------
-- Info/Warning box
----------------------------------------------------------------------

local popup = false   
local popup_text = ""
local popup_t_end_10ms = -1


local function setPopup(txt)
    popup = true
    popup_text = txt
    popup_t_end_10ms = getTime() + 100
end

local function setPopupWTmo(txt, tmo_10ms)
    popup = true
    popup_text = txt
    popup_t_end_10ms = getTime() + tmo_10ms
end

local function setPopupBlocked(txt)
    popup = true
    popup_text = txt
    popup_t_end_10ms = -1
end

local function isPopupBlocked()
    if popup and popup_t_end_10ms < 0 then return true; end
    return false;
end    

local function clearPopup()
    popup = false
end


local function drawPopup()
    lcd.drawFilledRectangle(LCD_W/2-160-2, 74, 320+4, 84, TEXT_COLOR) --TITLE_BGCOLOR)
    lcd.drawFilledRectangle(LCD_W/2-160, 76, 320, 80, TITLE_BGCOLOR) --TEXT_BGCOLOR) --TITLE_BGCOLOR)
    
    local i = string.find(popup_text, "\n")
    local attr = MENU_TITLE_COLOR+MIDSIZE+CENTER
    if i == nil then
        lcd.drawText(LCD_W/2, 99, popup_text, attr)
    else
        local t1 = string.sub(popup_text, 1,i-1)
        local t2 = string.sub(popup_text, i+1)
        lcd.drawText(LCD_W/2, 85, t1, attr)
        lcd.drawText(LCD_W/2, 85+30, t2, attr)
    end  
end


local function doPopup()
    if popup then
        drawPopup()
        if popup_t_end_10ms > 0 then
            local t_10ms = getTime()
            if t_10ms > popup_t_end_10ms then clearPopup() end
        end    
    end  
end


----------------------------------------------------------------------
-- helper to handle connect
----------------------------------------------------------------------

local connected = false
local connected_has_changed = false
local has_connected = false
local has_disconnected = false

local function doConnected()
    local is_connected = isConnected()
    
    connected_has_changed = false
    if is_connected ~= connected then connected_has_changed = true end
    
    has_connected = false
    if connected_has_changed and is_connected then has_connected = true end
    
    has_disconnected = false
    if connected_has_changed and not is_connected then has_disconnected = true end
    
    connected = is_connected
end    
    

----------------------------------------------------------------------
-- variables for mBridge traffic
----------------------------------------------------------------------

local paramloop_t_last = 0
local DEVICE_ITEM_TX = nil
local DEVICE_ITEM_RX = nil
local DEVICE_INFO = nil
local DEVICE_PARAM_LIST = nil
local DEVICE_PARAM_LIST_expected_index = 0
local DEVICE_PARAM_LIST_current_index = -1
local DEVICE_PARAM_LIST_errors = 0
local DEVICE_PARAM_LIST_complete = false
local DEVICE_DOWNLOAD_is_running = true -- we start the script with this
local DEVICE_SAVE_t_last = 0


local function clearParams()
    DEVICE_ITEM_TX = nil
    DEVICE_ITEM_RX = nil
    DEVICE_INFO = nil
    DEVICE_PARAM_LIST = nil
    DEVICE_PARAM_LIST_expected_index = 0
    DEVICE_PARAM_LIST_current_index = -1
    DEVICE_PARAM_LIST_errors = 0
    DEVICE_PARAM_LIST_complete = false
    DEVICE_DOWNLOAD_is_running = true
end


local function paramsError(err)
    DEVICE_PARAM_LIST_errors = DEVICE_PARAM_LIST_errors + 1
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

local function mb_to_u8(payload, pos)
    return payload[pos]
end    

local function mb_to_i8(payload, pos)
    local v = payload[pos+0]
    if v >= 128 then v = v - 256 end
    return v
end    

local function mb_to_u16(payload, pos)
    return payload[pos+0] + payload[pos+1]*256
end    

local function mb_to_i16(payload, pos)
    local v = payload[pos+0] + payload[pos+1]*256
    if v >= 32768 then v = v - 65536 end
    return v
end    

local function mb_to_u24(payload, pos)
    return payload[pos+0] + payload[pos+1]*256 + payload[pos+2]*256*256
end    

local function mb_to_u32(payload, pos)
    return payload[pos+0] + payload[pos+1]*256 + payload[pos+2]*256*256 + payload[pos+2]*256*256*256
end    

local function mb_to_value(payload, pos, typ)
    if typ == MBRIDGE_PARAM_TYPE_UINT8 then -- UINT8
        return mb_to_u8(payload,pos)
    elseif typ == MBRIDGE_PARAM_TYPE_INT8 then -- INT8
        return mb_to_i8(payload,pos)
    elseif typ == MBRIDGE_PARAM_TYPE_UINT16 then -- UINT16
        return mb_to_u16(payload,pos)
    elseif typ == MBRIDGE_PARAM_TYPE_INT16 then -- INT16
        return mb_to_i16(payload,pos)
    elseif typ == MBRIDGE_PARAM_TYPE_LIST then -- LIST
        return payload[pos+0]
    end
    return 0
end    

local function mb_to_value_or_str6(payload, pos, typ)
    if typ == 5 then --MBRIDGE_PARAM_TYPE_STR6 then
        return mb_to_string(payload,pos,6)
    else
        return mb_to_value(payload,pos,typ)
    end
end    

local function mb_to_options(payload, pos, len)
    local str = ""
    for i = 0,len-1 do 
        if payload[pos+i] == 0 then break end
        str = str .. string.char(payload[pos+i]) 
    end
    str = str .. ","
    local opt = {};
    for s in string.gmatch(str, "([^,]+)") do
        table.insert(opt, s)
    end
    return opt
end    

local function mb_to_firmware_u16_string(u16)
    local major = bit32.rshift(bit32.band(u16, 0xF000), 12)
    local minor = bit32.rshift(bit32.band(u16, 0x0FC0), 6)
    local patch = bit32.band(u16, 0x003F)
    return string.format("v%d.%02d.%02d", major, minor, patch)
end

local function mb_to_u8_bits(payload, pos, bitpos, bitmask)
    local v = payload[pos]
    v = bit32.rshift(v, bitpos)
    v = bit32.band(v, bitmask)
    return v    
end

local function mb_allowed_mask_editable(allowed_mask) -- only one option allowed
    if allowed_mask == 1 then return false end
    if allowed_mask == 2 then return false end
    if allowed_mask == 4 then return false end
    if allowed_mask == 8 then return false end
    if allowed_mask == 16 then return false end
    if allowed_mask == 32 then return false end
    if allowed_mask == 64 then return false end
    if allowed_mask == 128 then return false end
    if allowed_mask == 256 then return false end
    return true
end

local diversity_list = {}
diversity_list[0] = "enabled"
diversity_list[1] = "antenna1"
diversity_list[2] = "antenna2"

local freq_band_list = {}
freq_band_list[0] = "2.4 GHz"
freq_band_list[1] = "915 MHz FCC"
freq_band_list[2] = "868 MHz"
freq_band_list[3] = "433 MHz"
freq_band_list[4] = "70 cm HAM"


----------------------------------------------------------------------
-- looper to send and read command frames
----------------------------------------------------------------------

local function doParamLoop()
    -- trigger getting device items and param items
    local t_10ms = getTime()
    if t_10ms - paramloop_t_last > 33 then -- was 10 = 100 ms
      paramloop_t_last = t_10ms
      if t_10ms < DEVICE_SAVE_t_last + paramLoadDeadTime_10ms then
          -- skip, we don't send a cmd if the last Save was recent
      elseif DEVICE_ITEM_TX == nil then
          cmdPush(MBRIDGE_CMD_REQUEST_INFO, {})
          --cmdPush(MBRIDGE_CMD_REQUEST_CMD, {MBRIDGE_CMD_REQUEST_INFO)
          -- these should have been set when we nil-ed DEVICE_PARAM_LIST
          DEVICE_PARAM_LIST_expected_index = 0
          DEVICE_PARAM_LIST_current_index = -1
          DEVICE_PARAM_LIST_errors = 0
          DEVICE_PARAM_LIST_complete = false
      elseif DEVICE_PARAM_LIST == nil then
          if DEVICE_INFO ~= nil then -- wait for it to be populated
              DEVICE_PARAM_LIST = {}
              cmdPush(MBRIDGE_CMD_PARAM_REQUEST_LIST, {})
              --cmdPush(MBRIDGE_CMD_REQUEST_CMD, {MBRIDGE_CMD_PARAM_REQUEST_LIST})
          end    
      end  
    end    
  
    -- handle received commands
    for ijk = 1,6 do -- handle only 6 at most per lua cycle
        local cmd = cmdPop()
        if cmd == nil then break end
        if cmd.cmd == MBRIDGE_CMD_DEVICE_ITEM_TX then 
            -- MBRIDGE_CMD_DEVICE_ITEM_TX
            DEVICE_ITEM_TX = cmd
            DEVICE_ITEM_TX.version_u16 = mb_to_u16(cmd.payload, 0)
            DEVICE_ITEM_TX.setuplayout = mb_to_u16(cmd.payload, 2)
            DEVICE_ITEM_TX.name = mb_to_string(cmd.payload, 4, 20)
            DEVICE_ITEM_TX.version_str = mb_to_firmware_u16_string(DEVICE_ITEM_TX.version_u16)
        elseif cmd.cmd == MBRIDGE_CMD_DEVICE_ITEM_RX then 
            -- MBRIDGE_CMD_DEVICE_ITEM_RX
            DEVICE_ITEM_RX = cmd
            DEVICE_ITEM_RX.version_u16 = mb_to_u16(cmd.payload, 0)
            DEVICE_ITEM_RX.setuplayout = mb_to_u16(cmd.payload, 2)
            DEVICE_ITEM_RX.name = mb_to_string(cmd.payload, 4, 20)
            DEVICE_ITEM_RX.version_str = mb_to_firmware_u16_string(DEVICE_ITEM_RX.version_u16)
        elseif cmd.cmd == MBRIDGE_CMD_INFO then 
            -- MBRIDGE_CMD_INFO
            DEVICE_INFO = cmd
            DEVICE_INFO.receiver_sensitivity = mb_to_i16(cmd.payload,0)
            DEVICE_INFO.tx_power_dbm = mb_to_i8(cmd.payload,3)
            DEVICE_INFO.rx_power_dbm = mb_to_i8(cmd.payload,4)
            DEVICE_INFO.rx_available = mb_to_u8_bits(cmd.payload,5,0,0x1)
            DEVICE_INFO.tx_diversity = mb_to_u8_bits(cmd.payload,5,1,0x3)
            DEVICE_INFO.rx_diversity = mb_to_u8_bits(cmd.payload,5,3,0x3)
        elseif cmd.cmd == MBRIDGE_CMD_PARAM_ITEM then 
            -- MBRIDGE_CMD_PARAM_ITEM
            local index = cmd.payload[0]
            if index ~= DEVICE_PARAM_LIST_expected_index and index ~= 255 then
                paramsError() -- ERROR: should not happen, but ??? => catch this error
            end
            DEVICE_PARAM_LIST_current_index = index -- inform potential Item2/3 calls
            DEVICE_PARAM_LIST_expected_index = index + 1 -- prepare for next
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
                DEVICE_PARAM_LIST[index].editable = true
            elseif index == 255 then -- EOL (end of list :)
                if DEVICE_PARAM_LIST_errors == 0 then
                    DEVICE_PARAM_LIST_complete = true
                elseif disableParamLoadErrorWarnings then -- ignore any errors 
                    DEVICE_PARAM_LIST_complete = true
                else
                    -- Huston, we have a proble,
                    DEVICE_PARAM_LIST_complete = false
                    setPopupWTmo("Param Upload Errors ("..tostring(DEVICE_PARAM_LIST_errors)..")!\nTry Reload", 200)
                end
                DEVICE_DOWNLOAD_is_running = false
            else
                paramsError() -- ERROR: should not happen, but ??? => catch this error
            end  
        elseif cmd.cmd == MBRIDGE_CMD_PARAM_ITEM2 then 
            -- MBRIDGE_CMD_PARAM_ITEM2
            local index = cmd.payload[0]
            if index ~= DEVICE_PARAM_LIST_current_index then
                paramsError() -- ERROR: should not happen, but ??? => catch this error
            elseif DEVICE_PARAM_LIST[index] == nil then
                paramsError() -- ERROR: should not happen, but ??? => catch this error
            else  
                if DEVICE_PARAM_LIST[index].typ < MBRIDGE_PARAM_TYPE_LIST then
                    DEVICE_PARAM_LIST[index].min = mb_to_value(cmd.payload, 1, DEVICE_PARAM_LIST[index].typ)
                    DEVICE_PARAM_LIST[index].max = mb_to_value(cmd.payload, 3, DEVICE_PARAM_LIST[index].typ)
                    DEVICE_PARAM_LIST[index].unit = mb_to_string(cmd.payload, 7, 6)
                elseif DEVICE_PARAM_LIST[index].typ == MBRIDGE_PARAM_TYPE_LIST then
                    DEVICE_PARAM_LIST[index].allowed_mask = mb_to_u16(cmd.payload, 1)
                    DEVICE_PARAM_LIST[index].options = mb_to_options(cmd.payload, 3, 21)
                    DEVICE_PARAM_LIST[index].item2payload = cmd.payload
                    DEVICE_PARAM_LIST[index].min = 0
                    DEVICE_PARAM_LIST[index].max = #DEVICE_PARAM_LIST[index].options - 1
                    DEVICE_PARAM_LIST[index].editable = mb_allowed_mask_editable(DEVICE_PARAM_LIST[index].allowed_mask)
                elseif DEVICE_PARAM_LIST[index].typ == MBRIDGE_PARAM_TYPE_STR6 then
                    -- nothing to do, is send but hasn't any content
                else    
                    paramsError() -- ERROR: should not happen, but ??? => catch this error
                end
            end 
        elseif cmd.cmd == MBRIDGE_CMD_PARAM_ITEM3 then 
            -- MBRIDGE_CMD_PARAM_ITEM3
            local index = cmd.payload[0]
            if index ~= DEVICE_PARAM_LIST_current_index then
                paramsError() -- ERROR: should not happen, but ??? => catch this error
            elseif DEVICE_PARAM_LIST[index] == nil then
                paramsError() -- ERROR: should not happen, but ??? => catch this error
            elseif DEVICE_PARAM_LIST[index].typ ~= MBRIDGE_PARAM_TYPE_LIST then
                paramsError() -- ERROR: should not happen, but ??? => catch this error
            elseif DEVICE_PARAM_LIST[index].item2payload == nil then
                paramsError() -- ERROR: should not happen, but ??? => catch this error
            else
                local s = DEVICE_PARAM_LIST[index].item2payload
                for i=1,23 do s[23+i] = cmd.payload[i] end  
                DEVICE_PARAM_LIST[index].options = mb_to_options(s, 3, 21+23)
                DEVICE_PARAM_LIST[index].max = #DEVICE_PARAM_LIST[index].options - 1
            end
        end
    end--for    
end    
   
   
local function sendParamSet(idx)
    if not DEVICE_PARAM_LIST_complete then return end -- needed here??
    local p = DEVICE_PARAM_LIST[idx]
    if p.typ < MBRIDGE_PARAM_TYPE_LIST then
        cmdPush(MBRIDGE_CMD_PARAM_SET, {idx, p.value})
    elseif p.typ == MBRIDGE_PARAM_TYPE_LIST then
        cmdPush(MBRIDGE_CMD_PARAM_SET, {idx, p.value})
    elseif p.typ == MBRIDGE_PARAM_TYPE_STR6 then
        local cmd = {idx}
        for i = 1,6 do
            cmd[i+1] = string.byte(string.sub(p.value, i,i))
        end    
        cmdPush(MBRIDGE_CMD_PARAM_SET, cmd)
    end  
end  
    
    
local function sendParamStore()
    if not DEVICE_PARAM_LIST_complete then return end -- needed here??
    cmdPush(MBRIDGE_CMD_PARAM_STORE, {})
    DEVICE_SAVE_t_last = getTime()
    setPopupWTmo("Save Parameters", 250)
end  


local function sendBind()
    if not DEVICE_PARAM_LIST_complete then return end -- needed here??
    cmdPush(MBRIDGE_CMD_BIND_START, {})
    setPopupBlocked("Binding")
end  

    
----------------------------------------------------------------------
-- Edit stuff
----------------------------------------------------------------------

local page_nr = 0 -- 0: main, 1: edit Tx, 2: edit Rx

local BindPhrase_idx = 0
local Mode_idx = 1
local RFBand_idx = 2
local EditTx_idx = 3
local EditRx_idx = 4
local Save_idx = 5
local Reload_idx = 6
local Bind_idx = 7

local cursor_idx = EditTx_idx
local edit = false
local option_value = 0
    
local cursor_pidx = 0 -- parameter idx which corresponds to the current cursor_idx
local page_param_cnt = 0 -- number of parameters available on page
    
local cursor_x_idx = 0 -- index into string for string edits
local bindphrase_chars = "abcdefghijklmnopqrstuvwxyz0123456789_#-."


local function cur_attr(idx) -- used in menu
    local attr = TEXT_COLOR
    if cursor_idx == idx then
        attr = attr + INVERS
        if edit then attr = attr + BLINK end    
    end    
    return attr
end    


local function cur_attr_x(idx, x_idx) -- for Bind Phrase character editing
    local attr = TEXT_COLOR
    if DEVICE_PARAM_LIST_complete and cursor_idx == idx then
        if edit then 
            if cursor_x_idx == x_idx then attr = attr + BLINK + INVERS end    
        else    
            attr = attr + INVERS
        end    
    end    
    return attr
end    


local function cur_attr_p(idx, pidx) -- used for parameters
    local attr = cur_attr(idx)
    if DEVICE_PARAM_LIST_complete and not DEVICE_PARAM_LIST[pidx].editable then 
        lcd.setColor(CUSTOM_COLOR, GREY)
        attr = CUSTOM_COLOR
    end
    return attr
end
    
    
local function param_value_inc(idx)
    if not DEVICE_PARAM_LIST_complete then return end -- needed here??
    local p = DEVICE_PARAM_LIST[idx]
    if p.typ < MBRIDGE_PARAM_TYPE_LIST then
        p.value = p.value + 1
    elseif p.typ == MBRIDGE_PARAM_TYPE_LIST then 
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
    if not DEVICE_PARAM_LIST_complete then return end -- needed here??
    local p = DEVICE_PARAM_LIST[idx]
    if p.typ < MBRIDGE_PARAM_TYPE_LIST then
        p.value = p.value - 1
    elseif p.typ == MBRIDGE_PARAM_TYPE_LIST then 
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
    
    
local function param_str6_inc(idx)
    if not DEVICE_PARAM_LIST_complete then return end -- needed here??
    local p = DEVICE_PARAM_LIST[idx]
    if p.typ == MBRIDGE_PARAM_TYPE_STR6 then 
        local c = string.sub(p.value, cursor_x_idx+1, cursor_x_idx+1)
        local i = string.find(bindphrase_chars, c, 1, true) -- true for plain search
        i = i + 1 
        if i > string.len(bindphrase_chars) then i = 1 end
        c = string.sub(bindphrase_chars, i,i)
        p.value = string.sub(p.value, 1, cursor_x_idx) .. c .. string.sub(p.value, cursor_x_idx+2, string.len(p.value))
    end
    DEVICE_PARAM_LIST[idx].value = p.value
end    


local function param_str6_dec(idx)
    if not DEVICE_PARAM_LIST_complete then return end -- needed here??
    local p = DEVICE_PARAM_LIST[idx]
    if p.typ == MBRIDGE_PARAM_TYPE_STR6 then 
        local c = string.sub(p.value, cursor_x_idx+1, cursor_x_idx+1)
        local i = string.find(bindphrase_chars, c, 1, true) -- true for plain search
        i = i - 1 
        if i < 1 then i = string.len(bindphrase_chars) end
        c = string.sub(bindphrase_chars, i,i)
        p.value = string.sub(p.value, 1, cursor_x_idx) .. c .. string.sub(p.value, cursor_x_idx+2, string.len(p.value))
    end
    DEVICE_PARAM_LIST[idx].value = p.value
end
    
    
local function param_str6_next(idx)
    if not DEVICE_PARAM_LIST_complete then return end -- needed here??
    local p = DEVICE_PARAM_LIST[idx]
    if p.typ == MBRIDGE_PARAM_TYPE_STR6 then 
        cursor_x_idx = cursor_x_idx + 1
        if cursor_x_idx >= string.len(p.value) then
            return true -- last char
        end  
    end
    return false
end


local function param_focusable(idx)
    if not DEVICE_PARAM_LIST_complete then return end -- needed here??
    local p = DEVICE_PARAM_LIST[idx]
    if p == nil then return false end
    if p.editable == nil then return false end
    return p.editable
end  


----------------------------------------------------------------------
-- Page Edit Tx/Rx
----------------------------------------------------------------------
    
local top_idx = 0 -- index of first displayed option
local page_N1 = 9 -- number of options displayed in left colum
local page_N = 18 -- number of options displayed on page
    
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
    page_param_cnt = 0
    for pidx = 2, #DEVICE_PARAM_LIST do
        local p = DEVICE_PARAM_LIST[pidx]
        if p ~= nil and string.sub(p.name,1,2) == page_str and p.allowed_mask > 0 then
        local name = string.sub(p.name, 4)
           
        if idx >= top_idx and idx < top_idx + page_N then
            local shifted_idx = idx - top_idx 

            y = y0 + shifted_idx * dy
        
            local xofs = 0
            if shifted_idx >= page_N1 then y = y - page_N1*dy; xofs = 230 end
            
            lcd.drawText(10+xofs, y, name, TEXT_COLOR)
            if p.typ < MBRIDGE_PARAM_TYPE_LIST then
                lcd.drawText(140+xofs, y, p.value.." "..p.unit, cur_attr_p(idx, pidx))  
            elseif p.typ == MBRIDGE_PARAM_TYPE_LIST then
                lcd.drawText(140+xofs, y, p.options[p.value+1], cur_attr_p(idx, pidx))  
            end
        end
        
        if cursor_idx == idx then cursor_pidx = pidx end
           
        idx = idx + 1
      end  
    end
    
    page_param_cnt = idx
    
    local x_mid = LCD_W/2 - 5 
    if top_idx > 0 then
        local y_base = y0 - 4
        --lcd.drawFilledTriangle(x_mid-6, y_base, x_mid+6, y_base, x_mid, y_base-6, TEXT_COLOR)
        drawFilledTriangle(x_mid-6, y_base, x_mid+6, y_base, x_mid, y_base-6, TEXT_COLOR)
    end
    if page_param_cnt > top_idx + page_N then
        local y_base = y0 + page_N1*dy + 4
        --lcd.drawFilledTriangle(x_mid-6, y_base, x_mid+6, y_base, x_mid, y_base+6, TEXT_COLOR)
        drawFilledTriangle(x_mid-6, y_base, x_mid+6, y_base, x_mid, y_base+6, TEXT_COLOR)
    end      
end    


local function doPageEdit(event, page_str)
    lcd.drawFilledRectangle(0, 0, LCD_W, 30, TITLE_BGCOLOR)
    lcd.drawText(5, 5, "mLRS Configurator: Edit "..page_str, MENU_TITLE_COLOR)

    if event == EVT_VIRTUAL_EXIT and not edit then
        page_nr = 0
        cursor_idx = 3
        return
    end
     
    drawPageEdit(page_str) -- call before event handling, ensures that page_param_cnt, cursor_pidx are set

    if not edit then
        if event == EVT_VIRTUAL_EXIT then
        elseif event == EVT_VIRTUAL_ENTER then
            edit = true
        elseif event == EVT_VIRTUAL_NEXT then
            cursor_idx = cursor_idx + 1
            if cursor_idx >= page_param_cnt then cursor_idx = page_param_cnt - 1 end
        elseif event == EVT_VIRTUAL_PREV then
            cursor_idx = cursor_idx - 1
            if cursor_idx < 0 then cursor_idx = 0 end
        end
    else
        if event == EVT_VIRTUAL_EXIT then
            sendParamSet(cursor_pidx)
            edit = false
        elseif event == EVT_VIRTUAL_ENTER then
            sendParamSet(cursor_pidx)
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
        -- don't do anything
    elseif not connected then
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
        local x = 140
        for i = 1,6 do
            local c = string.sub(DEVICE_PARAM_LIST[0].value, i, i)
            local attr = cur_attr_x(0, i-1)
            lcd.drawText(x, y, c, attr)
            --x = x + lcd.getTextWidth(c,1,attr)+1
            x = x + getCharWidth(c) + 1
        end
    end    
    
    lcd.drawText(10, y + 21, "Mode", TEXT_COLOR)  
    if DEVICE_PARAM_LIST_complete then --DEVICE_PARAM_LIST ~= nil and DEVICE_PARAM_LIST[1] ~= nil then
        local p = DEVICE_PARAM_LIST[1]
        if p.options[p.value+1] ~= nil then
            lcd.drawText(140, y+21, p.options[p.value+1], cur_attr_p(1,1))
        end  
    end
    
    lcd.drawText(10, y + 2*21, "RF Band", TEXT_COLOR)  
    if DEVICE_PARAM_LIST_complete then
        local p = DEVICE_PARAM_LIST[2]
        if p.options[p.value+1] ~= nil then
            --lcd.drawText(240+80, y, p.options[p.value+1], cur_attr(2))
            if p.value <= #freq_band_list then 
                lcd.drawText(140, y + 2*21, freq_band_list[p.value], cur_attr_p(2,2)) 
            else
                lcd.drawText(140, y + 2*21, p.options[p.value+1], cur_attr_p(2,2)) 
            end
        end  
    end
 
    y = 166
    lcd.drawText(10, y, "Edit Tx", cur_attr(3))  
    if not connected then 
        lcd.drawText(10 + 80, y, "Edit Rx", TEXT_DISABLE_COLOR)
    else  
        lcd.drawText(10 + 80, y, "Edit Rx", cur_attr(4))
    end  
    lcd.drawText(10 + 160, y, "Save", cur_attr(5))  
    lcd.drawText(10 + 225, y, "Reload", cur_attr(6))
    lcd.drawText(10 + 305, y, "Bind", cur_attr(7))
     
    -- show overview of some selected parameters
    y = 205
    lcd.setColor(CUSTOM_COLOR, GREY)
    lcd.drawFilledRectangle(0, y-6, LCD_W, 1, CUSTOM_COLOR)
    
    --if not DEVICE_PARAM_LIST_complete then
    if DEVICE_DOWNLOAD_is_running then
        lcd.drawText(130, y+20, "parameters loading ...", TEXT_COLOR+BLINK+INVERS)  
        return
    end
    
    lcd.drawText(10, y, "Tx Power", TEXT_COLOR)  
    lcd.drawText(10, y+20, "Tx Diversity", TEXT_COLOR) 
    if DEVICE_INFO ~= nil then
        lcd.drawText(140, y, tostring(DEVICE_INFO.tx_power_dbm).." dBm", TEXT_COLOR)  
        if DEVICE_INFO.tx_diversity <= #diversity_list then
            lcd.drawText(140, y+20, diversity_list[DEVICE_INFO.tx_diversity], TEXT_COLOR)  
        else
            lcd.drawText(140, y+20, "?", TEXT_COLOR)  
        end
    else
        lcd.drawText(140, y, "---", TEXT_COLOR)    
        lcd.drawText(140, y+20, "---", TEXT_COLOR)  
    end    
    
    local rx_attr = TEXT_COLOR
    if not connected then 
        rx_attr = TEXT_DISABLE_COLOR 
    end
    lcd.drawText(10+240, y, "Rx Power", rx_attr)
    lcd.drawText(10+240, y+20, "Rx Diversity", rx_attr)  
    if DEVICE_INFO ~= nil and connected then
        lcd.drawText(140+240, y, tostring(DEVICE_INFO.rx_power_dbm).." dBm", rx_attr)  
        if DEVICE_INFO.rx_diversity <= #diversity_list then
            lcd.drawText(140+240, y+20, diversity_list[DEVICE_INFO.rx_diversity], rx_attr)  
        else  
            lcd.drawText(140+240, y+20, "?", rx_attr)  
        end  
    else  
        lcd.drawText(140+240, y, "---", rx_attr)  
        lcd.drawText(140+240, y+20, "---", rx_attr)  
    end    
    
    y = y + 2*20
    lcd.drawText(10, y, "Sensitivity", TEXT_COLOR)  
    if DEVICE_INFO ~= nil then
        lcd.drawText(140, y, tostring(DEVICE_INFO.receiver_sensitivity).." dBm", TEXT_COLOR)  
    else    
        lcd.drawText(140, y, "---", TEXT_COLOR)  
    end    
end    


local function doPageMain(event)
    lcd.drawFilledRectangle(0, 0, LCD_W, 30, TITLE_BGCOLOR)
    lcd.drawText(5, 5, "mLRS Configurator: Main Page", MENU_TITLE_COLOR)
    lcd.drawText(LCD_W-1, 0, version, MENU_TITLE_COLOR+TINSIZE+RIGHT)
  
    if not edit then
        if event == EVT_VIRTUAL_EXIT then
            -- nothing to do
        elseif event == EVT_VIRTUAL_ENTER then --and DEVICE_PARAM_LIST_complete then
            if cursor_idx == EditTx_idx and DEVICE_PARAM_LIST_complete then -- EditTX pressed
                page_nr = 1
                cursor_idx = 0
                top_idx = 0
                return
            elseif cursor_idx == EditRx_idx and DEVICE_PARAM_LIST_complete then -- EditRX pressed
                page_nr = 2
                cursor_idx = 0
                top_idx = 0
                return
            elseif cursor_idx == Save_idx and DEVICE_PARAM_LIST_complete then -- Save pressed
                sendParamStore()
            elseif cursor_idx == Reload_idx then -- Reload pressed
                clearParams()
            elseif cursor_idx == Bind_idx then -- Bind pressed
                sendBind()
            elseif DEVICE_PARAM_LIST_complete then
                cursor_x_idx = 0
                edit = true
            end  
        elseif event == EVT_VIRTUAL_NEXT then -- and DEVICE_PARAM_LIST_complete then
            cursor_idx = cursor_idx + 1
            if cursor_idx > 7 then cursor_idx = 7 end
          
            if cursor_idx == Mode_idx and not param_focusable(1) then cursor_idx = cursor_idx + 1 end
            if cursor_idx == RFBand_idx and not param_focusable(2) then cursor_idx = cursor_idx + 1 end
            if cursor_idx == EditRx_idx and not connected then cursor_idx = cursor_idx + 1 end
        elseif event == EVT_VIRTUAL_PREV then -- and DEVICE_PARAM_LIST_complete then
            cursor_idx = cursor_idx - 1
            if cursor_idx < 0 then cursor_idx = 0 end
          
            if cursor_idx == EditRx_idx and not connected then cursor_idx = cursor_idx - 1 end
            if cursor_idx == RFBand_idx and not param_focusable(2) then cursor_idx = cursor_idx - 1 end
            if cursor_idx == Mode_idx and not param_focusable(1) then cursor_idx = cursor_idx - 1 end
        end
    else
        if event == EVT_VIRTUAL_EXIT then
            if cursor_idx <= 2 then -- BindPhrase, Mode, RF Band
                sendParamSet(cursor_idx)
            end  
            edit = false
        elseif event == EVT_VIRTUAL_ENTER then
            if cursor_idx == BindPhrase_idx then -- BindPhrase
                if param_str6_next(0) then 
                    sendParamSet(0)
                    edit = false
                end    
            elseif cursor_idx <= 2 then -- Mode, RF Band
                sendParamSet(cursor_idx)
                edit = false
            else          
                edit = false
            end  
        elseif event == EVT_VIRTUAL_NEXT then
            if cursor_idx == BindPhrase_idx then -- BindPhrase 
                param_str6_inc(0)
            elseif cursor_idx <= 2 then -- Mode, RF Band 
                param_value_inc(cursor_idx)
            end    
        elseif event == EVT_VIRTUAL_PREV then
            if cursor_idx == BindPhrase_idx then -- BindPhrase
                param_str6_dec(0)
            elseif cursor_idx <= 2 then -- Mode, RF Band
                param_value_dec(cursor_idx)
            end    
        end
    end  
  
    drawPageMain()
end


----------------------------------------------------------------------
----------------------------------------------------------------------

local function Do(event)
    lcd.clear()

    doConnected()
    
    if has_connected then
        clearParams()
        if not popup then setPopup("Receiver connected!") end
        if isPopupBlocked() then clearPopup() end
    end
    if has_disconnected then
        if not popup then setPopup("Receiver\nhas disconnected!") end
    end
    if not connected and page_nr == 2 then
        page_nr = 0
        cursor_idx = EditTx_idx
    end  

    doParamLoop()
    
    if page_nr == 1 then
        doPageEdit(event,"Tx")
    elseif page_nr == 2 then
        doPageEdit(event,"Rx")
    else
        doPageMain(event)
    end
    
    doPopup()
end


----------------------------------------------------------------------
-- Script OTX Interface
----------------------------------------------------------------------

local function scriptInit()
    DEVICE_DOWNLOAD_is_running = true -- we start the script with this
    local tnow_10ms = getTime()
    if tnow_10ms < 300 then
        DEVICE_SAVE_t_last = 300 - tnow_10ms -- treat script start like a Save 
    else
        DEVICE_SAVE_t_last = 0
    end
end


local function scriptRun(event)
    if event == nil then
        error("Cannot be run as a model script!")
        return 2
    end  
    if mbridge == nil or not mbridge.enabled() then
        if model.getModule(1).Type ~= 5 then
            error("mLRS not accessible: mBridge or CRSF not enabled!")
            return 2
        end
    end
    
    setupBridge()
    if isConnected == nil or cmdPush == nil or cmdPop == nil then --just to be sure for sure
        error("Unclear issue with mBridge or CRSF!")
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
