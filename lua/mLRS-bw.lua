--local toolName = "TNS|mLRS Configurator|TNE"
----------------------------------------------------------------------
-- Copyright (c) MLRS project
-- GPL3
-- https://www.gnu.org/licenses/gpl-3.0.de.html
-- OlliW @ www.olliw.eu
-- modify for B/W screen by Jason Wang
-- multi-page by Brad Bosch
----------------------------------------------------------------------
-- Lua TOOLS script
----------------------------------------------------------------------
-- copy script to SCRIPTS\TOOLS folder on OpenTx SD card
-- works with OTX, ETX, mOTX v33

-- local version = '2024-11-14.00'

local req_tx_ver = 1000 -- 'v1.0.0'
local req_rx_ver = 1000 -- 'v1.0.0'

local parmLoadDeadTime_10ms = 400 -- 150 was a bit too short

local page = 0
local Max_Page = 6

-- idxes of options on main page
local BindPhrase_idx = 0 -- must not be changed

-- tools (max 4)
local Bind_idx = 4
local Boot_idx = 5

-- save/load/nav (max 4)
local Save_idx = 7
local Reload_idx = 8
local Prev_idx = 9
local Next_idx = 10

local Page0_parm_max = 3
local Page_parm_max = 6
local Page_parm_diff = Page_parm_max - Page0_parm_max

-- convert param id to param list index
local function parm_to_idx(parm_id)
    if page == 0 then
        count = 4
        idx = parm_id
    else
        count = 7
        idx = parm_id + 3 - page * count
    end
    if idx < count and idx >= 0 then return true, idx; end
    return false
end

-- convert param list index to param id
local function idx_to_parm(idx)
    if page == 0 then
        count = 4
        parm_idx = idx
    else
        count = 7
        parm_idx = idx + page * count - 3
    end
    if idx < count and idx >= 0 then return true, parm_idx; end
    return false
end

--------lines and colors
local TEXT_COLOR = 0
local TITLE_COLOR = INVERS

local function liney(line)
    return 8*line;
end


----------------------------------------------------------------------
-- MBridge CRSF emulation
----------------------------------------------------------------------

local MB_CMDPACKET_STX       = 0xA0

local MB_CMD_TX_LINK_ST_LEN  = 22
local MB_CMD_DEV_ITEM_LEN    = 24
local MB_CMD_PARM_ITEM_LEN   = 24
local MB_CMD_REQ_CMD_LEN     = 18
local MB_CMD_INFO_LEN        = 24
local MB_CMD_PARM_SET_LEN    = 7
local MB_CMD_MODELID_SET_LEN = 3

local MB_PARM_TYPE_UINT8     = 0
local MB_PARM_TYPE_INT8      = 1
local MB_PARM_TYPE_UINT16    = 2
local MB_PARM_TYPE_INT16     = 3
local MB_PARM_TYPE_LIST      = 4
local MB_PARM_TYPE_STR6      = 5

local MB_CMD_TX_LINK_ST      = 2
local MB_CMD_REQ_INFO        = 3
local MB_CMD_DEV_ITEM_TX     = 4
local MB_CMD_DEV_ITEM_RX     = 5
local MB_CMD_PARM_REQ_LIST   = 6
local MB_CMD_PARM_ITEM       = 7
local MB_CMD_PARM_ITEM2      = 8
local MB_CMD_PARM_ITEM3      = 9
local MB_CMD_REQ_CMD         = 10
local MB_CMD_INFO            = 11
local MB_CMD_PARM_SET        = 12
local MB_CMD_PARM_STORE      = 13
local MB_CMD_BIND_START      = 14
local MB_CMD_BIND_STOP       = 15
local MB_CMD_MODELID_SET     = 16
local MB_CMD_SYS_BL          = 17

local function mbridgeCmdLen(cmd)
    if cmd == MB_CMD_TX_LINK_ST then return MB_CMD_TX_LINK_ST_LEN; end
    if cmd == MB_CMD_REQ_INFO then return 0; end
    if cmd == MB_CMD_DEV_ITEM_TX then return MB_CMD_DEV_ITEM_LEN; end
    if cmd == MB_CMD_DEV_ITEM_RX then return MB_CMD_DEV_ITEM_LEN; end
    if cmd == MB_CMD_PARM_REQ_LIST then return 0; end
    if cmd == MB_CMD_PARM_ITEM then return MB_CMD_PARM_ITEM_LEN; end
    if cmd == MB_CMD_PARM_ITEM2 then return MB_CMD_PARM_ITEM_LEN; end
    if cmd == MB_CMD_PARM_ITEM3 then return MB_CMD_PARM_ITEM_LEN; end
    if cmd == MB_CMD_REQ_CMD then return MB_CMD_REQ_CMD_LEN; end
    if cmd == MB_CMD_INFO then return MB_CMD_INFO_LEN; end
    if cmd == MB_CMD_PARM_SET then return MB_CMD_PARM_SET_LEN; end
    if cmd == MB_CMD_PARM_STORE then return 0; end
    if cmd == MB_CMD_BIND_START then return 0; end
    if cmd == MB_CMD_BIND_STOP then return 0; end
    if cmd == MB_CMD_MODELID_SET then return MB_CMD_MODELID_SET_LEN; end
    if cmd == MB_CMD_SYS_BL then return 0; end
    return 0;
end

local function isConnected()
    if getRSSI() ~= 0 then return true end
    return false
end

local function cmdPush(cmd, payload)
    -- 'O', 'W', len/cmd, payload bytes
    local data = { 79, 87, cmd + MB_CMDPACKET_STX }
    for i=1, mbridgeCmdLen(cmd) do data[#data + 1] = 0 end -- fill with zeros of correct length
    for i=1, #payload do data[3 + i] = payload[i] end -- fill in data
    -- crossfireTelemetryPush() extends it to
    -- 0xEE, len, 129, 'O', 'W', len/cmd, payload bytes, crc8
    return crossfireTelemetryPush(129, data)
end

local function cmdPop()
    -- crossfireTelemetryPop() is invoked if
    -- address = RADIO_ADDRESS (0xEA) or UART_SYNC (0xC8)
    -- frame id != normal crsf telemetry sensor id
    -- 0xEE, len, 130, len/cmd, payload bytes, crc8
    local cmd, data = crossfireTelemetryPop()
    -- cmd = 130
    -- data = len/cmd, payload bytes
    if cmd == nil then return nil end
    if data == nil or data[1] == nil then return nil end -- Huston, we have a problem
    local command = data[1] - MB_CMDPACKET_STX
    local res = {
        cmd = command,
        len = mbridgeCmdLen(command),
        payload = {}
    }
    for i=2, #data do res.payload[i-2] = data[i] end
    return res
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
    popup_t_end_10ms = getTime() + 150
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
    lcd.drawFilledRectangle(0, liney(1)+2, LCD_W, liney(4)-2, SOLID)

    local i = string.find(popup_text, "\n")
    local attr = TITLE_COLOR+MIDSIZE
    if i == nil then
        lcd.drawText(2, liney(1)+2, popup_text, attr)
    else
        local t1 = string.sub(popup_text, 1,i-1)
        local t2 = string.sub(popup_text, i+1)
        lcd.drawText(2, liney(1)+4, t1, attr)
        lcd.drawText(2, liney(1)+4+12, t2, attr)
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

local parmloop_t_last = 0
local DEV_ITEM_TX = nil
local DEV_ITEM_RX = nil
local DEV_INFO = nil
local DEV_PARM_LIST = nil
local extra_payload = nil
local list_idx = 0
local parm_idx_current = -1
local DEV_PARM_LIST_error = 0
local DEV_PARM_LIST_complete = false
local DEV_DOWNLOAD_is_running = true -- we start the script with this
local DEV_SAVE_t_last = 0

--[[ -- memory size
local mem_max1 = 0
local mem_max2 = 0

local function checkmem()
    local mem = collectgarbage("count")
    if mem > mem_max1 then mem_max1 = mem end
    collectgarbage("collect")
    local mem = collectgarbage("count")
    if mem > mem_max2 then mem_max2 = mem end
end
--]]

local function clearParms()
    -- checkmem() -- memory size
    DEV_ITEM_TX = nil
    DEV_ITEM_RX = nil
    DEV_INFO = nil
    DEV_PARM_LIST = nil
    list_idx = 0
    warned = 0
    parm_idx_current = -1
    DEV_PARM_LIST_error = 0
    DEV_PARM_LIST_complete = false
    DEV_DOWNLOAD_is_running = true
end


local function parmsError(err)
    DEV_PARM_LIST_error = DEV_PARM_LIST_error * 100 + err
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

local function mb_to_value(payload, pos, typ)
    if typ == MB_PARM_TYPE_UINT8 then -- UINT8
        return mb_to_u8(payload,pos)
    elseif typ == MB_PARM_TYPE_INT8 then -- INT8
        return mb_to_i8(payload,pos)
    elseif typ == MB_PARM_TYPE_UINT16 then -- UINT16
        return mb_to_u16(payload,pos)
    elseif typ == MB_PARM_TYPE_INT16 then -- INT16
        return mb_to_i16(payload,pos)
    elseif typ == MB_PARM_TYPE_LIST then -- LIST
        return payload[pos+0]
    end
    return 0
end

local function mb_to_value_or_str6(payload, pos, typ)
    if typ == 5 then --MB_PARM_TYPE_STR6 then
        return mb_to_string(payload,pos,6)
    else
        return mb_to_value(payload,pos,typ)
    end
end

local function mb_to_options(payload, pos, len)
    local r = {}
    local idx = 0
    local opt = ''
    for i = 0,len-1 do
        if payload[pos+i] == 0 then break end
        if payload[pos+i] == 44 then -- 44 = ','
            r[idx] = opt
            idx = idx + 1
            opt = ''
        else
          opt = opt .. string.char(payload[pos+i])
        end
    end
    r[idx] = opt
    return r
end

local function mb_to_firmware_int(u16)
    local major = bit32.rshift(bit32.band(u16, 0xF000), 12)
    local minor = bit32.rshift(bit32.band(u16, 0x0FC0), 6)
    local patch = bit32.band(u16, 0x003F)
    return major * 10000 + minor * 100 + patch
end

local function mb_to_firmware_string(u16)
    local major = bit32.rshift(bit32.band(u16, 0xF000), 12)
    local minor = bit32.rshift(bit32.band(u16, 0x0FC0), 6)
    local patch = bit32.band(u16, 0x003F)
    return string.format("v%d.%d.%02d", major, minor, patch)
end

local function mb_to_u8_bits(payload, pos, bitpos, bitmask)
    local v = payload[pos]
    v = bit32.rshift(v, bitpos)
    v = bit32.band(v, bitmask)
    return v
end

local function mb_allowed_mask_editable(allowed_mask)
    -- if none or only one option allowed -> not editable
    if allowed_mask == 0 then return false; end
    if allowed_mask == 1 then return false; end
    if allowed_mask == 2 then return false; end
    if allowed_mask == 4 then return false; end
    if allowed_mask == 8 then return false; end
    if allowed_mask == 16 then return false; end
    if allowed_mask == 32 then return false; end
    if allowed_mask == 64 then return false; end
    if allowed_mask == 128 then return false; end
    if allowed_mask == 256 then return false; end
    return true
end


----------------------------------------------------------------------
-- looper to send and read command frames
----------------------------------------------------------------------

local function doParmLoop()
    -- trigger getting device items and parm items
    local t_10ms = getTime()
    if t_10ms - parmloop_t_last > 15 then -- was 10 = 100 ms
      parmloop_t_last = t_10ms
      if t_10ms < DEV_SAVE_t_last + parmLoadDeadTime_10ms then
          -- skip, we don't send a cmd if the last Save was recent
      elseif DEV_ITEM_TX == nil and page == 0 then
          cmdPush(MB_CMD_REQ_INFO, {}) -- triggers sending DEV_ITEM_TX, DEV_ITEM_RX, INFO
      else
          if DEV_INFO ~= nil or page ~= 0 then -- wait for it to be populated
              extra_payload = nil
              if DEV_PARM_LIST == nil then
                  DEV_PARM_LIST = {}
              end
              local valid, parm_item = idx_to_parm(list_idx)
              if valid then
                  cmdPush(MB_CMD_REQ_CMD, {MB_CMD_PARM_ITEM, parm_item})
              else
                  DEV_PARM_LIST_complete = true
                  DEV_DOWNLOAD_is_running = false
                  return
              end
          end
      end
    end

    -- handle received commands
    for ijk = 1,6 do -- handle only up to 6 per lua cycle
        -- checkmem() -- memory size
        local cmd = cmdPop()
        if cmd == nil then
            -- list_idx = list_idx + 1 -- error, skip?
            break
        end
        if cmd.cmd == MB_CMD_DEV_ITEM_TX then
            DEV_ITEM_TX = cmd
            --DEV_ITEM_TX.name = mb_to_string(cmd.payload, 4, 20)
            DEV_ITEM_TX.version_int = mb_to_firmware_int(mb_to_u16(cmd.payload, 0))
            DEV_ITEM_TX.setuplayout_int = mb_to_firmware_int(mb_to_u16(cmd.payload, 2))
            DEV_ITEM_TX.version_str = mb_to_firmware_string(mb_to_u16(cmd.payload, 0))
        elseif cmd.cmd == MB_CMD_DEV_ITEM_RX then
            DEV_ITEM_RX = cmd
            --DEV_ITEM_RX.name = mb_to_string(cmd.payload, 4, 20)
            DEV_ITEM_RX.version_int = mb_to_firmware_int(mb_to_u16(cmd.payload, 0))
            DEV_ITEM_RX.setuplayout_int = mb_to_firmware_int(mb_to_u16(cmd.payload, 2))
            DEV_ITEM_RX.version_str = mb_to_firmware_string(mb_to_u16(cmd.payload, 0))
        elseif cmd.cmd == MB_CMD_INFO then
            DEV_INFO = cmd
            --DEV_INFO.receiver_sensitivity = mb_to_i16(cmd.payload,0)
            --DEV_INFO.has_status = mb_to_u8_bits(cmd.payload, 2, 0, 0x01)
            --DEV_INFO.binding = mb_to_u8_bits(cmd.payload, 2, 1, 0x01)
            --DEV_INFO.LQ_low = 0 -- mb_to_u8_bits(cmd.payload, 2, 3, 0x03)
            DEV_INFO.tx_power_dbm = mb_to_i8(cmd.payload,3)
            DEV_INFO.rx_power_dbm = mb_to_i8(cmd.payload,4)
            DEV_INFO.rx_available = mb_to_u8_bits(cmd.payload,5,0,0x1)
            --DEV_INFO.tx_config_id = mb_to_u8(cmd.payload,6)
            --DEV_INFO.tx_diversity = mb_to_u8_bits(cmd.payload,7,0,0x0F)
            --DEV_INFO.rx_diversity = mb_to_u8_bits(cmd.payload,7,4,0x0F)
            DEV_INFO.num_parms = mb_to_i8(cmd.payload,8)
            if DEV_INFO.num_parms ~= 0 then Max_Page = math.floor((DEV_INFO.num_parms + 2) / 7); end
        elseif cmd.cmd == MB_CMD_PARM_ITEM then
            local parm_idx = cmd.payload[0]
            local valid, parm_item = idx_to_parm(list_idx)
            list_idx = list_idx + 1 -- prepare for next
            if parm_idx ~= parm_item then
                break -- unexpected skip
            end
            if DEV_PARM_LIST == nil then
                parmsError(1)
            else
                local valid,idx = parm_to_idx(parm_idx)
                if valid then
                    parm_idx_current = parm_idx -- inform potential Item2/3 calls
                    DEV_PARM_LIST[idx] = cmd
                    DEV_PARM_LIST[idx].typ = mb_to_u8(cmd.payload, 1)
                    DEV_PARM_LIST[idx].name = mb_to_string(cmd.payload, 2, 16)
                    DEV_PARM_LIST[idx].value = mb_to_value_or_str6(cmd.payload, 18, DEV_PARM_LIST[idx].typ)
                    DEV_PARM_LIST[idx].min = 0
                    DEV_PARM_LIST[idx].max = 0
                    DEV_PARM_LIST[idx].unit = ""
                    DEV_PARM_LIST[idx].options = {}
                    DEV_PARM_LIST[idx].allowed_mask = 65536
                    DEV_PARM_LIST[idx].editable = true
                else
                    parmsError(2)
                end
            end
        elseif cmd.cmd == MB_CMD_PARM_ITEM2 then
            local parm_idx = cmd.payload[0]
            if parm_idx ~= parm_idx_current then
                parmsError(3)
            else
                local valid,idx = parm_to_idx(parm_idx)
                if valid then
                    if DEV_PARM_LIST[idx].typ < MB_PARM_TYPE_LIST then
                        DEV_PARM_LIST[idx].min = mb_to_value(cmd.payload, 1, DEV_PARM_LIST[idx].typ)
                        DEV_PARM_LIST[idx].max = mb_to_value(cmd.payload, 3, DEV_PARM_LIST[idx].typ)
                        DEV_PARM_LIST[idx].unit = mb_to_string(cmd.payload, 7, 6)
                    elseif DEV_PARM_LIST[idx].typ == MB_PARM_TYPE_LIST then
                        DEV_PARM_LIST[idx].allowed_mask = mb_to_u16(cmd.payload, 1)
                        DEV_PARM_LIST[idx].options = mb_to_options(cmd.payload, 3, 21)
                        extra_payload = cmd.payload
                        DEV_PARM_LIST[idx].min = 0
                        DEV_PARM_LIST[idx].max = #DEV_PARM_LIST[idx].options
                        DEV_PARM_LIST[idx].editable = mb_allowed_mask_editable(DEV_PARM_LIST[idx].allowed_mask)
                    elseif DEV_PARM_LIST[idx].typ == MB_PARM_TYPE_STR6 then
                        -- nothing to do, is sent but hasn't any content
                    else
                        parmsError(4)
                    end
                else
                    parmsError(5)
                end
            end
        elseif cmd.cmd == MB_CMD_PARM_ITEM3 then
            local parm_idx = cmd.payload[0]
            local is_item4 = false
            if (parm_idx >= 128) then -- this is actually ITEM4
                parm_idx = parm_idx - 128
                is_item4 = true
            end
            if parm_idx ~= parm_idx_current then
                parmsError(6)
            else
                local valid,idx = parm_to_idx(parm_idx)
                if valid then
                    if DEV_PARM_LIST[idx].typ ~= MB_PARM_TYPE_LIST then
                        parmsError(7)
                    elseif extra_payload == nil then
                        parmsError(8)
                        -- parmsError(idx)
                    else
                        if not is_item4 then
                            for i=1,23 do extra_payload[23+i] = cmd.payload[i]; end
                            DEV_PARM_LIST[idx].options = mb_to_options(extra_payload, 3, 21+23)
                        else
                            for i=1,23 do extra_payload[23+23+i] = cmd.payload[i]; end
                            DEV_PARM_LIST[idx].options = mb_to_options(extra_payload, 3, 21+23+23)
                            extra_payload = nil
                        end
                        DEV_PARM_LIST[idx].max = #DEV_PARM_LIST[idx].options
                    end
                end
            end
        end
        cmd = nil
    end --for
    
    if DEV_ITEM_TX ~= nil and DEV_ITEM_RX ~= nil and connected and warned == 0 and
           (DEV_ITEM_TX.setuplayout_int > 515 or DEV_ITEM_RX.setuplayout_int > 515) then -- 515 is old 335
        if DEV_ITEM_TX.setuplayout_int < DEV_ITEM_RX.setuplayout_int then
            setPopupWTmo("Tx param ver old\nUpdate Tx firmware", 500)
            warned = 1
        elseif DEV_ITEM_RX.setuplayout_int < DEV_ITEM_TX.setuplayout_int then
            setPopupWTmo("Rx param ver old\nUpdate Rx firmware", 500)
            warned = 1
        end
    end
    
    if DEV_PARM_LIST_error > 0 then
        -- Huston, we have a problem,
        setPopupWTmo("Er("..tostring(DEV_PARM_LIST_error)..")!\nTry Reload", 300)
    end
end


local function sendParmSet(idx)
    if not DEV_PARM_LIST_complete then return end -- needed here??
    local p = DEV_PARM_LIST[idx]
    local valid,idx = idx_to_parm(idx)
    if not valid then return end
    if p.typ < MB_PARM_TYPE_LIST then
        cmdPush(MB_CMD_PARM_SET, {idx, p.value})
    elseif p.typ == MB_PARM_TYPE_LIST then
        cmdPush(MB_CMD_PARM_SET, {idx, p.value})
    elseif p.typ == MB_PARM_TYPE_STR6 then
        local cmd = {idx}
        for i = 1,6 do
            cmd[i+1] = string.byte(string.sub(p.value, i,i))
        end
        cmdPush(MB_CMD_PARM_SET, cmd)
    end
end


local function sendParmStore()
    if not DEV_PARM_LIST_complete then return end -- needed here??
    cmdPush(MB_CMD_PARM_STORE, {})
    DEV_SAVE_t_last = getTime()
    setPopupWTmo("Save Parameters", 250)
end


local function sendBind()
    --if not DEV_PARM_LIST_complete then return end -- needed here??
    if DEV_DOWNLOAD_is_running then return end
    cmdPush(MB_CMD_BIND_START, {})
    setPopupBlocked("Binding")
end


local function sendBoot()
    --if not DEV_PARM_LIST_complete then return end -- needed here??
    if DEV_DOWNLOAD_is_running then return end
    cmdPush(MB_CMD_SYS_BL, {})
    setPopupBlocked("In System Bootloader")
end


----------------------------------------------------------------------
-- Edit stuff
----------------------------------------------------------------------

local cursor_idx = 9
local edit = false

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
    if DEV_PARM_LIST_complete and cursor_idx == idx then
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
    return attr
end


local function parm_value_inc(idx)
    if not DEV_PARM_LIST_complete then return end -- needed here??
    local p = DEV_PARM_LIST[idx]
    if p.typ < MB_PARM_TYPE_LIST then
        p.value = p.value + 1
    elseif p.typ == MB_PARM_TYPE_LIST then
        local value = p.value
        while value <= p.max do
            value = value + 1
            local m = bit32.lshift(1,value)
            if bit32.btest(m, p.allowed_mask) then p.value = value; break end
        end
    end
    if p.value > p.max then p.value = p.max end
    DEV_PARM_LIST[idx].value = p.value
end


local function parm_value_dec(idx)
    if not DEV_PARM_LIST_complete then return end -- needed here??
    local p = DEV_PARM_LIST[idx]
    if p.typ < MB_PARM_TYPE_LIST then
        p.value = p.value - 1
    elseif p.typ == MB_PARM_TYPE_LIST then
        local value = p.value
        while value >= p.min do
            value = value - 1
            local m = bit32.lshift(1,value)
            if bit32.btest(m, p.allowed_mask) then p.value = value; break end
        end
    end
    if p.value < p.min then p.value = p.min end
    DEV_PARM_LIST[idx].value = p.value
end


local function parm_str6_inc(idx)
    if not DEV_PARM_LIST_complete then return end -- needed here??
    local p = DEV_PARM_LIST[idx]
    if p.typ == MB_PARM_TYPE_STR6 then
        local c = string.sub(p.value, cursor_x_idx+1, cursor_x_idx+1)
        local i = string.find(bindphrase_chars, c, 1, true) -- true for plain search
        i = i + 1
        if i > string.len(bindphrase_chars) then i = 1 end
        c = string.sub(bindphrase_chars, i,i)
        p.value = string.sub(p.value, 1, cursor_x_idx) .. c .. string.sub(p.value, cursor_x_idx+2, string.len(p.value))
    end
    DEV_PARM_LIST[idx].value = p.value
end


local function parm_str6_dec(idx)
    if not DEV_PARM_LIST_complete then return end -- needed here??
    local p = DEV_PARM_LIST[idx]
    if p.typ == MB_PARM_TYPE_STR6 then
        local c = string.sub(p.value, cursor_x_idx+1, cursor_x_idx+1)
        local i = string.find(bindphrase_chars, c, 1, true) -- true for plain search
        i = i - 1
        if i < 1 then i = string.len(bindphrase_chars) end
        c = string.sub(bindphrase_chars, i,i)
        p.value = string.sub(p.value, 1, cursor_x_idx) .. c .. string.sub(p.value, cursor_x_idx+2, string.len(p.value))
    end
    DEV_PARM_LIST[idx].value = p.value
end


local function parm_str6_next(idx)
    if not DEV_PARM_LIST_complete then return false end -- needed here??
    local p = DEV_PARM_LIST[idx]
    if p.typ == MB_PARM_TYPE_STR6 then
        cursor_x_idx = cursor_x_idx + 1
        if cursor_x_idx >= string.len(p.value) then
            return true -- last char
        end
    end
    return false
end


----------------------------------------------------------------------
-- Render Pages
----------------------------------------------------------------------

local function drawPage()
    local x, y
    local s = 0
    local parm_max = Page_parm_max

    if DEV_DOWNLOAD_is_running then
        lcd.drawText(LCD_W/3, LCD_H-24, "MLRS", DBLSIZE+TEXT_COLOR+BLINK+INVERS)
        lcd.drawText(12, LCD_H-9, "parameters loading ...", TEXT_COLOR+BLINK+INVERS)
        return
    end

  if page == 0 then
    -- checkmem() -- memory size
    parm_max = Page0_parm_max
    local version_error = false
    if DEV_ITEM_TX ~= nil and DEV_ITEM_TX.version_int < req_tx_ver then
        version_error = true
        popup_text = "Tx version not supported\nby this Lua script!"
    end
    if DEV_ITEM_RX ~= nil and connected and DEV_ITEM_RX.version_int < req_rx_ver then
        version_error = true
        popup_text = "Rx version not supported\nby this Lua script!"
    end
    if version_error then
        drawPopup()
        return
    end

    y = liney(0)
    if DEV_PARM_LIST_complete then
        --lcd.drawText(0, y, DEV_PARM_LIST[0].name, TEXT_COLOR)
        lcd.drawText(0, y, "Phrase", TEXT_COLOR)
        for i = 1,6 do
            local c = string.sub(DEV_PARM_LIST[0].value, i, i) -- param_idx = 0 = BindPhrase
            local attr = cur_attr_x(0, i-1)
            lcd.drawText(LCD_W*2/3+(i-1)*6, y, c, attr)
        end
    end
    s = 1
  end

    if DEV_PARM_LIST_complete then
        for i=s,parm_max do
            if DEV_PARM_LIST[i] ~= nil and DEV_PARM_LIST[i].name ~= nil then
                lcd.drawText(0, liney(i), string.sub(DEV_PARM_LIST[i].name, 1, 14), TEXT_COLOR)
                if DEV_PARM_LIST[i].allowed_mask > 0 then
                    if DEV_PARM_LIST[i].typ < MB_PARM_TYPE_LIST then
                        lcd.drawText(LCD_W*2/3, liney(i), DEV_PARM_LIST[i].value.." "..DEV_PARM_LIST[i].unit, cur_attr(i))
                    else
                        lcd.drawText(LCD_W*2/3, liney(i), DEV_PARM_LIST[i].options[DEV_PARM_LIST[i].value], cur_attr(i))
                    end
                else
                    lcd.drawText(LCD_W*2/3, liney(i), "-", cur_attr(i))
                end
            else
                lcd.drawText(LCD_W*2/3, liney(i), ".", cur_attr(i))
            end
        end
    end

  if page == 0 then
    y = liney(4)
    attr = SMLSIZE + TEXT_COLOR
    lcd.drawText(0, y, "TxPwr", attr)
    if DEV_INFO ~= nil then
        lcd.drawText(LCD_W/4, y, tostring(DEV_INFO.tx_power_dbm).."dBm", attr)
    else
        lcd.drawText(LCD_W/4, y, "--dBm", attr)
    end

    if DEV_ITEM_TX ~= nil then
        lcd.drawText(LCD_W/2, y, DEV_ITEM_TX.version_str, attr)
    else
        lcd.drawText(LCD_W/2, y, "v-.--.--", attr)
    end

    y = liney(5)
    lcd.drawText(0, y, "RxPwr", attr)
    if DEV_INFO ~= nil and DEV_INFO.rx_available then
        lcd.drawText(LCD_W/4, y, tostring(DEV_INFO.rx_power_dbm).."dBm", attr)
    else
        lcd.drawText(LCD_W/4, y, "--dBm", attr)
    end

    if not connected then
        lcd.drawText(LCD_W/2, y, "disconnected", attr)
    elseif not DEV_PARM_LIST_complete then
        lcd.drawText(LCD_W/2, y, "loading..", attr)
    elseif DEV_ITEM_RX ~= nil then
        lcd.drawText(LCD_W/2, y, DEV_ITEM_RX.version_str, attr)
    else
        lcd.drawText(LCD_W/2, y, "v-.--.--", attr)
    end

    -- Tools
    y = liney(6)
    lcd.drawText(0, y, "bind", cur_attr(Bind_idx))
    lcd.drawText(LCD_W/4, y, "boot", cur_attr(Boot_idx))
    -- lcd.drawText(LCD_W/2, y, tostring(mem_max1), TEXT_COLOR) -- memory size
    -- lcd.drawText(LCD_W*3/4, y, tostring(mem_max2), TEXT_COLOR) -- memory size
  
  end
  
    -- Save/Load and Navigation
    y = liney(7)
    lcd.drawText(0, y, "save", cur_attr(Save_idx - s))
    lcd.drawText(LCD_W/4, y, "load", cur_attr(Reload_idx - s))
    lcd.drawText(LCD_W/2, y, "prev", cur_attr(Prev_idx - s))
    lcd.drawText(LCD_W*3/4, y, "next", cur_attr(Next_idx - s))
end

local function doPage(event)
    local s = 0
    if page == 0 then
      s = 1
    end

    if not edit then
        if event == EVT_VIRTUAL_EXIT then
            -- nothing to do
        elseif event == EVT_VIRTUAL_ENTER then
            if cursor_idx == Save_idx - s and DEV_PARM_LIST_complete then -- Save pressed
                sendParmStore()
                clearParms()
            elseif page == 0 and cursor_idx == Bind_idx then -- Bind pressed
                sendBind()
            elseif page == 0 and cursor_idx == Boot_idx then -- Boot pressed
                sendBoot()
            elseif cursor_idx == Reload_idx - s then -- Reload pressed
                clearParms()
            elseif cursor_idx == Prev_idx - s then -- Prev pressed
                clearParms()
                page = page - 1
                if page == 0 then
                    cursor_idx = cursor_idx - 1 -- 1 fewer positions on page 0; move back to "next"
                end
                if page < 0 then
                    page = Max_Page
                    cursor_idx = cursor_idx + 1 -- 1 more positions on subsequent pages; move forward to "next"
                 end
            elseif cursor_idx == Next_idx - s then -- Next pressed
                clearParms()
                if page == 0 then
                    cursor_idx = cursor_idx + 1 -- 1 more positions on subsequent pages; move forward to "next"
                end
                page = page + 1
                if page > Max_Page then
                    page = 0
                    cursor_idx = cursor_idx - 1 -- 1 fewer positions on page 0; move back to "next"
                end
            elseif DEV_PARM_LIST_complete and DEV_PARM_LIST[cursor_idx] ~= nil and DEV_PARM_LIST[cursor_idx].editable then -- edit option
                cursor_x_idx = 0
                edit = true
            else
                playHaptic(50,0)
            end
        elseif event == EVT_VIRTUAL_NEXT then -- and DEV_PARM_LIST_complete then
            cursor_idx = cursor_idx + 1
            if cursor_idx > Next_idx - s then cursor_idx = Next_idx - s end
        elseif event == EVT_VIRTUAL_PREV then -- and DEV_PARM_LIST_complete then
            cursor_idx = cursor_idx - 1
            if cursor_idx < 0 then cursor_idx = 0 end
        end
    else -- edit
        if event == EVT_VIRTUAL_EXIT then
            if cursor_idx <= Page_parm_max - Page_parm_diff * s then -- BindPhrase, user defined parmas
                sendParmSet(cursor_idx)
            end
            edit = false
        elseif event == EVT_VIRTUAL_ENTER then
            if page == 0 and cursor_idx == BindPhrase_idx then -- BindPhrase
                if parm_str6_next(0) then
                    sendParmSet(0)
                    edit = false
                end
            elseif cursor_idx <= Page_parm_max - Page_parm_diff * s then -- user defined params
                sendParmSet(cursor_idx)
                edit = false
            else
                edit = false
            end
        elseif event == EVT_VIRTUAL_NEXT then
            if page == 0 and cursor_idx == BindPhrase_idx then -- BindPhrase
                parm_str6_inc(0)
            elseif cursor_idx <= Page_parm_max - Page_parm_diff * s then -- user defined params
                parm_value_inc(cursor_idx)
            end
        elseif event == EVT_VIRTUAL_PREV then
            if page == 0 and cursor_idx == BindPhrase_idx then -- BindPhrase
                parm_str6_dec(0)
            elseif cursor_idx <= Page_parm_max - Page_parm_diff * s then -- user defined params
                parm_value_dec(cursor_idx)
            end
        end
    end

    drawPage()
end

----------------------------------------------------------------------
----------------------------------------------------------------------

local function Do(event)
    lcd.clear()

    doConnected()

    if has_connected then
        clearParms()
        if not popup then setPopup("Receiver connected!") end
        if isPopupBlocked() then clearPopup() end
    end
    if has_disconnected then
        if not popup then setPopup("Receiver\nhas disconnected!") end
    end

    doParmLoop()
    doPage(event)

    doPopup()
end


----------------------------------------------------------------------
-- Script OTX Interface
----------------------------------------------------------------------

local function scriptInit()
    -- checkmem() -- memory size
    DEV_DOWNLOAD_is_running = true -- we start the script with this
    local tnow_10ms = getTime()
    if tnow_10ms < 300 then
        DEV_SAVE_t_last = 300 - tnow_10ms -- treat script start like a Save
    else
        DEV_SAVE_t_last = 0
    end
end


local function scriptRun(event)
    -- checkmem() -- memory size
    if event == nil then
        error("Cannot be run as a model script!")
        return 2
    end
    if model.getModule(0).Type ~= 5 and model.getModule(1).Type ~= 5 then
        error("mLRS not accessible: CRSF not enabled!")
        return 2
    end

    if not edit then
        if event == EVT_VIRTUAL_EXIT then
            return 2
        end
    end

    Do(event)

    return 0
end

-- checkmem() -- memory size
return { init=scriptInit, run=scriptRun }
