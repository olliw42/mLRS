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
-- works with mLRS v1.3.03 and later, mOTX v33
-- 13.02.2026: Many constants/variables put into tables. 
-- Tables are less efficient memory and cpu wise, but are being used to avoid the 200 local limit.

local VERSION = {
    script = '2026-02-13', -- add a '.01' if needed for the day
    required_tx_version_int = 10303,  -- 'v1.3.03'
    required_rx_version_int = 10303,  -- 'v1.3.03'
}


-- experimental
local paramLoadDeadTime_10ms = 300 -- 150 was a bit too short, also 200 was too short


----------------------------------------------------------------------
-- Screen
----------------------------------------------------------------------
-- TX16, T16, etc.:    480 x 272
-- T15, TX15:          480 x 320
-- PA01:               320 x 240

local THEME = {
    screenSize = nil,
    textColor = nil,
    textBgColor = nil,
    titleBgColor = nil,
    menuTitleColor = nil,
    textDisableColor = nil,
}

local LAYOUT = {
    page_N1 = 9, -- number of options displayed in left column
    page_N = 18, -- number of options displayed on page, should be just 2 * page_N1
    W = LCD_W,
    W_HALF = LCD_W / 2,
    DY = 21, -- default line distance
    -- popup box, location of popup box 
    POPUP_X = 80, -- LCD_W/2-160
    POPUP_Y = 76,
    POPUP_W = 320,
    POPUP_H = 80,
    -- warning box, location of setup layout issue warning box in info section
    WARN_X = 30, -- LCD_W/2-210
    WARN_W = 420,
    WARN_H = 50,
    -- main page buttons, location of menu buttons on main page
    BUTTONS_Y = 171,
    EDIT_TX_X = 10,
    EDIT_RX_X = 10 + 80,
    SAVE_X = 10 + 160,
    RELOAD_X = 10 + 225,
    BIND_X = 10 + 305,
    TOOLS_X = 10 + 365,
    -- info section, location of info section on main page
    INFO_Y = 210,
    INFO_DY = 20,
    INFO_LEFT_X = 10,
    INFO_LEFT_VAL_X = 140,
    INFO_RIGHT_X = 10 + LCD_W / 2, -- W_HALF,
    INFO_RIGHT_VALUE_X = 140 + LCD_W / 2, -- W_HALF,
}

local function setupScreen()
    THEME.screenSize = LCD_W * 1000 + LCD_H
    if THEME.screenSize == 320240 then -- 320x240, PA01
        LAYOUT.page_N1 = 7
        LAYOUT.page_N = 7 -- single column
        LAYOUT.POPUP_X = 10
        LAYOUT.POPUP_W = 300
        LAYOUT.WARN_X = 10
        LAYOUT.WARN_W = 300
        LAYOUT.BUTTONS_Y = 158
        LAYOUT.EDIT_TX_X = 2
        LAYOUT.EDIT_RX_X = 66
        LAYOUT.SAVE_X = 130
        LAYOUT.RELOAD_X = 175
        LAYOUT.BIND_X = 234
        LAYOUT.TOOLS_X = 277
        LAYOUT.INFO_Y = 180
        LAYOUT.INFO_DY = 15
        LAYOUT.INFO_LEFT_X = 10
        LAYOUT.INFO_LEFT_VAL_X = 90
        LAYOUT.INFO_RIGHT_X = 165
        LAYOUT.INFO_RIGHT_VALUE_X = 245
    elseif THEME.screenSize == 480320 then -- 480x320, T15
        LAYOUT.page_N1 = 11
        LAYOUT.page_N = 2 * LAYOUT.page_N1
    else
        LAYOUT.page_N1 = 9
        LAYOUT.page_N = 2 * LAYOUT.page_N1
    end
end

local function setupColors()
    local ver, radio, maj, minor, rev, osname = getVersion()

    if maj == nil then maj = 0 end
    if minor == nil then minor = 0 end

    if (osname == 'EdgeTX') and (maj > 2 or (maj == 2 and minor >= 4)) then
        THEME.textColor = COLOR_THEME_SECONDARY1
        THEME.textBgColor = COLOR_THEME_SECONDARY3
        THEME.titleBgColor = COLOR_THEME_SECONDARY1
        THEME.menuTitleColor = COLOR_THEME_PRIMARY2
        THEME.textDisableColor = COLOR_THEME_DISABLED
    else
        THEME.textColor = TEXT_COLOR
        THEME.textBgColor = TEXT_BGCOLOR -- doesn't work in OTX !!
        THEME.titleBgColor = TITLE_BGCOLOR
        THEME.menuTitleColor = MENU_TITLE_COLOR
        THEME.textDisableColor = TEXT_DISABLE_COLOR
    end
end


----------------------------------------------------------------------
-- it would be so nice to have
----------------------------------------------------------------------

local function drawFilledTriangle(x0, y0, x1, y1, x2, y2, flags)
    if lcd.drawFilledTriangle == nil then return end
    lcd.drawFilledTriangle(x0, y0, x1, y1, x2, y2, flags)
end

local charSize = {
    ["a"] = 10,
    ["b"] = 10,
    ["c"] = 9,
    ["d"] = 10,
    ["e"] = 9,
    ["f"] = 7,
    ["g"] = 10,
    ["h"] = 10,
    ["i"] = 5,
    ["j"] = 6,
    ["k"] = 10,
    ["l"] = 5,
    ["m"] = 15,
    ["n"] = 10,
    ["o"] = 10,
    ["p"] = 10,
    ["q"] = 10,
    ["r"] = 7,
    ["s"] = 9,
    ["t"] = 6,
    ["u"] = 10,
    ["v"] = 9,
    ["w"] = 13,
    ["x"] = 9,
    ["y"] = 9,
    ["z"] = 9,
    ["0"] = 10,
    ["1"] = 10,
    ["2"] = 10,
    ["3"] = 10,
    ["4"] = 10,
    ["5"] = 10,
    ["6"] = 10,
    ["7"] = 10,
    ["8"] = 10,
    ["9"] = 10,
    ["_"] = 9,
    ["#"] = 11,
    ["-"] = 6,
    ["."] = 5,
}

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

local MBRIDGE_CMD = {
    TX_LINK_STATS      = 2, -- len 22
    REQUEST_INFO       = 3,
    DEVICE_ITEM_TX     = 4, -- len 24
    DEVICE_ITEM_RX     = 5, -- len 24
    PARAM_REQUEST_LIST = 6,
    PARAM_ITEM         = 7, -- len 24
    PARAM_ITEM2        = 8, -- len 24
    PARAM_ITEM3_4      = 9, -- len24, this can be ITEM3 or ITEM4, is signaled by index >= 128
    REQUEST_CMD        = 10, -- len 18
    INFO               = 11, -- len 24
    PARAM_SET          = 12, -- len 7
    PARAM_STORE        = 13,
    BIND_START         = 14,
    BIND_STOP          = 15,
    MODELID_SET        = 16,
    SYSTEM_BOOTLOADER  = 17,
    FLASH_ESP          = 18,
}

local MBRIDGE_CMD_LEN = {
    [MBRIDGE_CMD.TX_LINK_STATS]   = 22, -- MBRIDGE_CMD_TX_LINK_STATS_LEN
    [MBRIDGE_CMD.DEVICE_ITEM_TX]  = 24, -- MBRIDGE_CMD_DEVICE_ITEM_LEN
    [MBRIDGE_CMD.DEVICE_ITEM_RX]  = 24, -- MBRIDGE_CMD_DEVICE_ITEM_LEN
    [MBRIDGE_CMD.PARAM_ITEM]      = 24, -- MBRIDGE_CMD_PARAM_ITEM_LEN
    [MBRIDGE_CMD.PARAM_ITEM2]     = 24, -- MBRIDGE_CMD_PARAM_ITEM_LEN
    [MBRIDGE_CMD.PARAM_ITEM3_4]   = 24, -- MBRIDGE_CMD_PARAM_ITEM_LEN
    [MBRIDGE_CMD.REQUEST_CMD]     = 18, -- MBRIDGE_CMD_REQUEST_CMD_LEN
    [MBRIDGE_CMD.INFO]            = 24, -- MBRIDGE_CMD_INFO_LEN
    [MBRIDGE_CMD.PARAM_SET]       = 7,  -- MBRIDGE_CMD_PARAM_SET_LEN
    [MBRIDGE_CMD.MODELID_SET]     = 3,  -- MBRIDGE_CMD_MODELID_SET_LEN
}
    
local MBRIDGE_PARAM_TYPE = {
    UINT8       = 0,
    INT8        = 1,
    UINT16      = 2,
    INT16       = 3,
    LIST        = 4,
    STR6        = 5,
}

local function mbridgeCmdLen(cmd)
    return MBRIDGE_CMD_LEN[cmd] or 0 -- short for: if MBRIDGE_CMD_LEN[cmd] ~= nil then return MBRIDGE_CMD_LEN[cmd] else return 0 end
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

local POPUP = {
    active = false,
    text = "",
    tend_10ms = 0,
}

local function setPopupWTmo(txt, tmo_10ms)
    if POPUP.tend_10ms < 0 then return end -- blocked popup on display
    POPUP.active = true
    POPUP.text = txt
    POPUP.tend_10ms = getTime() + tmo_10ms
end

local function setPopupBlocked(txt)
    POPUP.active = true
    POPUP.text = txt
    POPUP.tend_10ms = -1
end

local function clearPopup()
    POPUP.active = false
    POPUP.tend_10ms = 0
end

local function clearPopupIfBlocked()
    if POPUP.active and POPUP.tend_10ms < 0 then clearPopup(); end
end

local function drawPopup()
    lcd.drawFilledRectangle(LAYOUT.POPUP_X-2, LAYOUT.POPUP_Y-2, LAYOUT.POPUP_W+4, LAYOUT.POPUP_H+4, THEME.textColor) --TITLE_BGCOLOR)
    lcd.drawFilledRectangle(LAYOUT.POPUP_X, LAYOUT.POPUP_Y, LAYOUT.POPUP_W, LAYOUT.POPUP_H, THEME.titleBgColor) --TEXT_BGCOLOR) --TITLE_BGCOLOR)

    local i = string.find(POPUP.text, "\n")
    local attr = THEME.menuTitleColor + MIDSIZE + CENTER
    if i == nil then
        lcd.drawText(LAYOUT.W_HALF, 99, POPUP.text, attr)
    else
        local t1 = string.sub(POPUP.text, 1,i-1)
        local t2 = string.sub(POPUP.text, i+1)
        lcd.drawText(LAYOUT.W_HALF, 85, t1, attr)
        lcd.drawText(LAYOUT.W_HALF, 85+30, t2, attr)
    end
end

local function doPopup()
    if POPUP.active then
        drawPopup()
        if POPUP.tend_10ms > 0 then
            local t_10ms = getTime()
            if t_10ms > POPUP.tend_10ms then clearPopup() end
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
local DEVICE_PARAM_LIST_max_index = 0 -- 0: unknown, will be updated after each param upload
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
    return payload[pos+0] + payload[pos+1]*256 + payload[pos+2]*256*256 + payload[pos+3]*256*256*256
end

local function mb_to_value(payload, pos, typ)
    if typ == MBRIDGE_PARAM_TYPE.UINT8 then -- UINT8
        return mb_to_u8(payload,pos)
    elseif typ == MBRIDGE_PARAM_TYPE.INT8 then -- INT8
        return mb_to_i8(payload,pos)
    elseif typ == MBRIDGE_PARAM_TYPE.UINT16 then -- UINT16
        return mb_to_u16(payload,pos)
    elseif typ == MBRIDGE_PARAM_TYPE.INT16 then -- INT16
        return mb_to_i16(payload,pos)
    elseif typ == MBRIDGE_PARAM_TYPE.LIST then -- LIST
        return payload[pos+0]
    end
    return 0
end

local function mb_to_value_or_str6(payload, pos, typ)
    if typ == 5 then --MBRIDGE_PARAM_TYPE.STR6 then
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

local function mb_to_version_int(u16)
    local major = bit32.rshift(bit32.band(u16, 0xF000), 12)
    local minor = bit32.rshift(bit32.band(u16, 0x0FC0), 6)
    local patch = bit32.band(u16, 0x003F)
    return major * 10000 + minor * 100 + patch
end

local function mb_to_version_string(u16)
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

local diversity_list = { -- used for displaying on main page bottom info section
    [0] = "enabled",
    [1] = "antenna1",
    [2] = "antenna2",
    [3] = "r:en. t:ant1",
    [4] = "r:en. t:ant2",
}

local freq_band_list = { -- used for displaying on main page instead of received options
    [0] = "2.4 GHz",
    [1] = "915 MHz FCC",
    [2] = "868 MHz",
    [3] = "433 MHz",
    [4] = "70 cm HAM",
    [5] = "866 MHz IN",
}

local function getExceptNoFromChar(c)
    if (c >= 'a' and c <= 'z') then return (string.byte(c) - string.byte('a')) % 5; end
    if (c >= '0' and c <= '9') then return (string.byte(c) - string.byte('0')) % 5; end
    if (c == '_') then return 1; end
    if (c == '#') then return 2; end
    if (c == '-') then return 3; end
    if (c == '.') then return 4; end
    return 0
end

local function getExceptStrFromChar(c)
    local n = getExceptNoFromChar(c)
    if (n == 1) then return "/e1"; end
    if (n == 2) then return "/e6"; end
    if (n == 3) then return "/e11"; end
    if (n == 4) then return "/e13"; end
    return "/--"
end


----------------------------------------------------------------------
-- looper to send and read command frames
----------------------------------------------------------------------
-- 10.Jan.2026: Protocol change, switched from push-based to request-response-based parameter loading
-- This protocol ensures that the next parameter is explicitely fetched after the current one is processed
----------------------------------------------------------------------

local function updateDeviceParamCounts()
    DEVICE_PARAM_LIST_max_index = #DEVICE_PARAM_LIST
end  


local function doParamLoop()
    -- trigger getting device items and param items
    local t_10ms = getTime()
    if t_10ms - paramloop_t_last > 33 then -- was 10 = 100 ms
      paramloop_t_last = t_10ms
      if t_10ms < DEVICE_SAVE_t_last + paramLoadDeadTime_10ms then
          -- skip, we don't send a cmd if the last Save was recent
          -- TODO: we could make deadtime dependend on whether a receiver was connected before the Save
      elseif DEVICE_ITEM_TX == nil then
          cmdPush(MBRIDGE_CMD.REQUEST_INFO, {}) -- triggers sending DEVICE_ITEM_TX, DEVICE_ITEM_RX, DEVICE_INFO
          --cmdPush(MBRIDGE_CMD.REQUEST_CMD, {MBRIDGE_CMD.REQUEST_INFO) -- alternative command for the same effect
          -- these should have been set when we nil-ed DEVICE_PARAM_LIST
          DEVICE_PARAM_LIST_expected_index = 0
          DEVICE_PARAM_LIST_current_index = -1
          DEVICE_PARAM_LIST_errors = 0
          DEVICE_PARAM_LIST_complete = false
      elseif DEVICE_PARAM_LIST == nil then
          if DEVICE_INFO ~= nil then -- wait for DEVICE_INFO to be populated, indicates that MBRIDGE_CMD.REQUEST_INFO is completed
              DEVICE_PARAM_LIST = {}
              -- Old:
              --cmdPush(MBRIDGE_CMD.PARAM_REQUEST_LIST, {}) -- triggers sending full list of PARAM_ITEMs
              --cmdPush(MBRIDGE_CMD.REQUEST_CMD, {MBRIDGE_CMD.PARAM_REQUEST_LIST}) -- alternative command for the same effect
              -- New: request parameters by index (request-response protocol)
              -- request first parameter by index (is index = 0)
              cmdPush(MBRIDGE_CMD.REQUEST_CMD, {MBRIDGE_CMD.PARAM_ITEM, DEVICE_PARAM_LIST_expected_index})
          end
      end
    end

    -- handle received commands
    for ijk = 1,24 do -- handle only up to 24 per lua cycle
        local cmd = cmdPop()
        if cmd == nil then break end
        if cmd.cmd == MBRIDGE_CMD.DEVICE_ITEM_TX then
            -- MBRIDGE_CMD.DEVICE_ITEM_TX
            DEVICE_ITEM_TX = cmd
            DEVICE_ITEM_TX.version_u16 = mb_to_u16(cmd.payload, 0)
            DEVICE_ITEM_TX.setuplayout_u16 = mb_to_u16(cmd.payload, 2)
            DEVICE_ITEM_TX.name = mb_to_string(cmd.payload, 4, 20)
            DEVICE_ITEM_TX.version_int = mb_to_version_int(DEVICE_ITEM_TX.version_u16)
            DEVICE_ITEM_TX.version_str = mb_to_version_string(DEVICE_ITEM_TX.version_u16)
            DEVICE_ITEM_TX.setuplayout_int = mb_to_version_int(DEVICE_ITEM_TX.setuplayout_u16)
        elseif cmd.cmd == MBRIDGE_CMD.DEVICE_ITEM_RX then
            -- MBRIDGE_CMD.DEVICE_ITEM_RX
            DEVICE_ITEM_RX = cmd
            DEVICE_ITEM_RX.version_u16 = mb_to_u16(cmd.payload, 0)
            DEVICE_ITEM_RX.setuplayout_u16 = mb_to_u16(cmd.payload, 2)
            DEVICE_ITEM_RX.name = mb_to_string(cmd.payload, 4, 20)
            DEVICE_ITEM_RX.version_int = mb_to_version_int(DEVICE_ITEM_RX.version_u16)
            DEVICE_ITEM_RX.version_str = mb_to_version_string(DEVICE_ITEM_RX.version_u16)
            DEVICE_ITEM_RX.setuplayout_int = mb_to_version_int(DEVICE_ITEM_RX.setuplayout_u16)
        elseif cmd.cmd == MBRIDGE_CMD.INFO then
            -- MBRIDGE_CMD.INFO
            DEVICE_INFO = cmd
            DEVICE_INFO.receiver_sensitivity = mb_to_i16(cmd.payload, 0)
            DEVICE_INFO.has_status = mb_to_u8_bits(cmd.payload, 2, 0, 0x01)
            DEVICE_INFO.binding = mb_to_u8_bits(cmd.payload, 2, 1, 0x01)
            DEVICE_INFO.LQ_low = 0 -- mb_to_u8_bits(cmd.payload, 2, 3, 0x03)
            DEVICE_INFO.tx_power_dbm = mb_to_i8(cmd.payload, 3)
            DEVICE_INFO.rx_power_dbm = mb_to_i8(cmd.payload, 4)
            DEVICE_INFO.rx_available = mb_to_u8_bits(cmd.payload, 5, 0, 0x1)
            --DEVICE_INFO.tx_diversity = mb_to_u8_bits(cmd.payload, 5, 1, 0x3)
            --DEVICE_INFO.rx_diversity = mb_to_u8_bits(cmd.payload, 5, 3, 0x3)
            DEVICE_INFO.tx_config_id = mb_to_u8(cmd.payload, 6)
            DEVICE_INFO.tx_diversity = mb_to_u8_bits(cmd.payload, 7, 0, 0x0F)
            DEVICE_INFO.rx_diversity = mb_to_u8_bits(cmd.payload, 7, 4, 0x0F)
        elseif cmd.cmd == MBRIDGE_CMD.PARAM_ITEM then
            -- MBRIDGE_CMD.PARAM_ITEM
            local index = cmd.payload[0]
            if index ~= DEVICE_PARAM_LIST_expected_index and index ~= 255 then
                paramsError()
            end
            DEVICE_PARAM_LIST_current_index = index -- inform potential Item2/3/4 calls
            DEVICE_PARAM_LIST_expected_index = index + 1 -- prepare for next
            if DEVICE_PARAM_LIST == nil then
                paramsError()
            elseif index < 128 then
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
            elseif index == 255 then -- EOL (end of list)
                if DEVICE_PARAM_LIST_errors == 0 then
                    updateDeviceParamCounts()
                    DEVICE_PARAM_LIST_complete = true
                else
                    -- Huston, we have a problem
                    DEVICE_PARAM_LIST_complete = false
                    setPopupWTmo("Param Upload Errors ("..tostring(DEVICE_PARAM_LIST_errors)..")!\nTry Reload", 200)
                end
                DEVICE_DOWNLOAD_is_running = false
            else
                paramsError()
            end
        elseif cmd.cmd == MBRIDGE_CMD.PARAM_ITEM2 then
            -- MBRIDGE_CMD.PARAM_ITEM2
            local index = cmd.payload[0]
            if index ~= DEVICE_PARAM_LIST_current_index then
                paramsError()
            elseif DEVICE_PARAM_LIST == nil or DEVICE_PARAM_LIST[index] == nil then
                paramsError()
            else
                local item3_needed = false
                if DEVICE_PARAM_LIST[index].typ < MBRIDGE_PARAM_TYPE.LIST then
                    DEVICE_PARAM_LIST[index].min = mb_to_value(cmd.payload, 1, DEVICE_PARAM_LIST[index].typ)
                    DEVICE_PARAM_LIST[index].max = mb_to_value(cmd.payload, 3, DEVICE_PARAM_LIST[index].typ)
                    DEVICE_PARAM_LIST[index].unit = mb_to_string(cmd.payload, 7, 6)
                elseif DEVICE_PARAM_LIST[index].typ == MBRIDGE_PARAM_TYPE.LIST then
                    DEVICE_PARAM_LIST[index].allowed_mask = mb_to_u16(cmd.payload, 1)
                    DEVICE_PARAM_LIST[index].options = mb_to_options(cmd.payload, 3, 21)
                    DEVICE_PARAM_LIST[index].item2payload = cmd.payload
                    DEVICE_PARAM_LIST[index].min = 0
                    DEVICE_PARAM_LIST[index].max = #DEVICE_PARAM_LIST[index].options - 1
                    DEVICE_PARAM_LIST[index].editable = mb_allowed_mask_editable(DEVICE_PARAM_LIST[index].allowed_mask)
                    -- determine if we should expect an ITEM3
                    local s = mb_to_string(cmd.payload, 3, 21)
                    if string.len(s) == 21 then item3_needed = true end
                elseif DEVICE_PARAM_LIST[index].typ == MBRIDGE_PARAM_TYPE.STR6 then
                    -- nothing to do, is send but hasn't any content
                else
                    paramsError()
                end
                if not item3_needed then
                    -- request next parameter by index
                    cmdPush(MBRIDGE_CMD.REQUEST_CMD, {MBRIDGE_CMD.PARAM_ITEM, DEVICE_PARAM_LIST_expected_index})
                end
            end
        elseif cmd.cmd == MBRIDGE_CMD.PARAM_ITEM3_4 then -- can be ITEM3 or ITEM4
            -- MBRIDGE_CMD.PARAM_ITEM3_4
            local index = cmd.payload[0]
            local is_item4 = false
            if (index >= 128) then -- this is actually ITEM4
                index = index - 128;
                is_item4 = true
            end
            if index ~= DEVICE_PARAM_LIST_current_index then
                paramsError()
            elseif DEVICE_PARAM_LIST == nil or DEVICE_PARAM_LIST[index] == nil then
                paramsError()
            elseif DEVICE_PARAM_LIST[index].typ ~= MBRIDGE_PARAM_TYPE.LIST then
                paramsError()
            elseif DEVICE_PARAM_LIST[index].item2payload == nil then
                paramsError()
            elseif is_item4 and DEVICE_PARAM_LIST[index].item3payload == nil then
                paramsError()
            else
                local s = DEVICE_PARAM_LIST[index].item2payload
                local item4_needed = false
                if not is_item4 then
                    DEVICE_PARAM_LIST[index].item3payload = cmd.payload
                    for i=1,23 do s[23+i] = cmd.payload[i] end
                    DEVICE_PARAM_LIST[index].options = mb_to_options(s, 3, 21+23)
                    -- determine if we should expect an ITEM4
                    local opts = mb_to_string(cmd.payload, 1, 23)
                    if string.len(opts) == 23 then item4_needed = true end
                else
                    local s3 = DEVICE_PARAM_LIST[index].item3payload
                    for i=1,23 do s[23+i] = s3[i]; s[23+23+i] = cmd.payload[i]; end
                    DEVICE_PARAM_LIST[index].options = mb_to_options(s, 3, 21+23+23)
                end
                DEVICE_PARAM_LIST[index].max = #DEVICE_PARAM_LIST[index].options - 1
                s = nil
                if not item4_needed then
                    -- request next parameter by index
                    cmdPush(MBRIDGE_CMD.REQUEST_CMD, {MBRIDGE_CMD.PARAM_ITEM, DEVICE_PARAM_LIST_expected_index})
                end
            end
        end
        cmd = nil
    end --for
end


local function sendParamSet(idx)
    if not DEVICE_PARAM_LIST_complete then return end -- needed here??
    local p = DEVICE_PARAM_LIST[idx]
    if p.typ < MBRIDGE_PARAM_TYPE.LIST then
        cmdPush(MBRIDGE_CMD.PARAM_SET, {idx, p.value})
    elseif p.typ == MBRIDGE_PARAM_TYPE.LIST then
        cmdPush(MBRIDGE_CMD.PARAM_SET, {idx, p.value})
    elseif p.typ == MBRIDGE_PARAM_TYPE.STR6 then
        local cmd = {idx}
        for i = 1,6 do
            cmd[i+1] = string.byte(string.sub(p.value, i,i))
        end
        cmdPush(MBRIDGE_CMD.PARAM_SET, cmd)
    end
end


local function sendParamStore()
    if not DEVICE_PARAM_LIST_complete then return end -- needed here??
    cmdPush(MBRIDGE_CMD.PARAM_STORE, {})
    DEVICE_SAVE_t_last = getTime()
    setPopupWTmo("Save Parameters", 250)
end


local function sendBind()
    --if not DEVICE_PARAM_LIST_complete then return end -- needed here??
    if DEVICE_DOWNLOAD_is_running then return end
    cmdPush(MBRIDGE_CMD.BIND_START, {})
    setPopupBlocked("Binding")
end


local function sendBoot()
    --if not DEVICE_PARAM_LIST_complete then return end -- needed here??
    if DEVICE_DOWNLOAD_is_running then return end
    cmdPush(MBRIDGE_CMD.SYSTEM_BOOTLOADER, {})
    setPopupBlocked("In System Bootloader")
end


local function sendFlashEsp()
    --if not DEVICE_PARAM_LIST_complete then return end -- needed here??
    if DEVICE_DOWNLOAD_is_running then return end
    cmdPush(MBRIDGE_CMD.FLASH_ESP, {})
    setPopupBlocked("In Flash ESP")
end


local function checkBind()
    if DEVICE_DOWNLOAD_is_running then return end
    if DEVICE_INFO ~= nil and DEVICE_INFO.has_status == 1 and DEVICE_INFO.binding == 1 then
        setPopupBlocked("Binding")
    end
end


----------------------------------------------------------------------
-- Edit stuff
----------------------------------------------------------------------

local PAGENR = {
    MAIN = 0,
    EDIT_TX = 1,
    EDIT_RX = 2,
    TOOLS = 3,
}

local IDX = {
    -- Page Main, idxes of options on main page
    BindPhrase_idx = 0,
    Mode_idx = 1,
    RFBand_idx = 2,
    RFOrtho_idx = 3,
    EditTx_idx = 4,
    EditRx_idx = 5,
    Save_idx = 6,
    Reload_idx = 7,
    Bind_idx = 8,
    Tools_idx = 9,
    MAIN_CURSOR_IDX_MAX = 9,
    COMMON_PARAM_IDX_MAX = 3, -- note: for common parameters it is assumed that cursor idx = param idx
    -- Page Tools, idxes of options on tools page
    Tools_Boot_idx = 0,
    Tools_FlashEsp_idx = 1,
    TOOLS_CURSOR_IDX_MAX = 1,
}

local CURSOR = {
    page_nr = PAGENR.MAIN, -- 0: main, 1: edit Tx, 2: edit Rx, 3: tools
    idx = IDX.EditTx_idx,
    edit = false,
    pidx = 0, -- parameter idx which corresponds to the current CURSOR.idx
    param_cnt = 0, -- number of parameters available on page
    sidx = 0, -- index into string for string edits
    top_idx = 0, -- index of first displayed option on Edit Tx/Rx pages
}
  
local bindphrase_chars = "abcdefghijklmnopqrstuvwxyz0123456789_#-."


local function cur_attr(idx) -- used in menu
    local attr = THEME.textColor
    if CURSOR.idx == idx then
        attr = attr + INVERS
        if CURSOR.edit then attr = attr + BLINK end
    end
    return attr
end


local function cur_attr_x(idx, sidx) -- for Bind Phrase character editing
    local attr = THEME.textColor
    if DEVICE_PARAM_LIST_complete and CURSOR.idx == idx then
        if CURSOR.edit then
            if CURSOR.sidx == sidx then attr = attr + BLINK + INVERS end
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
    if p.typ < MBRIDGE_PARAM_TYPE.LIST then
        p.value = p.value + 1
    elseif p.typ == MBRIDGE_PARAM_TYPE.LIST then
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
    if p.typ < MBRIDGE_PARAM_TYPE.LIST then
        p.value = p.value - 1
    elseif p.typ == MBRIDGE_PARAM_TYPE.LIST then
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
    if p.typ == MBRIDGE_PARAM_TYPE.STR6 then
        local c = string.sub(p.value, CURSOR.sidx+1, CURSOR.sidx+1)
        local i = string.find(bindphrase_chars, c, 1, true) -- true for plain search
        i = i + 1
        if i > string.len(bindphrase_chars) then i = 1 end
        c = string.sub(bindphrase_chars, i,i)
        p.value = string.sub(p.value, 1, CURSOR.sidx) .. c .. string.sub(p.value, CURSOR.sidx+2, string.len(p.value))
    end
    DEVICE_PARAM_LIST[idx].value = p.value
end


local function param_str6_dec(idx)
    if not DEVICE_PARAM_LIST_complete then return end -- needed here??
    local p = DEVICE_PARAM_LIST[idx]
    if p.typ == MBRIDGE_PARAM_TYPE.STR6 then
        local c = string.sub(p.value, CURSOR.sidx+1, CURSOR.sidx+1)
        local i = string.find(bindphrase_chars, c, 1, true) -- true for plain search
        i = i - 1
        if i < 1 then i = string.len(bindphrase_chars) end
        c = string.sub(bindphrase_chars, i,i)
        p.value = string.sub(p.value, 1, CURSOR.sidx) .. c .. string.sub(p.value, CURSOR.sidx+2, string.len(p.value))
    end
    DEVICE_PARAM_LIST[idx].value = p.value
end


local function param_str6_next(idx)
    if not DEVICE_PARAM_LIST_complete then return false end -- needed here??
    local p = DEVICE_PARAM_LIST[idx]
    if p.typ == MBRIDGE_PARAM_TYPE.STR6 then
        CURSOR.sidx = CURSOR.sidx + 1
        if CURSOR.sidx >= string.len(p.value) then
            return true -- last char
        end
    end
    return false
end


local function param_focusable(idx)
    if not DEVICE_PARAM_LIST_complete then return false end -- needed here??
    local p = DEVICE_PARAM_LIST[idx]
    if p == nil then return false end
    if p.editable == nil then return false end
    return p.editable
end


----------------------------------------------------------------------
-- Page Edit Tx/Rx
----------------------------------------------------------------------

local function drawPageEdit(page_str)
    local y = 35
    if page_str == "Tx" then
        if DEVICE_INFO ~= nil then
            lcd.drawText(5, y, "Tx - "..tostring(DEVICE_INFO.tx_config_id)..":", THEME.textColor)
        else
            lcd.drawText(5, y, "Tx - ?:", THEME.textColor)
        end
    else
        lcd.drawText(5, y, page_str..":", THEME.textColor)
    end

    y = 60
    local y0 = y

    if CURSOR.idx < CURSOR.top_idx then CURSOR.top_idx = CURSOR.idx end
    if CURSOR.idx >= CURSOR.top_idx + LAYOUT.page_N then CURSOR.top_idx = CURSOR.idx - LAYOUT.page_N + 1 end

    local idx = 0
    CURSOR.param_cnt = 0
    for pidx = 2, #DEVICE_PARAM_LIST do
        local p = DEVICE_PARAM_LIST[pidx]
        if p ~= nil and string.sub(p.name,1,2) == page_str and p.allowed_mask > 0 then
        local name = string.sub(p.name, 4)

        if idx >= CURSOR.top_idx and idx < CURSOR.top_idx + LAYOUT.page_N then
            local shifted_idx = idx - CURSOR.top_idx

            y = y0 + shifted_idx * LAYOUT.DY

            local xofs = 0
            if shifted_idx >= LAYOUT.page_N1 then y = y - LAYOUT.page_N1 * LAYOUT.DY; xofs = 230 end

            lcd.drawText(10+xofs, y, name, THEME.textColor)
            if p.typ < MBRIDGE_PARAM_TYPE.LIST then
                lcd.drawText(140+xofs, y, p.value.." "..p.unit, cur_attr_p(idx, pidx))
            elseif p.typ == MBRIDGE_PARAM_TYPE.LIST then
                lcd.drawText(140+xofs, y, p.options[p.value+1], cur_attr_p(idx, pidx))
            end
        end

        if CURSOR.idx == idx then CURSOR.pidx = pidx end

        idx = idx + 1
      end
    end

    CURSOR.param_cnt = idx

    local x_mid = LAYOUT.W_HALF - 5
    if CURSOR.top_idx > 0 then
        local y_base = y0 - 4
        --lcd.drawFilledTriangle(x_mid-6, y_base, x_mid+6, y_base, x_mid, y_base-6, THEME.textColor)
        drawFilledTriangle(x_mid-6, y_base, x_mid+6, y_base, x_mid, y_base-6, THEME.textColor)
    end
    if CURSOR.param_cnt > CURSOR.top_idx + LAYOUT.page_N then
        local y_base = y0 + LAYOUT.page_N1 * LAYOUT.DY + 4
        --lcd.drawFilledTriangle(x_mid-6, y_base, x_mid+6, y_base, x_mid, y_base+6, THEME.textColor)
        drawFilledTriangle(x_mid-6, y_base, x_mid+6, y_base, x_mid, y_base+6, THEME.textColor)
    end
end


local function doPageEdit(event, page_str)
    lcd.drawFilledRectangle(0, 0, LAYOUT.W, 30, THEME.titleBgColor)
    lcd.drawText(5, 5, "mLRS Configurator: Edit "..page_str, THEME.menuTitleColor)

    if event == EVT_VIRTUAL_EXIT and not CURSOR.edit then
        CURSOR.page_nr = PAGENR.MAIN
        CURSOR.idx = IDX.EditTx_idx
        return
    end

    drawPageEdit(page_str) -- call before event handling, ensures that CURSOR.param_cnt, CURSOR.pidx are set

    if not CURSOR.edit then
        if event == EVT_VIRTUAL_EXIT then
        elseif event == EVT_VIRTUAL_ENTER then
            CURSOR.edit = true
        elseif event == EVT_VIRTUAL_NEXT then
            CURSOR.idx = CURSOR.idx + 1
            if CURSOR.idx >= CURSOR.param_cnt then CURSOR.idx = CURSOR.param_cnt - 1 end
        elseif event == EVT_VIRTUAL_PREV then
            CURSOR.idx = CURSOR.idx - 1
            if CURSOR.idx < 0 then CURSOR.idx = 0 end
        end
    else
        if event == EVT_VIRTUAL_EXIT then
            sendParamSet(CURSOR.pidx)
            CURSOR.edit = false
        elseif event == EVT_VIRTUAL_ENTER then
            sendParamSet(CURSOR.pidx)
            CURSOR.edit = false
        elseif event == EVT_VIRTUAL_NEXT then
            param_value_inc(CURSOR.pidx)
        elseif event == EVT_VIRTUAL_PREV then
            param_value_dec(CURSOR.pidx)
        end
    end

end


----------------------------------------------------------------------
-- Page Tools
----------------------------------------------------------------------

local function drawPageTools()
    local x, y;

    y = 60
    lcd.drawText(10, y, "System Bootloader", cur_attr(IDX.Tools_Boot_idx))
    y = y + LAYOUT.DY
    lcd.drawText(10, y, "Flash ESP", cur_attr(IDX.Tools_FlashEsp_idx))
end


local function doPageTools(event)
    lcd.drawFilledRectangle(0, 0, LAYOUT.W, 30, THEME.titleBgColor)
    lcd.drawText(5, 5, "mLRS Configurator: Tools Page", THEME.menuTitleColor)

    drawPageTools()

    if event == EVT_VIRTUAL_ENTER then
        if CURSOR.idx == IDX.Tools_Boot_idx then -- Boot
            CURSOR.page_nr = PAGENR.MAIN
            CURSOR.idx = IDX.EditTx_idx
            sendBoot()
        elseif CURSOR.idx == IDX.Tools_FlashEsp_idx then -- Flash ESP
            CURSOR.page_nr = PAGENR.MAIN
            CURSOR.idx = IDX.EditTx_idx
            sendFlashEsp()
        end
    elseif event == EVT_VIRTUAL_EXIT then
        CURSOR.page_nr = PAGENR.MAIN
        CURSOR.idx = IDX.EditTx_idx
        return
    elseif event == EVT_VIRTUAL_NEXT then
        CURSOR.idx = CURSOR.idx + 1
        if CURSOR.idx > IDX.TOOLS_CURSOR_IDX_MAX then CURSOR.idx = IDX.TOOLS_CURSOR_IDX_MAX end
    elseif event == EVT_VIRTUAL_PREV then
        CURSOR.idx = CURSOR.idx - 1
        if CURSOR.idx < 0 then CURSOR.idx = 0 end
    end
end


----------------------------------------------------------------------
-- Page Main
----------------------------------------------------------------------

local isEdgeTx = false
local isFirstParamDownload = true


local function drawSetuplayoutWarning(txt)
    local y = LAYOUT.INFO_Y
    lcd.drawFilledRectangle(LAYOUT.WARN_X-2, y, LAYOUT.WARN_W+4, LAYOUT.WARN_H+4, THEME.textColor) --TITLE_BGCOLOR)
    lcd.drawFilledRectangle(LAYOUT.WARN_X, y+2, LAYOUT.WARN_W, LAYOUT.WARN_H, THEME.titleBgColor) --TEXT_BGCOLOR) --TITLE_BGCOLOR)
    local attr = THEME.menuTitleColor+CENTER
    local s = txt
    local i = string.find(s, "\n")
    local lines = 0
    while i ~= nil do
        local s1 = string.sub(s, 1,i-1)
        lcd.drawText(LAYOUT.W_HALF, y+6 + lines, s1, attr)
        s = string.sub(s, i+1)
        i = string.find(s, "\n")
        lines = lines + LAYOUT.DY
    end
    lcd.drawText(LAYOUT.W_HALF, y+6 + lines, s, attr)
end


local function drawParamDownload()
    local y = LAYOUT.INFO_Y
    lcd.drawText(130, y+LAYOUT.INFO_DY, "parameters loading ...", THEME.textColor+BLINK+INVERS)
    local idx = DEVICE_PARAM_LIST_current_index
    if idx < 0 then idx = 0 end
    -- lcd.drawText(330, y+LAYOUT.INFO_DY, "("..tostring(idx)..")", THEME.textColor)
    local s = "("..tostring(idx)
    if DEVICE_PARAM_LIST_max_index > 0 then
        s = s.."/"..tostring(DEVICE_PARAM_LIST_max_index)
    end    
    s = s..")"
    if isEdgeTx then -- on OTX TEXT_BGCOLOR doesn't seem to work correctly
        lcd.setColor(CUSTOM_COLOR, THEME.textBgColor)
        lcd.drawFilledRectangle(320, y+LAYOUT.INFO_DY-5, 70, 30, CUSTOM_COLOR)
    end    
    lcd.drawText(330, y+LAYOUT.INFO_DY, s, THEME.textColor)
end


local function drawPageMain()
    lcd.setColor(CUSTOM_COLOR, RED)

    local y = 35
    lcd.drawText(5, y, "Tx:", THEME.textColor)
    if DEVICE_ITEM_TX == nil then
        lcd.drawText(35, y, "---", THEME.textColor)
    else
        lcd.drawText(35, y, DEVICE_ITEM_TX.name, THEME.textColor+SMLSIZE)
        lcd.drawText(35, y+16, DEVICE_ITEM_TX.version_str, THEME.textColor+SMLSIZE)
        if DEVICE_INFO ~= nil then
            lcd.drawText(35, y+32, "ConfigId "..tostring(DEVICE_INFO.tx_config_id), THEME.textColor+SMLSIZE)
        end
    end

    lcd.drawText(LAYOUT.W_HALF, y, "Rx:", THEME.textColor)
    if not DEVICE_PARAM_LIST_complete then
        -- don't do anything
    elseif not connected then
        lcd.drawText(LAYOUT.W_HALF+30, y, "not connected", THEME.textColor)
    elseif DEVICE_ITEM_RX == nil then
        lcd.drawText(LAYOUT.W_HALF+30, y, "---", THEME.textColor)
    else
        lcd.drawText(LAYOUT.W_HALF+30, y, DEVICE_ITEM_RX.name, THEME.textColor+SMLSIZE)
        lcd.drawText(LAYOUT.W_HALF+30, y+16, DEVICE_ITEM_RX.version_str, THEME.textColor+SMLSIZE)
    end

    local version_error = false
    if DEVICE_ITEM_TX ~= nil and DEVICE_ITEM_TX.version_int < VERSION.required_tx_version_int then
        version_error = true
        POPUP.text = "Tx version not supported\nby this Lua script!"
    end
    if DEVICE_ITEM_RX ~= nil and connected and DEVICE_ITEM_RX.version_int < VERSION.required_rx_version_int then
        version_error = true
        POPUP.text = "Rx version not supported\nby this Lua script!"
    end
    if version_error then
        drawPopup()
        return
    end

    y = 95 --90
    lcd.drawText(10, y, "Bind Phrase", THEME.textColor)
    if DEVICE_PARAM_LIST_complete then
        local x = 140
        for i = 1,6 do
            local c = string.sub(DEVICE_PARAM_LIST[0].value, i, i) -- param_idx = 0 = BindPhrase
            local attr = cur_attr_x(0, i-1)
            lcd.drawText(x, y, c, attr)
            --x = x + lcd.getTextWidth(c,1,attr)+1
            x = x + getCharWidth(c) + 1
            if i == 6 and DEVICE_PARAM_LIST[2].value == 0 then -- do only for 2.4GHz band
                lcd.drawText(140 + 70, y, getExceptStrFromChar(c), THEME.textColor)
            end
        end
    end

    lcd.drawText(10, y + LAYOUT.DY, "Mode", THEME.textColor)
    if DEVICE_PARAM_LIST_complete then
        local p = DEVICE_PARAM_LIST[1] -- param_idx = 1 = Mode
        if p.options[p.value+1] ~= nil then
            lcd.drawText(140, y+LAYOUT.DY, p.options[p.value+1], cur_attr_p(IDX.Mode_idx,1))
        end
    end

    lcd.drawText(10, y + 2*LAYOUT.DY, "RF Band", THEME.textColor)
    if DEVICE_PARAM_LIST_complete then
        local p = DEVICE_PARAM_LIST[2] -- param_idx = 2 = RfBand
        if p.options[p.value+1] ~= nil then
            --lcd.drawText(240+80, y, p.options[p.value+1], cur_attr(2))
            if p.value <= #freq_band_list then
                lcd.drawText(140, y + 2*LAYOUT.DY, freq_band_list[p.value], cur_attr_p(IDX.RFBand_idx,2))
            else
                lcd.drawText(140, y + 2*LAYOUT.DY, p.options[p.value+1], cur_attr_p(IDX.RFBand_idx,2))
            end
        end
    end

    if DEVICE_PARAM_LIST_complete and DEVICE_PARAM_LIST[3].allowed_mask > 0 then
        local y_ortho = y
        if THEME.screenSize == 320240 then y_ortho = y + 42 end

        lcd.drawText(LAYOUT.W_HALF+30, y_ortho, "Ortho", THEME.textColor)
        local p = DEVICE_PARAM_LIST[3] -- param_idx = 3 = RfOrtho
        if p.options[p.value+1] ~= nil then
            lcd.drawText(LAYOUT.W_HALF+90, y_ortho, p.options[p.value+1], cur_attr_p(IDX.RFOrtho_idx,3))
        end
    end

    y = LAYOUT.BUTTONS_Y
    lcd.drawText(LAYOUT.EDIT_TX_X, y, "Edit Tx", cur_attr(IDX.EditTx_idx))
    if not connected then
        lcd.drawText(LAYOUT.EDIT_RX_X, y, "Edit Rx", THEME.textDisableColor)
    else
        lcd.drawText(LAYOUT.EDIT_RX_X, y, "Edit Rx", cur_attr(IDX.EditRx_idx))
    end
    lcd.drawText(LAYOUT.SAVE_X, y, "Save", cur_attr(IDX.Save_idx))
    lcd.drawText(LAYOUT.RELOAD_X, y, "Reload", cur_attr(IDX.Reload_idx))
    lcd.drawText(LAYOUT.BIND_X, y, "Bind", cur_attr(IDX.Bind_idx))
    lcd.drawText(LAYOUT.TOOLS_X, y, "Tools", cur_attr(IDX.Tools_idx))

    -- show overview of some selected parameters
    y = LAYOUT.INFO_Y
    lcd.setColor(CUSTOM_COLOR, GREY)
    lcd.drawFilledRectangle(0, y-5, LAYOUT.W, 1, CUSTOM_COLOR)
    
    if DEVICE_DOWNLOAD_is_running then
        drawParamDownload()
        return
    end    

    lcd.drawText(LAYOUT.INFO_LEFT_X, y, "Tx Power", THEME.textColor)
    lcd.drawText(LAYOUT.INFO_LEFT_X, y+LAYOUT.INFO_DY, "Tx Diversity", THEME.textColor)
    if DEVICE_INFO ~= nil then
        lcd.drawText(LAYOUT.INFO_LEFT_VAL_X, y, tostring(DEVICE_INFO.tx_power_dbm).." dBm", THEME.textColor)
        if DEVICE_INFO.tx_diversity <= #diversity_list then
            lcd.drawText(LAYOUT.INFO_LEFT_VAL_X, y+LAYOUT.INFO_DY, diversity_list[DEVICE_INFO.tx_diversity], THEME.textColor)
        else
            lcd.drawText(LAYOUT.INFO_LEFT_VAL_X, y+LAYOUT.INFO_DY, "?", THEME.textColor)
        end
    else
        lcd.drawText(LAYOUT.INFO_LEFT_VAL_X, y, "---", THEME.textColor)
        lcd.drawText(LAYOUT.INFO_LEFT_VAL_X, y+LAYOUT.INFO_DY, "---", THEME.textColor)
    end

    local rx_attr = THEME.textColor
    if not connected then
        rx_attr = THEME.textDisableColor
    end
    lcd.drawText(LAYOUT.INFO_RIGHT_X, y, "Rx Power", rx_attr)
    lcd.drawText(LAYOUT.INFO_RIGHT_X, y+LAYOUT.INFO_DY, "Rx Diversity", rx_attr)
    if DEVICE_INFO ~= nil and connected then
        lcd.drawText(LAYOUT.INFO_RIGHT_VALUE_X, y, tostring(DEVICE_INFO.rx_power_dbm).." dBm", rx_attr)
        if DEVICE_INFO.rx_diversity <= #diversity_list then
            lcd.drawText(LAYOUT.INFO_RIGHT_VALUE_X, y+LAYOUT.INFO_DY, diversity_list[DEVICE_INFO.rx_diversity], rx_attr)
        else
            lcd.drawText(LAYOUT.INFO_RIGHT_VALUE_X, y+LAYOUT.INFO_DY, "?", rx_attr)
        end
    else
        lcd.drawText(LAYOUT.INFO_RIGHT_VALUE_X, y, "---", rx_attr)
        lcd.drawText(LAYOUT.INFO_RIGHT_VALUE_X, y+LAYOUT.INFO_DY, "---", rx_attr)
    end

    y = y + 2*LAYOUT.INFO_DY
    lcd.drawText(LAYOUT.INFO_LEFT_X, y, "Sensitivity", THEME.textColor)
    if DEVICE_INFO ~= nil then
        lcd.drawText(LAYOUT.INFO_LEFT_VAL_X, y, tostring(DEVICE_INFO.receiver_sensitivity).." dBm", THEME.textColor)
    else
        lcd.drawText(LAYOUT.INFO_LEFT_VAL_X, y, "---", THEME.textColor)
    end

    -- setup layout warning
    if DEVICE_ITEM_TX ~= nil and DEVICE_ITEM_RX ~= nil and connected and DEVICE_PARAM_LIST_complete and
           (DEVICE_ITEM_TX.setuplayout_int > 515 or DEVICE_ITEM_RX.setuplayout_int > 515) then -- 515 is old 335
        if DEVICE_ITEM_TX.setuplayout_int < DEVICE_ITEM_RX.setuplayout_int then
            drawSetuplayoutWarning("Tx param version smaller than Rx param version.\nPlease update Tx firmware!")
        elseif DEVICE_ITEM_RX.setuplayout_int < DEVICE_ITEM_TX.setuplayout_int then
            drawSetuplayoutWarning("Rx param version smaller than Tx param version.\nPlease update Rx firmware!")
        end
    end
end


local function doPageMain(event)
    lcd.drawFilledRectangle(0, 0, LAYOUT.W, 30, THEME.titleBgColor)
    lcd.drawText(5, 5, "mLRS Configurator: Main Page", THEME.menuTitleColor)
    lcd.drawText(LAYOUT.W-1, 0, VERSION.script, THEME.menuTitleColor+TINSIZE+RIGHT)

    if not CURSOR.edit then
        if event == EVT_VIRTUAL_EXIT then
            -- nothing to do
        elseif event == EVT_VIRTUAL_ENTER then
            if CURSOR.idx == IDX.EditTx_idx and DEVICE_PARAM_LIST_complete then -- EditTX pressed
                CURSOR.page_nr = PAGENR.EDIT_TX
                CURSOR.idx = 0
                CURSOR.top_idx = 0
                return
            elseif CURSOR.idx == IDX.EditRx_idx and DEVICE_PARAM_LIST_complete then -- EditRX pressed
                CURSOR.page_nr = PAGENR.EDIT_RX
                CURSOR.idx = 0
                CURSOR.top_idx = 0
                return
            elseif CURSOR.idx == IDX.Save_idx and DEVICE_PARAM_LIST_complete then -- Save pressed
                sendParamStore()
                clearParams()
            elseif CURSOR.idx == IDX.Reload_idx then -- Reload pressed
                clearParams()
            elseif CURSOR.idx == IDX.Bind_idx then -- Bind pressed
                sendBind()
            elseif CURSOR.idx == IDX.Tools_idx and DEVICE_PARAM_LIST_complete then -- Tools pressed
                CURSOR.page_nr = PAGENR.TOOLS
                CURSOR.idx = 0
                CURSOR.top_idx = 0
                return
            elseif DEVICE_PARAM_LIST_complete then -- edit option
                CURSOR.sidx = 0
                CURSOR.edit = true
            end
        elseif event == EVT_VIRTUAL_NEXT then -- and DEVICE_PARAM_LIST_complete then
            CURSOR.idx = CURSOR.idx + 1
            if CURSOR.idx > IDX.MAIN_CURSOR_IDX_MAX then CURSOR.idx = IDX.MAIN_CURSOR_IDX_MAX end

            if CURSOR.idx == IDX.Mode_idx and not param_focusable(1) then CURSOR.idx = CURSOR.idx + 1 end
            if CURSOR.idx == IDX.RFBand_idx and not param_focusable(2) then CURSOR.idx = CURSOR.idx + 1 end
            if CURSOR.idx == IDX.RFOrtho_idx and not param_focusable(3) then CURSOR.idx = CURSOR.idx + 1 end
            if CURSOR.idx == IDX.EditRx_idx and not connected then CURSOR.idx = CURSOR.idx + 1 end
        elseif event == EVT_VIRTUAL_PREV then -- and DEVICE_PARAM_LIST_complete then
            CURSOR.idx = CURSOR.idx - 1
            if CURSOR.idx < 0 then CURSOR.idx = 0 end

            if CURSOR.idx == IDX.EditRx_idx and not connected then CURSOR.idx = CURSOR.idx - 1 end
            if CURSOR.idx == IDX.RFOrtho_idx and not param_focusable(3) then CURSOR.idx = CURSOR.idx - 1 end
            if CURSOR.idx == IDX.RFBand_idx and not param_focusable(2) then CURSOR.idx = CURSOR.idx - 1 end
            if CURSOR.idx == IDX.Mode_idx and not param_focusable(1) then CURSOR.idx = CURSOR.idx - 1 end
        end
    else
        if event == EVT_VIRTUAL_EXIT then
            if CURSOR.idx <= IDX.COMMON_PARAM_IDX_MAX then -- BindPhrase, Mode, RF Band, RF Ortho
                sendParamSet(CURSOR.idx)
            end
            CURSOR.edit = false
        elseif event == EVT_VIRTUAL_ENTER then
            if CURSOR.idx == IDX.BindPhrase_idx then -- BindPhrase
                if param_str6_next(0) then
                    sendParamSet(0)
                    CURSOR.edit = false
                end
            elseif CURSOR.idx <= IDX.COMMON_PARAM_IDX_MAX then -- Mode, RF Band, RF Ortho
                sendParamSet(CURSOR.idx)
                CURSOR.edit = false
            else
                CURSOR.edit = false
            end
        elseif event == EVT_VIRTUAL_NEXT then
            if CURSOR.idx == IDX.BindPhrase_idx then -- BindPhrase
                param_str6_inc(0)
            elseif CURSOR.idx <= IDX.COMMON_PARAM_IDX_MAX then -- Mode, RF Band, RF Ortho
                param_value_inc(CURSOR.idx)
            end
        elseif event == EVT_VIRTUAL_PREV then
            if CURSOR.idx == IDX.BindPhrase_idx then -- BindPhrase
                param_str6_dec(0)
            elseif CURSOR.idx <= IDX.COMMON_PARAM_IDX_MAX then -- Mode, RF Band, RF Ortho
                param_value_dec(CURSOR.idx)
            end
        end
    end

    drawPageMain()
end


----------------------------------------------------------------------
----------------------------------------------------------------------

local function Do(event)
    doConnected()

    if has_connected then
        clearParams()
        if not POPUP.active then setPopupWTmo("Receiver connected!", 100) end
        clearPopupIfBlocked()
    end
    if has_disconnected then
        if not POPUP.active then setPopupWTmo("Receiver\nhas disconnected!", 100) end
    end
    if not connected and CURSOR.page_nr == PAGENR.EDIT_RX then
        CURSOR.page_nr = PAGENR.MAIN
        CURSOR.idx = IDX.EditTx_idx
    end

    doParamLoop()

    -- OpenTx: must display
    -- EdgeTx: don't display everything in param upload, EdgeTx is super slow
    if isEdgeTx and DEVICE_DOWNLOAD_is_running then
        if not isFirstParamDownload then drawParamDownload(); end
        return 
    end
    isFirstParamDownload = false

    lcd.clear()

    if CURSOR.page_nr == PAGENR.EDIT_TX then
        doPageEdit(event,"Tx")
    elseif CURSOR.page_nr == PAGENR.EDIT_RX then
        doPageEdit(event,"Rx")
    elseif CURSOR.page_nr == PAGENR.TOOLS then
        doPageTools(event)
    else
        doPageMain(event)
    end

    checkBind()
    doPopup()
end


----------------------------------------------------------------------
-- Script OTX Interface
----------------------------------------------------------------------

local function scriptInit()
    local ver, radio, maj, minor, rev, osname = getVersion()
    isEdgeTx = (osname == 'EdgeTX')

    setupScreen()
    setupColors()
    setupBridge()

    DEVICE_DOWNLOAD_is_running = true -- we start the script with this
    isFirstParamDownload = true
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
        if model.getModule(0).Type ~= 5 and model.getModule(1).Type ~= 5 then
            error("mLRS not accessible: mBridge or CRSF not enabled!")
            return 2
        end
    end

    if isConnected == nil or cmdPush == nil or cmdPop == nil then --just to be sure for sure
        error("Unclear issue with mBridge or CRSF!")
        return 2
    end

    if not CURSOR.edit and CURSOR.page_nr == PAGENR.MAIN then
        if event == EVT_VIRTUAL_EXIT then
            return 2
        end
    end

    Do(event)

    return 0
end

return { init=scriptInit, run=scriptRun }
