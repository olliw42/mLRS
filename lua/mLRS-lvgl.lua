--local toolName = "TNS|mLRS LVGL|TNE"
----------------------------------------------------------------------
-- Copyright (c) MLRS project
-- GPL3
-- https://www.gnu.org/licenses/gpl-3.0.de.html
-- OlliW @ www.olliw.eu
----------------------------------------------------------------------
-- Lua TOOLS script - LVGL touch UI variant
----------------------------------------------------------------------
-- copy script to SCRIPTS\TOOLS folder on the SD card
-- requires EdgeTX 2.11 or later on a color touch radio
-- supported screens: 800x480 (TX16S MK3, reference layout),
-- 480x272 (TX16S/mkII, T16, T18, Horus X10S/X12S) and
-- 480x320 (T15/T15 Pro, TX15, PL18) via a compact layout variant;
-- portrait radios (NV14/EL18) are not supported, use the classic mLRS.lua
-- UI is built on native LVGL widgets: touch everywhere, popup keyboard
-- for text entry, popup menus for list options
-- the mBridge/CRSF protocol part is identical to the classic mLRS.lua,
-- which remains the script of choice for radios without LVGL/touch
-- works with mLRS v1.3.03 and later, mOTX v33

local VERSION = {
    script = '2026-07-06 LVGL',
    required_tx_version_int = 10303,  -- 'v1.3.03'
    required_rx_version_int = 10303,  -- 'v1.3.03'
}


-- experimental
local paramLoadDeadTime_10ms = 300 -- 150 was a bit too short, also 200 was too short


----------------------------------------------------------------------
-- Color themes, switchable on the Tools page
----------------------------------------------------------------------

local C_DARK = { -- dark cards with framed input fields
    BG           = lcd.RGB(0x16, 0x1F, 0x2A), -- screen background
    HEADER       = lcd.RGB(0x22, 0x30, 0x3F), -- header bar
    CARD         = lcd.RGB(0x1C, 0x28, 0x36), -- card fill
    BORDER       = lcd.RGB(0x35, 0x50, 0x6B), -- card border
    BTN          = lcd.RGB(0x26, 0x36, 0x4A), -- button fill
    BTN_TEXT     = lcd.RGB(0xE8, 0xEE, 0xF4), -- button label
    ACCENT       = lcd.RGB(0x3B, 0x82, 0xC4), -- focused/primary
    TEXT         = lcd.RGB(0xE8, 0xEE, 0xF4),
    TEXT_DIM     = lcd.RGB(0x8F, 0xA3, 0xB5),
    TEXT_OFF     = lcd.RGB(0x5C, 0x6D, 0x7D),
    TITLE_TEXT   = lcd.RGB(0xE8, 0xEE, 0xF4), -- text on header bar and popup
    GREEN        = lcd.RGB(0x4C, 0xAF, 0x7D),
    GREY         = lcd.RGB(0x6B, 0x7C, 0x8C),
    WARN         = lcd.RGB(0xE0, 0x6C, 0x5A),
}

local C_CONTRAST = { -- high contrast: dark text on white, strong colors,
                     -- easier to read in sunlight and for older eyes
    BG           = lcd.RGB(0xFF, 0xFF, 0xFF),
    HEADER       = lcd.RGB(0x00, 0x48, 0x90),
    CARD         = lcd.RGB(0xF2, 0xF2, 0xF2),
    BORDER       = lcd.RGB(0x00, 0x00, 0x00),
    BTN          = lcd.RGB(0x35, 0x45, 0x58),
    BTN_TEXT     = lcd.RGB(0xFF, 0xFF, 0xFF),
    ACCENT       = lcd.RGB(0x00, 0x64, 0xC8),
    TEXT         = lcd.RGB(0x00, 0x00, 0x00),
    TEXT_DIM     = lcd.RGB(0x30, 0x30, 0x30),
    TEXT_OFF     = lcd.RGB(0x90, 0x90, 0x90),
    TITLE_TEXT   = lcd.RGB(0xFF, 0xFF, 0xFF),
    GREEN        = lcd.RGB(0x00, 0x80, 0x30),
    GREY         = lcd.RGB(0x80, 0x80, 0x80),
    WARN         = lcd.RGB(0xC8, 0x14, 0x00),
}

local C = C_DARK -- active palette; all build functions read through this
local theme_nr = 1

local THEME_FILE = "/SCRIPTS/TOOLS/mLRS_theme.txt"

local function applyTheme(nr)
    theme_nr = nr
    if nr == 2 then C = C_CONTRAST else C = C_DARK end
end

local function loadThemeChoice()
    local f = io.open(THEME_FILE, "r")
    if f == nil then return 1 end
    local s = io.read(f, 1)
    io.close(f)
    if s == "2" then return 2 end
    return 1
end

local function saveThemeChoice(nr)
    local f = io.open(THEME_FILE, "w")
    if f ~= nil then
        io.write(f, tostring(nr))
        io.close(f)
    end
end


----------------------------------------------------------------------
-- Screen layouts
----------------------------------------------------------------------
-- 800x480 (TX16S MK3) is the reference layout. Radios that are 480
-- wide get a compact variant with the same structure: Tx/Rx cards
-- side by side, two-column parameter card, one dense button row.
-- 480x320 reuses the compact variant with more vertical breathing room.

local L = {} -- selected once in setupLayout(), read by the build functions

local function setupLayout()
    if LCD_W >= 800 then -- 800x480, TX16S MK3
        L.TITLE_H = 44;  L.TITLE_X = 16;  L.TITLE_Y = 8;  L.TITLE_FONT = MIDSIZE
        L.VER_X = LCD_W - 170;  L.VER_Y = 14
        L.TXC_X = 12;  L.CARD_Y = 56;  L.CARD_W = 382;  L.CARD_H = 88;  L.RXC_X = 406
        L.DOT_W = 12;  L.DOT_XOFS = 18;  L.DOT_YOFS = 20
        L.TXT_XOFS = 42;  L.TXT_Y1OFS = 14;  L.TXT_Y2OFS = 44
        L.PC_X = 12;  L.PC_Y = 152;  L.PC_W = 776;  L.PC_H = 176
        L.LBL_X = 32;  L.ROW_Y1 = 170;  L.ROW_Y2 = 222;  L.ROW_Y3 = 274
        L.FLD_X = 210;  L.FLD_Y1 = 162;  L.FLD_Y2 = 214;  L.FLD_Y3 = 266;  L.FLD_W = 180
        L.HINT_X = 410;  L.ORTHO_LBL_X = 430;  L.ORTHO_FLD_X = 570
        L.LOAD_X = 300;  L.LOAD_Y = 222;  L.LOAD_FONT = MIDSIZE
        L.BTN_X0 = 12;  L.BTN_Y = 344;  L.BTN_W = 118;  L.BTN_H = 40;  L.BTN_DX = 130;  L.BTN_FONT = 0
        L.ST_Y1 = 400;  L.ST_Y2 = 424;  L.ST_RX_X = 406
        L.WARN_Y = 452;  L.STATUS_HIDES_ON_WARN = false
        L.ED_BOX_Y = 52;  L.ED_ROW_H = 52;  L.ED_LBL_X = 32;  L.ED_LBL_YOFS = 10
        L.ED_CTL_X = 400;  L.ED_CTL_W = 240
        L.BK_W = 88;  L.BK_H = 32;  L.BK_Y = 6;  L.BK_FONT = 0
        L.TL_X = 32;  L.TL_Y1 = 80;  L.TL_Y2 = 140;  L.TL_Y3 = 200;  L.TL_W = 280;  L.TL_H = 44
    else -- 480 wide, compact
        L.TITLE_H = 30;  L.TITLE_X = 8;  L.TITLE_Y = 4;  L.TITLE_FONT = 0
        L.VER_X = LCD_W - 115;  L.VER_Y = 8
        L.TXC_X = 4;  L.CARD_Y = 34;  L.CARD_W = 234;  L.CARD_H = 52;  L.RXC_X = 242
        L.DOT_W = 10;  L.DOT_XOFS = 10;  L.DOT_YOFS = 12
        L.TXT_XOFS = 26;  L.TXT_Y1OFS = 4;  L.TXT_Y2OFS = 28
        L.PC_X = 4;  L.PC_Y = 92;  L.PC_W = 472;  L.PC_H = 110
        L.LBL_X = 14;  L.ROW_Y1 = 104;  L.ROW_Y2 = 138;  L.ROW_Y3 = 172
        L.FLD_X = 112;  L.FLD_Y1 = 98;  L.FLD_Y2 = 132;  L.FLD_Y3 = 166;  L.FLD_W = 140
        L.HINT_X = 258;  L.ORTHO_LBL_X = 262;  L.ORTHO_FLD_X = 330
        L.LOAD_X = 150;  L.LOAD_Y = 132;  L.LOAD_FONT = 0
        L.BTN_X0 = 4;  L.BTN_Y = 206;  L.BTN_W = 74;  L.BTN_H = 30;  L.BTN_DX = 78;  L.BTN_FONT = SMLSIZE
        L.ST_Y1 = 240;  L.ST_Y2 = 256;  L.ST_RX_X = 242
        -- no room for a dedicated warning line: it replaces the status line
        L.WARN_Y = 240;  L.STATUS_HIDES_ON_WARN = true
        L.ED_BOX_Y = 32;  L.ED_ROW_H = 40;  L.ED_LBL_X = 12;  L.ED_LBL_YOFS = 8
        L.ED_CTL_X = 240;  L.ED_CTL_W = 200
        L.BK_W = 64;  L.BK_H = 24;  L.BK_Y = 3;  L.BK_FONT = SMLSIZE
        L.TL_X = 12;  L.TL_Y1 = 44;  L.TL_Y2 = 90;  L.TL_Y3 = 136;  L.TL_W = 240;  L.TL_H = 38
        if LCD_H >= 320 then -- 480x320: same, with more breathing room
            L.PC_H = 130
            L.ROW_Y1 = 106;  L.ROW_Y2 = 146;  L.ROW_Y3 = 186
            L.FLD_Y1 = 100;  L.FLD_Y2 = 140;  L.FLD_Y3 = 180
            L.LOAD_Y = 140
            L.BTN_Y = 232
            L.ST_Y1 = 272;  L.ST_Y2 = 290;  L.WARN_Y = 272
            L.ED_ROW_H = 44
        end
    end
end


----------------------------------------------------------------------
-- MBridge CRSF emulation
----------------------------------------------------------------------

local isConnected = nil
local cmdPush = nil
local cmdPop = nil
local deviceInfoRxIsAvailable = false -- workaround for AX12
local isEdgeTx = false
local isAX12 = false -- set once in scriptInit(); read by crsfIsConnected() below

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
    -- isAX12 is determined once in scriptInit(); avoids a getVersion() call on every tick
    if not isAX12 and getRSSI() ~= 0 then
        return true
    end
    -- substitute for getRSSI() on AX12, needs reload to discover receiver (dis)connect.
    if isAX12 and deviceInfoRxIsAvailable then
       return true
    end
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
-- Info/Warning popup state (rendered by a reactive LVGL overlay)
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

local function doPopupTimeout()
    if POPUP.active and POPUP.tend_10ms > 0 then
        if getTime() > POPUP.tend_10ms then clearPopup() end
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
    if typ == MBRIDGE_PARAM_TYPE.STR6 then
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
    -- a value with at most one bit set satisfies (v & (v-1)) == 0; this also covers 0
    -- (unlike the previous fixed list of literals, this is correct for any bit position)
    return bit32.band(allowed_mask, allowed_mask - 1) ~= 0
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

local bindphrase_chars = "abcdefghijklmnopqrstuvwxyz0123456789_#-."

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

local function updateDeviceParamCounts(f)
    if f <= 1 then -- called when DEVICE_INFO is received
        if DEVICE_INFO.param_num > 0 then DEVICE_PARAM_LIST_max_index = DEVICE_INFO.param_num - 1 end
    else -- called when end of param list is received
        if DEVICE_INFO.param_num == 0 then DEVICE_PARAM_LIST_max_index = #DEVICE_PARAM_LIST end
    end
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
            DEVICE_INFO.tx_power_dbm = mb_to_i8(cmd.payload, 3)
            DEVICE_INFO.rx_power_dbm = mb_to_i8(cmd.payload, 4)
            DEVICE_INFO.rx_available = mb_to_u8_bits(cmd.payload, 5, 0, 0x1)
            DEVICE_INFO.tx_config_id = mb_to_u8(cmd.payload, 6)
            DEVICE_INFO.tx_diversity = mb_to_u8_bits(cmd.payload, 7, 0, 0x0F)
            DEVICE_INFO.rx_diversity = mb_to_u8_bits(cmd.payload, 7, 4, 0x0F)
            DEVICE_INFO.param_num = mb_to_u8(cmd.payload, 8)
            updateDeviceParamCounts(1)
            deviceInfoRxIsAvailable = (DEVICE_INFO.rx_available == 1) -- to signal if reciever is available
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
                    updateDeviceParamCounts(2)
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
-- UI state
----------------------------------------------------------------------

local PAGENR = {
    MAIN = 0,
    EDIT_TX = 1,
    EDIT_RX = 2,
    TOOLS = 3,
}

local UI = {
    page_nr = PAGENR.MAIN,
    sig = "", -- signature of the state the current LVGL tree was built for
    dirty = true, -- force rebuild
    exit = false,
    baud_warn = false,
}

local isFirstParamDownload = true
local FirstParamDownloadTmo_10ms = 0


local function gotoPage(nr)
    UI.page_nr = nr
    UI.dirty = true
end


local function sanitizeStr6(s)
    -- native keyboard allows any characters; restrict to the mLRS charset
    s = string.lower(s or "")
    local out = ""
    for i = 1, string.len(s) do
        local c = string.sub(s, i, i)
        if string.find(bindphrase_chars, c, 1, true) ~= nil then out = out .. c end
        if string.len(out) >= 6 then break end
    end
    while string.len(out) < 6 do out = out .. "." end
    return out
end


local function paramOrNil(pidx)
    if not DEVICE_PARAM_LIST_complete then return nil end
    if DEVICE_PARAM_LIST == nil then return nil end
    return DEVICE_PARAM_LIST[pidx]
end


local function optionAllowed(p, i) -- i is 1-based option index
    if p.typ ~= MBRIDGE_PARAM_TYPE.LIST then return true end
    return bit32.btest(bit32.lshift(1, i-1), p.allowed_mask)
end


local function warnText()
    if DEVICE_ITEM_TX ~= nil and DEVICE_ITEM_TX.version_int < VERSION.required_tx_version_int then
        return "Tx version not supported by this Lua script!"
    end
    if DEVICE_ITEM_RX ~= nil and connected and DEVICE_ITEM_RX.version_int < VERSION.required_rx_version_int then
        return "Rx version not supported by this Lua script!"
    end
    if UI.baud_warn and DEVICE_DOWNLOAD_is_running then
        return "!! Please check if CRSF baudrate is 400k !!"
    end
    if DEVICE_ITEM_TX ~= nil and DEVICE_ITEM_RX ~= nil and connected and DEVICE_PARAM_LIST_complete and
           (DEVICE_ITEM_TX.setuplayout_int > 515 or DEVICE_ITEM_RX.setuplayout_int > 515) then
        if DEVICE_ITEM_TX.setuplayout_int < DEVICE_ITEM_RX.setuplayout_int then
            return "Tx param version smaller than Rx param version. Please update Tx firmware!"
        elseif DEVICE_ITEM_RX.setuplayout_int < DEVICE_ITEM_TX.setuplayout_int then
            return "Rx param version smaller than Tx param version. Please update Rx firmware!"
        end
    end
    return ""
end


----------------------------------------------------------------------
-- UI building blocks
----------------------------------------------------------------------

local function uiCard(x, y, w, h)
    lvgl.rectangle({x=x, y=y, w=w, h=h, color=C.CARD, filled=true, rounded=8})
    lvgl.rectangle({x=x, y=y, w=w, h=h, color=C.BORDER, filled=false, rounded=8, thickness=2})
end


local function uiHeader(title)
    lvgl.rectangle({x=0, y=0, w=LCD_W, h=LCD_H, color=C.BG, filled=true})
    lvgl.rectangle({x=0, y=0, w=LCD_W, h=L.TITLE_H, color=C.HEADER, filled=true})
    lvgl.label({x=L.TITLE_X, y=L.TITLE_Y, font=L.TITLE_FONT, color=C.TITLE_TEXT, text=title})
    lvgl.label({x=L.VER_X, y=L.VER_Y, font=SMLSIZE, color=C.TITLE_TEXT, text=VERSION.script})
end


local function uiPopupOverlay()
    -- reactive popup box, drawn last so it sits on top; centered on any screen
    local w = 400
    if w > LCD_W - 60 then w = LCD_W - 60 end
    local h = 110
    local x = (LCD_W - w) / 2
    local y = (LCD_H - h) / 2
    lvgl.rectangle({x=x, y=y, w=w, h=h, color=C.HEADER, filled=true, rounded=10,
                    visible=function() return POPUP.active end})
    lvgl.rectangle({x=x, y=y, w=w, h=h, color=C.ACCENT, filled=false, rounded=10, thickness=2,
                    visible=function() return POPUP.active end})
    lvgl.label({x=x+24, y=y+34, w=w-48, font=MIDSIZE, color=C.TITLE_TEXT,
                text=function() return POPUP.text end,
                visible=function() return POPUP.active end})
end


----------------------------------------------------------------------
-- Page Main
----------------------------------------------------------------------

local function statusLineVisible()
    -- on small screens the warning replaces the first status line
    if not L.STATUS_HIDES_ON_WARN then return true end
    return warnText() == ""
end


local function buildPageMain()
    uiHeader("mLRS Configurator")

    -- Tx card
    uiCard(L.TXC_X, L.CARD_Y, L.CARD_W, L.CARD_H)
    lvgl.rectangle({x=L.TXC_X+L.DOT_XOFS, y=L.CARD_Y+L.DOT_YOFS, w=L.DOT_W, h=L.DOT_W,
                    color=C.GREEN, filled=true, rounded=L.DOT_W/2,
                    visible=function() return DEVICE_ITEM_TX ~= nil end})
    lvgl.rectangle({x=L.TXC_X+L.DOT_XOFS, y=L.CARD_Y+L.DOT_YOFS, w=L.DOT_W, h=L.DOT_W,
                    color=C.GREY, filled=true, rounded=L.DOT_W/2,
                    visible=function() return DEVICE_ITEM_TX == nil end})
    lvgl.label({x=L.TXC_X+L.TXT_XOFS, y=L.CARD_Y+L.TXT_Y1OFS, color=C.TEXT,
                text=function()
                    if DEVICE_ITEM_TX == nil then return "Tx  ·  ---" end
                    return "Tx  ·  "..DEVICE_ITEM_TX.name
                end})
    lvgl.label({x=L.TXC_X+L.TXT_XOFS, y=L.CARD_Y+L.TXT_Y2OFS, font=SMLSIZE, color=C.TEXT_DIM,
                text=function()
                    if DEVICE_ITEM_TX == nil then return "" end
                    local s = DEVICE_ITEM_TX.version_str
                    if DEVICE_INFO ~= nil then s = s.."  ·  ConfigId "..tostring(DEVICE_INFO.tx_config_id) end
                    return s
                end})

    -- Rx card
    uiCard(L.RXC_X, L.CARD_Y, L.CARD_W, L.CARD_H)
    lvgl.rectangle({x=L.RXC_X+L.DOT_XOFS, y=L.CARD_Y+L.DOT_YOFS, w=L.DOT_W, h=L.DOT_W,
                    color=C.GREEN, filled=true, rounded=L.DOT_W/2,
                    visible=function() return connected end})
    lvgl.rectangle({x=L.RXC_X+L.DOT_XOFS, y=L.CARD_Y+L.DOT_YOFS, w=L.DOT_W, h=L.DOT_W,
                    color=C.GREY, filled=true, rounded=L.DOT_W/2,
                    visible=function() return not connected end})
    lvgl.label({x=L.RXC_X+L.TXT_XOFS, y=L.CARD_Y+L.TXT_Y1OFS, color=C.TEXT,
                text=function()
                    if not connected then return "Rx  ·  not connected" end
                    if DEVICE_ITEM_RX == nil then return "Rx  ·  ---" end
                    return "Rx  ·  "..DEVICE_ITEM_RX.name
                end})
    lvgl.label({x=L.RXC_X+L.TXT_XOFS, y=L.CARD_Y+L.TXT_Y2OFS, font=SMLSIZE, color=C.TEXT_DIM,
                text=function()
                    if not connected or DEVICE_ITEM_RX == nil then return "" end
                    return DEVICE_ITEM_RX.version_str
                end})

    -- parameters card
    uiCard(L.PC_X, L.PC_Y, L.PC_W, L.PC_H)

    lvgl.label({x=L.LBL_X, y=L.ROW_Y1, color=C.TEXT_DIM, text="Bind Phrase"})
    lvgl.label({x=L.LBL_X, y=L.ROW_Y2, color=C.TEXT_DIM, text="Mode"})
    lvgl.label({x=L.LBL_X, y=L.ROW_Y3, color=C.TEXT_DIM, text="RF Band"})

    if DEVICE_PARAM_LIST_complete then
        -- Bind Phrase: native text editor, pops up the on-screen keyboard
        lvgl.textEdit({x=L.FLD_X, y=L.FLD_Y1, w=L.FLD_W,
                       value=DEVICE_PARAM_LIST[0].value,
                       set=function(s)
                           local p = paramOrNil(0)
                           if p == nil then return end
                           p.value = sanitizeStr6(s)
                           sendParamSet(0)
                           UI.dirty = true -- redraw so the sanitized value is shown
                       end})
        -- exception hint, only for 2.4 GHz band
        lvgl.label({x=L.HINT_X, y=L.ROW_Y1, font=SMLSIZE, color=C.TEXT_DIM,
                    text=function()
                        local p = paramOrNil(0)
                        if p == nil then return "" end
                        return getExceptStrFromChar(string.sub(p.value, 6, 6))
                    end,
                    visible=function()
                        local p = paramOrNil(2)
                        return p ~= nil and p.value == 0
                    end})

        -- Mode: native popup menu
        local pmode = DEVICE_PARAM_LIST[1]
        lvgl.choice({x=L.FLD_X, y=L.FLD_Y2, w=L.FLD_W, title="Mode",
                     values=pmode.options,
                     active=function() return DEVICE_PARAM_LIST_complete and pmode.editable end,
                     filter=function(i) return optionAllowed(pmode, i) end,
                     get=function() return pmode.value + 1 end,
                     set=function(i) pmode.value = i - 1; sendParamSet(1) end})

        -- RF Band: native popup menu with pretty names
        local pband = DEVICE_PARAM_LIST[2]
        local band_values = {}
        for i = 1, #pband.options do
            if freq_band_list[i-1] ~= nil then
                band_values[i] = freq_band_list[i-1]
            else
                band_values[i] = pband.options[i]
            end
        end
        lvgl.choice({x=L.FLD_X, y=L.FLD_Y3, w=L.FLD_W, title="RF Band",
                     values=band_values,
                     active=function() return DEVICE_PARAM_LIST_complete and pband.editable end,
                     filter=function(i) return optionAllowed(pband, i) end,
                     get=function() return pband.value + 1 end,
                     set=function(i) pband.value = i - 1; sendParamSet(2) end})

        -- Ortho: only shown when the device reports it as available
        local portho = DEVICE_PARAM_LIST[3]
        if portho ~= nil and portho.allowed_mask > 0 then
            lvgl.label({x=L.ORTHO_LBL_X, y=L.ROW_Y2, color=C.TEXT_DIM, text="Ortho"})
            lvgl.choice({x=L.ORTHO_FLD_X, y=L.FLD_Y2, w=L.FLD_W, title="Ortho",
                         values=portho.options,
                         active=function() return DEVICE_PARAM_LIST_complete and portho.editable end,
                         filter=function(i) return optionAllowed(portho, i) end,
                         get=function() return portho.value + 1 end,
                         set=function(i) portho.value = i - 1; sendParamSet(3) end})
        end
    else
        -- parameters still loading
        lvgl.label({x=L.LOAD_X, y=L.LOAD_Y, font=L.LOAD_FONT, color=C.ACCENT,
                    text=function()
                        local idx = DEVICE_PARAM_LIST_current_index
                        if idx < 0 then idx = 0 end
                        local s = "loading ... ("..tostring(idx)
                        if DEVICE_PARAM_LIST_max_index > 0 then
                            s = s.."/"..tostring(DEVICE_PARAM_LIST_max_index)
                        end
                        return s..")"
                    end,
                    visible=function() return DEVICE_DOWNLOAD_is_running end})
    end

    -- button row
    lvgl.button({x=L.BTN_X0, y=L.BTN_Y, w=L.BTN_W, h=L.BTN_H, text="Edit Tx", cornerRadius=8,
                 color=C.ACCENT, textColor=C.BTN_TEXT, font=L.BTN_FONT,
                 active=function() return DEVICE_PARAM_LIST_complete end,
                 press=function() if DEVICE_PARAM_LIST_complete then gotoPage(PAGENR.EDIT_TX) end end})
    lvgl.button({x=L.BTN_X0+L.BTN_DX, y=L.BTN_Y, w=L.BTN_W, h=L.BTN_H, text="Edit Rx", cornerRadius=8,
                 color=C.BTN, textColor=C.BTN_TEXT, font=L.BTN_FONT,
                 active=function() return DEVICE_PARAM_LIST_complete and connected end,
                 press=function() if DEVICE_PARAM_LIST_complete and connected then gotoPage(PAGENR.EDIT_RX) end end})
    lvgl.button({x=L.BTN_X0+2*L.BTN_DX, y=L.BTN_Y, w=L.BTN_W, h=L.BTN_H, text="Save", cornerRadius=8,
                 color=C.BTN, textColor=C.BTN_TEXT, font=L.BTN_FONT,
                 active=function() return DEVICE_PARAM_LIST_complete end,
                 press=function()
                     if DEVICE_PARAM_LIST_complete then sendParamStore(); clearParams(); UI.dirty = true end
                 end})
    lvgl.button({x=L.BTN_X0+3*L.BTN_DX, y=L.BTN_Y, w=L.BTN_W, h=L.BTN_H, text="Reload", cornerRadius=8,
                 color=C.BTN, textColor=C.BTN_TEXT, font=L.BTN_FONT,
                 press=function() clearParams(); UI.dirty = true end})
    lvgl.button({x=L.BTN_X0+4*L.BTN_DX, y=L.BTN_Y, w=L.BTN_W, h=L.BTN_H, text="Bind", cornerRadius=8,
                 color=C.BTN, textColor=C.BTN_TEXT, font=L.BTN_FONT,
                 press=function() sendBind() end})
    lvgl.button({x=L.BTN_X0+5*L.BTN_DX, y=L.BTN_Y, w=L.BTN_W, h=L.BTN_H, text="Tools", cornerRadius=8,
                 color=C.BTN, textColor=C.BTN_TEXT, font=L.BTN_FONT,
                 active=function() return DEVICE_PARAM_LIST_complete end,
                 press=function() if DEVICE_PARAM_LIST_complete then gotoPage(PAGENR.TOOLS) end end})

    -- status section
    lvgl.label({x=L.TXC_X, y=L.ST_Y1, font=SMLSIZE, color=C.TEXT_DIM,
                visible=statusLineVisible,
                text=function()
                    if DEVICE_INFO == nil then return "Tx Power ---  ·  Diversity ---" end
                    local div = "?"
                    if DEVICE_INFO.tx_diversity <= #diversity_list then div = diversity_list[DEVICE_INFO.tx_diversity] end
                    return "Tx Power "..tostring(DEVICE_INFO.tx_power_dbm).." dBm  ·  Diversity "..div
                end})
    lvgl.label({x=L.TXC_X, y=L.ST_Y2, font=SMLSIZE, color=C.TEXT_DIM,
                text=function()
                    if DEVICE_INFO == nil then return "Sensitivity ---" end
                    return "Sensitivity "..tostring(DEVICE_INFO.receiver_sensitivity).." dBm"
                end})
    lvgl.label({x=L.ST_RX_X, y=L.ST_Y1, font=SMLSIZE,
                visible=statusLineVisible,
                color=function() if connected then return C.TEXT_DIM end return C.TEXT_OFF end,
                text=function()
                    if DEVICE_INFO == nil or not connected then return "Rx Power ---  ·  Diversity ---" end
                    local div = "?"
                    if DEVICE_INFO.rx_diversity <= #diversity_list then div = diversity_list[DEVICE_INFO.rx_diversity] end
                    return "Rx Power "..tostring(DEVICE_INFO.rx_power_dbm).." dBm  ·  Diversity "..div
                end})

    -- warning line (version mismatch, baudrate hint, setup layout);
    -- on small screens it takes the place of the first status line
    lvgl.label({x=L.TXC_X, y=L.WARN_Y, font=SMLSIZE, color=C.WARN,
                text=function() return warnText() end,
                visible=function() return warnText() ~= "" end})

    uiPopupOverlay()
end


----------------------------------------------------------------------
-- Page Edit Tx/Rx
----------------------------------------------------------------------

local function buildPageEdit(page_str)
    local title = "Edit "..page_str
    if page_str == "Tx" and DEVICE_INFO ~= nil then
        title = "Edit Tx  ·  ConfigId "..tostring(DEVICE_INFO.tx_config_id)
    end
    uiHeader(title)

    lvgl.button({x=LCD_W-L.BK_W-12, y=L.BK_Y, w=L.BK_W, h=L.BK_H, text="Back", cornerRadius=8,
                 color=C.BTN, textColor=C.BTN_TEXT, font=L.BK_FONT,
                 press=function() gotoPage(PAGENR.MAIN) end})

    -- scrollable parameter list: content taller than the box scrolls automatically
    local box = lvgl.box({x=0, y=L.ED_BOX_Y, w=LCD_W, h=LCD_H-L.ED_BOX_Y})

    local row = 0
    if DEVICE_PARAM_LIST ~= nil then
        for pidx = 2, #DEVICE_PARAM_LIST do
            local p = DEVICE_PARAM_LIST[pidx]
            if p ~= nil and string.sub(p.name,1,2) == page_str and p.allowed_mask > 0 then
                local y = 8 + row * L.ED_ROW_H
                local name = string.sub(p.name, 4)
                local pi = pidx -- capture per parameter for the closures below

                box:label({x=L.ED_LBL_X, y=y+L.ED_LBL_YOFS, color=C.TEXT, text=name})

                if p.typ < MBRIDGE_PARAM_TYPE.LIST then
                    box:numberEdit({x=L.ED_CTL_X, y=y, w=L.ED_CTL_W, min=p.min, max=p.max,
                                    active=function() return p.editable end,
                                    get=function() return p.value end,
                                    display=function(v)
                                        if p.unit ~= "" then return tostring(v).." "..p.unit end
                                        return tostring(v)
                                    end,
                                    edited=function(v) p.value = v; sendParamSet(pi) end})
                elseif p.typ == MBRIDGE_PARAM_TYPE.LIST then
                    box:choice({x=L.ED_CTL_X, y=y, w=L.ED_CTL_W, title=name,
                                values=p.options,
                                active=function() return p.editable end,
                                filter=function(i) return optionAllowed(p, i) end,
                                get=function() return p.value + 1 end,
                                set=function(i) p.value = i - 1; sendParamSet(pi) end})
                elseif p.typ == MBRIDGE_PARAM_TYPE.STR6 then
                    box:textEdit({x=L.ED_CTL_X, y=y, w=L.ED_CTL_W,
                                  value=p.value,
                                  active=function() return p.editable end,
                                  set=function(s)
                                      p.value = sanitizeStr6(s)
                                      sendParamSet(pi)
                                      UI.dirty = true
                                  end})
                end

                row = row + 1
            end
        end
    end

    if row == 0 then
        box:label({x=L.ED_LBL_X, y=20, color=C.TEXT_DIM, text="no parameters"})
    end

    uiPopupOverlay()
end


----------------------------------------------------------------------
-- Page Tools
----------------------------------------------------------------------

local function buildPageTools()
    uiHeader("Tools")

    lvgl.button({x=LCD_W-L.BK_W-12, y=L.BK_Y, w=L.BK_W, h=L.BK_H, text="Back", cornerRadius=8,
                 color=C.BTN, textColor=C.BTN_TEXT, font=L.BK_FONT,
                 press=function() gotoPage(PAGENR.MAIN) end})

    lvgl.button({x=L.TL_X, y=L.TL_Y1, w=L.TL_W, h=L.TL_H, text="System Bootloader", cornerRadius=8,
                 color=C.BTN, textColor=C.BTN_TEXT,
                 press=function() gotoPage(PAGENR.MAIN); sendBoot() end})
    lvgl.button({x=L.TL_X, y=L.TL_Y2, w=L.TL_W, h=L.TL_H, text="Flash ESP", cornerRadius=8,
                 color=C.BTN, textColor=C.BTN_TEXT,
                 press=function() gotoPage(PAGENR.MAIN); sendFlashEsp() end})

    -- color theme toggle; the choice is persisted on the SD card
    lvgl.button({x=L.TL_X, y=L.TL_Y3, w=L.TL_W, h=L.TL_H, cornerRadius=8,
                 color=C.BTN, textColor=C.BTN_TEXT,
                 text=function()
                     if theme_nr == 2 then return "Theme: High Contrast" end
                     return "Theme: Dark"
                 end,
                 press=function()
                     if theme_nr == 2 then applyTheme(1) else applyTheme(2) end
                     saveThemeChoice(theme_nr)
                     UI.dirty = true -- rebuild with the new palette
                 end})

    uiPopupOverlay()
end


----------------------------------------------------------------------
-- UI rebuild logic
----------------------------------------------------------------------

local function uiSignature()
    return tostring(UI.page_nr).."|"..tostring(DEVICE_PARAM_LIST_complete).."|"
           ..tostring(connected).."|"..tostring(DEVICE_DOWNLOAD_is_running)
end


local function rebuildUI()
    lvgl.clear()
    if UI.page_nr == PAGENR.EDIT_TX then
        buildPageEdit("Tx")
    elseif UI.page_nr == PAGENR.EDIT_RX then
        buildPageEdit("Rx")
    elseif UI.page_nr == PAGENR.TOOLS then
        buildPageTools()
    else
        buildPageMain()
    end
    UI.sig = uiSignature()
    UI.dirty = false
end


----------------------------------------------------------------------
-- Script OTX Interface
----------------------------------------------------------------------

local function scriptInit()
    local ver, radio, maj, minor, rev, osname = getVersion()
    isEdgeTx = (osname == 'EdgeTX')
    isAX12 = (osname == 'EdgeTXqw')

    setupBridge()
    setupLayout()
    applyTheme(loadThemeChoice())

    local tnow_10ms = getTime()

    DEVICE_DOWNLOAD_is_running = true -- we start the script with this
    isFirstParamDownload = true
    FirstParamDownloadTmo_10ms = tnow_10ms + 500 -- drop a warning after 5 secs
    if tnow_10ms < 300 then
        DEVICE_SAVE_t_last = 300 - tnow_10ms -- treat script start like a Save
    else
        DEVICE_SAVE_t_last = 0
    end

    UI.page_nr = PAGENR.MAIN
    UI.dirty = true
    UI.exit = false
    UI.baud_warn = false

    if lvgl ~= nil then
        rebuildUI()
    end
end


local function scriptRun(event, touchState)
    if lvgl == nil then
        lcd.clear()
        lcd.drawText(10, 10, "This script needs EdgeTX 2.11+ (LVGL).", COLOR_THEME_WARNING)
        lcd.drawText(10, 40, "Please use the classic mLRS.lua instead.", COLOR_THEME_WARNING)
        if event == EVT_VIRTUAL_EXIT then return 2 end
        return 0
    end
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

    doConnected()

    if has_connected then
        clearParams()
        if not POPUP.active then setPopupWTmo("Receiver connected!", 100) end
        clearPopupIfBlocked()
    end
    if has_disconnected then
        if not POPUP.active then setPopupWTmo("Receiver has disconnected!", 100) end
    end
    if not connected and UI.page_nr == PAGENR.EDIT_RX then
        gotoPage(PAGENR.MAIN)
    end

    doParamLoop()
    checkBind()
    doPopupTimeout()

    -- baudrate hint if the very first parameter download stalls
    if DEVICE_DOWNLOAD_is_running and isFirstParamDownload then
        if FirstParamDownloadTmo_10ms > 0 and getTime() > FirstParamDownloadTmo_10ms then
            UI.baud_warn = true
            FirstParamDownloadTmo_10ms = 0
        end
    end
    if not DEVICE_DOWNLOAD_is_running and isFirstParamDownload then
        isFirstParamDownload = false
        UI.baud_warn = false
    end

    -- key handling: RTN leaves sub-pages, exits the tool from the main page
    if event == EVT_VIRTUAL_EXIT then
        if POPUP.active and POPUP.tend_10ms > 0 then
            clearPopup()
        elseif UI.page_nr ~= PAGENR.MAIN then
            gotoPage(PAGENR.MAIN)
        else
            UI.exit = true
        end
    end

    if UI.exit then return 2 end

    -- rebuild the LVGL tree when the page or the underlying data changed;
    -- everything else updates itself through reactive attributes
    if UI.dirty or UI.sig ~= uiSignature() then
        rebuildUI()
    end

    return 0
end

return { init=scriptInit, run=scriptRun, useLvgl=true }
