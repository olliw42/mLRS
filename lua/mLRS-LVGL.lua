-- TNS|mLRS Configurator LVGL|TNE
----------------------------------------------------------------------
-- Copyright (c) MLRS project
-- GPL3
-- https://www.gnu.org/licenses/gpl-3.0.de.html
----------------------------------------------------------------------
-- Lua TOOLS script, LVGL version for EdgeTX 2.12.X+
----------------------------------------------------------------------
-- copy script to SCRIPTS\TOOLS folder on EdgeTx SD card
-- works with mLRS v1.4.00 and later

local VERSION = {
    script = '2026-04-14 LVGL',
    required_tx_version_int = 10400,  -- 'v1.4.00'
    required_rx_version_int = 10400,  -- 'v1.4.00'
}


-- experimental
local paramLoadDeadTime_10ms = 300


----------------------------------------------------------------------
-- LVGL layout constants
----------------------------------------------------------------------

local IS_NARROW = LCD_W < 400
local LABEL_PCT = IS_NARROW and 42 or 50
local CTRL_PCT  = 100 - LABEL_PCT


-- ============================================================================
-- App Module: Application state and coordinators
-- ============================================================================

local App = {
    page = "main",           -- "main", "edit_tx", "edit_rx", "tools"
    shouldExit = false,
    moduleChecked = false,
    isFirstParamDownload = true,
    firstParamDownloadTmo_10ms = 0,
    baudRateWarningShown = false,
}


-- ============================================================================
-- Protocol Module: mBridge/CRSF communication
-- ============================================================================

local Protocol = {
    isConnected = nil,
    cmdPush = nil,
    cmdPop = nil,

    connected = false,
    connected_has_changed = false,
    has_connected = false,
    has_disconnected = false,

    paramloop_t_last = 0,
    DEVICE_ITEM_TX = nil,
    DEVICE_ITEM_RX = nil,
    DEVICE_INFO = nil,
    DEVICE_PARAM_LIST = nil,
    DEVICE_PARAM_LIST_expected_index = 0,
    DEVICE_PARAM_LIST_current_index = -1,
    DEVICE_PARAM_LIST_max_index = 0,
    DEVICE_PARAM_LIST_errors = 0,
    DEVICE_PARAM_LIST_complete = false,
    DEVICE_DOWNLOAD_is_running = true,
    DEVICE_SAVE_t_last = 0,
    DEVICE_SAVE_reload_pending = false,
    DEVICE_BINDING = false,
    DEVICE_BIND_t_start = 0,
}

local MBRIDGE_COMMANDPACKET_STX  = 0xA0

local MBRIDGE_CMD = {
    TX_LINK_STATS      = 2,
    REQUEST_INFO       = 3,
    DEVICE_ITEM_TX     = 4,
    DEVICE_ITEM_RX     = 5,
    PARAM_REQUEST_LIST = 6,
    PARAM_ITEM         = 7,
    PARAM_ITEM2        = 8,
    PARAM_ITEM3_4      = 9,
    REQUEST_CMD        = 10,
    INFO               = 11,
    PARAM_SET          = 12,
    PARAM_STORE        = 13,
    BIND_START         = 14,
    BIND_STOP          = 15,
    MODELID_SET        = 16,
    SYSTEM_BOOTLOADER  = 17,
    FLASH_ESP          = 18,
}

local MBRIDGE_CMD_LEN = {
    [MBRIDGE_CMD.TX_LINK_STATS]   = 22,
    [MBRIDGE_CMD.DEVICE_ITEM_TX]  = 24,
    [MBRIDGE_CMD.DEVICE_ITEM_RX]  = 24,
    [MBRIDGE_CMD.PARAM_ITEM]      = 24,
    [MBRIDGE_CMD.PARAM_ITEM2]     = 24,
    [MBRIDGE_CMD.PARAM_ITEM3_4]   = 24,
    [MBRIDGE_CMD.REQUEST_CMD]     = 18,
    [MBRIDGE_CMD.INFO]            = 24,
    [MBRIDGE_CMD.PARAM_SET]       = 7,
    [MBRIDGE_CMD.MODELID_SET]     = 3,
}

local MBRIDGE_PARAM_TYPE = {
    UINT8       = 0,
    INT8        = 1,
    UINT16      = 2,
    INT16       = 3,
    LIST        = 4,
    STR6        = 5,
}

local bindphrase_chars = "abcdefghijklmnopqrstuvwxyz0123456789_#-."

local diversity_list = {
    [0] = "enabled",
    [1] = "antenna1",
    [2] = "antenna2",
    [3] = "r:en. t:ant1",
    [4] = "r:en. t:ant2",
}

local freq_band_list = {
    [0] = "2.4 GHz",
    [1] = "915 MHz FCC",
    [2] = "868 MHz",
    [3] = "433 MHz",
    [4] = "70 cm HAM",
    [5] = "866 MHz IN",
}


----------------------------------------------------------------------
-- Protocol helpers
----------------------------------------------------------------------

local function mbridgeCmdLen(cmd)
    return MBRIDGE_CMD_LEN[cmd] or 0
end

local function mb_to_string(payload, pos, len)
    local str = ""
    for i = 0, len-1 do
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
    if typ == MBRIDGE_PARAM_TYPE.UINT8 then
        return mb_to_u8(payload, pos)
    elseif typ == MBRIDGE_PARAM_TYPE.INT8 then
        return mb_to_i8(payload, pos)
    elseif typ == MBRIDGE_PARAM_TYPE.UINT16 then
        return mb_to_u16(payload, pos)
    elseif typ == MBRIDGE_PARAM_TYPE.INT16 then
        return mb_to_i16(payload, pos)
    elseif typ == MBRIDGE_PARAM_TYPE.LIST then
        return payload[pos+0]
    end
    return 0
end

local function mb_to_value_or_str6(payload, pos, typ)
    if typ == MBRIDGE_PARAM_TYPE.STR6 then
        return mb_to_string(payload, pos, 6)
    else
        return mb_to_value(payload, pos, typ)
    end
end

local function mb_to_options(payload, pos, len)
    local str = ""
    for i = 0, len-1 do
        if payload[pos+i] == 0 then break end
        str = str .. string.char(payload[pos+i])
    end
    str = str .. ","
    local opt = {}
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
    if allowed_mask == 0 then return false end
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

local function getExceptNoFromChar(c)
    if (c >= 'a' and c <= 'z') then return (string.byte(c) - string.byte('a')) % 5 end
    if (c >= '0' and c <= '9') then return (string.byte(c) - string.byte('0')) % 5 end
    if (c == '_') then return 1 end
    if (c == '#') then return 2 end
    if (c == '-') then return 3 end
    if (c == '.') then return 4 end
    return 0
end

local function getExceptStrFromChar(c)
    local n = getExceptNoFromChar(c)
    if (n == 1) then return "/e1" end
    if (n == 2) then return "/e6" end
    if (n == 3) then return "/e11" end
    if (n == 4) then return "/e13" end
    return "/--"
end


----------------------------------------------------------------------
-- Protocol: Bridge setup
----------------------------------------------------------------------

local function crsfIsConnected()
    if getRSSI() ~= 0 then return true end
    return false
end

local function crsfCmdPush(cmd, payload)
    local data = { 79, 87, cmd + MBRIDGE_COMMANDPACKET_STX }
    for i = 1, mbridgeCmdLen(cmd) do data[#data + 1] = 0 end
    for i = 1, #payload do data[3 + i] = payload[i] end
    return crossfireTelemetryPush(129, data)
end

local function crsfCmdPop()
    local cmd, data = crossfireTelemetryPop()
    if cmd == nil then return nil end
    if data == nil or data[1] == nil then return nil end
    local command = data[1] - MBRIDGE_COMMANDPACKET_STX
    local res = {
        cmd = command,
        len = mbridgeCmdLen(command),
        payload = {}
    }
    for i = 2, #data do res.payload[i-2] = data[i] end
    return res
end

local function mbridgeIsConnected()
    local LStats = mbridge.getLinkStats()
    if LStats.LQ > 0 then return true end
    return false
end

function Protocol.setupBridge()
    if mbridge == nil or not mbridge.enabled() then
        Protocol.isConnected = crsfIsConnected
        Protocol.cmdPush = crsfCmdPush
        Protocol.cmdPop = crsfCmdPop
    else
        Protocol.isConnected = mbridgeIsConnected
        Protocol.cmdPush = mbridge.cmdPush
        Protocol.cmdPop = mbridge.cmdPop
    end
end


----------------------------------------------------------------------
-- Protocol: Connection tracking
----------------------------------------------------------------------

function Protocol.doConnected()
    local is_connected = Protocol.isConnected()

    Protocol.connected_has_changed = false
    if is_connected ~= Protocol.connected then Protocol.connected_has_changed = true end

    Protocol.has_connected = false
    if Protocol.connected_has_changed and is_connected then Protocol.has_connected = true end

    Protocol.has_disconnected = false
    if Protocol.connected_has_changed and not is_connected then Protocol.has_disconnected = true end

    Protocol.connected = is_connected
end


----------------------------------------------------------------------
-- Protocol: Parameter management
----------------------------------------------------------------------

function Protocol.clearParams()
    Protocol.DEVICE_ITEM_TX = nil
    Protocol.DEVICE_ITEM_RX = nil
    Protocol.DEVICE_INFO = nil
    Protocol.DEVICE_PARAM_LIST = nil
    Protocol.DEVICE_PARAM_LIST_expected_index = 0
    Protocol.DEVICE_PARAM_LIST_current_index = -1
    Protocol.DEVICE_PARAM_LIST_errors = 0
    Protocol.DEVICE_PARAM_LIST_complete = false
    Protocol.DEVICE_DOWNLOAD_is_running = true
end

local function paramsError()
    Protocol.DEVICE_PARAM_LIST_errors = Protocol.DEVICE_PARAM_LIST_errors + 1
end

local function updateDeviceParamCounts(f)
    if f <= 1 then
        if Protocol.DEVICE_INFO.param_num > 0 then
            Protocol.DEVICE_PARAM_LIST_max_index = Protocol.DEVICE_INFO.param_num - 1
        end
    else
        if Protocol.DEVICE_INFO.param_num == 0 then
            Protocol.DEVICE_PARAM_LIST_max_index = #Protocol.DEVICE_PARAM_LIST
        end
    end
end


----------------------------------------------------------------------
-- Protocol: Parameter loop (request-response state machine)
----------------------------------------------------------------------
-- Forward declaration for Dialogs (defined later)
local Dialogs = {}

function Protocol.doParamLoop()
    local t_10ms = getTime()
    if t_10ms - Protocol.paramloop_t_last > 33 then
      Protocol.paramloop_t_last = t_10ms
      if t_10ms < Protocol.DEVICE_SAVE_t_last + paramLoadDeadTime_10ms then
          -- skip, we don't send a cmd if the last Save was recent
      elseif Protocol.DEVICE_ITEM_TX == nil then
          Protocol.cmdPush(MBRIDGE_CMD.REQUEST_INFO, {})
          Protocol.DEVICE_PARAM_LIST_expected_index = 0
          Protocol.DEVICE_PARAM_LIST_current_index = -1
          Protocol.DEVICE_PARAM_LIST_errors = 0
          Protocol.DEVICE_PARAM_LIST_complete = false
      elseif Protocol.DEVICE_PARAM_LIST == nil then
          if Protocol.DEVICE_INFO ~= nil then
              Protocol.DEVICE_PARAM_LIST = {}
              Protocol.cmdPush(MBRIDGE_CMD.REQUEST_CMD, {MBRIDGE_CMD.PARAM_ITEM, Protocol.DEVICE_PARAM_LIST_expected_index})
          end
      end
    end

    -- handle received commands
    for ijk = 1, 24 do
        local cmd = Protocol.cmdPop()
        if cmd == nil then break end
        if cmd.cmd == MBRIDGE_CMD.DEVICE_ITEM_TX then
            Protocol.DEVICE_ITEM_TX = cmd
            Protocol.DEVICE_ITEM_TX.version_u16 = mb_to_u16(cmd.payload, 0)
            Protocol.DEVICE_ITEM_TX.setuplayout_u16 = mb_to_u16(cmd.payload, 2)
            Protocol.DEVICE_ITEM_TX.name = mb_to_string(cmd.payload, 4, 20)
            Protocol.DEVICE_ITEM_TX.version_int = mb_to_version_int(Protocol.DEVICE_ITEM_TX.version_u16)
            Protocol.DEVICE_ITEM_TX.version_str = mb_to_version_string(Protocol.DEVICE_ITEM_TX.version_u16)
            Protocol.DEVICE_ITEM_TX.setuplayout_int = mb_to_version_int(Protocol.DEVICE_ITEM_TX.setuplayout_u16)
        elseif cmd.cmd == MBRIDGE_CMD.DEVICE_ITEM_RX then
            Protocol.DEVICE_ITEM_RX = cmd
            Protocol.DEVICE_ITEM_RX.version_u16 = mb_to_u16(cmd.payload, 0)
            Protocol.DEVICE_ITEM_RX.setuplayout_u16 = mb_to_u16(cmd.payload, 2)
            Protocol.DEVICE_ITEM_RX.name = mb_to_string(cmd.payload, 4, 20)
            Protocol.DEVICE_ITEM_RX.version_int = mb_to_version_int(Protocol.DEVICE_ITEM_RX.version_u16)
            Protocol.DEVICE_ITEM_RX.version_str = mb_to_version_string(Protocol.DEVICE_ITEM_RX.version_u16)
            Protocol.DEVICE_ITEM_RX.setuplayout_int = mb_to_version_int(Protocol.DEVICE_ITEM_RX.setuplayout_u16)
        elseif cmd.cmd == MBRIDGE_CMD.INFO then
            Protocol.DEVICE_INFO = cmd
            Protocol.DEVICE_INFO.receiver_sensitivity = mb_to_i16(cmd.payload, 0)
            Protocol.DEVICE_INFO.has_status = mb_to_u8_bits(cmd.payload, 2, 0, 0x01)
            Protocol.DEVICE_INFO.binding = mb_to_u8_bits(cmd.payload, 2, 1, 0x01)
            Protocol.DEVICE_INFO.tx_power_dbm = mb_to_i8(cmd.payload, 3)
            Protocol.DEVICE_INFO.rx_power_dbm = mb_to_i8(cmd.payload, 4)
            Protocol.DEVICE_INFO.rx_available = mb_to_u8_bits(cmd.payload, 5, 0, 0x1)
            Protocol.DEVICE_INFO.tx_config_id = mb_to_u8(cmd.payload, 6)
            Protocol.DEVICE_INFO.tx_diversity = mb_to_u8_bits(cmd.payload, 7, 0, 0x0F)
            Protocol.DEVICE_INFO.rx_diversity = mb_to_u8_bits(cmd.payload, 7, 4, 0x0F)
            Protocol.DEVICE_INFO.param_num = mb_to_u8(cmd.payload, 8)
            updateDeviceParamCounts(1)
        elseif cmd.cmd == MBRIDGE_CMD.PARAM_ITEM then
            local index = cmd.payload[0]
            if index ~= Protocol.DEVICE_PARAM_LIST_expected_index and index ~= 255 then
                paramsError()
            end
            Protocol.DEVICE_PARAM_LIST_current_index = index
            Protocol.DEVICE_PARAM_LIST_expected_index = index + 1
            if Protocol.DEVICE_PARAM_LIST == nil then
                paramsError()
            elseif index < 128 then
                Protocol.DEVICE_PARAM_LIST[index] = cmd
                Protocol.DEVICE_PARAM_LIST[index].typ = mb_to_u8(cmd.payload, 1)
                Protocol.DEVICE_PARAM_LIST[index].name = mb_to_string(cmd.payload, 2, 16)
                Protocol.DEVICE_PARAM_LIST[index].value = mb_to_value_or_str6(cmd.payload, 18, Protocol.DEVICE_PARAM_LIST[index].typ)
                Protocol.DEVICE_PARAM_LIST[index].min = 0
                Protocol.DEVICE_PARAM_LIST[index].max = 0
                Protocol.DEVICE_PARAM_LIST[index].unit = ""
                Protocol.DEVICE_PARAM_LIST[index].options = {}
                Protocol.DEVICE_PARAM_LIST[index].allowed_mask = 65536
                Protocol.DEVICE_PARAM_LIST[index].editable = true
            elseif index == 255 then -- EOL
                if Protocol.DEVICE_PARAM_LIST_errors == 0 then
                    updateDeviceParamCounts(2)
                    Protocol.DEVICE_PARAM_LIST_complete = true
                else
                    Protocol.DEVICE_PARAM_LIST_complete = false
                    UI.setSubtitleMsg("Param errors ("..tostring(Protocol.DEVICE_PARAM_LIST_errors)..")! Try Reload", 200)
                end
                Protocol.DEVICE_DOWNLOAD_is_running = false
            else
                paramsError()
            end
        elseif cmd.cmd == MBRIDGE_CMD.PARAM_ITEM2 then
            local index = cmd.payload[0]
            if index ~= Protocol.DEVICE_PARAM_LIST_current_index then
                paramsError()
            elseif Protocol.DEVICE_PARAM_LIST == nil or Protocol.DEVICE_PARAM_LIST[index] == nil then
                paramsError()
            else
                local item3_needed = false
                if Protocol.DEVICE_PARAM_LIST[index].typ < MBRIDGE_PARAM_TYPE.LIST then
                    Protocol.DEVICE_PARAM_LIST[index].min = mb_to_value(cmd.payload, 1, Protocol.DEVICE_PARAM_LIST[index].typ)
                    Protocol.DEVICE_PARAM_LIST[index].max = mb_to_value(cmd.payload, 3, Protocol.DEVICE_PARAM_LIST[index].typ)
                    Protocol.DEVICE_PARAM_LIST[index].unit = mb_to_string(cmd.payload, 7, 6)
                elseif Protocol.DEVICE_PARAM_LIST[index].typ == MBRIDGE_PARAM_TYPE.LIST then
                    Protocol.DEVICE_PARAM_LIST[index].allowed_mask = mb_to_u16(cmd.payload, 1)
                    Protocol.DEVICE_PARAM_LIST[index].options = mb_to_options(cmd.payload, 3, 21)
                    Protocol.DEVICE_PARAM_LIST[index].item2payload = cmd.payload
                    Protocol.DEVICE_PARAM_LIST[index].min = 0
                    Protocol.DEVICE_PARAM_LIST[index].max = #Protocol.DEVICE_PARAM_LIST[index].options - 1
                    Protocol.DEVICE_PARAM_LIST[index].editable = mb_allowed_mask_editable(Protocol.DEVICE_PARAM_LIST[index].allowed_mask)
                    local s = mb_to_string(cmd.payload, 3, 21)
                    if string.len(s) == 21 then item3_needed = true end
                elseif Protocol.DEVICE_PARAM_LIST[index].typ == MBRIDGE_PARAM_TYPE.STR6 then
                    -- nothing to do
                else
                    paramsError()
                end
                if not item3_needed then
                    Protocol.cmdPush(MBRIDGE_CMD.REQUEST_CMD, {MBRIDGE_CMD.PARAM_ITEM, Protocol.DEVICE_PARAM_LIST_expected_index})
                end
            end
        elseif cmd.cmd == MBRIDGE_CMD.PARAM_ITEM3_4 then
            local index = cmd.payload[0]
            local is_item4 = false
            if (index >= 128) then
                index = index - 128
                is_item4 = true
            end
            if index ~= Protocol.DEVICE_PARAM_LIST_current_index then
                paramsError()
            elseif Protocol.DEVICE_PARAM_LIST == nil or Protocol.DEVICE_PARAM_LIST[index] == nil then
                paramsError()
            elseif Protocol.DEVICE_PARAM_LIST[index].typ ~= MBRIDGE_PARAM_TYPE.LIST then
                paramsError()
            elseif Protocol.DEVICE_PARAM_LIST[index].item2payload == nil then
                paramsError()
            elseif is_item4 and Protocol.DEVICE_PARAM_LIST[index].item3payload == nil then
                paramsError()
            else
                local s = Protocol.DEVICE_PARAM_LIST[index].item2payload
                local item4_needed = false
                if not is_item4 then
                    Protocol.DEVICE_PARAM_LIST[index].item3payload = cmd.payload
                    for i = 1, 23 do s[23+i] = cmd.payload[i] end
                    Protocol.DEVICE_PARAM_LIST[index].options = mb_to_options(s, 3, 21+23)
                    local opts = mb_to_string(cmd.payload, 1, 23)
                    if string.len(opts) == 23 then item4_needed = true end
                else
                    local s3 = Protocol.DEVICE_PARAM_LIST[index].item3payload
                    for i = 1, 23 do s[23+i] = s3[i]; s[23+23+i] = cmd.payload[i] end
                    Protocol.DEVICE_PARAM_LIST[index].options = mb_to_options(s, 3, 21+23+23)
                end
                Protocol.DEVICE_PARAM_LIST[index].max = #Protocol.DEVICE_PARAM_LIST[index].options - 1
                s = nil
                if not item4_needed then
                    Protocol.cmdPush(MBRIDGE_CMD.REQUEST_CMD, {MBRIDGE_CMD.PARAM_ITEM, Protocol.DEVICE_PARAM_LIST_expected_index})
                end
            end
        end
        cmd = nil
    end
end


----------------------------------------------------------------------
-- Protocol: Send commands
----------------------------------------------------------------------

function Protocol.sendParamSet(idx)
    if not Protocol.DEVICE_PARAM_LIST_complete then return end
    local p = Protocol.DEVICE_PARAM_LIST[idx]
    if p.typ < MBRIDGE_PARAM_TYPE.LIST then
        Protocol.cmdPush(MBRIDGE_CMD.PARAM_SET, {idx, p.value})
    elseif p.typ == MBRIDGE_PARAM_TYPE.LIST then
        Protocol.cmdPush(MBRIDGE_CMD.PARAM_SET, {idx, p.value})
    elseif p.typ == MBRIDGE_PARAM_TYPE.STR6 then
        local cmd = {idx}
        for i = 1, 6 do
            cmd[i+1] = string.byte(string.sub(p.value, i, i))
        end
        Protocol.cmdPush(MBRIDGE_CMD.PARAM_SET, cmd)
    end
end

function Protocol.sendParamStore()
    if not Protocol.DEVICE_PARAM_LIST_complete then return end
    Protocol.cmdPush(MBRIDGE_CMD.PARAM_STORE, {})
    Protocol.DEVICE_SAVE_t_last = getTime()
    Protocol.DEVICE_SAVE_reload_pending = true
end

function Protocol.sendBind()
    if Protocol.DEVICE_DOWNLOAD_is_running then return end
    Protocol.cmdPush(MBRIDGE_CMD.BIND_START, {})
    Protocol.DEVICE_BINDING = true
    Protocol.DEVICE_BIND_t_start = getTime()
end

function Protocol.sendBoot()
    if Protocol.DEVICE_DOWNLOAD_is_running then return end
    Protocol.cmdPush(MBRIDGE_CMD.SYSTEM_BOOTLOADER, {})
    Dialogs.showBlocked("In System Bootloader")
end

function Protocol.sendFlashEsp()
    if Protocol.DEVICE_DOWNLOAD_is_running then return end
    Protocol.cmdPush(MBRIDGE_CMD.FLASH_ESP, {})
    Dialogs.showBlocked("In Flash ESP")
end

function Protocol.checkBind()
    if Protocol.DEVICE_DOWNLOAD_is_running then return end
    if Protocol.DEVICE_INFO ~= nil and Protocol.DEVICE_INFO.has_status == 1 and Protocol.DEVICE_INFO.binding == 1 then
        Protocol.DEVICE_BINDING = true
    end
end


----------------------------------------------------------------------
-- Protocol: Parameter value editing
----------------------------------------------------------------------

function Protocol.paramValueInc(idx)
    if not Protocol.DEVICE_PARAM_LIST_complete then return end
    local p = Protocol.DEVICE_PARAM_LIST[idx]
    if p.typ < MBRIDGE_PARAM_TYPE.LIST then
        p.value = p.value + 1
    elseif p.typ == MBRIDGE_PARAM_TYPE.LIST then
        local value = p.value
        while value <= p.max do
            value = value + 1
            local m = bit32.lshift(1, value)
            if bit32.btest(m, p.allowed_mask) then p.value = value; break end
        end
    end
    if p.value > p.max then p.value = p.max end
    Protocol.DEVICE_PARAM_LIST[idx].value = p.value
end

function Protocol.paramValueDec(idx)
    if not Protocol.DEVICE_PARAM_LIST_complete then return end
    local p = Protocol.DEVICE_PARAM_LIST[idx]
    if p.typ < MBRIDGE_PARAM_TYPE.LIST then
        p.value = p.value - 1
    elseif p.typ == MBRIDGE_PARAM_TYPE.LIST then
        local value = p.value
        while value >= p.min do
            value = value - 1
            local m = bit32.lshift(1, value)
            if bit32.btest(m, p.allowed_mask) then p.value = value; break end
        end
    end
    if p.value < p.min then p.value = p.min end
    Protocol.DEVICE_PARAM_LIST[idx].value = p.value
end

function Protocol.paramFocusable(idx)
    if not Protocol.DEVICE_PARAM_LIST_complete then return false end
    local p = Protocol.DEVICE_PARAM_LIST[idx]
    if p == nil then return false end
    if p.editable == nil then return false end
    return p.editable
end


-- ============================================================================
-- Dialogs Module
-- ============================================================================

Dialogs.popup = nil       -- {dialog}

function Dialogs.showBlocked(text)
    -- Close existing popup if any
    if Dialogs.popup and Dialogs.popup.dialog then
        Dialogs.popup.dialog:close()
    end
    local dg = lvgl.dialog({
        title = "mLRS",
        flexFlow = lvgl.FLOW_COLUMN,
        flexPad = lvgl.PAD_TINY,
    })
    dg:build({
        {type="label", text=text, align=CENTER, font=MIDSIZE},
    })
    Dialogs.popup = {dialog = dg}
end

function Dialogs.clearBlocked()
    if Dialogs.popup and Dialogs.popup.dialog then
        Dialogs.popup.dialog:close()
    end
    Dialogs.popup = nil
end

function Dialogs.showVersionWarning(text)
    lvgl.message({
        title = "Version Warning",
        message = text,
    })
end


-- ============================================================================
-- UI Module: LVGL page/widget building
-- ============================================================================

local UI = {
    currentPage = nil,
    uiBuilt = false,
    paramsWereComplete = false,
    subtitle_msg = nil,
    subtitle_msg_tend = 0,
}

function UI.invalidate()
    UI.uiBuilt = false
end

function UI.setSubtitleMsg(msg, tmo_10ms)
    UI.subtitle_msg = msg
    UI.subtitle_msg_tend = getTime() + tmo_10ms
end


----------------------------------------------------------------------
-- UI: Subtitle
----------------------------------------------------------------------

function UI.getSubtitle()
    if Protocol.DEVICE_DOWNLOAD_is_running then
        local idx = Protocol.DEVICE_PARAM_LIST_current_index
        if idx < 0 then idx = 0 end
        local s = "Loading params... (" .. tostring(idx)
        if Protocol.DEVICE_PARAM_LIST_max_index > 0 then
            s = s .. "/" .. tostring(Protocol.DEVICE_PARAM_LIST_max_index)
        end
        return s .. ")"
    end
    return ""
end

function UI.getMainSubtitle()
    if Protocol.DEVICE_BINDING then return "Binding..." end
    if Protocol.DEVICE_SAVE_reload_pending then return "Saving..." end
    if UI.subtitle_msg then
        if getTime() < UI.subtitle_msg_tend then return UI.subtitle_msg end
        UI.subtitle_msg = nil
    end
    local s = UI.getSubtitle()
    if s ~= "" then return s end
    return VERSION.script
end


----------------------------------------------------------------------
-- UI: Helpers
----------------------------------------------------------------------

-- Build filtered values + index mappings from allowed_mask
function UI.buildFilteredValues(param, displayList)
    local filteredValues = {}
    local origToFiltered = {}
    local filteredToOrig = {}
    for i, opt in ipairs(param.options) do
        local idx0 = i - 1
        if bit32.btest(bit32.lshift(1, idx0), param.allowed_mask) then
            local name = (displayList and displayList[idx0]) or opt
            filteredValues[#filteredValues + 1] = name
            origToFiltered[idx0] = #filteredValues
            filteredToOrig[#filteredValues] = idx0
        end
    end
    return filteredValues, origToFiltered, filteredToOrig
end


----------------------------------------------------------------------
-- UI: Widget creators
----------------------------------------------------------------------

function UI.createInfoRowPair(pg, label1, valueFn1, label2, valueFn2)
    pg:build({
        {type="rectangle", w=lvgl.PERCENT_SIZE+100, thickness=0, flexFlow=lvgl.FLOW_ROW, flexPad=0, children={
            {type="rectangle", w=lvgl.PERCENT_SIZE+50, thickness=0, flexFlow=lvgl.FLOW_ROW, flexPad=0, children={
                {type="rectangle", w=lvgl.PERCENT_SIZE+LABEL_PCT, h=lvgl.UI_ELEMENT_HEIGHT, thickness=0, children={
                    {type="label", y=lvgl.PAD_SMALL, text=label1, color=COLOR_THEME_PRIMARY1},
                }},
                {type="rectangle", w=lvgl.PERCENT_SIZE+CTRL_PCT, h=lvgl.UI_ELEMENT_HEIGHT, thickness=0, children={
                    {type="label", y=lvgl.PAD_SMALL, text=valueFn1},
                }},
            }},
            {type="rectangle", w=lvgl.PERCENT_SIZE+50, thickness=0, flexFlow=lvgl.FLOW_ROW, flexPad=0, children={
                {type="rectangle", w=lvgl.PERCENT_SIZE+LABEL_PCT, h=lvgl.UI_ELEMENT_HEIGHT, thickness=0, children={
                    {type="label", y=lvgl.PAD_SMALL, text=label2, color=COLOR_THEME_PRIMARY1},
                }},
                {type="rectangle", w=lvgl.PERCENT_SIZE+CTRL_PCT, h=lvgl.UI_ELEMENT_HEIGHT, thickness=0, children={
                    {type="label", y=lvgl.PAD_SMALL, text=valueFn2},
                }},
            }},
        }},
    })
end

function UI.createInfoRow(pg, label, valueFn)
    pg:build({
        {type="rectangle", w=lvgl.PERCENT_SIZE+100, thickness=0, flexFlow=lvgl.FLOW_ROW, flexPad=0, children={
            {type="rectangle", w=lvgl.PERCENT_SIZE+LABEL_PCT, h=lvgl.UI_ELEMENT_HEIGHT, thickness=0, children={
                {type="label", y=lvgl.PAD_SMALL, text=label, color=COLOR_THEME_PRIMARY1},
            }},
            {type="rectangle", w=lvgl.PERCENT_SIZE+CTRL_PCT, h=lvgl.UI_ELEMENT_HEIGHT, thickness=0, children={
                {type="label", y=lvgl.PAD_SMALL, text=valueFn},
            }},
        }},
    })
end

function UI.createNumberRow(pg, param, pidx)
    local displayFn = function(val)
        if param.unit ~= "" then
            return tostring(val) .. " " .. param.unit
        end
        return tostring(val)
    end

    pg:build({
        {type="rectangle", w=lvgl.PERCENT_SIZE+100, thickness=0, flexFlow=lvgl.FLOW_ROW, flexPad=0, children={
            {type="rectangle", w=lvgl.PERCENT_SIZE+LABEL_PCT, h=lvgl.UI_ELEMENT_HEIGHT, thickness=0, children={
                {type="label", y=lvgl.PAD_SMALL, text=string.sub(param.name, 4), color=COLOR_THEME_PRIMARY1},
            }},
            {type="rectangle", w=lvgl.PERCENT_SIZE+CTRL_PCT, thickness=0, flexFlow=lvgl.FLOW_ROW, align=LEFT, children={
                {type="numberEdit", min=param.min, max=param.max,
                    get=function() return param.value or 0 end,
                    set=function(val)
                        param.value = val
                    end,
                    edited=function(val)
                        param.value = val
                        Protocol.sendParamSet(pidx)
                    end,
                    display=displayFn,
                    active=function() return param.editable end},
            }},
        }},
    })
end

function UI.createFilteredChoiceRow(pg, label, param, pidx)
    local row = pg:rectangle({
        w = lvgl.PERCENT_SIZE + 100,
        thickness = 0,
        flexFlow = lvgl.FLOW_ROW,
        flexPad = 0,
    })

    row:rectangle({
        w = lvgl.PERCENT_SIZE + LABEL_PCT,
        h = lvgl.UI_ELEMENT_HEIGHT,
        thickness = 0,
        children = {
            {
                type = lvgl.LABEL,
                y = lvgl.PAD_SMALL,
                text = label,
                color = COLOR_THEME_PRIMARY1,
            }
        }
    })

    local ctrlRect = row:rectangle({
        w = lvgl.PERCENT_SIZE + CTRL_PCT,
        thickness = 0,
        flexFlow = lvgl.FLOW_ROW,
        align = LEFT + VCENTER,
    })

    -- Build filtered values list using allowed_mask
    local filteredValues = {}
    local origToFiltered = {}
    local filteredToOrig = {}
    for i, opt in ipairs(param.options) do
        local idx0 = i - 1
        if bit32.btest(bit32.lshift(1, idx0), param.allowed_mask) then
            filteredValues[#filteredValues + 1] = opt
            origToFiltered[idx0] = #filteredValues
            filteredToOrig[#filteredValues] = idx0
        end
    end

    ctrlRect:choice({
        values = filteredValues,
        get = function() return origToFiltered[param.value] or 1 end,
        set = function(val)
            param.value = filteredToOrig[val] or 0
            Protocol.sendParamSet(pidx)
        end,
        active = function() return param.editable end,
    })
end

function UI.createFilteredChoiceRowWithDisplay(pg, label, param, pidx, displayList)
    local row = pg:rectangle({
        w = lvgl.PERCENT_SIZE + 100,
        thickness = 0,
        flexFlow = lvgl.FLOW_ROW,
        flexPad = 0,
    })

    row:rectangle({
        w = lvgl.PERCENT_SIZE + LABEL_PCT,
        h = lvgl.UI_ELEMENT_HEIGHT,
        thickness = 0,
        children = {
            {
                type = lvgl.LABEL,
                y = lvgl.PAD_SMALL,
                text = label,
                color = COLOR_THEME_PRIMARY1,
            }
        }
    })

    local ctrlRect = row:rectangle({
        w = lvgl.PERCENT_SIZE + CTRL_PCT,
        thickness = 0,
        flexFlow = lvgl.FLOW_ROW,
        align = LEFT + VCENTER,
    })

    local filteredValues = {}
    local origToFiltered = {}
    local filteredToOrig = {}
    for i, opt in ipairs(param.options) do
        local idx0 = i - 1
        if bit32.btest(bit32.lshift(1, idx0), param.allowed_mask) then
            local displayName = displayList[idx0] or opt
            filteredValues[#filteredValues + 1] = displayName
            origToFiltered[idx0] = #filteredValues
            filteredToOrig[#filteredValues] = idx0
        end
    end

    ctrlRect:choice({
        values = filteredValues,
        get = function() return origToFiltered[param.value] or 1 end,
        set = function(val)
            param.value = filteredToOrig[val] or 0
            Protocol.sendParamSet(pidx)
        end,
        active = function() return param.editable end,
    })
end

----------------------------------------------------------------------
-- UI: Main page helpers (grid-aligned label:value pairs)
----------------------------------------------------------------------

local ML = 25  -- label width % for full-width rows
local MV = 75  -- value width % for full-width rows
local ML2 = 40 -- left label width % within a 50% half (two-column rows)
local MV2 = 60 -- left value width % within a 50% half (two-column rows)
local ML2R = 22 -- right label width % within a 50% half
local MV2R = 78 -- right value width % within a 50% half

-- One label:value pair in the left half, right half empty
function UI.mainRow1(pg, label, valueFn)
    local row = pg:rectangle({
        w = lvgl.PERCENT_SIZE + 100, thickness = 0,
        flexFlow = lvgl.FLOW_ROW, flexPad = 0,
    })
    row:rectangle({
        w = lvgl.PERCENT_SIZE + ML, h = lvgl.UI_ELEMENT_HEIGHT, thickness = 0,
        children = {{type=lvgl.LABEL, y=lvgl.PAD_SMALL, text=label, font=BOLD, color=BLACK}},
    })
    row:rectangle({
        w = lvgl.PERCENT_SIZE + (100 - ML), h = lvgl.UI_ELEMENT_HEIGHT, thickness = 0,
        children = {{type=lvgl.LABEL, y=lvgl.PAD_SMALL, text=valueFn}},
    })
end

-- One label:control pair (full width)
function UI.mainControlRow1(pg, label, ctrlFn)
    local row = pg:rectangle({
        w = lvgl.PERCENT_SIZE + 100, thickness = 0,
        flexFlow = lvgl.FLOW_ROW, flexPad = 0,
    })
    row:rectangle({
        w = lvgl.PERCENT_SIZE + ML, h = lvgl.UI_ELEMENT_HEIGHT, thickness = 0,
        children = {{type=lvgl.LABEL, y=lvgl.PAD_SMALL, text=label, font=BOLD, color=BLACK}},
    })
    local ctrl = row:rectangle({
        w = lvgl.PERCENT_SIZE + (100 - ML), thickness = 0,
        flexFlow = lvgl.FLOW_ROW, flexPad = 0,
        align = LEFT,
    })
    ctrlFn(ctrl)
    -- Invisible placeholder keeps height consistent when no control is added
    ctrl:rectangle({w = 1, h = lvgl.UI_ELEMENT_HEIGHT, thickness = 0})
end

-- Two label:value pairs side by side (each gets 50%)
function UI.mainRow2(pg, label1, val1, label2, val2)
    local row = pg:rectangle({
        w = lvgl.PERCENT_SIZE + 100, thickness = 0,
        flexFlow = lvgl.FLOW_ROW, flexPad = 0,
    })
    -- Left half
    local left = row:rectangle({
        w = lvgl.PERCENT_SIZE + 50, thickness = 0,
        flexFlow = lvgl.FLOW_ROW, flexPad = 0,
    })
    left:rectangle({
        w = lvgl.PERCENT_SIZE + ML2, h = lvgl.UI_ELEMENT_HEIGHT, thickness = 0,
        children = {{type=lvgl.LABEL, y=lvgl.PAD_SMALL, text=label1, font=BOLD, color=BLACK}},
    })
    left:rectangle({
        w = lvgl.PERCENT_SIZE + MV2, h = lvgl.UI_ELEMENT_HEIGHT, thickness = 0,
        children = {{type=lvgl.LABEL, y=lvgl.PAD_SMALL, text=val1}},
    })
    -- Right half
    local right = row:rectangle({
        w = lvgl.PERCENT_SIZE + 50, thickness = 0,
        flexFlow = lvgl.FLOW_ROW, flexPad = 0,
    })
    right:rectangle({
        w = lvgl.PERCENT_SIZE + ML2R, h = lvgl.UI_ELEMENT_HEIGHT, thickness = 0,
        children = {{type=lvgl.LABEL, y=lvgl.PAD_SMALL, text=label2, font=BOLD, color=BLACK}},
    })
    right:rectangle({
        w = lvgl.PERCENT_SIZE + MV2R, h = lvgl.UI_ELEMENT_HEIGHT, thickness = 0,
        children = {{type=lvgl.LABEL, y=lvgl.PAD_SMALL, text=val2}},
    })
end

-- Two label:control pairs side by side (each gets 50%)
function UI.mainControlRow2(pg, label1, ctrl1Fn, label2, ctrl2Fn)
    local row = pg:rectangle({
        w = lvgl.PERCENT_SIZE + 100, thickness = 0,
        flexFlow = lvgl.FLOW_ROW, flexPad = 0,
    })
    -- Left half
    local left = row:rectangle({
        w = lvgl.PERCENT_SIZE + 50, thickness = 0,
        flexFlow = lvgl.FLOW_ROW, flexPad = 0,
    })
    left:rectangle({
        w = lvgl.PERCENT_SIZE + ML2, h = lvgl.UI_ELEMENT_HEIGHT, thickness = 0,
        children = {{type=lvgl.LABEL, y=lvgl.PAD_SMALL, text=label1, font=BOLD, color=BLACK}},
    })
    local leftCtrl = left:rectangle({
        w = lvgl.PERCENT_SIZE + MV2, thickness = 0,
        flexFlow = lvgl.FLOW_ROW, flexPad = 0,
    })
    ctrl1Fn(leftCtrl)
    -- Right half
    local right = row:rectangle({
        w = lvgl.PERCENT_SIZE + 50, thickness = 0,
        flexFlow = lvgl.FLOW_ROW, flexPad = 0,
    })
    right:rectangle({
        w = lvgl.PERCENT_SIZE + ML2R, h = lvgl.UI_ELEMENT_HEIGHT, thickness = 0,
        children = {{type=lvgl.LABEL, y=lvgl.PAD_SMALL, text=label2, font=BOLD, color=BLACK}},
    })
    local rightCtrl = right:rectangle({
        w = lvgl.PERCENT_SIZE + MV2R, thickness = 0,
        flexFlow = lvgl.FLOW_ROW, flexPad = 0,
    })
    ctrl2Fn(rightCtrl)
end

----------------------------------------------------------------------
-- UI: Page builders
----------------------------------------------------------------------

function UI.buildMainPage()
    UI.currentPage = lvgl.page({
        title = "mLRS Configurator",
        subtitle = UI.getMainSubtitle,
        back = function() App.shouldExit = true end,
    })

    local pg = UI.currentPage:box({
        w = lvgl.PERCENT_SIZE + 100,
        flexFlow = lvgl.FLOW_COLUMN,
        flexPad = lvgl.PAD_TINY,
    })

    -- Version check
    if Protocol.DEVICE_ITEM_TX ~= nil and Protocol.DEVICE_ITEM_TX.version_int < VERSION.required_tx_version_int then
        Dialogs.showVersionWarning("Tx version not supported by this Lua script!")
    end
    if Protocol.DEVICE_ITEM_RX ~= nil and Protocol.connected and Protocol.DEVICE_ITEM_RX.version_int < VERSION.required_rx_version_int then
        Dialogs.showVersionWarning("Rx version not supported by this Lua script!")
    end

    -- Tx, Rx, Bind Phrase, Mode, RF Band, Ortho (consistent spacing)
    local ctrlBox = pg:box({
        w = lvgl.PERCENT_SIZE + 100, flexFlow = lvgl.FLOW_COLUMN, flexPad = 5,
    })
    UI.mainRow1(ctrlBox, "Tx:", function()
        if Protocol.DEVICE_ITEM_TX == nil then return "---" end
        local s = Protocol.DEVICE_ITEM_TX.name .. " | " .. Protocol.DEVICE_ITEM_TX.version_str
        if Protocol.DEVICE_INFO ~= nil then
            s = s .. " | ConfigId " .. tostring(Protocol.DEVICE_INFO.tx_config_id)
        end
        return s
    end)
    UI.mainRow1(ctrlBox, "Rx:", function()
        if not Protocol.connected then return "not connected" end
        if Protocol.DEVICE_ITEM_RX == nil then return "---" end
        return Protocol.DEVICE_ITEM_RX.name .. " | " .. Protocol.DEVICE_ITEM_RX.version_str
    end)
    local bpParam = Protocol.DEVICE_PARAM_LIST_complete and Protocol.DEVICE_PARAM_LIST[0] or nil
    local modeParam = Protocol.DEVICE_PARAM_LIST_complete and Protocol.DEVICE_PARAM_LIST[1] or nil
    local rfParam = Protocol.DEVICE_PARAM_LIST_complete and Protocol.DEVICE_PARAM_LIST[2] or nil
    local orthoParam = Protocol.DEVICE_PARAM_LIST_complete and Protocol.DEVICE_PARAM_LIST[3] or nil

    UI.mainControlRow2(ctrlBox, "Bind Phrase:", function(parent)
        if bpParam then
            parent:textEdit({
                value = bpParam.value or "",
                length = 6,
                w = 80,
                set = function(newVal)
                    local s = ""
                    for i = 1, math.min(6, #newVal) do
                        local c = string.sub(newVal, i, i)
                        if string.find(bindphrase_chars, c, 1, true) then s = s .. c end
                    end
                    while #s < 6 do s = s .. "a" end
                    bpParam.value = s
                    Protocol.sendParamSet(0)
                end,
            })
        end
    end, "Mode:", function(parent)
        if modeParam and Protocol.paramFocusable(1) then
            local mfv, mo2f, mf2o = UI.buildFilteredValues(modeParam)
            parent:choice({
                values = mfv,
                get = function() return mo2f[modeParam.value] or 1 end,
                set = function(val) modeParam.value = mf2o[val] or 0; Protocol.sendParamSet(1) end,
                active = function() return modeParam.editable end,
            })
        end
    end)

    UI.mainControlRow2(ctrlBox, "RF Band:", function(parent)
        if rfParam and Protocol.paramFocusable(2) then
            local rfv, ro2f, rf2o = UI.buildFilteredValues(rfParam, freq_band_list)
            parent:choice({
                values = rfv,
                get = function() return ro2f[rfParam.value] or 1 end,
                set = function(val) rfParam.value = rf2o[val] or 0; Protocol.sendParamSet(2) end,
                active = function() return rfParam.editable end,
            })
        end
    end, "Ortho:", function(parent)
        if orthoParam and orthoParam.allowed_mask > 0 and Protocol.paramFocusable(3) then
            local ofv, oo2f, of2o = UI.buildFilteredValues(orthoParam)
            parent:choice({
                values = ofv,
                get = function() return oo2f[orthoParam.value] or 1 end,
                set = function(val) orthoParam.value = of2o[val] or 0; Protocol.sendParamSet(3) end,
                active = function() return orthoParam.editable end,
            })
        end
    end)

    -- Setup layout warning
    if Protocol.DEVICE_ITEM_TX ~= nil and Protocol.DEVICE_ITEM_RX ~= nil and Protocol.connected and Protocol.DEVICE_PARAM_LIST_complete and
           (Protocol.DEVICE_ITEM_TX.setuplayout_int > 515 or Protocol.DEVICE_ITEM_RX.setuplayout_int > 515) then
        if Protocol.DEVICE_ITEM_TX.setuplayout_int < Protocol.DEVICE_ITEM_RX.setuplayout_int then
            pg:build({{type="label", text="Tx param version < Rx. Update Tx firmware!", color=COLOR_THEME_PRIMARY1}})
        elseif Protocol.DEVICE_ITEM_RX.setuplayout_int < Protocol.DEVICE_ITEM_TX.setuplayout_int then
            pg:build({{type="label", text="Rx param version < Tx. Update Rx firmware!", color=COLOR_THEME_PRIMARY1}})
        end
    end

    -- Action buttons (at bottom)
    local btnRow = pg:box({
        w = lvgl.PERCENT_SIZE + 100, flexFlow = lvgl.FLOW_ROW,
        flexPad = lvgl.PAD_TINY,
    })
    btnRow:button({ text = "Edit Tx", w = lvgl.PERCENT_SIZE + 16, press = function()
        if Protocol.DEVICE_PARAM_LIST_complete then App.switchPage("edit_tx") end
    end })
    btnRow:button({ text = "Edit Rx", w = lvgl.PERCENT_SIZE + 16, press = function()
        if Protocol.DEVICE_PARAM_LIST_complete and Protocol.connected then App.switchPage("edit_rx") end
    end, active = function() return Protocol.connected and Protocol.DEVICE_PARAM_LIST_complete end })
    btnRow:button({ text = "Save", w = lvgl.PERCENT_SIZE + 16, press = function()
        if Protocol.DEVICE_PARAM_LIST_complete then Protocol.sendParamStore() end
    end })
    btnRow:button({ text = "Reload", w = lvgl.PERCENT_SIZE + 16, press = function()
        Protocol.clearParams(); UI.invalidate()
    end })
    btnRow:button({ text = "Bind", w = lvgl.PERCENT_SIZE + 16, press = function()
        Protocol.sendBind()
    end })
    btnRow:button({ text = "Tools", w = lvgl.PERCENT_SIZE + 16, press = function()
        if Protocol.DEVICE_PARAM_LIST_complete then App.switchPage("tools") end
    end })
end


function UI.buildEditPage(prefix)
    local title = "mLRS: Edit " .. prefix

    local pageOptions = {
        title = title,
        subtitle = UI.getSubtitle,
        backButton = true,
        back = function()
            App.switchPage("main")
        end,
    }

    UI.currentPage = lvgl.page(pageOptions)

    local container = UI.currentPage:box({
        w = lvgl.PERCENT_SIZE + 100,
        flexFlow = lvgl.FLOW_COLUMN,
        flexPad = lvgl.PAD_TINY,
    })

    -- Header
    if prefix == "Tx" then
        local headerText = "Tx"
        if Protocol.DEVICE_INFO ~= nil then
            headerText = "Tx - " .. tostring(Protocol.DEVICE_INFO.tx_config_id) .. ":"
        end
        container:build({{type="label", text=headerText, color=COLOR_THEME_PRIMARY1, font=BOLD}})
    else
        container:build({{type="label", text="Rx:", color=COLOR_THEME_PRIMARY1, font=BOLD}})
    end

    if not Protocol.DEVICE_PARAM_LIST_complete then
        container:build({{type="label", text="Parameters not loaded"}})
        return
    end

    -- Iterate parameters, filter by prefix
    for pidx = 2, #Protocol.DEVICE_PARAM_LIST do
        local p = Protocol.DEVICE_PARAM_LIST[pidx]
        if p ~= nil and string.sub(p.name, 1, 2) == prefix and p.allowed_mask > 0 then
            if p.typ < MBRIDGE_PARAM_TYPE.LIST then
                UI.createNumberRow(container, p, pidx)
            elseif p.typ == MBRIDGE_PARAM_TYPE.LIST then
                local displayName = string.sub(p.name, 4)
                UI.createFilteredChoiceRow(container, displayName, p, pidx)
            end
        end
    end

    container:rectangle({w = lvgl.PERCENT_SIZE + 100, h = lvgl.PAD_SMALL, thickness = 0})
end


function UI.buildToolsPage()
    local pageOptions = {
        title = "mLRS: Tools",
        backButton = true,
        back = function()
            App.switchPage("main")
        end,
    }

    UI.currentPage = lvgl.page(pageOptions)

    local container = UI.currentPage:box({
        w = lvgl.PERCENT_SIZE + 100,
        flexFlow = lvgl.FLOW_COLUMN,
        flexPad = lvgl.PAD_MEDIUM,
    })

    container:button({
        text = "System Bootloader",
        w = lvgl.PERCENT_SIZE + 100,
        press = function()
            Protocol.sendBoot()
            App.switchPage("main")
        end,
    })

    container:button({
        text = "Flash ESP",
        w = lvgl.PERCENT_SIZE + 100,
        press = function()
            Protocol.sendFlashEsp()
            App.switchPage("main")
        end,
    })
end


----------------------------------------------------------------------
-- UI: Build dispatcher
----------------------------------------------------------------------

function UI.build()
    -- Close any open dialog before clearing - dialogs may be on a separate
    -- LVGL layer that lvgl.clear() does not destroy
    Dialogs.clearBlocked()
    lvgl.clear()

    if App.page == "main" then
        UI.buildMainPage()
    elseif App.page == "edit_tx" then
        UI.buildEditPage("Tx")
    elseif App.page == "edit_rx" then
        UI.buildEditPage("Rx")
    elseif App.page == "tools" then
        UI.buildToolsPage()
    end

    UI.uiBuilt = true
end


-- ============================================================================
-- App: Page switching
-- ============================================================================

function App.switchPage(name)
    App.page = name
    UI.invalidate()
end


-- ============================================================================
-- Mock data for EdgeTX Companion simulator
-- ============================================================================

local function setMock()
    local _, rv = getVersion()
    if string.sub(rv, -5) ~= "-simu" then return end

    -- Parameter set based on mLRS PARAMETERS.md documentation
    local mockParams = {
        -- Common parameters (shown on main page)
        [0]  = {typ=5, name="Bind Phrase",      value="mlrs.1"},
        [1]  = {typ=4, name="Mode",             value=0, options="50 Hz,31 Hz,19 Hz",  mask=7},
        [2]  = {typ=4, name="RF Band",          value=0, options="2.4 GHz,915 MHz FCC", mask=3},
        [3]  = {typ=4, name="Ortho",            value=0, options="off,1/3,2/3,3/3",     mask=15},
        -- Tx parameters
        [4]  = {typ=4, name="Tx Power",         value=2, options="10 mW,50 mW,100 mW",  mask=7},
        [5]  = {typ=4, name="Tx Diversity",     value=0, options="enabled,antenna1,antenna2,r:en. t:ant1,r:en. t:ant2", mask=31},
        [6]  = {typ=4, name="Tx Ch Source",     value=1, options="none,crsf,in,mbridge", mask=15},
        [7]  = {typ=4, name="Tx Ch Order",      value=0, options="AETR,TAER,ETAR",      mask=7},
        [8]  = {typ=4, name="Tx In Mode",       value=0, options="sbus,sbus inv",        mask=3},
        [9]  = {typ=4, name="Tx Ser Dest",      value=0, options="serial,serial2,mbridge", mask=7},
        [10] = {typ=4, name="Tx Ser Baudrate",  value=4, options="9600,19200,38400,57600,115200,230400", mask=63},
        [11] = {typ=4, name="Tx Snd RadioStat", value=0, options="off,1 Hz",            mask=3},
        [12] = {typ=4, name="Tx Buzzer",        value=0, options="off,LP,rxLQ",          mask=7},
        -- Rx parameters
        [13] = {typ=4, name="Rx Power",         value=2, options="10 mW,50 mW,100 mW",  mask=7},
        [14] = {typ=4, name="Rx Diversity",     value=0, options="enabled,antenna1,antenna2,r:en. t:ant1,r:en. t:ant2", mask=31},
        [15] = {typ=4, name="Rx Ch Order",      value=0, options="AETR,TAER,ETAR",      mask=7},
        [16] = {typ=4, name="Rx Out Mode",      value=0, options="sbus,crsf,sbus inv",  mask=7},
        [17] = {typ=4, name="Rx FailSafe Mode", value=0, options="no sig,low thr,by cnf,low thr cnt,ch1ch4 cnt", mask=31},
        [18] = {typ=4, name="Rx Ser Baudrate",  value=3, options="9600,19200,38400,57600,115200,230400", mask=63},
        [19] = {typ=4, name="Rx Ser Link Mode", value=0, options="transp.,mavlink,mavlinkX,mspX", mask=15},
        [20] = {typ=4, name="Rx Out Rssi Ch",   value=0, options="off,5,6,7,8,9,10,11,12,13,14,15,16", mask=8191},
        [21] = {typ=4, name="Rx Out LQ Ch",     value=0, options="off,5,6,7,8,9,10,11,12,13,14,15,16", mask=8191},
        [22] = {typ=4, name="Rx Buzzer",        value=0, options="off,LP",               mask=3},
    }
    local MOCK_PARAM_COUNT = 23

    -- Version v1.4.01: (1<<12)|(4<<6)|1
    local VER_U16 = 4353
    -- Setup layout v0.5.15: (0<<12)|(5<<6)|15  ->  setuplayout_int = 515
    local LAYOUT_U16 = 335

    local responseQueue = {}

    local function enqueue(cmd, payload)
        responseQueue[#responseQueue + 1] = {
            cmd = cmd,
            len = MBRIDGE_CMD_LEN[cmd] or 24,
            payload = payload,
        }
    end

    local function encStr(p, pos, str, maxLen)
        for i = 0, maxLen - 1 do
            if i < #str then
                p[pos + i] = string.byte(str, i + 1)
            else
                p[pos + i] = 0
            end
        end
    end

    local function encU16(p, pos, val)
        p[pos]     = bit32.band(val, 0xFF)
        p[pos + 1] = bit32.rshift(val, 8)
    end

    local function buildDeviceItem(name)
        local p = {}
        encU16(p, 0, VER_U16)
        encU16(p, 2, LAYOUT_U16)
        encStr(p, 4, name, 20)
        return p
    end

    local function buildInfo()
        local p = {}
        encU16(p, 0, 65424)     -- receiver_sensitivity = -112 dBm (as u16)
        p[2] = 1                -- has_status=1, binding=0
        p[3] = 20               -- tx_power_dbm
        p[4] = 14               -- rx_power_dbm
        p[5] = 1                -- rx_available
        p[6] = 42               -- tx_config_id
        p[7] = 0                -- diversity flags
        p[8] = MOCK_PARAM_COUNT
        for i = 9, 23 do p[i] = 0 end
        return p
    end

    local function buildParamItem(idx)
        local mp = mockParams[idx]
        if not mp then return nil end
        local p = {}
        p[0] = idx
        p[1] = mp.typ
        encStr(p, 2, mp.name, 16)
        if mp.typ == MBRIDGE_PARAM_TYPE.STR6 then
            encStr(p, 18, mp.value, 6)
        else
            p[18] = (type(mp.value) == "number") and mp.value or 0
            for i = 19, 23 do p[i] = 0 end
        end
        return p
    end

    local function buildParamItem2(idx)
        local mp = mockParams[idx]
        if not mp then return nil end
        local p = {}
        p[0] = idx
        if mp.typ == MBRIDGE_PARAM_TYPE.LIST then
            encU16(p, 1, mp.mask)
            encStr(p, 3, mp.options, 21)
        elseif mp.typ < MBRIDGE_PARAM_TYPE.LIST then
            p[1] = mp.min or 0; p[2] = 0
            p[3] = mp.max or 0; p[4] = 0
            p[5] = 0; p[6] = 0
            encStr(p, 7, mp.unit or "", 6)
            for i = 13, 23 do p[i] = 0 end
        else -- STR6
            for i = 1, 23 do p[i] = 0 end
        end
        return p
    end

    -- ITEM3: continuation bytes for LIST options > 20 chars
    local function buildItem3(idx, opts)
        local p = {}
        p[0] = idx
        for i = 0, 22 do
            local si = 21 + i
            if si < #opts then
                p[1 + i] = string.byte(opts, si + 1)
            else
                p[1 + i] = 0
            end
        end
        return p
    end

    -- ITEM4: continuation bytes for LIST options > 43 chars
    local function buildItem4(idx, opts)
        local p = {}
        p[0] = idx + 128
        for i = 0, 22 do
            local si = 44 + i
            if si < #opts then
                p[1 + i] = string.byte(opts, si + 1)
            else
                p[1 + i] = 0
            end
        end
        return p
    end

    Protocol.isConnected = function() return true end

    Protocol.cmdPush = function(cmd, payload)
        if cmd == MBRIDGE_CMD.REQUEST_INFO then
            enqueue(MBRIDGE_CMD.DEVICE_ITEM_TX, buildDeviceItem("Mock Tx Module"))
            enqueue(MBRIDGE_CMD.DEVICE_ITEM_RX, buildDeviceItem("Mock Rx Module"))
            enqueue(MBRIDGE_CMD.INFO, buildInfo())
        elseif cmd == MBRIDGE_CMD.REQUEST_CMD then
            local paramIdx = payload[2]
            if mockParams[paramIdx] then
                enqueue(MBRIDGE_CMD.PARAM_ITEM, buildParamItem(paramIdx))
                enqueue(MBRIDGE_CMD.PARAM_ITEM2, buildParamItem2(paramIdx))
                -- Queue ITEM3/ITEM4 for LIST options that overflow ITEM2's 21 bytes
                local mp = mockParams[paramIdx]
                if mp.typ == MBRIDGE_PARAM_TYPE.LIST and mp.options and #mp.options >= 21 then
                    enqueue(MBRIDGE_CMD.PARAM_ITEM3_4, buildItem3(paramIdx, mp.options))
                    if #mp.options >= 44 then
                        enqueue(MBRIDGE_CMD.PARAM_ITEM3_4, buildItem4(paramIdx, mp.options))
                    end
                end
            else
                -- End-of-list marker
                local eol = {}; eol[0] = 255
                for i = 1, 23 do eol[i] = 0 end
                enqueue(MBRIDGE_CMD.PARAM_ITEM, eol)
            end
        elseif cmd == MBRIDGE_CMD.PARAM_SET then
            local idx = payload[1]
            if mockParams[idx] then
                if mockParams[idx].typ == MBRIDGE_PARAM_TYPE.STR6 then
                    local s = ""
                    for i = 2, 7 do
                        if payload[i] and payload[i] > 0 then
                            s = s .. string.char(payload[i])
                        end
                    end
                    mockParams[idx].value = s
                else
                    mockParams[idx].value = payload[2] or 0
                end
            end
        end
        -- PARAM_STORE, BIND_START, etc: silently accepted
    end

    Protocol.cmdPop = function()
        if #responseQueue > 0 then
            return table.remove(responseQueue, 1)
        end
        return nil
    end
end


-- ============================================================================
-- LVGL support check fallback
-- ============================================================================

local function showLvglRequired()
    lcd.clear()
    lcd.drawText(10, 10, "LVGL support required", MIDSIZE)
    lcd.drawText(10, 30, "Color LCD radio with", 0)
    lcd.drawText(10, 50, "EdgeTX 2.11.5+ needed", 0)
end


-- ============================================================================
-- Init
-- ============================================================================

local function init()
    if lvgl == nil then
        return
    end

    Protocol.setupBridge()
    setMock()

    local tnow_10ms = getTime()

    Protocol.DEVICE_DOWNLOAD_is_running = true
    App.isFirstParamDownload = true
    App.firstParamDownloadTmo_10ms = tnow_10ms + 500

    if tnow_10ms < 300 then
        Protocol.DEVICE_SAVE_t_last = 300 - tnow_10ms
    else
        Protocol.DEVICE_SAVE_t_last = 0
    end
end


-- ============================================================================
-- Run (main coordinator)
-- ============================================================================

local function run(event, touchState)
    if event == nil then return 2 end

    -- Check for LVGL support
    if lvgl == nil then
        showLvglRequired()
        return 0
    end

    -- Check for mBridge/CRSF module
    if not App.moduleChecked then
        App.moduleChecked = true
        if Protocol.isConnected == nil then
            if mbridge == nil or not mbridge.enabled() then
                if model.getModule(0).Type ~= 5 and model.getModule(1).Type ~= 5 then
                    lvgl.clear()
                    local dg = lvgl.dialog({
                        title = "mLRS: No Module Found",
                        flexFlow = lvgl.FLOW_COLUMN,
                        flexPad = lvgl.PAD_SMALL,
                        close = function() App.shouldExit = true end,
                    })
                    dg:build({
                        {type="label", text="mLRS not accessible:"},
                        {type="label", text="mBridge or CRSF not enabled!"},
                        {type="box", flexFlow=lvgl.FLOW_ROW, w=lvgl.PERCENT_SIZE+100, align=CENTER, children={
                            {type="button", text="Exit", w=lvgl.PERCENT_SIZE+98, press=function()
                                dg:close()
                                App.shouldExit = true
                            end},
                        }},
                    })
                    UI.uiBuilt = true
                end
            end
        end
    end

    if App.shouldExit then return 2 end

    if Protocol.isConnected == nil then
        return 0
    end

    -- Protocol polling
    Protocol.doConnected()

    -- Connection state transitions
    if Protocol.has_connected then
        Protocol.clearParams()
        Protocol.DEVICE_BINDING = false
        UI.setSubtitleMsg("Receiver connected!", 100)
    end
    if Protocol.has_disconnected then
        UI.setSubtitleMsg("Receiver disconnected!", 100)
    end
    if not Protocol.connected and App.page == "edit_rx" then
        App.switchPage("main")
    end

    Protocol.doParamLoop()
    Protocol.checkBind()

    -- Auto-reload params after save dead time expires
    if Protocol.DEVICE_SAVE_reload_pending and getTime() > Protocol.DEVICE_SAVE_t_last + paramLoadDeadTime_10ms then
        Protocol.DEVICE_SAVE_reload_pending = false
        Protocol.clearParams()
        UI.invalidate()
    end

    -- CRSF baudrate warning on first param download
    if App.isFirstParamDownload and Protocol.DEVICE_DOWNLOAD_is_running then
        if App.firstParamDownloadTmo_10ms > 0 and getTime() > App.firstParamDownloadTmo_10ms then
            if not App.baudRateWarningShown then
                App.baudRateWarningShown = true
                UI.setSubtitleMsg("Check CRSF baudrate is 400k!", 300)
            end
            App.firstParamDownloadTmo_10ms = 0
        end
    end

    -- Detect param loading state transitions
    local paramsNowComplete = Protocol.DEVICE_PARAM_LIST_complete
    if paramsNowComplete ~= UI.paramsWereComplete then
        if paramsNowComplete then
            App.isFirstParamDownload = false
            collectgarbage("collect")
        end
        UI.invalidate()
    end
    UI.paramsWereComplete = paramsNowComplete

    -- Build/rebuild UI when needed (skip while blocked dialog is showing)
    if not UI.uiBuilt and Dialogs.popup == nil then
        UI.build()
    end

    if App.shouldExit then return 2 end

    return 0
end


-- ============================================================================
-- Return
-- ============================================================================

return { init = init, run = run, useLvgl = true }
