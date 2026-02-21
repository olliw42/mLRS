-- mlrs.lua
-- Lightweight queued API for mLRS modules (Ethos / CRSF sensor frames)
-- Pull-based parameter loading (REQUEST_CMD -> PARAM_ITEM by index), plus full ITEM2/3/4 enrichment.

local M = {}

-- mBridge constants
local A0 = 0xA0
local CMD_TX_LINK_STATS       = 2
local CMD_REQUEST_INFO        = 3
local CMD_DEVICE_ITEM_TX      = 4
local CMD_DEVICE_ITEM_RX      = 5
local CMD_PARAM_REQUEST_LIST  = 6
local CMD_PARAM_ITEM          = 7
local CMD_PARAM_ITEM2         = 8
local CMD_PARAM_ITEM3         = 9
local CMD_REQUEST_CMD         = 10
local CMD_INFO                = 11
local CMD_PARAM_SET           = 12
local CMD_PARAM_STORE         = 13
local CMD_BIND_START          = 14
local CMD_BIND_STOP           = 15
local CMD_MODELID_SET         = 16
local CMD_SYSTEM_BOOTLOADER   = 17

-- types
local T_UINT8, T_INT8, T_UINT16, T_INT16, T_LIST, T_STR6 = 0, 1, 2, 3, 4, 5

local function now() return os.clock() end

-- ---------- sensor wrapper ----------
local function makeSensor()
  if crsf and crsf.getSensor then
    return crsf.getSensor()
  end
  return {
    popFrame  = function(_, id) return crsf.popFrame(id) end,
    pushFrame = function(_, id, data) return crsf.pushFrame(id, data) end,
  }
end

local function drainFrames(sensor, max)
  max = max or 64
  for _ = 1, max do
    local cmd = sensor:popFrame(130)
    if not cmd then break end
  end
end

-- ---------- bit helpers (Ethos-friendly) ----------
local function band(a, b) return (a or 0) & (b or 0) end
local function rshift(a, n) return (a or 0) >> (n or 0) end
local function lshift(a, n) return (a or 0) << (n or 0) end
local function btest(mask, bits) return band(bits, mask) ~= 0 end

local function popcount16(x)
  x = x or 0
  local c = 0
  for _ = 1, 16 do
    if (x & 1) ~= 0 then c = c + 1 end
    x = x >> 1
  end
  return c
end

local function allowed_mask_editable(mask)
  -- same intent as the script: if none or only one option allowed -> not editable
  if not mask or mask == 0 then return false end
  return popcount16(mask) > 1
end

-- ---------- payload helpers (0-based offsets) ----------
local function u8(p, ofs0) return (p[ofs0 + 1] or 0) & 0xFF end
local function i8(p, ofs0)
  local v = u8(p, ofs0)
  if v >= 128 then v = v - 256 end
  return v
end
local function u16(p, ofs0) return u8(p, ofs0) + (u8(p, ofs0 + 1) << 8) end
local function i16(p, ofs0)
  local v = u16(p, ofs0)
  if v >= 32768 then v = v - 65536 end
  return v
end

local function mb_str(p, ofs0, n)
  local s = {}
  for k = 0, n - 1 do
    local b = p[ofs0 + 1 + k]
    if not b or b == 0 then break end
    s[#s + 1] = string.char(b)
  end
  return table.concat(s)
end

local function mb_value_by_type(p, ofs0, typ)
  if typ == T_UINT8 then return u8(p, ofs0) end
  if typ == T_INT8 then return i8(p, ofs0) end
  if typ == T_UINT16 then return u16(p, ofs0) end
  if typ == T_INT16 then return i16(p, ofs0) end
  if typ == T_LIST then return u8(p, ofs0) end
  return u8(p, ofs0)
end

local function mb_value_or_str6(p, ofs0, typ)
  if typ == T_STR6 then
    return mb_str(p, ofs0, 6)
  end
  return mb_value_by_type(p, ofs0, typ)
end

local function bytes_to_string(bytes)
  local s = {}
  for i = 1, #bytes do
    local b = bytes[i]
    if not b or b == 0 then break end
    s[#s + 1] = string.char(b)
  end
  return table.concat(s)
end

local function split_csv(str)
  local opts = {}
  if not str or str == "" then return opts end
  for part in string.gmatch(str .. ",", "([^,]+)") do
    part = part:match("^%s*(.-)%s*$")
    if part ~= "" then opts[#opts + 1] = part end
  end
  return opts
end

local function take_bytes(payload, ofs0, len)
  local out = {}
  for i = 0, len - 1 do
    out[#out + 1] = u8(payload, ofs0 + i)
  end
  return out
end

local function segment_is_full(payload, ofs0, len)
  -- "full" means no terminator encountered within len bytes (same heuristic as string.len==len)
  -- i.e. last byte nonzero and all previous nonzero up to last; easiest: scan for 0
  for i = 0, len - 1 do
    if u8(payload, ofs0 + i) == 0 then return false end
  end
  return true
end

-- ---------- command lengths ----------
local function cmd_len(cmd)
  if cmd == CMD_TX_LINK_STATS then return 22 end
  if cmd == CMD_DEVICE_ITEM_TX or cmd == CMD_DEVICE_ITEM_RX then return 24 end
  if cmd == CMD_PARAM_ITEM or cmd == CMD_PARAM_ITEM2 or cmd == CMD_PARAM_ITEM3 then return 24 end
  if cmd == CMD_REQUEST_CMD then return 18 end
  if cmd == CMD_INFO then return 24 end
  if cmd == CMD_PARAM_SET then return 7 end
  if cmd == CMD_MODELID_SET then return 3 end
  return 0
end

local function pushMB(sensor, cmd, payload)
  local data = { string.byte("O"), string.byte("W"), A0 + cmd }
  local need = cmd_len(cmd)
  for _ = 1, need do data[#data + 1] = 0 end
  for i = 1, #payload do data[3 + i] = payload[i] end
  return sensor:pushFrame(129, data)
end

-- ---------- internal model ----------
local function newModel()
  return {
    gotInfo = false,

    txItem = nil,
    rxItem = nil,
    info = nil,

    params = {},               -- [idx1] = param
    paramsComplete = false,

    lastParamRxAt = nil,
    lastRxAt = nil,

    lastInfoReqAt = nil,
    lastDeviceReqAt = nil,
    lastListReqAt = nil,
    lastIndexReqAt = nil,
  }
end

local function ensureParam(model, idx0)
  local idx1 = idx0 + 1
  model.params[idx1] = model.params[idx1] or {
    idx0 = idx0,
    -- enrichment state
    _gotItem = false,
    _gotItem2 = false,
    _needItem3 = false,
    _gotItem3 = false,
    _needItem4 = false,
    _gotItem4 = false,
    _optBytes = nil,
  }
  return model.params[idx1]
end

-- ---------- decode DEVICE_ITEM_TX/RX ----------
local function decode_device_item(payload)
  local version_u16 = u16(payload, 0)
  local setuplayout_u16 = u16(payload, 2)
  local name = mb_str(payload, 4, 20)

  -- The OlliW script maps version bits; we’ll keep simple here:
  local function version_to_int(v)
    local major = (v & 0xF000) >> 12
    local minor = (v & 0x0FC0) >> 6
    local patch = (v & 0x003F)
    return major * 10000 + minor * 100 + patch
  end
  local function version_to_str(v)
    local major = (v & 0xF000) >> 12
    local minor = (v & 0x0FC0) >> 6
    local patch = (v & 0x003F)
    return string.format("v%d.%d.%02d", major, minor, patch)
  end

  return {
    version_u16 = version_u16,
    setuplayout_u16 = setuplayout_u16,
    name = name,
    version_int = version_to_int(version_u16),
    version_str = version_to_str(version_u16),
    setuplayout_int = version_to_int(setuplayout_u16),
  }
end

-- ---------- decode INFO ----------
local function decode_info(payload)
  -- matches positions used in the pasted script
  local receiver_sensitivity = i16(payload, 0)
  local flags2 = u8(payload, 2)
  local tx_power_dbm = i8(payload, 3)
  local rx_power_dbm = i8(payload, 4)
  local flags5 = u8(payload, 5)
  local tx_config_id = u8(payload, 6)
  local div = u8(payload, 7)

  return {
    receiver_sensitivity = receiver_sensitivity,
    has_status = (flags2 & 0x01) ~= 0 and 1 or 0,
    binding = (flags2 & 0x02) ~= 0 and 1 or 0,
    tx_power_dbm = tx_power_dbm,
    rx_power_dbm = rx_power_dbm,
    rx_available = (flags5 & 0x01) ~= 0 and 1 or 0,
    tx_config_id = tx_config_id,
    tx_diversity = div & 0x0F,
    rx_diversity = (div >> 4) & 0x0F,
  }
end

-- ---------- decode PARAM items ----------
local function on_PARAM_ITEM(model, payload)
  local idx0 = payload[1]
  if idx0 == nil then return end
  model.lastParamRxAt = now()

  if idx0 == 255 then
    model.paramsComplete = true
    return
  end

  local p = ensureParam(model, idx0)
  p.typ = u8(payload, 1)
  p.name = mb_str(payload, 2, 16)
  p.value = mb_value_or_str6(payload, 18, p.typ)

  -- defaults
  p.min = p.min or 0
  p.max = p.max or 0
  p.unit = p.unit or ""
  p.options = p.options or {}
  p.allowed_mask = p.allowed_mask or 0
  p.editable = (p.editable ~= nil) and p.editable or true

  p._gotItem = true
end

local function finalize_list_options(p)
  if not p._optBytes then return end
  local s = bytes_to_string(p._optBytes)
  p.options = split_csv(s)
  p.min = 0
  p.max = math.max(#p.options - 1, 0)
end

local function on_PARAM_ITEM2(model, payload)
  local idx0 = payload[1]
  if idx0 == nil or idx0 == 255 then return end
  model.lastParamRxAt = now()

  local p = model.params[idx0 + 1]
  if not p then return end

  if p.typ ~= nil and p.typ < T_LIST then
    p.min = mb_value_by_type(payload, 1, p.typ)
    p.max = mb_value_by_type(payload, 3, p.typ)
    p.unit = mb_str(payload, 7, 6)
    p._gotItem2 = true
    return
  end

  if p.typ == T_LIST then
    p.allowed_mask = u16(payload, 1)
    p.editable = allowed_mask_editable(p.allowed_mask)

    -- options chunk from ITEM2 at ofs3 len21
    p._optBytes = p._optBytes or {}
    local chunk = take_bytes(payload, 3, 21)
    for i = 1, #chunk do p._optBytes[#p._optBytes + 1] = chunk[i] end
    finalize_list_options(p)

    -- if chunk fully packed (no 0), expect ITEM3
    p._needItem3 = segment_is_full(payload, 3, 21)
    p._gotItem2 = true

    -- if not expecting ITEM3, we are fully ready now
    if not p._needItem3 then
      p._gotItem3, p._needItem4, p._gotItem4 = false, false, false
    end
  end
end

local function on_PARAM_ITEM3(model, payload)
  -- supports "ITEM4" encoding via index>=128 (same trick as pasted script)
  local rawIndex = payload[1]
  if rawIndex == nil then return end
  model.lastParamRxAt = now()

  local is_item4 = false
  local idx0 = rawIndex
  if idx0 >= 128 then
    idx0 = idx0 - 128
    is_item4 = true
  end

  local p = model.params[idx0 + 1]
  if not p or p.typ ~= T_LIST then return end

  p._optBytes = p._optBytes or {}

  -- ITEM3/ITEM4 options chunk at ofs1 len23
  local chunk = take_bytes(payload, 1, 23)
  for i = 1, #chunk do p._optBytes[#p._optBytes + 1] = chunk[i] end
  finalize_list_options(p)

  if not is_item4 then
    p._gotItem3 = true
    p._needItem4 = segment_is_full(payload, 1, 23) -- if fully packed, expect ITEM4
  else
    p._gotItem4 = true
    p._needItem4 = false
  end
end

local function param_is_ready(p, full)
  if not p or not p._gotItem then return false end
  if not full then return true end

  -- full means:
  --  numeric: wait for ITEM2 (min/max/unit)
  --  list: wait for ITEM2, and if needItem3/4 then wait for them
  if p.typ ~= nil and p.typ < T_LIST then
    return p._gotItem2
  end
  if p.typ == T_LIST then
    if not p._gotItem2 then return false end
    if p._needItem3 and not p._gotItem3 then return false end
    if p._needItem4 and not p._gotItem4 then return false end
    return true
  end
  if p.typ == T_STR6 then
    -- item2 is usually empty; ITEM is enough
    return true
  end
  return true
end

-- ---------- frame handler ----------
local function handleFrame(model, cmd, data)
  if cmd ~= 130 or not data or not data[1] then return end
  local mcmd = data[1] - A0

  local payload = {}
  for i = 2, #data do payload[#payload + 1] = data[i] end
  model.lastRxAt = now()

  if mcmd == CMD_DEVICE_ITEM_TX then
    model.txItem = decode_device_item(payload)
  elseif mcmd == CMD_DEVICE_ITEM_RX then
    model.rxItem = decode_device_item(payload)
  elseif mcmd == CMD_INFO then
    model.gotInfo = true
    model.info = decode_info(payload)
  elseif mcmd == CMD_PARAM_ITEM then
    on_PARAM_ITEM(model, payload)
  elseif mcmd == CMD_PARAM_ITEM2 then
    on_PARAM_ITEM2(model, payload)
  elseif mcmd == CMD_PARAM_ITEM3 then
    on_PARAM_ITEM3(model, payload)
  end
end

-- ---------- queue ----------
local function finish(req, ok, dataOrErr)
  req.done = true
  if req.cb then
    if ok then req.cb({ ok = true, type = req.type, data = dataOrErr })
    else req.cb({ ok = false, type = req.type, err = dataOrErr }) end
  end
end

local function activeReq(self)
  for i = 1, #self.queue do
    if not self.queue[i].done then return self.queue[i] end
  end
  return nil
end

local function enqueue(self, req)
  req.id = self.nextReqId
  self.nextReqId = self.nextReqId + 1
  self.queue[#self.queue + 1] = req
  return req.id
end

-- ---------- public API ----------
function M.new(opts)
  local self = {
    sensor = makeSensor(),
    model = newModel(),
    queue = {},
    nextReqId = 1,

    settleS = (opts and opts.settleS) or 1.5,
    createdAt = now(),

    -- by_index uses REQUEST_CMD {PARAM_ITEM, idx}
    -- list uses PARAM_REQUEST_LIST (streaming)
    strategy = (opts and opts.strategy) or "by_index",

    -- after STORE, pause requests for a bit (mirrors paramLoadDeadTime idea)
    saveDeadS = (opts and opts.saveDeadS) or 3.0,
    lastStoreAt = nil,
  }

  drainFrames(self.sensor)
  setmetatable(self, { __index = M })
  return self
end

function M.reset(self)
  self.model = newModel()
  self.queue = {}
  self.createdAt = now()
  self.lastStoreAt = nil
  drainFrames(self.sensor)
end

function M.getInfo(self, cb)
  return enqueue(self, { type = "GET_INFO", cb = cb, deadline = now() + 8.0 })
end

function M.getDeviceItems(self, cb)
  return enqueue(self, { type = "GET_DEVICE_ITEMS", cb = cb, deadline = now() + 8.0 })
end

function M.getAllParams(self, cb, opts)
  opts = opts or {}
  return enqueue(self, {
    type = "GET_ALL_PARAMS",
    cb = cb,
    deadline = now() + (opts.deadlineS or 25.0),
    full = (opts.full ~= false), -- default true
  })
end

function M.getParam(self, idx0, cb, opts)
  -- backwards compatible: getParam(idx0, cb)
  opts = opts or {}
  return enqueue(self, {
    type = "GET_PARAM",
    idx0 = idx0,
    cb = cb,
    deadline = now() + (opts.deadlineS or 10.0),
    full = (opts.full ~= false), -- default true
  })
end

function M.setParam(self, idx0, value, cb)
  return enqueue(self, { type = "SET_PARAM", idx0 = idx0, value = value, cb = cb, deadline = now() + 6.0 })
end

function M.store(self, cb)
  return enqueue(self, { type = "STORE", cb = cb, deadline = now() + 25.0 })
end

function M.bindStart(self, cb)
  return enqueue(self, { type = "BIND_START", cb = cb, deadline = now() + 6.0 })
end

function M.bindStop(self, cb)
  return enqueue(self, { type = "BIND_STOP", cb = cb, deadline = now() + 6.0 })
end

function M.bootloader(self, cb)
  return enqueue(self, { type = "BOOT", cb = cb, deadline = now() + 6.0 })
end

-- ---------- request primitives ----------
local function maybeRequestInfo(self)
  local m = self.model
  local t = now()
  if m.gotInfo then return end
  if (not m.lastInfoReqAt) or (t - m.lastInfoReqAt > 1.0) then
    pushMB(self.sensor, CMD_REQUEST_INFO, {})
    m.lastInfoReqAt = t
  end
end

local function maybeRequestDeviceItems(self)
  -- In mLRS, REQUEST_INFO triggers DEVICE_ITEM_TX, DEVICE_ITEM_RX, and INFO (per pasted script).
  -- So we just re-use REQUEST_INFO, but gate frequency.
  local m = self.model
  local t = now()
  if m.txItem and m.rxItem then return end
  if (not m.lastDeviceReqAt) or (t - m.lastDeviceReqAt > 1.0) then
    pushMB(self.sensor, CMD_REQUEST_INFO, {})
    m.lastDeviceReqAt = t
  end
end

local function requestParamList(self)
  local m = self.model
  local t = now()
  if (not m.lastListReqAt) or (t - m.lastListReqAt > 2.0) then
    m.paramsComplete = false
    pushMB(self.sensor, CMD_PARAM_REQUEST_LIST, {})
    m.lastListReqAt = t
  end
end

local function requestParamByIndex(self, idx0)
  local m = self.model
  local t = now()
  -- keep this modest; the upstream script drives ~30Hz (33 * 10ms)
  if (not m.lastIndexReqAt) or (t - m.lastIndexReqAt > 0.10) then
    pushMB(self.sensor, CMD_REQUEST_CMD, { CMD_PARAM_ITEM, idx0 & 0xFF })
    m.lastIndexReqAt = t
  end
end

-- ---------- pump ----------
function M.processQueue(self, maxFrames)
  maxFrames = maxFrames or 24

  -- settle window
  if (now() - self.createdAt) < self.settleS then return end

  -- after store deadtime (avoid hammering while device reboots / re-enumerates)
  if self.lastStoreAt and (now() - self.lastStoreAt) < self.saveDeadS then
    -- still read frames though
  end

  -- RX: consume frames
  for _ = 1, maxFrames do
    local cmd, data = self.sensor:popFrame(130)
    if not cmd then break end
    handleFrame(self.model, cmd, data)
  end

  local req = activeReq(self)
  if not req then return end

  if now() > (req.deadline or 0) then
    finish(req, false, "timeout")
    return
  end

  -- deadtime blocks TX-side requests, but allows RX consumption
  if self.lastStoreAt and (now() - self.lastStoreAt) < self.saveDeadS then
    return
  end

  -- ---- GET_INFO ----
  if req.type == "GET_INFO" then
    if self.model.gotInfo and self.model.info then
      finish(req, true, { info = self.model.info })
    else
      maybeRequestInfo(self)
    end
    return
  end

  -- ---- GET_DEVICE_ITEMS ----
  if req.type == "GET_DEVICE_ITEMS" then
    -- IMPORTANT:
    -- Some setups legitimately have no RX (or it's offline).
    -- Use INFO.rx_available to decide whether RX is expected.
    local haveTx = (self.model.txItem ~= nil)
    local haveRx = (self.model.rxItem ~= nil)
    local rxAvail = (self.model.info and self.model.info.rx_available) -- 1 or 0 or nil

    if haveTx and (haveRx or rxAvail == 0) then
      finish(req, true, { tx = self.model.txItem, rx = self.model.rxItem, info = self.model.info })
    else
      maybeRequestDeviceItems(self)
      maybeRequestInfo(self)
    end
    return
  end

  -- for everything else: require INFO first (keeps behaviour sane)
  if not self.model.gotInfo then
    maybeRequestInfo(self)
    return
  end

  -- ---- SET_PARAM ----
  if req.type == "SET_PARAM" then
  if not req.sent then
    local payload
    if type(req.value) == "string" then
      -- STR6: send idx0 + 6 ASCII bytes (padded/truncated by UI)
      payload = { req.idx0 & 0xFF }
      local s = tostring(req.value or "")
      for i = 1, 6 do
        local b = string.byte(s, i) or 0
        payload[#payload + 1] = b & 0xFF
      end
    else
      -- numeric / list: 1 byte value
      payload = { req.idx0 & 0xFF, (req.value or 0) & 0xFF }
    end
    pushMB(self.sensor, CMD_PARAM_SET, payload)
    req.sent = true
    finish(req, true, { sent = true })
  end
  return
end


  -- ---- STORE ----
  if req.type == "STORE" then
    if not req.sent then
      pushMB(self.sensor, CMD_PARAM_STORE, {})
      req.sent = true
      self.lastStoreAt = now()
      -- After store, your UI can enqueue a reload/getAllParams later
      finish(req, true, { sent = true })
    end
    return
  end

  -- ---- BIND ----
  if req.type == "BIND_START" then
    if not req.sent then
      pushMB(self.sensor, CMD_BIND_START, {})
      req.sent = true
      finish(req, true, { sent = true })
    end
    return
  end

  -- ---- BIND_STOP ----
  if req.type == "BIND_STOP" then
    if not req.sent then
      pushMB(self.sensor, CMD_BIND_STOP, {})
      req.sent = true
      finish(req, true, { sent = true })
    end
    return
  end

  -- ---- BOOT ----
  if req.type == "BOOT" then
    if not req.sent then
      pushMB(self.sensor, CMD_SYSTEM_BOOTLOADER, {})
      req.sent = true
      finish(req, true, { sent = true })
    end
    return
  end

  -- ---- GET_PARAM ----
  if req.type == "GET_PARAM" then
    local p = self.model.params[(req.idx0 or 0) + 1]
    if param_is_ready(p, req.full) then
      finish(req, true, { param = p })
      return
    end

    if self.strategy == "by_index" then
      requestParamByIndex(self, req.idx0 or 0)
    else
      if not req.streamStarted then
        req.streamStarted = true
        requestParamList(self)
      end
    end
    return
  end

  -- ---- GET_ALL_PARAMS ----
  if req.type == "GET_ALL_PARAMS" then
    if self.model.paramsComplete then
      finish(req, true, { params = self.model.params, complete = true })
      return
    end

    if self.strategy == "by_index" then
      if req.nextIdx0 == nil then req.nextIdx0 = 0 end

      -- When current index is fully ready, advance to next.
      local p = self.model.params[req.nextIdx0 + 1]
      if param_is_ready(p, req.full) then
        req.nextIdx0 = req.nextIdx0 + 1
        if req.nextIdx0 > 255 then
          -- safety (normally complete triggered by idx0==255 item)
          finish(req, true, { params = self.model.params, complete = true })
          return
        end
      end

      requestParamByIndex(self, req.nextIdx0)
    else
      requestParamList(self)
    end
    return
  end
end

return M
