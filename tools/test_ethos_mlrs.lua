-- Headless replay harness for lua/Ethos/mlrs.lua.
-- Stubs the Ethos crsf sensor and os.clock with a fake mLRS Tx module, then
-- drives the request queue through load / retry / reset / store scenarios.
-- Run with Lua 5.4 (the Ethos dialect); an optional argument names another mlrs.lua:
--   lua5.4 tools/test_ethos_mlrs.lua [path/to/mlrs.lua]

local A0 = 0xA0
local CMD = {
  REQUEST_INFO = 3, DEVICE_ITEM_TX = 4, DEVICE_ITEM_RX = 5, PARAM_ITEM = 7,
  PARAM_ITEM2 = 8, PARAM_ITEM3 = 9, REQUEST_CMD = 10, INFO = 11,
  PARAM_SET = 12, PARAM_STORE = 13, BIND_START = 14, BIND_STOP = 15,
  SYSTEM_BOOTLOADER = 17, FLASH_ESP = 18,
}
local T_UINT8, T_UINT16, T_LIST, T_STR6 = 0, 2, 4, 5

-- ---------- fake clock ----------
local T = 0
os.clock = function() return T end

-- ---------- fake module ----------
local module = {}

local function version_u16(major, minor, patch)
  return (major << 12) | (minor << 6) | patch
end

local function put_str(bytes, ofs0, s, n)
  for i = 1, n do
    local b = string.byte(s, i) or 0
    bytes[ofs0 + i] = b
  end
end

local function payload(len)
  local p = {}
  for i = 1, len do p[i] = 0 end
  return p
end

local function frame(cmd, p)
  local data = { A0 + cmd }
  for i = 1, #p do data[#data + 1] = p[i] end
  return data
end

local function device_item(version, layout, name)
  local p = payload(24)
  p[1] = version & 0xFF; p[2] = (version >> 8) & 0xFF
  p[3] = layout & 0xFF; p[4] = (layout >> 8) & 0xFF
  put_str(p, 4, name, 20)
  return p
end

local function info_payload(rx_available, binding, param_num)
  local p = payload(24)
  p[3] = 0x01 | (binding and 0x02 or 0) -- has_status, binding
  p[6] = rx_available and 1 or 0
  p[9] = param_num
  return p
end

local function param_item(idx0, def)
  local p = payload(24)
  p[1] = idx0
  p[2] = def.typ
  put_str(p, 2, def.name, 16)
  if def.typ == T_STR6 then
    put_str(p, 18, def.value, 6)
  else
    p[19] = def.value & 0xFF
    p[20] = (def.value >> 8) & 0xFF
  end
  return p
end

-- returns the list of frames a module sends for one PARAM_ITEM request
local function param_frames(idx0, def)
  local out = { frame(CMD.PARAM_ITEM, param_item(idx0, def)) }
  if def.typ == T_STR6 then return out end

  local p2 = payload(24)
  p2[1] = idx0
  if def.typ ~= T_LIST then
    p2[2] = def.min & 0xFF; p2[3] = (def.min >> 8) & 0xFF
    p2[4] = def.max & 0xFF; p2[5] = (def.max >> 8) & 0xFF
    put_str(p2, 7, def.unit or "", 6)
    out[#out + 1] = frame(CMD.PARAM_ITEM2, p2)
    return out
  end

  local mask = def.allowed_mask or 0xFFFF
  p2[2] = mask & 0xFF; p2[3] = (mask >> 8) & 0xFF
  local opt = table.concat(def.options, ",")
  put_str(p2, 3, opt:sub(1, 21), 21)
  out[#out + 1] = frame(CMD.PARAM_ITEM2, p2)
  if #opt >= 21 then
    local p3 = payload(24)
    p3[1] = idx0
    put_str(p3, 1, opt:sub(22, 44), 23)
    out[#out + 1] = frame(CMD.PARAM_ITEM3, p3)
    if #opt >= 44 then
      local p4 = payload(24)
      p4[1] = idx0 + 128
      put_str(p4, 1, opt:sub(45, 67), 23)
      out[#out + 1] = frame(CMD.PARAM_ITEM3, p4)
    end
  end
  return out
end

function module.reset(opts)
  module.params = opts.params
  module.rx_available = opts.rx_available
  module.binding = opts.binding or false
  module.outq = {}            -- { at = time, data = frame }
  module.sent = {}            -- commands received from the script
  module.info_requests = 0
  module.drop = opts.drop or function() return false end
  module.latency = opts.latency or 0
  module.item_requests = {}
end

local function send(data, delay)
  module.outq[#module.outq + 1] = { at = T + module.latency + (delay or 0), data = data }
end

local function on_push(data)
  local cmd = data[3] - A0
  module.sent[#module.sent + 1] = { cmd = cmd, t = T, arg = data[4], arg2 = data[5] }
  if cmd == CMD.REQUEST_INFO then
    module.info_requests = module.info_requests + 1
    send(frame(CMD.DEVICE_ITEM_TX, device_item(version_u16(1, 3, 5), version_u16(1, 3, 5), "FakeTx")))
    if module.rx_available then
      send(frame(CMD.DEVICE_ITEM_RX, device_item(version_u16(1, 3, 4), version_u16(1, 3, 4), "FakeRx")))
    else
      send(frame(CMD.DEVICE_ITEM_RX, device_item(0, 0, "")))
    end
    send(frame(CMD.INFO, info_payload(module.rx_available, module.binding, #module.params)), module.info_delay or 0)
  elseif cmd == CMD.REQUEST_CMD and data[4] == CMD.PARAM_ITEM then
    local idx0 = data[5]
    module.item_requests[idx0] = (module.item_requests[idx0] or 0) + 1
    local def = module.params[idx0 + 1]
    if not def then
      send(frame(CMD.PARAM_ITEM, param_item(255, { typ = 0, name = "", value = 0 })))
      return
    end
    local frames = param_frames(idx0, def)
    for i = 1, #frames do
      if not module.drop(idx0, i, module.item_requests[idx0]) then send(frames[i]) end
    end
  end
end

local sensor = {
  popFrame = function(_, id)
    local q = module.outq
    if q[1] and q[1].at <= T then
      local f = table.remove(q, 1)
      return id, f.data
    end
    return nil
  end,
  pushFrame = function(_, _, data) on_push(data); return true end,
}

crsf = { getSensor = function() return sensor end }

-- ---------- load the library ----------
local script_dir = arg[0]:match("^(.*)/[^/]*$") or "."
local mlrs = assert(loadfile(arg[1] or (script_dir .. "/../lua/Ethos/mlrs.lua")))()

local function pump(api, seconds)
  local steps = math.floor(seconds / 0.02 + 0.5)
  for _ = 1, steps do
    T = T + 0.02
    api:processQueue(24)
  end
end

local function count_sent(cmd)
  local n = 0
  for _, s in ipairs(module.sent) do if s.cmd == cmd then n = n + 1 end end
  return n
end

-- ---------- test support ----------
local failures = 0
local function check(cond, msg)
  if not cond then
    failures = failures + 1
    print("  FAIL: " .. msg)
  end
end

local function test(name, fn)
  T = 0
  io.write(name, "\n")
  fn()
end

local function new_api()
  local api = mlrs.new({ settleS = 0.2, strategy = "by_index", saveDeadS = 3.0 })
  return api
end

-- Tx params first, Rx params form the tail, like the real list
local LONG_OPTS = { "off", "1/3", "2/3", "3/3", "ant1", "ant2", "r:e t:ant1", "r:e t:ant2" } -- 44+ chars
local PARAMS = {
  { typ = T_STR6, name = "Bind Phrase", value = "mlrs.0" },
  { typ = T_LIST, name = "Mode", value = 1, options = { "50 Hz", "31 Hz", "19 Hz" } },
  { typ = T_LIST, name = "Tx Diversity", value = 0, options = LONG_OPTS },
  { typ = T_UINT8, name = "Tx Power", value = 3, min = 0, max = 7, unit = "" },
  { typ = T_UINT16, name = "Tx Ch Order", value = 1, min = 0, max = 300, unit = "ms" },
  { typ = T_LIST, name = "Rx Diversity", value = 2, options = LONG_OPTS },
  { typ = T_UINT8, name = "Rx Power", value = 1, min = 0, max = 4, unit = "" },
}
local TX_COUNT = 5

local function load_all(api, want_tx_only)
  local items, infos
  api:getDeviceItems(function(res)
    check(res.ok, "device items ok")
    items = res.data
  end)
  pump(api, 1.0)
  check(items ~= nil, "device items received")
  check(items and items.info ~= nil, "INFO delivered together with device items")

  local got
  api:getAllParams(function(res) got = res end, { full = true, deadlineS = 30 })
  pump(api, 5.0)
  check(got ~= nil and got.ok, "params load completes")
  local params = got and got.data and got.data.params or {}
  local expect = want_tx_only and TX_COUNT or #PARAMS
  check(#params == expect, "param count " .. #params .. " expected " .. expect)
  return params, items
end

local function check_params(params)
  for i, p in ipairs(params) do
    local def = PARAMS[i]
    check(p.name == def.name, "name " .. tostring(p.name))
    if def.typ == T_LIST then
      check(#p.options == #def.options, def.name .. " options " .. #p.options .. " expected " .. #def.options)
      check(p.options[#p.options] == def.options[#def.options], def.name .. " last option " .. tostring(p.options[#p.options]))
      check(p.max == #def.options - 1, def.name .. " max")
    elseif def.typ ~= T_STR6 then
      check(p.min == def.min and p.max == def.max, def.name .. " min/max")
      check(p.unit == (def.unit or ""), def.name .. " unit")
    end
    check(p.value == def.value, def.name .. " value")
  end
end

-- ---------- scenarios ----------
test("full load with receiver", function()
  module.reset({ params = PARAMS, rx_available = true })
  local api = new_api()
  local params = load_all(api, false)
  check_params(params)
  check(module.info_requests == 1, "exactly one REQUEST_INFO, got " .. module.info_requests)
  for idx0 = 0, #PARAMS do
    check((module.item_requests[idx0] or 0) == 1, "index " .. idx0 .. " requested once")
  end
end)

test("no receiver stops at first Rx param", function()
  module.reset({ params = PARAMS, rx_available = false })
  local api = new_api()
  local params = load_all(api, true)
  check_params(params)
  check(module.item_requests[TX_COUNT + 1] == nil, "no request beyond the first Rx param")
end)

test("INFO arriving after the device items", function()
  module.reset({ params = PARAMS, rx_available = true })
  module.info_delay = 0.5
  local api = new_api()
  local done_at
  api:getDeviceItems(function(res) done_at = T; check(res.data.info ~= nil, "info present") end)
  pump(api, 2.0)
  check(done_at ~= nil and done_at >= 0.5 + 0.2, "device items wait for INFO")
  check(module.info_requests == 1, "no second REQUEST_INFO, got " .. module.info_requests)
  module.info_delay = nil
end)

test("lost ITEM3 is retried without duplicating options", function()
  module.reset({ params = PARAMS, rx_available = true,
    drop = function(idx0, frame_no, attempt) return idx0 == 2 and frame_no == 3 and attempt == 1 end })
  local api = new_api()
  local params = load_all(api, false)
  check_params(params)
  check(module.item_requests[2] == 2, "index 2 retried once, got " .. tostring(module.item_requests[2]))
end)

test("stale frames after reset do not end the new load", function()
  module.reset({ params = PARAMS, rx_available = true, latency = 0.3 })
  local api = new_api()
  load_all(api, false)
  -- request the end-of-list marker, then reset while the reply is still in flight
  api:getParam(#PARAMS, function() end)
  pump(api, 0.02)
  check(#module.outq > 0, "reply in flight at reset")
  api:reset()
  T = T + 0.4 -- stale ITEM(255) becomes readable after the drain
  local params = load_all(api, false)
  check_params(params)
end)

test("store dead time holds back a queued set", function()
  module.reset({ params = PARAMS, rx_available = true })
  local api = new_api()
  load_all(api, false)
  local n0 = #module.sent
  api:store(function() end)
  api:setParam(3, 5, function() end)
  pump(api, 0.02)
  check(module.sent[n0 + 1] and module.sent[n0 + 1].cmd == CMD.PARAM_STORE, "store sent")
  check(#module.sent == n0 + 1, "set held back in the store pump")
  pump(api, 2.5)
  check(#module.sent == n0 + 1, "set still held during dead time")
  pump(api, 1.0)
  check(#module.sent == n0 + 2 and module.sent[n0 + 2].cmd == CMD.PARAM_SET, "set sent after dead time")
end)

test("single-shot commands", function()
  module.reset({ params = PARAMS, rx_available = true })
  local api = new_api()
  load_all(api, false)
  local n0 = #module.sent
  local oks = 0
  api:bindStart(function(r) if r.ok then oks = oks + 1 end end)
  api:bindStop(function(r) if r.ok then oks = oks + 1 end end)
  api:bootloader(function(r) if r.ok then oks = oks + 1 end end)
  api:flashEsp(function(r) if r.ok then oks = oks + 1 end end)
  pump(api, 0.1)
  check(oks == 4, "all four commands acknowledged")
  check(module.sent[n0 + 1].cmd == CMD.BIND_START and module.sent[n0 + 2].cmd == CMD.BIND_STOP
    and module.sent[n0 + 3].cmd == CMD.SYSTEM_BOOTLOADER and module.sent[n0 + 4].cmd == CMD.FLASH_ESP,
    "commands sent in order with the right ids")
end)

test("load progress", function()
  module.reset({ params = PARAMS, rx_available = true, latency = 0.1 })
  local api = new_api()
  api:getDeviceItems(function() end)
  pump(api, 1.0)
  check(api:loadProgress() == nil, "no progress while idle")
  api:getAllParams(function() end)
  pump(api, 0.3)
  local done, total = api:loadProgress()
  check(done ~= nil and done > 0 and done < #PARAMS, "progress mid-load " .. tostring(done))
  check(total == #PARAMS, "total from INFO param_num")
  pump(api, 5.0)
  check(api:loadProgress() == nil, "no progress after load")
end)

if failures > 0 then
  print(failures .. " check(s) failed")
  os.exit(1)
end
print("all checks passed")
