-- MSP-over-CRSF Test Tool for EdgeTX
-- Validates MSP V1 + V2 communication through CRSF interface
-- Works with: Crossfire, ELRS, mLRS
-- Place as /SCRIPTS/TOOLS/msp-crsf-test.lua on SD card
--
-- Usage:
--   1. Connect radio to TX module with CRSF, FC must be connected and powered
--   2. Run from EdgeTX TOOLS menu
--   3. [ENTER] start test sequence
--   4. [UP/DN] scroll results
--   5. Results show pass/fail, response time, and hex payload
--
-- Protocol notes:
--   Commands <= 254 use MSP V1 chunk format (status version bits 01 = 0x20)
--   Commands >  254 use MSP V2 chunk format (status version bits 10 = 0x40)
--   mLRS internally converts V1 requests to V2, responses always come back V2

-- CRSF protocol constants
local CRSF_MSP_REQ    = 0x7A
local CRSF_MSP_RESP   = 0x7B
local ADDR_FC         = 0xC8
local ADDR_RADIO      = 0xEA

-- MSP chunk status byte flags
local ST_START    = 0x10
local ST_VER_V1   = 0x20  -- bits5-6 = 01 = MSP V1
local ST_VER_V2   = 0x40  -- bits5-6 = 10 = MSP V2
local ST_ERROR    = 0x80
local ST_SEQ_MASK = 0x0F
local ST_VER_MASK = 0x60

-- Test definitions: MSP commands with no payload
local tests = {
    -- V1 commands (func <= 254): sent with V1 chunk format
    { name = "API_VERSION", func = 1,      minLen = 3,  info = "proto,major,minor" },
    { name = "FC_VARIANT",  func = 2,      minLen = 4,  info = "4-char ID" },
    { name = "FC_VERSION",  func = 3,      minLen = 3,  info = "major,minor,patch" },
    { name = "BOARD_INFO",  func = 4,      minLen = 4,  info = "4-char board" },
    { name = "STATUS",      func = 101,    minLen = 6,  info = "FC status" },
    { name = "ATTITUDE",    func = 108,    minLen = 6,  info = "roll,pitch,yaw" },
    { name = "ANALOG",      func = 110,    minLen = 7,  info = "vbat,mah,rssi,a" },
    -- V2 command (func > 254): sent with V2 chunk format
    { name = "INAV_STATUS", func = 0x2000, minLen = 1,  info = "V2 INAV status" },
}

-- State machine
local S_IDLE = 0
local S_SEND = 1
local S_WAIT = 2
local S_NEXT = 3
local S_DONE = 4
local S_CH_LOOP = 5

-- Channel test constants
local MSP_SET_RAW_RC = 200   -- 0xC8
local CH_INTERVAL    = 10    -- 100ms in 10ms ticks
local CH_SENDS       = 10    -- sends per value before switching
local CH_NUM         = 17    -- channel number to toggle (1-based)

-- Global state
local state     = S_IDLE
local testIdx   = 0
local results   = {}

-- RX reassembly
local rxBuf     = {}
local rxPos     = 0
local rxLen     = 0
local rxFunc    = 0
local rxFlag    = 0
local rxSeq     = 0
local rxStart   = 0
local rxStarted = false
local rxVersion = 0  -- 1=V1, 2=V2

-- Timing (getTime() returns 10ms ticks)
local TIMEOUT   = 200    -- 2000ms
local COOLDOWN  = 5      -- 50ms between requests
local lastTime  = 0
local scrollOfs = 0

-- Channel test state
local chValue    = 1800
local chCount    = 0
local chSendN    = 0
local chRespN    = 0
local chErrN     = 0
local chCycles   = 0
local chLastSend = 0

-- ============================================================
-- CRC8 DVB-S2 for MSP V2
-- ============================================================

local function crc8_dvb_s2(crc, byte)
    crc = bit32.bxor(crc, byte)
    for _ = 1, 8 do
        if bit32.btest(crc, 0x80) then
            crc = bit32.bxor(bit32.lshift(bit32.band(crc, 0xFF), 1), 0xD5)
        else
            crc = bit32.lshift(bit32.band(crc, 0xFF), 1)
        end
        crc = bit32.band(crc, 0xFF)
    end
    return crc
end

-- ============================================================
-- MSP-over-CRSF send
-- ============================================================

local function sendMspRequest(func)
    local data = { ADDR_FC, ADDR_RADIO }

    if func > 254 then
        -- MSP V2: [dest][origin][status][flag][cmd_lo][cmd_hi][size_lo][size_hi][crc8]
        data[3] = bit32.bor(ST_START, ST_VER_V2)  -- 0x50: start + V2 + seq 0
        data[4] = 0x00                              -- flag
        data[5] = bit32.band(func, 0xFF)            -- cmd low
        data[6] = bit32.rshift(func, 8)             -- cmd high
        data[7] = 0x00                              -- size low
        data[8] = 0x00                              -- size high
        -- DVB-S2 CRC over flag, cmd_lo, cmd_hi, size_lo, size_hi (no payload)
        local crc = 0
        for i = 4, 8 do
            crc = crc8_dvb_s2(crc, data[i])
        end
        data[9] = crc
    else
        -- MSP V1: [dest][origin][status][size][cmd]
        data[3] = bit32.bor(ST_START, ST_VER_V1)  -- 0x30: start + V1 + seq 0
        data[4] = 0x00                              -- payload size = 0
        data[5] = bit32.band(func, 0xFF)            -- cmd
    end

    return crossfireTelemetryPush(CRSF_MSP_REQ, data)
end

-- ============================================================
-- MSP_SET_RAW_RC (200) sender for channel test
-- Sends CH_NUM channels: 1-16 safe defaults, CH_NUM = test value
-- ============================================================

local function sendSetRawRC(value)
    local data = { ADDR_FC, ADDR_RADIO }
    local payloadSize = CH_NUM * 2  -- 17 channels * 2 bytes = 34
    -- V1 chunk: [dest][origin][status][size][cmd][payload...]
    data[3] = bit32.bor(ST_START, ST_VER_V1)  -- 0x30: start + V1
    data[4] = payloadSize
    data[5] = MSP_SET_RAW_RC
    local idx = 6
    -- CH1-16: send 0 (ignored by FC when mLRS provides them via its own MSP_SET_RAW_RC)
    for ch = 1, CH_NUM - 1 do
        data[idx]     = 0
        data[idx + 1] = 0
        idx = idx + 2
    end
    -- Channel 17: test value
    data[idx]     = bit32.band(value, 0xFF)
    data[idx + 1] = bit32.rshift(value, 8)
    return crossfireTelemetryPush(CRSF_MSP_REQ, data)
end

-- ============================================================
-- MSP-over-CRSF receive with V1/V2 auto-detect
-- ============================================================

local function pollResponse()
    local cmd, data = crossfireTelemetryPop()
    if not cmd or not data then return nil end
    if cmd ~= CRSF_MSP_RESP then return nil end
    if #data < 4 then return nil end

    -- data[1]=dest, data[2]=origin, data[3]=status, data[4+]=msp chunk data
    local status  = data[3]
    local seq     = bit32.band(status, ST_SEQ_MASK)
    local isStart = bit32.band(status, ST_START) ~= 0
    local isError = bit32.band(status, ST_ERROR) ~= 0
    local verBits = bit32.band(status, ST_VER_MASK)

    if isStart then
        -- Determine version from response status byte
        local ver = 1
        if verBits == ST_VER_V2 then ver = 2 end
        rxVersion = ver

        local idx = 4  -- first MSP data byte after dest+origin+status

        if ver == 2 then
            -- V2 start: [flag][cmd_lo][cmd_hi][size_lo][size_hi][payload...]
            if #data < 8 then return "SHORT" end
            rxFlag = data[idx]; idx = idx + 1
            rxFunc = data[idx] + data[idx + 1] * 256; idx = idx + 2
            rxLen  = data[idx] + data[idx + 1] * 256; idx = idx + 2
        else
            -- V1 start: [size][cmd][payload...]
            if #data < 5 then return "SHORT" end
            rxFlag = 0
            rxLen  = data[idx]; idx = idx + 1
            rxFunc = data[idx]; idx = idx + 1
        end

        rxBuf  = {}
        rxPos  = 0
        rxSeq  = bit32.band(seq + 1, ST_SEQ_MASK)
        rxStarted = true

        for i = idx, #data do
            if rxPos < rxLen then
                rxPos = rxPos + 1
                rxBuf[rxPos] = data[i]
            end
        end
    else
        -- Continuation chunk
        if not rxStarted then return nil end
        if bit32.band(seq, ST_SEQ_MASK) ~= rxSeq then
            rxStarted = false
            return "SEQ_ERR"
        end
        rxSeq = bit32.band(seq + 1, ST_SEQ_MASK)
        for i = 4, #data do
            if rxPos < rxLen then
                rxPos = rxPos + 1
                rxBuf[rxPos] = data[i]
            end
        end
    end

    if isError then
        rxStarted = false
        return "MSP_ERR"
    end
    if rxPos >= rxLen then
        rxStarted = false
        return "OK"
    end
    return nil -- need more chunks
end

-- ============================================================
-- Display helpers
-- ============================================================

local function toHex(buf, maxN)
    local s = ""
    local n = math.min(#buf, maxN or 12)
    for i = 1, n do
        if i > 1 then s = s .. " " end
        s = s .. string.format("%02X", buf[i])
    end
    if #buf > n then s = s .. ".." end
    return s
end

-- ============================================================
-- EdgeTX tool script interface
-- ============================================================

local function init()
    state   = S_IDLE
    testIdx = 0
    results = {}
    scrollOfs = 0
end

local function run(event)
    local now = getTime()

    -- ---- Input handling ----
    if event == EVT_VIRTUAL_EXIT then
        if state == S_IDLE or state == S_DONE then
            return 2
        end
        if state == S_CH_LOOP then
            state = S_DONE
            return 0
        end
        state = S_DONE
        return 0
    end

    if state == S_IDLE then
        if event == EVT_VIRTUAL_ENTER then
            testIdx   = 1
            results   = {}
            state     = S_SEND
            lastTime  = now
            scrollOfs = 0
        end
    elseif state == S_DONE then
        if event == EVT_VIRTUAL_ENTER then
            -- Start channel test loop
            state      = S_CH_LOOP
            chValue    = 1800
            chCount    = 0
            chSendN    = 0
            chRespN    = 0
            chErrN     = 0
            chCycles   = 0
            chLastSend = now
        elseif event == EVT_VIRTUAL_NEXT or event == EVT_VIRTUAL_NEXT_REPT then
            scrollOfs = math.min(scrollOfs + 1, math.max(0, #results * 2 - 4))
        elseif event == EVT_VIRTUAL_PREV or event == EVT_VIRTUAL_PREV_REPT then
            scrollOfs = math.max(0, scrollOfs - 1)
        end
    end

    -- ---- State machine ----
    if state == S_SEND then
        if (now - lastTime) >= COOLDOWN then
            if sendMspRequest(tests[testIdx].func) then
                rxStart   = now
                rxStarted = false
                state     = S_WAIT
            end
            lastTime = now
        end
    elseif state == S_WAIT then
        local result = pollResponse()
        if result then
            local elapsed = (now - rxStart) * 10
            local vStr = rxVersion == 2 and "V2" or "V1"
            results[#results + 1] = {
                name   = tests[testIdx].name,
                status = result,
                hex    = toHex(rxBuf, 10),
                ms     = elapsed,
                func   = rxFunc,
                len    = rxPos,
                flag   = rxFlag,
                ver    = vStr,
            }
            lastTime = now
            state    = S_NEXT
        elseif (now - rxStart) > TIMEOUT then
            results[#results + 1] = {
                name   = tests[testIdx].name,
                status = "TIMEOUT",
                hex    = "",
                ms     = TIMEOUT * 10,
                func   = tests[testIdx].func,
                len    = 0,
                flag   = 0,
                ver    = "",
            }
            lastTime = now
            state    = S_NEXT
        end
    elseif state == S_NEXT then
        testIdx = testIdx + 1
        if testIdx > #tests then
            state = S_DONE
        else
            state = S_SEND
        end
    elseif state == S_CH_LOOP then
        -- Poll responses (fire-and-forget, just count)
        local result = pollResponse()
        if result == "OK" then
            chRespN = chRespN + 1
        elseif result then  -- any non-nil string = error
            chErrN = chErrN + 1
        end
        -- Send at interval
        if (now - chLastSend) >= CH_INTERVAL then
            if sendSetRawRC(chValue) then
                chSendN    = chSendN + 1
                chCount    = chCount + 1
                chLastSend = now
                if chCount >= CH_SENDS then
                    chCount = 0
                    if chValue == 1800 then
                        chValue = 1200
                    else
                        chValue = 1800
                        chCycles = chCycles + 1
                    end
                end
            end
        end
    end

    -- ---- Drawing ----
    lcd.clear()

    if state == S_IDLE then
        lcd.drawText(1, 0, "MSP-CRSF Test", MIDSIZE)
        lcd.drawText(1, 18, #tests .. " tests (V1+V2)", SMLSIZE)
        lcd.drawText(1, 28, "CRSF / ELRS / mLRS", SMLSIZE)
        lcd.drawText(1, 42, "[ENTER] start", SMLSIZE)
        lcd.drawText(1, 52, "[EXIT] quit", SMLSIZE)

    elseif state == S_DONE then
        local ok = 0
        for i = 1, #results do
            if results[i].status == "OK" then ok = ok + 1 end
        end
        lcd.drawText(1, 0, ok .. "/" .. #results .. " OK", MIDSIZE)

        local y = 16
        local lineH = 9
        local lineIdx = 0
        for i = 1, #results do
            local r = results[i]
            -- Line 1: status
            if lineIdx >= scrollOfs then
                if y > 55 then break end
                local mark = r.status == "OK" and "+" or "x"
                local line = mark .. " " .. r.name
                if r.status == "OK" then
                    line = line .. " " .. r.ver .. " " .. r.ms .. "ms"
                else
                    line = line .. " " .. r.status
                end
                lcd.drawText(1, y, line, SMLSIZE)
                y = y + lineH
            end
            lineIdx = lineIdx + 1

            -- Line 2: hex data
            if r.hex ~= "" then
                if lineIdx >= scrollOfs then
                    if y > 55 then break end
                    lcd.drawText(8, y, "[" .. r.len .. "] " .. r.hex, SMLSIZE)
                    y = y + lineH
                end
                lineIdx = lineIdx + 1
            end
        end

        lcd.drawText(1, 56, "[ENTER] CH17 test [UP/DN] scroll", SMLSIZE)

    elseif state == S_CH_LOOP then
        lcd.drawText(1, 0, "CH17 Test", MIDSIZE)
        lcd.drawText(1, 18, "Value: " .. chValue .. "  [" .. (chCount + 1) .. "/" .. CH_SENDS .. "]", SMLSIZE)
        lcd.drawText(1, 28, "Sent: " .. chSendN .. "  Resp: " .. chRespN .. "  Err: " .. chErrN, SMLSIZE)
        lcd.drawText(1, 38, "Cycles: " .. chCycles, SMLSIZE)
        lcd.drawText(1, 48, "SET_RAW_RC #200, 17ch", SMLSIZE)
        lcd.drawText(1, 56, "[EXIT] stop", SMLSIZE)

    else
        local t = tests[testIdx]
        lcd.drawText(1, 0, "Testing...", MIDSIZE)
        lcd.drawText(1, 18, t.name, 0)
        local vStr = t.func > 254 and "V2" or "V1"
        lcd.drawText(1, 30, vStr .. " 0x" .. string.format("%04X", t.func), SMLSIZE)
        lcd.drawText(1, 40, string.format("%d / %d", testIdx, #tests), SMLSIZE)
        local stStr = state == S_SEND and "Sending..." or "Waiting..."
        lcd.drawText(1, 50, stStr, SMLSIZE)
    end

    return 0
end

return { init = init, run = run }
