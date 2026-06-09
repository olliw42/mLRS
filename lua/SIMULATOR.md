# Developing mLRS-LVGL.lua with the EdgeTX Companion Simulator

The mLRS LVGL Lua script includes built-in mock data that activates automatically
when running in the EdgeTX Companion simulator, so you can develop and test the UI
without real radio hardware.

## Setup

1. **Install EdgeTX Companion** from the
   [EdgeTX releases](https://github.com/EdgeTX/edgetx/releases). Pick the
   version matching your target firmware (2.11.5+ for LVGL support).

2. **Create a radio profile** in Companion:
   - Settings > Radio Profile
   - Select a **color LCD** radio type (TX16S, Boxer, X20S, etc.)
   - Set the SD Structure Path to a folder on your computer

3. **Copy the script** into the simulated SD card:
   ```
   <SD card path>/SCRIPTS/TOOLS/mLRS-LVGL.lua
   ```

4. **Launch the simulator** (click the Simulate button in Companion).

5. **Run the script**: navigate to SYS > Tools and select
   "mLRS Configurator LVGL".

## How the Mock Works

The script detects the simulator via `getVersion()`, which returns a string
ending in `"-simu"`. When detected, `setMock()` replaces the three protocol
bridge functions:

| Function               | Normal behaviour                   | Mock behaviour                        |
|------------------------|------------------------------------|---------------------------------------|
| `Protocol.isConnected` | Checks RSSI / mBridge link stats   | Always returns `true`                 |
| `Protocol.cmdPush`     | Sends mBridge commands via CRSF    | Queues matching mock responses        |
| `Protocol.cmdPop`      | Reads mBridge responses via CRSF   | Dequeues from the mock response queue |

On real hardware (or when the `-simu` suffix is absent) the mock is never
loaded and has zero overhead.

## Mock Data

The mock provides:
- **Device info** for a "Mock Tx Module" and "Mock Rx Module" (v1.4.01)
- **17 parameters** (indices 0-16) covering all UI control types:
  - Bind Phrase (STR6 text edit)
  - Mode, RF Band, Ortho (LIST choices on main page)
  - Tx and Rx parameters (LIST choices and UINT8 number edits on edit pages)
- `PARAM_SET` commands update mock values in memory (no persistence)
- `PARAM_STORE`, `BIND_START`, etc. are silently accepted

## Editing Mock Data

To change the mock parameters, edit the `mockParams` table inside `setMock()`
in `mLRS-LVGL.lua`. Each entry has:

```lua
[index] = {
    typ     = 4,                    -- MBRIDGE_PARAM_TYPE (0=UINT8, 1=INT8, 4=LIST, 5=STR6)
    name    = "Tx Power",           -- up to 16 chars; "Tx"/"Rx" prefix controls edit page routing
    value   = 2,                    -- initial value (number for numeric/list, string for STR6)
    options = "min,10mW,50mW,100mW",-- comma-separated (LIST only, max 21 chars without ITEM3)
    mask    = 15,                   -- allowed_mask bitmask (LIST only)
    min     = 0,                    -- min value (numeric only)
    max     = 255,                  -- max value (numeric only)
    unit    = "kbps",               -- display unit (numeric only, max 6 chars)
}
```

Update `MOCK_PARAM_COUNT` if you add or remove entries.

## Tips

- The simulator runs Lua faster than real hardware; don't rely on it for
  performance profiling.
- Touch/scroll behaviour may differ slightly from a real radio.
- The initial parameter download has a ~3 second dead-time delay (same as on
  real hardware) before data appears. This is normal.
- To test the disconnected state, change `Protocol.isConnected` in `setMock()`
  to return `false`.
