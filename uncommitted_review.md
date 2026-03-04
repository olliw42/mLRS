# Review of Uncommitted Changes

**Files changed:** 3 modified, 2 new, 1 unrelated (`uncommitted_review.md`)

## Overall Assessment

The logic is clean. The state machine is well-structured, the blocking loop correctly pumps CAN while driving the OTA, and the separation of concerns is good (`dronecan_firmware_update.h` is MCU-agnostic, `esp-ota.h` is ESP-specific, and the blocking loop lives in the platform-aware DroneCAN interface).

## Issues Found

### 1. `esp-ota.h` — static globals in header (medium risk)

`_ota_handle`, `_ota_partition`, `_ota_state`, `_ota_bytes_written` are `static` globals + the functions are fully defined in a header. If `esp-ota.h` is ever included from a second translation unit, each TU gets its own copy of the state — silently broken, no linker error.

**Current risk:** low — the include chain is narrow (`esp-ota.h` → `dronecan_firmware_update.h` → `dronecan_interface_rx_types.h` → `common.h` → one `.cpp`). But it's fragile.

**Suggested fix (optional):** wrap the state + functions in a struct/class or move to a `.cpp`. Not urgent, but worth noting for future maintainability.

### 2. `dronecan_firmware_update.h` — unused `#define` (cosmetic)

`FWUPDATE_STATUS_INTERVAL_MS` (500) is defined but never used. The node status interval in the blocking loop uses a hardcoded `1000`.

**Fix:** remove the unused define, or use it from the blocking loop.

### 3. `dronecan_firmware_update.h` — date has trailing asterisk (cosmetic)

`// 2026-03-03 *` — stray `*` from the editing session.

### 4. `dronecan_firmware_update.h` — duplicate includes

The four `uavcan.protocol.file.*` headers are included here AND in `dronecan_interface_rx_types.h`. The header guards protect against double inclusion, but it's redundant. Since the types header already pulls them in before including `dronecan_firmware_update.h`, the includes in the firmware update header are technically unnecessary — though they do make the file self-documenting.

### 5. `dronecan_interface_rx_types.h` — union members unused (style)

The `_p` union now includes `fw_update_req`, `fw_update_res`, `file_read_req`, `file_read_res`, but `dronecan_firmware_update.h` allocates these on the stack instead of using the union. This was flagged in the previous review. Not a bug — the stack-allocated structs are small and short-lived — but the union members are dead code.

**Fix:** remove the four union members from `_p`, or refactor the firmware update class to use them.

## No Issues Found With

- **Blocking loop logic** — correctly pumps CAN, drives state machine, emits node status, exits on abort
- **State machine transitions** — all paths lead to either `esp_restart()` or `_abort()` → IDLE
- **`BeginFirmwareUpdate` accept/reject** — correctly handles in-progress rejection, decode errors, source_node_id fallback
- **file.Read timeout/retry** — proper 2s timeout with 5 retries
- **End-of-file detection** — `data.len < 256` per DroneCAN spec
- **CAN filter setup** — correctly accepts `BeginFirmwareUpdate` requests and `file.Read` responses
- **`dronecan_should_accept_transfer`** — properly gates `file.Read` responses on `IsActive()`
