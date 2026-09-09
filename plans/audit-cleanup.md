# Plan: over-engineering audit cleanup + opt-in device-timer cancel

Branch: `cleanup-audit` (off `master`). One PR at the end.

## 1. Summary

Two independent pieces of work, shipped together:

1. **Audit cleanup (9 findings).** Remove machinery that costs more lines than it
   buys in the two ESPHome components: enum+switch string lookups, redundant
   fan-speed bounds constants, a write-only member, a subsumed length guard, two
   one-use constants, a do-nothing TLV case, and a CI artifact-upload step.
2. **Timer behaviour becomes opt-in.** The air purifier currently cancels any
   timer the user starts from the device's own timer button, unconditionally.
   Turn that into a `cancel_device_timer:` boolean config key
   (**recommended default: `false`**, see §4.7).

**Hard constraint: the protocol is frozen.** No TX byte may change. Every
`uint8_t packet[] = {...}` array and `calc_checksum_` stays byte-identical, and
entity `name:` values in both sample YAMLs are untouched. See §6 for the
verification procedure.

Expected net: **about -50 lines** (about -55 inside `components/`, -7 in CI,
about +10 of new config/docs).

## 2. Relevant files and current behaviour

| File | Role |
|---|---|
| `/Users/luke/esphome-levoit/components/air_purifier_vital200s/types.h` | 211 lines. RX consts, packet offsets, addresses, TLV enums, `Mode`, `AirQuality`, fan-speed consts, `DisplayBrightness`, WiFi-LED. |
| `/Users/luke/esphome-levoit/components/air_purifier_vital200s/air_purifier_vital200s.cpp` | 459 lines. Entities, UART read/write, commands, timer, `parse_packet_` / `parse_tlvs_` / `handle_status_tlv_`. |
| `/Users/luke/esphome-levoit/components/air_purifier_vital200s/air_purifier_vital200s.h` | 123 lines. Entity classes + `AirPurifier` (setters, `send_*`, private helpers, state members). |
| `/Users/luke/esphome-levoit/components/air_purifier_vital200s/__init__.py` | Config schema + `to_code`. Currently only entity keys, no plain scalar options. |
| `/Users/luke/esphome-levoit/components/humidifier_oasismist1000s/types.h` | 147 lines. Same shape, deliberately spelled to match. |
| `/Users/luke/esphome-levoit/components/humidifier_oasismist1000s/humidifier_oasismist1000s.cpp` | 378 lines. |
| `/Users/luke/esphome-levoit/components/humidifier_oasismist1000s/humidifier_oasismist1000s.h` | 122 lines. |
| `/Users/luke/esphome-levoit/air_purifier_vital200s.yaml` | Sample config; component block at lines 91-106. |
| `/Users/luke/esphome-levoit/README.md` | Config reference tables (lines 83-113), protocol TODO (lines 145-174). |
| `/Users/luke/esphome-levoit/.github/workflows/ci.yaml` | clang-format job + build matrix; artifact upload at lines 52-57. |
| `/Users/luke/esphome-levoit/.clang-format` | Google, IndentWidth 2, **ColumnLimit 120**, PointerAlignment Right, short ifs/loops on one line allowed, SortIncludes off. Pinned to clang-format 20.1.7. |

Facts confirmed by grep (do not re-derive):

- `AirQuality` / `air_quality_to_string` / `uint8_to_air_quality` / the five
  `AIR_QUALITY_*` consts are referenced **only** in air `types.h` and air
  `.cpp:424-426`.
- `FAN_SPEED_MIN`/`MAX` are used at air `.cpp:67` (dead guard), `:239`
  (`std::clamp`), `:402` (RX speed sanity check). `FAN_SPEED_COUNT` only at
  `:25`.
- Air's `last_mode_` (`.h:119`) is written and read only inside
  `handle_status_tlv_` `case TLV::MODE` (`.cpp:396-398`). The humidifier's
  `last_mode_` **is** read elsewhere (`.cpp:216`, `:229`) - leave it alone.
- `DisplayBrightness` is used once (air `.cpp:246`); humidifier `VALUE_ACTIVE`
  is used once (humid `.cpp:336`, RX comparison only).
- `RX_MIN_PACKET_LEN` is used once per component (air `.cpp:352`, humid
  `.cpp:281`). It equals `Offset::TLV_START_TIMER` (air) and `Offset::TLV_START`
  (humid) - all three are `10`.
- ESPHome's `fan::FanCall::validate_()` clamps speed to
  `clamp(*speed_, 1, traits.supported_speed_count())` before `control()` runs,
  so `speed >= FAN_SPEED_MIN` in `PurifierFan::control()` can never be false.
  (Verified against esphome/dev `components/fan/fan.cpp`.)

## 3. Two corrections to the audit - read these first

The audit text is right about *what* to remove but wrong about two mechanics.
Both are non-negotiable in this plan.

### 3.1 Do not put a `static` local array inside a `constexpr` function

`mode_to_string()` and `air_quality_to_string()` are declared `inline
constexpr`. A function-local `static` variable is only legal in a `constexpr`
function from C++23; ESPHome builds ESP-IDF at C++17/20 depending on version, so
`inline constexpr const char *f() { static const char *const N[] = ...; }`
may not compile.

**Resolution:** put the table at namespace scope as an `inline constexpr
std::array<...>` and keep the functions `inline constexpr`. `<array>` is already
included in both `types.h` (used by `Address`), so no new include is needed and
`.size()` gives the bound check for free.

(The alternative - drop `constexpr` from the function and use a function-local
`static` - is also correct. Do not combine the two.)

### 3.2 The table lookup needs a real bound check

`handle_status_tlv_` does `static_cast<Mode>(v)` on an unvalidated MCU byte, and
today `mode_to_string()`'s `default:` catches anything out of range. A bare
`NAMES[static_cast<uint8_t>(mode)]` on a 3-element array would be an
out-of-bounds read for any byte >= 3. Every table lookup added by this plan
**must** be bounds-checked (`i < NAMES.size()`), including the mode one.

### 3.3 `parse_packet_` (air): reorder, do not just delete the guard

The audit says the `len < RX_MIN_PACKET_LEN` guard is subsumed by the
`len >= TLV_START_*` checks. It is not, because of `&&` evaluation order:
`match_address_(data, ADDR_STATUS)` reads `data[6..9]` and runs *before* the
length check. Removing the guard as-is would over-read for a status frame whose
`payload_len` byte is 0-3 (`expected_size` 6-9).

**Resolution:** swap the operands so the length test comes first. Same line
saving, no over-read.

## 4. Proposed changes, file by file

### 4.1 `components/air_purifier_vital200s/types.h`

**(a) Air quality (lines 138-176) - replace the whole block.**

Before:

```cpp
enum class AirQuality : uint8_t { UNKNOWN = 0, VERY_GOOD = 1, /* ... */ BAD = 4 };
inline constexpr const char *AIR_QUALITY_UNKNOWN = "Unknown";
/* ...4 more consts... */
inline constexpr const char *air_quality_to_string(AirQuality quality) { /* 5-way switch */ }
inline constexpr AirQuality uint8_to_air_quality(uint8_t value) { /* range check */ }
```

After (keep the `// ==== Air Quality ====` banner comment):

```cpp
// index = MCU air-quality value; 0 doubles as the out-of-range fallback
inline constexpr std::array<const char *, 5> AIR_QUALITY_NAMES = {"Unknown", "Very Good", "Good", "Moderate", "Bad"};

inline constexpr const char *air_quality_to_string(uint8_t value) {
  return value < AIR_QUALITY_NAMES.size() ? AIR_QUALITY_NAMES[value] : AIR_QUALITY_NAMES[0];
}
```

The `AIR_QUALITY_NAMES` line is 117 chars - under the 120 limit but tight. Do not
lengthen it; if clang-format wraps it, keep the wrapped form.

**(b) Mode (lines 111-130).** Keep `enum class Mode`, keep `MODE_AUTO` /
`MODE_MANUAL` / `MODE_SLEEP` (still used by `set_supported_preset_modes` and
`string_to_mode`), keep `string_to_mode` unchanged. Replace only the switch:

```cpp
// index = Mode value: MANUAL=0, SLEEP=1, AUTO=2
inline constexpr std::array<const char *, 3> MODE_NAMES = {MODE_MANUAL, MODE_SLEEP, MODE_AUTO};

inline constexpr const char *mode_to_string(Mode mode) {
  const size_t i = static_cast<size_t>(mode);
  return i < MODE_NAMES.size() ? MODE_NAMES[i] : MODE_AUTO;
}
```

**Order matters and differs per component.** Air is `MANUAL=0, SLEEP=1,
AUTO=2` (types.h:111-115). Re-read the enum before writing the array.

**(c) Fan speed (lines 182-184).** Delete `FAN_SPEED_MIN` and `FAN_SPEED_MAX`,
keep:

```cpp
inline constexpr uint8_t FAN_SPEED_COUNT = 4;  // speeds are 1..4
```

**(d) Display brightness (lines 193-196).** Delete the `DisplayBrightness` enum;
add one line to the `Values` block under `VALUE_ON`:

```cpp
inline constexpr uint8_t DISPLAY_ON_BRIGHTNESS = 0x64;  // display takes a brightness byte, not VALUE_ON
```

**(e) `RX_MIN_PACKET_LEN` (line 17).** Delete (unused after 4.2f).

**(f) `TLV::DISPLAY_CURRENT` (line 78).** Replace the enumerator with a comment
so the reverse-engineered knowledge survives:

```cpp
  SPEED = 0x04,
  // 0x06 = live display brightness; ignored, DISPLAY_SAVED is the switch state
  DISPLAY_SAVED = 0x07,
```

Do not touch the file's final line / EOF newline - CI's clang-format check
passes on it today.

### 4.2 `components/air_purifier_vital200s/air_purifier_vital200s.cpp`

**(a) `PurifierFan::control()` (lines 65-71)** - drop the dead guard:

```cpp
  if (call.get_speed().has_value()) {
    parent_->send_mode(Mode::MANUAL);
    parent_->send_fan_speed(static_cast<uint8_t>(*call.get_speed()));
  }
```

**(b) `send_fan_speed()` (line 239)** - `std::clamp` needs matching argument
types, so pin the template parameter:

```cpp
  speed = std::clamp<uint8_t>(speed, 1, FAN_SPEED_COUNT);
```

**(c) `send_display()` (lines 246-247)** - two lines become one:

```cpp
  send_command_(ADDR_DISPLAY, on ? DISPLAY_ON_BRIGHTNESS : VALUE_OFF);
```

`DisplayBrightness::ON == 0x64 == DISPLAY_ON_BRIGHTNESS` and
`DisplayBrightness::OFF == 0x00 == VALUE_OFF`, so the TX byte is unchanged.

**(d) `send_wifi_status()` (lines 260-275)** - drop `status_str`, log in-branch:

```cpp
  WifiLedStatus status;
  if (ha_connected) {
    status = WifiLedStatus::SOLID;
    ESP_LOGI(TAG, "WiFi LED: solid (HA connected)");
  } else if (wifi_connected) {
    status = WifiLedStatus::BLINKING;
    ESP_LOGI(TAG, "WiFi LED: blinking (WiFi only)");
  } else {
    status = WifiLedStatus::OFF;
    ESP_LOGI(TAG, "WiFi LED: off (disconnected)");
  }
```

The rest of the function (the `packet[]` block onward) is untouched.

**(e) `parse_packet_()` (lines 351-363)** - delete the `RX_MIN_PACKET_LEN` guard
**and swap each `&&`** (see §3.3):

```cpp
void AirPurifier::parse_packet_(const uint8_t *data, size_t len) {
  if (len >= static_cast<size_t>(Offset::TLV_START_STATUS) && match_address_(data, ADDR_STATUS)) {
    parse_tlvs_(data, len, static_cast<size_t>(Offset::TLV_START_STATUS),
                [this](uint8_t t, uint8_t l, const uint8_t *v) { handle_status_tlv_(t, l, v); });
  } else if (len >= static_cast<size_t>(Offset::TLV_START_TIMER) && match_address_(data, ADDR_TIMER)) {
    parse_tlvs_(data, len, static_cast<size_t>(Offset::TLV_START_TIMER),
                [this](uint8_t t, uint8_t l, const uint8_t *v) { handle_timer_tlv_(t, l, v); });
  }
}
```

Behaviour is identical: the two addresses are distinct, so no frame can change
which branch it lands in.

**(f) `handle_status_tlv_` `case TLV::MODE` (lines 395-399)** - local instead of
member. **Braces are required** (a declaration with an initializer directly under
a `case` label is ill-formed when later labels follow):

```cpp
    case TLV::MODE: {
      const Mode mode = static_cast<Mode>(v);
      if (fan_ != nullptr) fan_->publish_mode(mode);
      ESP_LOGD(TAG, "Mode: %s", mode_to_string(mode));
      break;
    }
```

**(g) `case TLV::SPEED` (line 402)**:

```cpp
      if (v >= 1 && v <= FAN_SPEED_COUNT) {
```

Keep the lower bound - `v` comes from the MCU and `fan_->speed = 0` is not a
valid ESPHome fan speed.

**(h) `case TLV::DISPLAY_CURRENT` (lines 411-413)** - delete all three lines.
0x06 now falls to `default:` and logs `Unknown TLV 0x06` at VERBOSE, which is
documented behaviour (README lines 148-151) and matches the comment added in
4.1f.

**(i) `case TLV::AIR_QUALITY` (lines 422-428)** - one lookup instead of three
calls:

```cpp
    case TLV::AIR_QUALITY:
      if (air_quality_sensor_ != nullptr) {
        const char *quality = air_quality_to_string(v);
        air_quality_sensor_->publish_state(quality);
        ESP_LOGD(TAG, "Air quality: %s", quality);
      }
      break;
```

Published strings are unchanged, including "Unknown" for any out-of-range byte.

### 4.3 `components/air_purifier_vital200s/air_purifier_vital200s.h`

- Delete `Mode last_mode_{Mode::AUTO};` (line 119).
- Add the timer option (see 4.7): a public setter next to the entity setters and
  a `bool cancel_device_timer_{false};` in the `// State` block.

### 4.4 `components/humidifier_oasismist1000s/types.h`

- **Mode (lines 103-112):** same transformation as 4.1b, but the enum order is
  `AUTO=0, MANUAL=1, SLEEP=2`, so:

```cpp
// index = Mode value: AUTO=0, MANUAL=1, SLEEP=2
inline constexpr std::array<const char *, 3> MODE_NAMES = {MODE_AUTO, MODE_MANUAL, MODE_SLEEP};
```

  The `mode_to_string` body is character-for-character the same as the air one.
- **Line 126:** rename `VALUE_ACTIVE` to `DISPLAY_ON_BRIGHTNESS` so both
  components spell the same 0x64 the same way, and move/keep it with the same
  trailing comment as 4.1d.
- **Line 17:** delete `RX_MIN_PACKET_LEN` (replaced in 4.5b).
- Do **not** touch `HUMIDITY_MIN/MAX` or `MIST_LEVEL_MIN/MAX` - all four have
  multiple live uses (traits, `std::clamp`, RX validation, `control()`).

### 4.5 `components/humidifier_oasismist1000s/humidifier_oasismist1000s.cpp`

**(a) `send_wifi_status()` (lines 239-254)** - identical rewrite to 4.2d, same
wording, so the two components stay spelled alike.

**(b) `parse_packet_()` (line 281)** - `RX_MIN_PACKET_LEN` and
`Offset::TLV_START` are both `10`; keep one guard, expressed as the offset the
parser actually needs:

```cpp
void Humidifier::parse_packet_(const uint8_t *data, size_t len) {
  if (len < static_cast<size_t>(Offset::TLV_START)) return;
```

Keep it *before* `match_address_` - it is what protects the `data[6..9]` read
here (same reasoning as §3.3).

**(c) Line 336** - `v == DISPLAY_ON_BRIGHTNESS` after the rename.

### 4.6 `.github/workflows/ci.yaml`

Delete lines 52-57 (the `Upload firmware` step) and the now-unreferenced
`id: build` on line 47. Nothing else in the workflow reads
`steps.build.outputs.*`. The `build` job name and matrix stay.

Result:

```yaml
      - name: Build firmware
        uses: esphome/build-action@v8.1.0
        with:
          yaml-file: ${{ matrix.config }}
          version: latest
```

### 4.7 Timer opt-in (`cancel_device_timer`)

**Current behaviour:** `parse_packet_` routes `ADDR_TIMER` frames to
`handle_timer_tlv_`; on `TimerTLV::TOTAL` with `seconds > 0` it calls
`send_timer_cancel_()`. Pressing the timer button on the device therefore cancels
the timer immediately, always.

**Default: `false`.** Reasoning:

- The behaviour is invisible and un-guessable. A user who presses the timer
  button and watches it disappear has no log line at DEBUG telling them the
  integration did it (the WARN exists, but nobody reads WARN before filing a
  bug), and nothing in the entity list hints at it.
- It is an unsolicited *write* to the device in response to a user's physical
  action. Defaults that quietly override the user at the hardware level should be
  opt-in.
- It is one person's preference ("my purifier must never turn itself off"), and
  the maintainer can flip it in their own YAML with one line.
- Cost: users who update the component get the old behaviour switched off. That
  is a real regression for them, so it must be called out in the PR description
  and in the README row. It is a mild, self-correcting surprise ("my timer works
  again") versus the current permanent, silent one.

If the maintainer prefers backward compatibility over least-surprise, changing
`default=False` to `default=True` is a one-token edit in `__init__.py` plus the
member initialiser - flagged in §9.

**Where the flag is checked:** inside `handle_timer_tlv_`, in the
`TimerTLV::TOTAL` case - **not** in `parse_packet_`. Both are one-token edits, so
"lighter" is a tie on lines; gating late keeps the `Timer remaining` VERBOSE log
working regardless of the setting (README lines 154-157 advertise it as the only
window into the timer), and keeps one code path instead of two.

**(a) `components/air_purifier_vital200s/__init__.py`**

Add next to the other config-key constants (after `CONF_AIR_QUALITY`):

```python
CONF_CANCEL_DEVICE_TIMER = "cancel_device_timer"
```

In `CONFIG_SCHEMA`, immediately after `cv.GenerateID()`:

```python
            cv.GenerateID(): cv.declare_id(AirPurifier),
            # Behaviour
            cv.Optional(CONF_CANCEL_DEVICE_TIMER, default=False): cv.boolean,
            # Sensors
```

In `to_code`, right after `await uart.register_uart_device(var, config)`:

```python
    cg.add(var.set_cancel_device_timer(config[CONF_CANCEL_DEVICE_TIMER]))
```

(`default=False` means the key is always present - no `.get()` needed.)

**(b) `air_purifier_vital200s.h`** - after `set_light_detection_switch(...)`:

```cpp
  // Options
  void set_cancel_device_timer(bool enable) { cancel_device_timer_ = enable; }
```

and in the `// State` block (where `last_mode_` was removed):

```cpp
  bool cancel_device_timer_{false};
```

**(c) `air_purifier_vital200s.cpp`** - in `handle_timer_tlv_`:

```cpp
    case TimerTLV::TOTAL:
      if (cancel_device_timer_ && seconds > 0) {
        ESP_LOGW(TAG, "Timer %lu seconds detected, cancelling...", static_cast<unsigned long>(seconds));
        send_timer_cancel_();
      }
      break;
```

and in `dump_config()`, after the update-interval line:

```cpp
  ESP_LOGCONFIG(TAG, "  Cancel device timer: %s", YESNO(cancel_device_timer_));
```

(`YESNO` comes from `esphome/core/helpers.h`, already included.)

Keep `send_timer_cancel_()`, `handle_timer_tlv_`, `TimerTLV`, `ADDR_TIMER`,
`ADDR_TIMER_SET`, `Offset::TLV_START_TIMER` and `PayloadLen::TIMER_CANCEL`
exactly as they are - all still reachable when the key is on.

**(d) `air_purifier_vital200s.yaml`** - inside the component block (currently
lines 91-93), before `fan:`:

```yaml
air_purifier_vital200s:
  id: air_purifier
  uart_id: uart_bus
  # Cancel any timer started from the device's own timer button, so the purifier
  # never switches itself off. Off by default.
  # cancel_device_timer: true
  fan:
```

Leave every `name:` value untouched.

**(e) `README.md`** - after the `air_purifier_vital200s` entity table (line 102),
add:

```markdown
Plus one behaviour option:

| Key | Default | Description |
|---|---|---|
| `cancel_device_timer` | `false` | Cancel any timer started from the device's own timer button, so the purifier never switches itself off |
```

And amend the protocol-TODO timer bullet (lines 154-157) so it reads that the
cancel command is only sent when `cancel_device_timer:` is enabled.

## 5. Pre-flight: grep-to-confirm-unused

Run these from the repo root **before** deleting anything, and confirm the only
hits are the ones this plan already accounts for. Stop and report if anything
else shows up.

```bash
rg -n 'last_mode_'            components/air_purifier_vital200s   # expect .h:119 + .cpp:396-398 only
rg -n 'FAN_SPEED_(MIN|MAX)'   components                          # expect types.h:182-183, .cpp:67,239,402
rg -n 'FAN_SPEED_COUNT'       components                          # expect types.h:184, .cpp:25
rg -n 'RX_MIN_PACKET_LEN'     components                          # expect one decl + one use per component
rg -n 'DisplayBrightness'     components                          # expect types.h:193 + .cpp:246
rg -n 'VALUE_ACTIVE'          components                          # expect humid types.h:126 + .cpp:336
rg -n 'AirQuality|air_quality_to_string|uint8_to_air_quality|AIR_QUALITY_' components
rg -n 'DISPLAY_CURRENT'       components                          # expect types.h:78 + .cpp:411
rg -n 'steps\.build'          .github                             # expect only the upload step being deleted
```

Also `rg -n 'air_quality_to_string|mode_to_string' -g '*.yaml'` - confirm no
sample/test YAML lambda calls these helpers (none should).

Re-run the same greps after the edits: every deleted name must return zero hits.

## 6. Byte-freeze verification

All four TX builders live inside `// clang-format off` / `// clang-format on`
blocks:

| Function | File:line (pre-change) |
|---|---|
| `AirPurifier::send_ping_` | air .cpp:180-191 |
| `AirPurifier::send_command_` | air .cpp:194-207 |
| `AirPurifier::send_wifi_status` (packet) | air .cpp:280-295 |
| `AirPurifier::send_timer_cancel_` | air .cpp:305-318 |
| `Humidifier::send_ping_` | humid .cpp:150-160 |
| `Humidifier::send_command_` | humid .cpp:163-175 |
| `Humidifier::send_wifi_status` (packet) | humid .cpp:259-274 |

Checks before opening the PR:

1. `git diff master -- components` must contain **zero** changed lines inside any
   `clang-format off` region, and zero changes to either `calc_checksum_`.
2. `git diff master -- components | rg -n '^[+-].*(packet\[|write_array|0x)'` -
   review every hit. The only expected value-bearing hits are:
   `DISPLAY_ON_BRIGHTNESS = 0x64` (moved constant, same value),
   `// 0x06 = ...` (comment), and hex in log format strings.
3. Argue each of the three call sites that feed TX bytes:
   - `send_display`: `DisplayBrightness::ON (0x64)` -> `DISPLAY_ON_BRIGHTNESS
     (0x64)`; `DisplayBrightness::OFF (0x00)` -> `VALUE_OFF (0x00)`. Identical.
   - `send_fan_speed`: `clamp(speed, 1, 4)` before and after. Identical.
   - `send_wifi_status`: only the log lines moved; `status` is computed by the
     same three-way branch and still feeds `WifiLedTLV::STATUS`. Identical.
   - `send_timer_cancel_`: body untouched; only *whether* it is called changes,
     and only when the new key is off.
4. Sanity-check entity names: `git diff master -- '*.yaml' | rg 'name:'` must
   show only the new commented `# cancel_device_timer: true` context lines, no
   `name:` changes.

## 7. Implementation checklist

Work in this order; each step ends with a buildable tree.

**Step 0 - setup**
1. Confirm `git branch --show-current` is `cleanup-audit` and it is up to date
   with `master`.
2. Run the §5 greps; record the output. Abort and report if anything unexpected.
3. `git ls-files plans/` - if `plans/` is tracked (it is), this plan file gets
   committed with commit 1.

**Step 1 - table lookups (commit 1)**
4. Air `types.h`: replace the Air Quality block per 4.1a.
5. Air `types.h`: replace `mode_to_string` per 4.1b (order `MANUAL, SLEEP,
   AUTO` - re-read `enum class Mode` first).
6. Humid `types.h`: replace `mode_to_string` per 4.4 (order `AUTO, MANUAL,
   SLEEP` - re-read the enum first).
7. Air `.cpp`: rewrite `case TLV::AIR_QUALITY` per 4.2i.
8. Build check (§8). Commit.

**Step 2 - dead constants (commit 2)**
9. Air `types.h`: drop `FAN_SPEED_MIN`/`MAX` (4.1c); drop `DisplayBrightness`,
   add `DISPLAY_ON_BRIGHTNESS` (4.1d).
10. Air `.cpp`: `control()` guard removal (4.2a), `std::clamp<uint8_t>` (4.2b),
    `send_display` (4.2c), `case TLV::SPEED` bound (4.2g).
11. Humid `types.h`: rename `VALUE_ACTIVE` -> `DISPLAY_ON_BRIGHTNESS` (4.4).
12. Humid `.cpp:336`: use the new name (4.5c).
13. Build check. Commit.

**Step 3 - guards, logging, dead members (commit 3)**
14. Air `.cpp`: `parse_packet_` reorder + guard removal (4.2e) - re-read §3.3.
15. Air `types.h`: delete `RX_MIN_PACKET_LEN`.
16. Humid `.cpp`: `parse_packet_` guard -> `Offset::TLV_START` (4.5b); humid
    `types.h`: delete `RX_MIN_PACKET_LEN`.
17. Both `.cpp`: `send_wifi_status` logging (4.2d / 4.5a) - keep the two
    versions textually identical.
18. Air `.cpp`: `case TLV::MODE` local + braces (4.2f); air `.h`: delete
    `last_mode_`.
19. Air `.cpp`: delete `case TLV::DISPLAY_CURRENT` (4.2h); air `types.h`: swap
    the enumerator for the comment (4.1f).
20. Re-run the §5 greps: deleted names must have zero hits. Build check. Commit.

**Step 4 - timer opt-in (commit 4)**
21. `__init__.py`: const, schema entry, `to_code` line (4.7a).
22. Air `.h`: setter + member (4.7b).
23. Air `.cpp`: flag check in `TimerTLV::TOTAL` + `dump_config` line (4.7c).
24. `air_purifier_vital200s.yaml`: commented key + two-line explanation (4.7d).
25. `README.md`: option table + TODO bullet amendment (4.7e).
26. `esphome config tests/air_purifier_vital200s.yaml` must pass; then flip the
    sample to `cancel_device_timer: true` temporarily and confirm it also
    validates, then revert to the commented-out form. Commit.

**Step 5 - CI (commit 5)**
27. `.github/workflows/ci.yaml`: delete the upload step and `id: build` (4.6).
    Commit.

**Step 6 - verify and PR**
28. Run the full §6 byte-freeze checklist.
29. Run the §8 checks once more over the whole branch.
30. Push and open one PR from `cleanup-audit` -> `master`. The description must
    state the `cancel_device_timer` default change as a **behaviour change for
    existing users** and link this plan.

## 8. Build / format checks

```bash
# formatting (pin matters: 20.1.7)
clang-format --version
find components \( -name '*.cpp' -o -name '*.h' \) -print0 | xargs -0 clang-format --dry-run --Werror

# config validation (needs secrets; both copies, see README "Development")
cp -n secrets.yaml.example secrets.yaml
cp -n secrets.yaml.example tests/secrets.yaml
esphome config tests/air_purifier_vital200s.yaml
esphome config tests/humidifier_oasismist1000s.yaml

# compile at least the air purifier - that is where the C++ changed
esphome compile tests/air_purifier_vital200s.yaml
```

If `clang-format --dry-run` fails, run `clang-format -i` on **only the files you
edited**, then `git diff` and revert any hunk you did not intend (a mismatched
local clang-format version can reflow untouched code; if that happens, hand-fix
instead and note it in the PR). Never commit `secrets.yaml` or `tests/secrets.yaml`
(git-ignored), and never add `.claude/` config to the repo.

## 9. Commits

Five commits, each of which compiles on its own (this is why the types.h and
.cpp halves of a change are not split into separate "audit-types" /
"audit-cpp" commits - a types-only commit would not build).

| # | Message |
|---|---|
| 1 | `refactor: table-driven mode and air-quality lookups` |
| 2 | `refactor: drop redundant fan-speed and display-brightness constants` |
| 3 | `refactor: tighten packet guards, wifi-led logging and dead state` |
| 4 | `feat(air_purifier): opt-in cancel_device_timer` |
| 5 | `ci: drop firmware artifact upload` |

Trailer style, confirmed against the repo's last commit message
(`git log -1 --format=%B master` / `.git/COMMIT_EDITMSG`) - blank line, then
exactly two lines:

```
Co-Authored-By: Claude <Model> <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_012RUYiRNZCXJk84M5MsBbKL
```

Use the model name from your own session's attribution instructions (history
shows e.g. `Claude Sonnet 5`); keep the `Claude-Session` URL above. PR
description ends with the `🤖 Generated with [Claude Code](https://claude.com/claude-code)`
line plus the same session URL.

## 10. Net line-count estimate

| Area | Delta |
|---|---|
| air `types.h` (air quality -29, mode -4, fan speed -2, brightness -3, RX const -1, TLV enum 0) | ~ -39 |
| humid `types.h` (mode -4, RX const -1) | ~ -5 |
| air `.cpp` (control -3, wifi log -5, display -1, DISPLAY_CURRENT -3, air quality -1, parse guard -3, MODE braces +1, dump_config +1) | ~ -14 |
| humid `.cpp` (wifi log -5) | ~ -5 |
| air `.h` (-1 member, +2 option) | ~ +1 |
| `__init__.py` | ~ +5 |
| `.github/workflows/ci.yaml` | -7 |
| sample YAML + README | ~ +10 |
| **Net (excluding this plan file)** | **~ -54** |

## 11. Risks and open questions

1. **Open question for the maintainer: the `cancel_device_timer` default.** This
   plan ships `false` (§4.7). That silently disables today's behaviour for
   anyone who updates the component. If the maintainer wants continuity instead,
   switch `default=False` -> `default=True` in `__init__.py` *and*
   `bool cancel_device_timer_{true}` in the header, and change the sample YAML
   comment to show `cancel_device_timer: false` as the opt-out. Do not change
   one without the other.
2. **Table indexing.** The two `MODE_NAMES` arrays are ordered differently per
   device. Getting one wrong silently mislabels the fan's preset in Home
   Assistant *and* breaks the `set_preset_mode_` round-trip (the string must
   still be one of the registered presets). Verify against each `enum class Mode`
   and, ideally, watch the DEBUG `Mode:` log on real hardware.
3. **`constexpr` + `static` local** would not compile on C++17/20 - §3.1. If the
   compiler complains about the tables, the fix is to drop `constexpr` from the
   function, not to move the array into it.
4. **`parse_packet_` operand order** - §3.3. Deleting the guard without swapping
   the `&&` operands reintroduces a short over-read.
5. **`case TLV::MODE` needs braces** - §4.2f - or the build fails with "jump to
   case label crosses initialization".
6. **`std::clamp` type mismatch** - `std::clamp(speed, 1, FAN_SPEED_COUNT)`
   without the explicit `<uint8_t>` will not compile.
7. **Timer code stays resident.** `send_timer_cancel_()` is guarded by a runtime
   bool, not compiled out, so flash usage is unchanged whichever way the key is
   set. That is intentional and fine.
8. **`Unknown TLV 0x06`** will now appear in VERBOSE logs on the air purifier.
   Acceptable and documented, but if the maintainer dislikes noise in captures,
   the alternative is to keep an empty `case TLV::DISPLAY_CURRENT: break;`
   (2 lines) instead of removing it.
9. **Air-quality byte 5+**: unchanged behaviour ("Unknown"), so the README's
   "confirm the device never emits a 5th" TODO stays accurate.
10. **Line-limit fragility**: the `AIR_QUALITY_NAMES` initialiser is 117/120
    chars. Any rename or added entry forces a clang-format reflow - accept the
    reflow rather than shortening the displayed strings (they are user-visible
    entity states).
