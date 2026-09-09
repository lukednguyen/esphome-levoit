# Plan: opt-in Wi-Fi status LED + retuned blink states

Branch: `cleanup-audit` (continues from `06b0115`, `ci: drop firmware artifact
upload`). Same branch, same single PR as the audit-cleanup work.

## 1. Summary

The Wi-Fi-status-LED feature drives the device's front LED from
Home-Assistant/Wi-Fi connection state. Today it is unconditional and its state
mapping is opinionated. Two changes, shipped together:

1. **Make it opt-in.** New `wifi_status_led:` boolean config key on *both*
   component blocks, **default `false`**. When off, `send_wifi_status()` returns
   on its first line. The sample YAMLs keep calling it from a lambda, so the C++
   guard is the whole off-switch — no structural YAML change.
2. **Retune the states.** Three visually distinct states instead of two plus
   "dark":

| Condition | Today | After |
|---|---|---|
| HA (API) connected | solid | solid |
| Wi-Fi up, HA not connected | blink 500/500 ms | **slow** blink, on-heavy (900 on / 300 off) |
| No Wi-Fi | **LED off** | **fast** blink 200/200 ms ("searching") |

**This deliberately changes the TX bytes of the Wi-Fi-LED packet** — and only
that packet. Every other TX builder (`send_ping_`, `send_command_`,
`send_timer_cancel_`) and both `calc_checksum_` stay byte-identical, the
Wi-Fi-LED packet's structure/length/TLV order is unchanged, and
`PayloadLen::WIFI_LED` (18) still matches. See §6 for the verification
procedure.

Expected net: **about +55 lines** (§10).

## 2. Relevant files and current behaviour

| File | Role / current state |
|---|---|
| `/Users/luke/esphome-levoit/components/air_purifier_vital200s/types.h` | 172 lines. `WifiLedStatus` (OFF/SOLID/BLINKING) + `WifiLedTLV` (STATUS/BLINK_ON/BLINK_OFF/RESET_FLAG) + `WIFI_BLINK_MS = 500` at lines 159-169. |
| `/Users/luke/esphome-levoit/components/air_purifier_vital200s/air_purifier_vital200s.cpp` | 445 lines. `send_wifi_status()` at 257-289; `dump_config()` at 93-105 (already logs `Cancel device timer`). |
| `/Users/luke/esphome-levoit/components/air_purifier_vital200s/air_purifier_vital200s.h` | 126 lines. Has an `// Options` block (line 78-79) with `set_cancel_device_timer`, and a `// State` block ending in `bool cancel_device_timer_{false};`. |
| `/Users/luke/esphome-levoit/components/air_purifier_vital200s/__init__.py` | Schema + `to_code`. `CONF_CANCEL_DEVICE_TIMER` (line 32), `cv.Optional(..., default=False): cv.boolean` under a `# Behaviour` comment (38-39), `cg.add(var.set_cancel_device_timer(...))` (84). **This is the pattern to copy.** |
| `/Users/luke/esphome-levoit/components/humidifier_oasismist1000s/types.h` | 143 lines. Same Wi-Fi-LED block at 129-139. |
| `/Users/luke/esphome-levoit/components/humidifier_oasismist1000s/humidifier_oasismist1000s.cpp` | 374 lines. `send_wifi_status()` at 239-270 — identical to the air one **except it has no `ESP_LOGD(TAG, "TX WiFi LED: ...")` line**; `dump_config()` at 77-89. |
| `/Users/luke/esphome-levoit/components/humidifier_oasismist1000s/humidifier_oasismist1000s.h` | 122 lines. **No `// Options` block yet**; state members end at `bool power_on_{false};`. |
| `/Users/luke/esphome-levoit/components/humidifier_oasismist1000s/__init__.py` | Schema + `to_code`. No scalar options yet; no `# Behaviour` comment. |
| `/Users/luke/esphome-levoit/air_purifier_vital200s.yaml` | Sample. `script: update_wifi_led` (67-75), `binary_sensor: platform: status` (77-83), component block 91-109 (commented `cancel_device_timer` at 94-96). |
| `/Users/luke/esphome-levoit/humidifier_oasismist1000s.yaml` | Sample. Same script/binary_sensor wiring; component block 87-104. |
| `/Users/luke/esphome-levoit/README.md` | "Common to both components" table (85-90); air option table (103-107, headed "Plus one behaviour option:"); humidifier section (109-119); protocol TODO bullet about echoed Wi-Fi-LED TLVs (181). |
| `/Users/luke/esphome-levoit/tests/*.yaml` | CI wrappers that `!include` the samples. **No change needed** — they inherit whatever the samples do. |

Facts confirmed by grep (do not re-derive):

- `send_wifi_status` is called **only** from the two sample YAML lambdas. No C++
  caller, no other component code path. Its signature must not change.
- `WIFI_BLINK_MS` is referenced only inside each component's own
  `send_wifi_status` (air `.cpp:270-271`, humid `.cpp:252-253`).
- `WifiLedStatus::OFF` is referenced only at air `.cpp:266` / humid `.cpp:248`
  — the branch this plan removes.
- `YESNO()` comes from `esphome/core/helpers.h`, already included by **both**
  `.cpp` files.
- `.clang-format`: Google, IndentWidth 2, ColumnLimit 120, short `if` on one
  line allowed, pinned to clang-format **20.1.7** in CI.

### The current packet (24 bytes)

```
idx  0     1     2     3     4     5     6..9        10 11 12    13 14 15 16   17 18 19 20   21 22 23
     A5    22    seq   12    00    cks   ADDR_WIFI   01 01 st    02 02 lo hi   03 02 lo hi   04 01 00
                       (18)              _LED        STATUS      BLINK_ON      BLINK_OFF     RESET_FLAG
```

`sizeof(packet) == 24 == 6 header + 18 payload`, and payload `18 == 4 addr +
3 + 4 + 4 + 3 TLV bytes`. This matches the convention of the other builders
(`PING = 4 = addr`, `COMMAND = 7 = addr + 3`, `TIMER_CANCEL = 10 = addr + 6`).

## 3. Design decisions

### 3.1 Blink timings — **fast 200/200, slow 900/300**

```cpp
inline constexpr uint16_t WIFI_BLINK_FAST_ON_MS = 200;
inline constexpr uint16_t WIFI_BLINK_FAST_OFF_MS = 200;
inline constexpr uint16_t WIFI_BLINK_SLOW_ON_MS = 900;
inline constexpr uint16_t WIFI_BLINK_SLOW_OFF_MS = 300;
```

Reasoning:

- **The two blink states must be unmistakable from across a room**, not merely
  measurably different. 200/200 (2.5 Hz) versus 900/300 (0.83 Hz) is a 4.5x
  period ratio *and* a duty-cycle difference (50 % vs 75 %), so they differ on
  two axes.
- **Asymmetry encodes progress.** "Mostly on with a short dropout" reads as
  *nearly there* (Wi-Fi up, waiting for HA); symmetric fast flicker reads as
  *hunting* (no Wi-Fi). That gives a monotonic ladder: solid (100 % on) ->
  900/300 (75 %) -> 200/200 (50 %, fast).
- **200 rather than 150** for the fast state: the MCU's timer granularity is
  unknown (10 ms or 100 ms ticks are both plausible), and 200 is safe under
  either; 150 could be rounded to 100 or 200 by the firmware and silently change
  the ratio. 200 ms is also the low end of "clearly blinking, not flickering".
  If it does not read as *fast* on hardware, 150/150 is a drop-in edit of one
  constant (§11.1).
- Four separate `constexpr` values rather than two "period" constants: the MCU
  genuinely takes independent on-time and off-time TLVs, and the slow state uses
  that. Naming them per-edge keeps the packet code free of arithmetic.
- **Both components get byte-identical constants and comments.**

### 3.2 What blink values are sent in the SOLID state?

The blink TLVs are still present (structure is frozen) but presumably ignored by
the MCU when `STATUS = SOLID`. **Send the slow pair.** Do *not* send zeros: a
0 ms on- or off-time is the kind of value that trips divide-by-zero or
"always off" edge cases in unknown firmware, and today's code already sends a
nonzero pair (500/500) in this state. Sending the slow pair also means the
implementation needs only one "default then override" assignment (§4.2).

### 3.3 `WifiLedStatus::OFF` — **keep the enumerator**

After this change nothing sends `0x00`. Keep it anyway, with a comment marking
it as protocol documentation. Rationale: it is one line, it is reverse-
engineered knowledge that is expensive to re-derive, an unused `enum class`
enumerator produces no warning and no code, and the repo already prefers
preserving protocol facts as comments (see the filter-reset note and the
`0x06 = live display brightness` note in air `types.h`). It is also the obvious
building block if a future "LED off at night" option is added.

### 3.4 `RESET_FLAG` — untouched

The trailing `0x04, 0x01, 0x00` TLV stays exactly as written. Its meaning is not
understood; changing it is not part of this task.

### 3.5 Guard placement and the `default=False` schema

Guard on the **first line of `send_wifi_status()`**, before any packet
construction or logging: `if (!wifi_status_led_) return;`. That makes the
disabled path a single branch and a return — the sample's `mode: restart` script
can fire on every Wi-Fi event with no cost.

Schema: use `cv.Optional(CONF_WIFI_STATUS_LED, default=False): cv.boolean` plus
an unconditional `cg.add(var.set_wifi_status_led(config[CONF_WIFI_STATUS_LED]))`
— i.e. exactly the shape `cancel_device_timer` already uses in the air
purifier's `__init__.py`. This is a deliberate reading of "set from `to_code`
when the key is present and true": with `default=False` the key is *always*
present after validation, `set_wifi_status_led(false)` is a no-op against the
member's `{false}` initialiser, and the value shows up in `esphome config`
output. The alternative (`cv.Optional` with no default + `if config.get(...)`)
saves nothing and would make the two options in the same file look different.
The C++ member default and the schema default must stay in agreement: both
`false`.

### 3.6 Add the missing TX log line to the humidifier

Air's `send_wifi_status` ends with
`ESP_LOGD(TAG, "TX WiFi LED: %s", format_hex_pretty(packet, sizeof(packet)).c_str());`;
the humidifier's does not. Add it, so the two functions are textually identical
(a stated constraint) and so the new timing bytes can actually be verified on
hardware from the log. This adds a log line, not a TX byte.

### 3.7 Sample YAML: key **uncommented** (`wifi_status_led: true`)

Both samples already ship the whole mechanism — the `update_wifi_led` script,
the `on_boot` hook, the wifi `on_connect`/`on_disconnect` hooks and the
`platform: status` binary sensor. Shipping all of that wired to a function that
returns immediately would make the sample look broken and would give a reader no
way to discover the feature. So the sample opts in explicitly, with a comment
above it stating what the three states mean and that switching the feature off
means deleting *both* the key and the script. (`cancel_device_timer` is
commented out in the sample for the opposite reason: it has no other YAML
machinery to explain, and it overrides a user's physical button press.)

### 3.8 The key alone does not make the feature self-contained

`wifi_status_led: true` in a config that does *not* also contain the
`update_wifi_led` script does nothing at all, because nothing ever calls
`send_wifi_status()`. That is a real usability trap for anyone copying only the
component block out of the sample. It is **not** fixed here (doing so means the
component polling API/Wi-Fi state itself — a bigger change, §11.6); instead the
README text must say the key has to be paired with the script. Word it that way,
not as "enables the LED".

## 4. Proposed changes, file by file

### 4.1 `types.h` (both components — identical edits)

Air lines 159-169, humidifier lines 129-139.

Before:

```cpp
enum class WifiLedStatus : uint8_t {
  OFF = 0x00,       // disconnected
  SOLID = 0x01,     // HA connected
  BLINKING = 0x02,  // WiFi only, connecting to HA
};

inline constexpr uint16_t WIFI_BLINK_MS = 500;
```

After:

```cpp
enum class WifiLedStatus : uint8_t {
  OFF = 0x00,       // protocol only; never sent
  SOLID = 0x01,     // HA connected
  BLINKING = 0x02,  // WiFi only (slow) or no WiFi (fast)
};

// fast = no WiFi (searching), slow = WiFi up but no HA
inline constexpr uint16_t WIFI_BLINK_FAST_ON_MS = 200;
inline constexpr uint16_t WIFI_BLINK_FAST_OFF_MS = 200;
inline constexpr uint16_t WIFI_BLINK_SLOW_ON_MS = 900;
inline constexpr uint16_t WIFI_BLINK_SLOW_OFF_MS = 300;
```

Keep the `// ==== WiFi LED ====` banner above it. Do not touch `WifiLedTLV`.

### 4.2 `send_wifi_status()` (both components — identical bodies)

Air `.cpp:257-289`, humid `.cpp:239-270`.

Before (air; humid is the same minus the final `ESP_LOGD`):

```cpp
void AirPurifier::send_wifi_status(bool ha_connected, bool wifi_connected) {
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

  constexpr uint8_t blink_lo = WIFI_BLINK_MS & 0xFF;
  constexpr uint8_t blink_hi = (WIFI_BLINK_MS >> 8) & 0xFF;
  ...
      static_cast<uint8_t>(WifiLedTLV::BLINK_ON), 0x02, blink_lo, blink_hi,
      static_cast<uint8_t>(WifiLedTLV::BLINK_OFF), 0x02, blink_lo, blink_hi,
```

After (write this verbatim in both files, changing only the class name):

```cpp
void AirPurifier::send_wifi_status(bool ha_connected, bool wifi_connected) {
  if (!wifi_status_led_) return;

  // default = WiFi up, no HA; the other two states adjust from here
  WifiLedStatus status = WifiLedStatus::BLINKING;
  uint16_t on_ms = WIFI_BLINK_SLOW_ON_MS;
  uint16_t off_ms = WIFI_BLINK_SLOW_OFF_MS;

  if (ha_connected) {
    status = WifiLedStatus::SOLID;
    ESP_LOGI(TAG, "WiFi LED: solid (HA connected)");
  } else if (wifi_connected) {
    ESP_LOGI(TAG, "WiFi LED: slow blink (WiFi only)");
  } else {
    on_ms = WIFI_BLINK_FAST_ON_MS;
    off_ms = WIFI_BLINK_FAST_OFF_MS;
    ESP_LOGI(TAG, "WiFi LED: fast blink (no WiFi)");
  }

  const uint8_t on_lo = static_cast<uint8_t>(on_ms & 0xFF);
  const uint8_t on_hi = static_cast<uint8_t>(on_ms >> 8);
  const uint8_t off_lo = static_cast<uint8_t>(off_ms & 0xFF);
  const uint8_t off_hi = static_cast<uint8_t>(off_ms >> 8);

  // clang-format off
  uint8_t packet[] = {
      PACKET_HEADER, static_cast<uint8_t>(PacketType::STATUS),  // header, packet type
      seq_++, static_cast<uint8_t>(PayloadLen::WIFI_LED),       // sequence, payload length
      0x00, 0x00,                                               // reserved, checksum placeholder
      ADDR_WIFI_LED[0], ADDR_WIFI_LED[1], ADDR_WIFI_LED[2], ADDR_WIFI_LED[3],
      static_cast<uint8_t>(WifiLedTLV::STATUS), 0x01, static_cast<uint8_t>(status),
      static_cast<uint8_t>(WifiLedTLV::BLINK_ON), 0x02, on_lo, on_hi,
      static_cast<uint8_t>(WifiLedTLV::BLINK_OFF), 0x02, off_lo, off_hi,
      static_cast<uint8_t>(WifiLedTLV::RESET_FLAG), 0x01, 0x00,
  };
  // clang-format on

  packet[static_cast<size_t>(Offset::CHECKSUM)] = calc_checksum_(packet, sizeof(packet));
  write_array(packet, sizeof(packet));
  ESP_LOGD(TAG, "TX WiFi LED: %s", format_hex_pretty(packet, sizeof(packet)).c_str());
}
```

Three things that are **not** optional:

1. **The `on_lo`/`on_hi`/`off_lo`/`off_hi` locals must be `uint8_t` before the
   array.** Writing `on_ms & 0xFF` directly inside the braced initialiser is a
   narrowing conversion from a non-constant `int` and is ill-formed — the
   current code only gets away with it because `blink_lo`/`blink_hi` are
   `constexpr`. The `static_cast<uint8_t>` is what makes the intent explicit and
   silences any `-Wconversion` build.
2. **Inside the `// clang-format off` block, only the four identifiers
   `blink_lo`/`blink_hi` -> `on_lo`/`on_hi`/`off_lo`/`off_hi` change.** Leave the
   alignment of the three trailing comments exactly as-is.
3. **The two component versions must stay textually identical** apart from the
   class name (see the §6 diff check).

### 4.3 Headers

`air_purifier_vital200s.h` — extend the existing `// Options` block (line 78-79):

```cpp
  // Options
  void set_cancel_device_timer(bool enable) { cancel_device_timer_ = enable; }
  void set_wifi_status_led(bool enable) { wifi_status_led_ = enable; }
```

and the `// State` block, after `bool cancel_device_timer_{false};`:

```cpp
  bool wifi_status_led_{false};
```

`humidifier_oasismist1000s.h` — new `// Options` block after
`set_target_humidity_number(...)` and before `// Commands`:

```cpp
  // Options
  void set_wifi_status_led(bool enable) { wifi_status_led_ = enable; }
```

and in `// State`, after `bool power_on_{false};`:

```cpp
  bool wifi_status_led_{false};
```

### 4.4 `dump_config()`

Air (after the `Cancel device timer` line, before `LOG_SENSOR`):

```cpp
  ESP_LOGCONFIG(TAG, "  WiFi status LED: %s", YESNO(wifi_status_led_));
```

Humidifier (after the `Update interval` line, before `LOG_SENSOR`): the same
line, verbatim. `YESNO` is already available via `esphome/core/helpers.h` in
both files.

### 4.5 `components/air_purifier_vital200s/__init__.py`

Config-key constant, after `CONF_CANCEL_DEVICE_TIMER` (line 32):

```python
CONF_WIFI_STATUS_LED = "wifi_status_led"
```

Schema, under the existing `# Behaviour` comment (after line 39):

```python
            # Behaviour
            cv.Optional(CONF_CANCEL_DEVICE_TIMER, default=False): cv.boolean,
            cv.Optional(CONF_WIFI_STATUS_LED, default=False): cv.boolean,
```

`to_code`, after the existing `set_cancel_device_timer` line:

```python
    cg.add(var.set_wifi_status_led(config[CONF_WIFI_STATUS_LED]))
```

### 4.6 `components/humidifier_oasismist1000s/__init__.py`

Config-key constant, after `CONF_TARGET_HUMIDITY` (line 35):

```python
CONF_WIFI_STATUS_LED = "wifi_status_led"
```

Schema, immediately after `cv.GenerateID(): cv.declare_id(Humidifier),` — this
introduces the `# Behaviour` comment to this file so both components read the
same way:

```python
            cv.GenerateID(): cv.declare_id(Humidifier),
            # Behaviour
            cv.Optional(CONF_WIFI_STATUS_LED, default=False): cv.boolean,
            # Sensor
```

`to_code`, immediately after `await uart.register_uart_device(var, config)`
(mirrors the air purifier's placement):

```python
    cg.add(var.set_wifi_status_led(config[CONF_WIFI_STATUS_LED]))
```

### 4.7 Sample YAMLs

`air_purifier_vital200s.yaml`, inside the component block after the commented
`cancel_device_timer` lines and before `fan:`:

```yaml
  # cancel_device_timer: true
  # Drive the front LED from connection state: solid = Home Assistant connected,
  # slow blink = Wi-Fi only, fast blink = searching for Wi-Fi. Needs the
  # update_wifi_led script above; delete both to leave the LED alone.
  wifi_status_led: true
  fan:
```

`humidifier_oasismist1000s.yaml`, the same three comment lines plus
`wifi_status_led: true`, inserted after `uart_id: uart_bus` and before `fan:`.

**Do not touch any `name:` value in either file.** No other YAML change — the
script, the `on_boot` hook, the wifi hooks and the status binary sensor all stay
exactly as they are.

### 4.8 `README.md`

**(a)** "Common to both components" table (lines 87-90) gains a row:

```markdown
| Key | Default | Description |
|---|---|---|
| `uart_id` | — | ID of the `uart:` bus |
| `update_interval` | `250ms` | MCU poll interval |
| `wifi_status_led` | `false` | Drive the device's front LED from connection state (see below) |
```

**(b)** Directly under that table, one short paragraph:

```markdown
With `wifi_status_led: true` the front LED shows **solid** when Home Assistant
is connected, a **slow blink** when Wi-Fi is up but Home Assistant is not, and a
**fast blink** while searching for Wi-Fi. The key only arms the feature — the
`update_wifi_led` script in the sample YAMLs is what actually calls it, so copy
both. With the key off (the default) the component never sends the LED command
and the LED keeps whatever state the device's MCU gave it.
```

This is one place rather than one per component **because the key, the states
and the wiring are identical in both** and duplicating six lines of prose is
exactly what the previous cleanup pass was trimming. The per-component tables
stay as they are; a reader of either section still finds the key in the common
table two screens up. (If the maintainer prefers per-component rows, the fix is
to copy the single table row into the air purifier's "Plus one behaviour
option" table — retitled "Plus behaviour options" — and into a new one-row table
under the humidifier's entity table.)

**(c)** Leave the protocol-TODO bullet at line 181 ("The Wi-Fi-LED TLVs the MCU
echoes back are intentionally ignored (TX only)") unchanged — still true.

## 5. Pre-flight greps

Run from the repo root before editing; stop and report anything not listed:

```bash
rg -n 'WIFI_BLINK_MS'        components   # expect 1 decl + 2 uses per component
rg -n 'WifiLedStatus::OFF'   components   # expect 1 use per component (the branch being removed)
rg -n 'send_wifi_status'     . -g '!plans/*'  # expect 2 .h decls, 2 .cpp defs, 2 YAML lambdas
rg -n 'wifi_status_led'      .            # expect zero hits outside plans/
```

After editing, re-run the first two: `WIFI_BLINK_MS` must be gone entirely, and
`WifiLedStatus::OFF` must appear only in the two `types.h` enum declarations.

## 6. Byte-freeze verification

### 6.1 No other packet changes (constraint a)

The seven TX builders and their `// clang-format off` regions:

| Function | File |
|---|---|
| `AirPurifier::send_ping_` | air `.cpp:177-189` |
| `AirPurifier::send_command_` | air `.cpp:191-205` |
| `AirPurifier::send_wifi_status` | air `.cpp:257-289` — **the only one that may change** |
| `AirPurifier::send_timer_cancel_` | air `.cpp:295-311` |
| `Humidifier::send_ping_` | humid `.cpp:149-160` |
| `Humidifier::send_command_` | humid `.cpp:162-175` |
| `Humidifier::send_wifi_status` | humid `.cpp:239-270` — **the only one that may change** |

Checks before opening the PR:

1. `git diff 06b0115 -- components` must show changed lines inside a
   `clang-format off` region **only** in the two `send_wifi_status` bodies, and
   there only the four identifier substitutions of §4.2.
2. `git diff 06b0115 -- components | rg '^[+-].*(calc_checksum_|write_array|PayloadLen::|packet\[)'`
   — the only expected hits are the unchanged-context lines of the two
   `send_wifi_status` functions. Neither `calc_checksum_` body may appear.
3. `git diff 06b0115 -- components | rg '^[+-].*0x'` — expected hits are limited
   to: the `types.h` enum comment edits, `on_ms & 0xFF` / `off_ms & 0xFF`, and
   the `on_lo/on_hi/off_lo/off_hi` packet lines. Anything else is a bug.
4. Entity names: `git diff 06b0115 -- '*.yaml' | rg 'name:'` must return nothing.

### 6.2 Structure, length and TLV order unchanged (constraint b)

Byte-by-byte, the packet keeps all 24 positions and the TLV order
`STATUS, BLINK_ON, BLINK_OFF, RESET_FLAG`. Only these indices can differ:

| idx | Meaning | Before | After |
|---|---|---|---|
| 5 | checksum | derived | derived (changes because payload does) |
| 12 | `STATUS` value | `01` / `02` / **`00`** | `01` / `02` / **`02`** |
| 15,16 | `BLINK_ON` lo,hi | `F4 01` (500) | `84 03` (900) or `C8 00` (200) |
| 19,20 | `BLINK_OFF` lo,hi | `F4 01` (500) | `2C 01` (300) or `C8 00` (200) |

Unchanged: `A5`, `22`, seq, the `18` payload-length byte, the reserved `00`, the
four address bytes, all four TLV **type** bytes (`01 02 03 04`), all four TLV
**length** bytes (`01 02 02 01`), and the `RESET_FLAG` value `00`.

Expected TX per state after the change (seq and checksum elided):

```
HA connected : A5 22 ss 12 00 cc 02 18 50 00  01 01 01  02 02 84 03  03 02 2C 01  04 01 00
WiFi only    : A5 22 ss 12 00 cc 02 18 50 00  01 01 02  02 02 84 03  03 02 2C 01  04 01 00
No WiFi      : A5 22 ss 12 00 cc 02 18 50 00  01 01 02  02 02 C8 00  03 02 C8 00  04 01 00
```

### 6.3 `PayloadLen::WIFI_LED` still matches (constraint c)

`sizeof(packet)` is still 24: 6 header bytes + 4 address + 3 (`STATUS`) +
4 (`BLINK_ON`) + 4 (`BLINK_OFF`) + 3 (`RESET_FLAG`) = 6 + 18. No TLV was added,
removed or resized, so the declared `PayloadLen::WIFI_LED = 18` remains correct
and `types.h` needs no `PayloadLen` edit. Confirm by counting the elements in
the edited array (24) — or, once, by temporarily adding
`static_assert(sizeof(packet) == 6 + static_cast<size_t>(PayloadLen::WIFI_LED));`
after the array, compiling, and **removing it again** (no other builder carries
one; do not commit it).

## 7. Implementation checklist

Each step ends with a tree that compiles.

**Step 0 — setup**

1. Confirm `git branch --show-current` is `cleanup-audit` and `git log -1
   --format=%h` is `06b0115` (or later on the same branch); working tree clean.
2. Run the §5 pre-flight greps; record the output.

**Step 1 — C++: constants, guard, new states (commit 1)**

3. Air `types.h`: replace the `WifiLedStatus` comments and `WIFI_BLINK_MS` per
   §4.1.
4. Humidifier `types.h`: the identical replacement (same four constants, same
   comment text).
5. Air `.h`: add `set_wifi_status_led` to `// Options` and
   `bool wifi_status_led_{false};` to `// State` (§4.3).
6. Humidifier `.h`: add the new `// Options` block and the state member (§4.3).
7. Air `.cpp`: rewrite `send_wifi_status` per §4.2. Re-read the three
   "not optional" points first.
8. Humidifier `.cpp`: paste the same body (class name differs), **including the
   trailing `ESP_LOGD(TAG, "TX WiFi LED: ...")` line that this file is currently
   missing** (§3.6).
9. Both `.cpp`: add the `WiFi status LED: %s` line to `dump_config()` (§4.4).
10. Diff the two `send_wifi_status` bodies against each other; they must be
    identical apart from `AirPurifier::` / `Humidifier::`.
11. Format + build checks (§8). Commit 1.

**Step 2 — config key (commit 2)**

12. Air `__init__.py`: constant, schema line, `to_code` line (§4.5).
13. Humidifier `__init__.py`: constant, `# Behaviour` + schema line, `to_code`
    line (§4.6).
14. `esphome config tests/air_purifier_vital200s.yaml` and
    `esphome config tests/humidifier_oasismist1000s.yaml` must both pass with
    the key still absent from the samples (proves the default path).
15. Commit 2.

**Step 3 — samples + docs (commit 3)**

16. `air_purifier_vital200s.yaml`: comment block + `wifi_status_led: true`
    (§4.7).
17. `humidifier_oasismist1000s.yaml`: the same (§4.7).
18. `README.md`: common-table row + paragraph (§4.8).
19. `esphome config` both test wrappers again — now exercising the key set to
    `true`.
20. `esphome compile tests/air_purifier_vital200s.yaml` and
    `esphome compile tests/humidifier_oasismist1000s.yaml` (both components'
    C++ changed, so compile both, not one).
21. Commit 3.

**Step 4 — verify and PR**

22. Run the whole §6 byte-freeze checklist against `06b0115`.
23. Re-run the §5 greps (post-edit expectations).
24. Push. The PR (one, covering the audit cleanup **and** this work) must state
    two behaviour changes for anyone tracking the branch: `cancel_device_timer`
    now defaults off, and the Wi-Fi LED is now opt-in *and* re-tuned (no-Wi-Fi
    is a fast blink, no longer dark).

## 8. Build / format checks

```bash
clang-format --version                     # must be 20.1.7
find components \( -name '*.cpp' -o -name '*.h' \) -print0 \
  | xargs -0 clang-format --dry-run --Werror

cp -n secrets.yaml.example secrets.yaml
cp -n secrets.yaml.example tests/secrets.yaml
esphome config  tests/air_purifier_vital200s.yaml
esphome config  tests/humidifier_oasismist1000s.yaml
esphome compile tests/air_purifier_vital200s.yaml
esphome compile tests/humidifier_oasismist1000s.yaml
```

If `clang-format --dry-run` fails, run `clang-format -i` on **only** the files
you edited, then inspect the diff and revert any hunk you did not intend. Never
commit `secrets.yaml` / `tests/secrets.yaml`, and never add `.claude/` to the
repo.

## 9. Commits

Three commits, each buildable on its own. After commit 1 the feature is present
but unreachable (no config key sets the flag) — that is fine and intentional.

| # | Message | Files |
|---|---|---|
| 1 | `feat: gate the wifi status LED and retune its blink states` | both `types.h`, both `.h`, both `.cpp` |
| 2 | `feat: add wifi_status_led config key to both components` | both `__init__.py` |
| 3 | `docs: enable and document the wifi status LED in the samples` | both sample YAMLs, `README.md`, this plan file |

Bodies: a two- or three-line body on commit 1 stating the new state mapping and
that the Wi-Fi-LED packet's TX bytes change deliberately while every other
packet is byte-identical.

Trailer style, confirmed from the branch's last commit message
(`git log -1 --format=%B` / `.git/COMMIT_EDITMSG` — `ci: drop firmware artifact
upload`): subject in lowercase conventional form, blank line, optional body,
blank line, then exactly two trailer lines:

```
Co-Authored-By: Claude <Model> <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_012RUYiRNZCXJk84M5MsBbKL
```

Use the model name from your own session's attribution instructions (the last
five commits on this branch used `Claude Sonnet 5`); keep the `Claude-Session`
URL above. The PR description ends with the
`🤖 Generated with [Claude Code](https://claude.com/claude-code)` line plus the
same session URL.

## 10. Net line-count estimate

| Area | Delta |
|---|---|
| air `types.h` (−1 const, +4 consts, +1 comment) | +4 |
| humid `types.h` (same) | +4 |
| air `.cpp` (guard +2, state decls +4−1, branch bodies −1, byte locals +2, dump_config +1) | +7 |
| humid `.cpp` (same, plus the TX log line) | +8 |
| air `.h` (setter +1, member +1) | +2 |
| humid `.h` (Options block +3, member +1) | +4 |
| air `__init__.py` (const +2, schema +1, to_code +1) | +4 |
| humid `__init__.py` (const +2, comment+schema +2, to_code +1) | +5 |
| sample YAMLs (4 lines each) | +8 |
| `README.md` (table row + paragraph) | +7 |
| **Net (excluding this plan file)** | **≈ +53** |

## 11. Risks and open questions

1. **The timings are guesses — no hardware in the loop.** 200/200 and 900/300
   were chosen for human legibility (§3.1), not measured against the MCU. The
   MCU may quantise, clamp or ignore values outside a range it accepts. Both
   `send_wifi_status` functions now log the full TX frame at `DEBUG`
   (`TX WiFi LED: ...`), so the first hardware test is: watch the log, confirm
   `84 03` / `2C 01` / `C8 00` land where §6.2 says, then look at the LED. All
   four values are single-line constants in `types.h`; retuning is a one-commit
   follow-up.
2. **With the feature on, the LED is never fully dark.** Previously "no Wi-Fi"
   meant the LED went out; now it blinks fast forever. On a bedroom humidifier
   that is a real annoyance if the router goes down overnight, and a device that
   permanently loses Wi-Fi becomes a permanent flashing beacon. Judged
   acceptable **because the feature is now opt-in**: a user who wants a dark LED
   simply omits the key (the default), which is strictly better than today,
   where the behaviour is forced. `WifiLedStatus::OFF` is kept (§3.3) so a
   future `wifi_status_led: off_when_disconnected`-style option, or a night-mode
   hook, has the protocol value ready. Call this out in the PR description.
3. **Silent behaviour change for anyone already running this branch.** After
   the update the LED stops being driven unless they add the key. Same class of
   change as `cancel_device_timer`, and it must be listed in the PR description
   alongside it.
4. **Narrowing conversion.** Putting `on_ms & 0xFF` straight into the braced
   initialiser will not compile (or will warn) now that the value is not
   `constexpr`. The `uint8_t` locals in §4.2 are mandatory, not stylistic.
5. **Divergence between the two components.** The two `send_wifi_status` bodies
   and the two `types.h` Wi-Fi-LED blocks must stay character-identical. Step
   1.10 exists specifically to catch a copy-paste that drifts (a different blink
   constant in one component would be invisible until someone owns both
   devices).
6. **Open question / out of scope: the YAML script.** The component could drive
   the LED itself — poll `api::global_api_server->is_connected()` and
   `wifi::global_wifi_component->is_connected()` from `update()` and send only
   on change — which would delete ~20 lines of YAML from both samples, remove
   the §3.8 trap, and make `wifi_status_led: true` genuinely self-contained. It
   also adds an `api` dependency and a per-poll state comparison. **Not part of
   this change**; worth raising in the PR as the natural next step.
7. **`static_assert` on packet size** would permanently lock §6.3 in place for
   one line per component. Left out for consistency (no other builder has one),
   but it is the cheapest guard if the Wi-Fi-LED packet is ever edited again.
8. **README placement** (§4.8b) puts the LED documentation in the shared section
   rather than once per component, deviating from the task's literal "one line
   per component". Trivially reversible; flagged so the maintainer can decide.
