# Implementation Plan: one first-class `fan` entity for the OasisMist 1000S

> Status: ready to implement. Every judgement call is resolved below — do not
> re-derive them. Follow the **Checklist** in order.
>
> **Do not touch `PLAN.md`.** That plan is already implemented on branch
> `polish-components-and-ci`; this file is a separate, follow-on task that builds
> on top of it.

---

## 1. Task and goal

Make the Levoit OasisMist 1000S humidifier a first-class Home Assistant device by
collapsing three control entities into one:

| Today (3 entities) | After (1 entity) |
|---|---|
| `power` → `switch` | `fan` state (on/off) |
| `mist_level` → `number` 1-9 | `fan` speed 1-9 (`set_supported_speed_count(9)`) |
| `mode` → `select` Auto/Manual/Sleep | `fan` preset modes Auto/Manual/Sleep |

Unchanged: `target_humidity` stays a `number` (only meaningful in Auto/Sleep),
`humidity` stays a `sensor`, `reservoir`/`water`/`misting` stay diagnostic
`binary_sensor`s, `display` stays a `switch`.

This mirrors `components/air_purifier_vital200s/`, which already exposes a single
`PurifierFan`.

ESPHome has **no `humidifier` platform**, so the HA `humidifier` domain cannot be
produced from an ESPHome device; `fan` is the closest first-class representation
(`climate` was rejected — it is semantically a thermostat and would need
temperature traits). This must be stated in the README.

Two extra items are folded into the same coder pass:

- **GitHub issue #1** — ESPHome 2026.4.0 moved preset-mode vectors off `FanTraits`
  onto the `fan::Fan` entity. The existing `PurifierFan` uses the deprecated
  setter (removed in 2026.11.0) and must be migrated; the new `HumidifierFan` is
  written against the new API from the start. See §3 F4 and Phase 3.
- **Packet-array formatting** — the `clang-format` run from the previous plan
  reflowed the TX packet byte arrays in both components into ragged multi-column
  blocks. Wrap each `uint8_t packet[] = { ... };` in `// clang-format off` /
  `// clang-format on` and restore a readable hand layout, **with zero change to
  the emitted bytes**.

### Definition of done

- [ ] `humidifier_oasismist1000s.yaml` has a single `fan:` block and **no**
      `power:` / `mode:` / `mist_level:` keys.
- [ ] Pasting that YAML into ESPHome Builder yields one HA fan card with on/off,
      9 speeds and 3 presets, plus the target-humidity number, humidity sensor,
      display switch and three diagnostic binary sensors.
- [ ] `esphome config tests/humidifier_oasismist1000s.yaml` passes and its output
      contains a `fan` entity and no `select`, no power `switch`, no mist-level
      `number`.
- [ ] `esphome compile tests/humidifier_oasismist1000s.yaml` **and**
      `tests/air_purifier_vital200s.yaml` succeed with **no
      `set_supported_preset_modes` deprecation warning** (issue #1 closed).
- [ ] `clang-format --dry-run --Werror` over `components/` is silent.
- [ ] Every TX byte sequence is provably unchanged (§6).
- [ ] Work lands as 3 new commits on `polish-components-and-ci` with the repo's
      commit trailers; the PR closes #1.

---

## 2. Relevant files and current behaviour

All paths absolute.

| File | Current behaviour / what matters |
|---|---|
| `/Users/luke/esphome-levoit/components/humidifier_oasismist1000s/__init__.py` | 151 lines. `AUTO_LOAD = ["sensor", "switch", "select", "number", "binary_sensor"]`. Declares `PowerSwitch`, `DisplaySwitch`, `ModeSelect`, `TargetHumidityNumber`, `MistLevelNumber`. Config keys incl. `CONF_POWER`, `CONF_MODE`, `CONF_MIST_LEVEL`. Module constants `MODE_OPTIONS`, `HUMIDITY_MIN/MAX`, `MIST_LEVEL_MIN/MAX` kept in sync with `types.h`. |
| `/Users/luke/esphome-levoit/components/humidifier_oasismist1000s/humidifier_oasismist1000s.h` | 130 lines. Entity classes at the top, `Humidifier` component below with setters, `send_*` commands, private UART/parse helpers, entity members, and state `Mode last_mode_{Mode::AUTO}; bool power_on_{false};`. |
| `/Users/luke/esphome-levoit/components/humidifier_oasismist1000s/humidifier_oasismist1000s.cpp` | 347 lines. Entity impls (lines 16-24), `setup()`/`loop()`/`update()`/`dump_config()`, UART TX/RX, `send_power`/`send_display`/`send_mode`/`send_mist_level`/`send_target_humidity`/`send_wifi_status`, `parse_packet_`, `parse_tlvs_`, `handle_status_tlv_`, `invalidate_diagnostic_sensors_`. |
| `/Users/luke/esphome-levoit/components/humidifier_oasismist1000s/types.h` | 147 lines. `Mode { AUTO=0, MANUAL=1, SLEEP=2 }` (**different order from the purifier — do not "fix"**), `MODE_AUTO/MANUAL/SLEEP` string constants, `mode_to_string`/`string_to_mode`, `MIST_LEVEL_MIN=1`, `MIST_LEVEL_MAX=9`, `HUMIDITY_MIN=40`, `HUMIDITY_MAX=80`, `TLV::{POWER=0x02, MODE=0x0B, MIST_LEVEL=0x0C, ...}`. |
| `/Users/luke/esphome-levoit/humidifier_oasismist1000s.yaml` | Sample config. Component block at lines 85-105 lists `power`, `mode`, `target_humidity`, `mist_level`, `humidity`, `display`, `reservoir`, `water`, `misting`. |
| `/Users/luke/esphome-levoit/tests/humidifier_oasismist1000s.yaml` | 9-line CI wrapper; `!include`s the sample and overrides `component_source: ../components`. **Needs no edit** — only verification. |
| `/Users/luke/esphome-levoit/README.md` | Humidifier entity table at lines 72-84 lists `power`/`mode`/`mist_level`. |
| `/Users/luke/esphome-levoit/components/air_purifier_vital200s/*` | Reference implementation: `PurifierFan` in `__init__.py` (lines 20, 49-52, 89-92), `.h` (lines 34-39, 58-61, 107), `.cpp` (`get_traits` 22-30, `setup` 32-49, `control` 51-68, RX at 398-423). `AirPurifier::setup()` calls `fan_->setup()` at lines 78-81. |

### Command semantics already implemented (reuse, do not rewrite)

- `send_power(bool)` — raw power on/off.
- `send_mode(Mode)` — sends `ADDR_POWER=ON` first **if `power_on_` is false**,
  then `ADDR_MODE`. i.e. selecting a preset auto-powers the device on.
- `send_mist_level(level, auto_switch_mode)` — clamps to 1-9; sends power-on if
  needed; sends `MODE=MANUAL` if `auto_switch_mode && last_mode_ != MANUAL`
  (or if it had to power on); then `ADDR_MANUAL=level`.
- `send_target_humidity(humidity, auto_switch_mode)` — clamps 40-80; power-on if
  needed; switches to `AUTO` when the current mode is `MANUAL`.
- `power_on_` and `last_mode_` are maintained **only** from RX (`handle_status_tlv_`)
  and are read by the `send_*` side-effect logic. They must keep working.

---

## 3. Load-bearing findings (verified — do not re-investigate)

### F1 — ESPHome's fan preset API changed; the repo's `PurifierFan` uses the removed API

Verified against `https://raw.githubusercontent.com/esphome/esphome/release/esphome/components/fan/fan.h`,
`https://api-docs.esphome.io/fan_8h_source` and ESPHome PRs #11632 / #13092
("Entity getters now return StringRef", ESPHome 2026.1.0):

- ESPHome **2025.5.0** `class Fan` public members:
  `bool state; bool oscillating; int speed; FanDirection direction; std::string preset_mode{};`
  and `FanCall::get_preset_mode()` returned `std::string`.
- ESPHome **current release / dev** `class Fan` public members:
  `bool state; bool oscillating; int speed; FanDirection direction;`
  — **`preset_mode` is gone.** It is now `private: const char *preset_mode_{nullptr};`
  with public `StringRef get_preset_mode() const`, `bool has_preset_mode() const`
  and **protected** `bool set_preset_mode_(const char *)`, `clear_preset_mode_()`,
  `apply_preset_mode_(const FanCall &)`.
  `FanCall` exposes `const char *get_preset_mode() const` **plus**
  `bool has_preset_mode() const`.
- Migration published with PR #13092: `fan->preset_mode = "Auto";` →
  `fan->set_preset_mode_("Auto");`, and null checks → `has_preset_mode()`.

Consequence: `components/air_purifier_vital200s/air_purifier_vital200s.cpp`
lines 37, 39-40 and 409 (`fan_->preset_mode = ...`, `this->preset_mode.c_str()`,
`const std::string &preset = call.get_preset_mode()`) **will not compile** against
a current ESPHome. The repo has **zero GitHub Actions runs** (checked via the
GitHub API — the CI workflow has never executed), so this has never been caught.

### F4 — ESPHome 2026.4.0 moved preset-mode vectors onto the `Fan` entity (**issue #1**)

Verified verbatim against `https://raw.githubusercontent.com/esphome/esphome/dev/esphome/components/fan/fan.h`
and `.../fan/fan_traits.h` (ESPHome PR #15209, developer blog post
`https://developers.esphome.io/blog/2026/04/09/climate-and-fan-custom-mode-vectors-moved-to-entity/`):

`fan.h`, **public** on `class Fan`:

```cpp
  /// Set the supported preset modes (stored on Fan, referenced by FanTraits via pointer).
  void set_supported_preset_modes(std::initializer_list<const char *> preset_modes) {
    this->ensure_preset_modes_().assign(preset_modes.begin(), preset_modes.end());
  }
  void set_supported_preset_modes(const std::vector<const char *> &preset_modes) {
    this->ensure_preset_modes_() = preset_modes;
  }
```

`fan.h`, **protected** on `class Fan` (and `private: std::vector<const char *> *supported_preset_modes_{nullptr};`):

```cpp
  /// Wire the Fan-owned preset modes pointer into the given traits object.
  void wire_preset_modes_(FanTraits &traits) {
    if (this->supported_preset_modes_) {
      traits.set_supported_preset_modes_(this->supported_preset_modes_);
    }
  }
```

`get_traits()` is unchanged and is still the correct override point for speed,
oscillation, direction and speed count:

```cpp
  virtual FanTraits get_traits() = 0;
```

`fan_traits.h` — the old setters still exist but are deprecated, and the
`std::string` overloads are hard errors:

```cpp
  ESPDEPRECATED("Call set_supported_preset_modes() on the Fan entity instead. Removed in 2026.11.0", "2026.5.0")
  void set_supported_preset_modes(std::initializer_list<const char *> preset_modes) { this->compat_preset_modes_ = preset_modes; }
  ...
  void set_supported_preset_modes(const std::vector<std::string> &preset_modes) = delete;
```

`class FanTraits { friend class Fan; ... protected: void set_supported_preset_modes_(const std::vector<const char *> *preset_modes); }`
— i.e. the entity-owned vector can **only** reach the traits through
`Fan::wire_preset_modes_()`.

Conclusions that drive the plan:

1. CI's ESPHome (`latest` via `esphome/build-action@v8.1.0`, which requires
   ≥ 2026.7.0) still **has** the old setter — it is deprecated, not removed, so
   today's purifier code compiles but emits a deprecation warning. Removal lands
   in **2026.11.0**.
2. The new idiom is: `this->set_supported_preset_modes({...})` **once** in the
   fan's `setup()`, and `get_traits()` builds the traits **without** preset modes
   but **must** call `this->wire_preset_modes_(traits)` — otherwise the traits
   pointer stays null and Home Assistant sees no presets. (The snippet in issue #1
   omits `wire_preset_modes_`; the developer blog post includes it. Follow the
   blog and the header.)
3. `MODE_*` in both components are `inline constexpr const char *`, so the
   `const char *` overload is selected and the `= delete`d `std::string` overloads
   are never a problem.

### F2 — `Fan::restore_state_()` does not apply anything

`optional<FanRestoreState> restore_state_();` — it *returns* the recovered state.
Canonical ESPHome usage (`speed/fan/speed_fan.cpp`):

```cpp
auto restore = this->restore_state_();
if (restore.has_value()) {
  restore->apply(*this);
  this->write_state_();
}
```

`PurifierFan::setup()` calls `this->restore_state_();` and **discards the result**,
then reads `this->state`, which is therefore always the default `false`. So the
purifier unconditionally sends `send_power(false)` at boot — which is exactly why
its sample YAML needs `- fan.turn_on: air_purifier_fan` in `on_boot` (PLAN.md Q1).

Additionally `fan_schema`'s `restore_mode` defaults to **`ALWAYS_OFF`**, so even a
*correct* restore would force the device off after every reboot/OTA.

Note also that `fan::Fan` is **not** a `Component`: `PurifierFan::setup()` is a
plain, non-virtual method invoked manually from `AirPurifier::setup()`. Writing
`void setup() override;` on these classes is a compile error.

### F3 — Fan speed is 1..`supported_speed_count`

`FanCall::validate_()` clamps speed to `[1, traits.supported_speed_count()]`.
So `set_supported_speed_count(9)` gives speeds 1-9 that map **1:1** onto mist
levels 1-9. No scaling anywhere. (HA renders it as a percentage slider with
9 steps; `fan.set_percentage` round-trips through the same mapping.)

`validate_()` also: (a) clears an active preset when a speed is explicitly set —
HA convention, and exactly what we want since a mist-level change switches the
device to Manual; (b) if the fan is turned on with **no** active preset **and** no
speed, it forces speed to 100%. (b) is unreachable in practice here because the
MCU reports MODE and MIST_LEVEL every 250 ms poll, so both are populated within
one poll of boot (see R5).

---

## 4. Decisions

### D1 — Replace, don't add alongside. **Remove `power`, `mode` and `mist_level` entirely.**

Resolution: **replace**. Rationale:

1. *Two sources of truth is the opposite of first-class.* A `select` "Mode" and a
   fan preset row that both drive `send_mode()` produce a card where changing one
   silently moves the other; every automation author then has to pick. The air
   purifier — the model we are converging on — has never had duplicates.
2. *Nothing is lost.* Power = `fan.turn_on/off`, mist level =
   `fan.set_percentage` / `speed_level`, mode = `fan.set_preset_mode`. All three
   are first-class HA services with full automation/voice support.
3. *Power users are still covered.* The component's `send_*` methods are public
   and callable from a lambda, so anyone wanting the old shape can rebuild it in
   their own YAML in six lines — document this snippet in the README:

   ```yaml
   number:
     - platform: template
       name: "Mist Level"
       min_value: 1
       max_value: 9
       step: 1
       optimistic: true
       set_action:
         - lambda: id(humidifier).send_mist_level((uint8_t) x, true);
   ```

   Keeping the keys "just in case" costs schema surface, three extra entity
   classes, three extra RX publish paths and a permanently ambiguous README.
4. *Cost, stated honestly:* this is a **breaking change** for anyone already
   running the component — `switch.*_power`, `select.*_mode` and
   `number.*_mist_level` disappear. Mitigated by an explicit breaking-change note
   in the README and the PR body with a migration table (D10).

Minor accepted regression: the old `number` gave an exact 1-9 slider; the fan card
gives a percentage slider that snaps to 9 steps. The template-number snippet above
covers anyone who needs the exact widget.

### D2 — Speed semantics
`traits.set_speed(true); traits.set_supported_speed_count(MIST_LEVEL_MAX);` —
`MIST_LEVEL_MAX` is `9` and `MIST_LEVEL_MIN` is `1`, matching ESPHome's 1..count
convention exactly (F3). No scaling, no new constant.

### D3 — Preset modes: use the 2026.4.0 entity-level API (issue #1)

`HumidifierFan::setup()` registers the list once:

```cpp
this->set_supported_preset_modes({MODE_AUTO, MODE_MANUAL, MODE_SLEEP});
```

and `get_traits()` builds the traits without presets but calls
`this->wire_preset_modes_(traits);` (F4). Reuse the `types.h` constants; **order
the list Auto, Manual, Sleep** (matches the option order users see in the current
`select`).

⚠️ The list order is load-bearing beyond cosmetics: `FanRestoreState` stores the
preset as an **index into this list**, so reordering it silently remaps any
restored preset. Do not reorder it later, and do not reorder the purifier's
existing `{MODE_AUTO, MODE_SLEEP, MODE_MANUAL}` during the Phase 3 migration.

Do **not** reorder the `Mode` enum in `types.h`; its numeric values are the wire
protocol.

### D4 — `control()` mapping
Mirror `PurifierFan::control` structurally: state → `send_power`;
preset → `send_mode` (which auto-powers on); speed → `send_mist_level(speed, true)`
(which auto-powers on and switches to Manual). Guard the speed with
`>= MIST_LEVEL_MIN && <= MIST_LEVEL_MAX` like the purifier does.

`control()` must **not** write `this->state/speed` or call `publish_state()` — the
MCU is authoritative and echoes the real state within one 250 ms poll. This is the
purifier's existing behaviour and it is deliberate.

Known, accepted wart: a single HA "turn on at 50%" call carries both state and
speed, so we emit `POWER=ON` and then `send_mist_level()` emits a second
`POWER=ON` (because `power_on_` has not been refreshed from RX yet), then
`MODE=MANUAL`, then the level. Repeated identical commands are harmless on this
MCU (the existing `send_mode`/`send_mist_level` paths already rely on that).
Do not add optimistic mutation of `power_on_` to "fix" it.

### D5 — `HumidifierFan::setup()` registers presets and **nothing else**; no state is pushed at boot

`setup()` exists only because the 2026.4.0 preset API requires it (D3/F4). Its
body is exactly one statement. It must **not** restore or transmit anything.
Rationale, on the strength of F2:

- `PurifierFan::setup()` is buggy (`restore_state_()` result discarded), so what
  it actually does is `send_power(false)` on every boot.
- Even fixed, `restore_mode` defaults to `ALWAYS_OFF`, so the "restore" would
  power the humidifier **off** after every reboot and OTA update.
- The MCU reports POWER / MODE / MIST_LEVEL every poll (250 ms), so the entity
  self-syncs almost immediately with zero forced writes.

So: `Humidifier::setup()` calls `fan_->setup()` (needed for the preset
registration), the sample YAML gets **no** `fan.turn_on` in `on_boot`, and a
comment in the header records why. If the maintainer later wants symmetry with
the purifier, the correct implementation is:

```cpp
void HumidifierFan::setup() {
  this->set_supported_preset_modes({MODE_AUTO, MODE_MANUAL, MODE_SLEEP});
  auto restore = this->restore_state_();
  if (!restore.has_value())
    return;
  restore->apply(*this);            // writes state/speed/preset + publish_state()
  if (this->state) {
    parent_->send_power(true);
    parent_->send_mist_level(this->speed, true);
  } else {
    parent_->send_power(false);
  }
}
```

…together with `restore_mode: RESTORE_DEFAULT_OFF` in the YAML. Recorded as open
question **Q1**; do not implement it now.

### D6 — RX status → fan
In `handle_status_tlv_`: `TLV::POWER` → `fan_->state` + `publish_state()`;
`TLV::MODE` → `fan_->publish_mode(last_mode_)` (a small public wrapper on
`HumidifierFan`, so the RX code is identical under either fan API variant);
`TLV::MIST_LEVEL` → `fan_->speed` + `publish_state()`, guarded to 1-9 so an
out-of-range/0 report in Auto mode cannot blank the slider. `power_on_` and
`last_mode_` tracking stays exactly as it is.

Publish frequency (up to 3 `publish_state()` per 250 ms poll) is **not** changed
here — it matches the purifier and today's number/select publishes. Recorded as
open question **Q2**.

### D7 — `target_humidity` unchanged
Still a `number`, still `send_target_humidity(value, true)`, still 40-80. Its
`auto_switch_mode` behaviour (switch to Auto when currently in Manual) is
unchanged and now interacts with the fan preset via the normal RX echo.

### D8 — Schema
`AUTO_LOAD = ["sensor", "fan", "switch", "number", "binary_sensor"]` — gains
`fan`, **drops `select`** (no select entities remain). `CONF_POWER`, `CONF_MODE`,
`CONF_MIST_LEVEL` and their `to_code` branches are deleted; `CONF_FAN` added with
`fan.fan_schema(HumidifierFan, icon="mdi:air-humidifier")` and the purifier's
`new_Pvariable` + `register_fan` + `set_fan` codegen shape.

Unused Python constants `MODE_OPTIONS`, `MIST_LEVEL_MIN`, `MIST_LEVEL_MAX` are
removed (presets and speed count now live in C++); `HUMIDITY_MIN/MAX` stay.

### D9 — Packet-array formatting
`// clang-format off` / `// clang-format on` around each `uint8_t packet[] = {...}`
initializer in both components' `.cpp` files, with a hand layout of one logical
group per line. This is the accepted exception to PLAN.md R9 ("don't fight
clang-format") — byte tables are exactly what upstream ESPHome uses these markers
for, and the reflowed ragged columns are unreadable. **Bytes must not change**
(§6).

### D10 — Breaking-change communication
Add a short **Breaking changes** subsection to the README under the humidifier
table (a `CHANGELOG.md` is overkill for a repo with no releases; add one only if
the maintainer asks). Repeat it in the PR body.

---

## 5. Implementation checklist

Work in order. Commit boundaries are in Phase 4.

### Step 0 — Probe the ESPHome fan API (**mandatory, 2 minutes, blocks everything**)

1. Find the ESPHome package that will be used to build:
   ```bash
   python3 -c "import esphome, os; print(os.path.dirname(esphome.__file__))"
   esphome version
   ```
   (or, after any `esphome compile`, use
   `/Users/luke/esphome-levoit/tests/.esphome/build/humidifier/src/esphome/components/fan/fan.h`).
2. ```bash
   grep -n "preset_mode" <that path>/components/fan/fan.h
   grep -n "preset_mode" <that path>/components/fan/fan_traits.h
   ```
3. Classify:
   - **Variant C (expected, ESPHome ≥ 2026.4)** — `class Fan` has public
     `set_supported_preset_modes(std::initializer_list<const char *>)` **and**
     protected `wire_preset_modes_(FanTraits &)`, and `fan_traits.h` marks its
     own setters `ESPDEPRECATED`. Private `const char *preset_mode_{nullptr};`.
   - **Variant B (2026.1 - 2026.3)** — no `wire_preset_modes_`, but `preset_mode`
     is already private with protected `set_preset_mode_()`.
   - **Variant A (legacy, < 2026.1)** — public `std::string preset_mode{};` on
     `class Fan`.
4. Also note the return type of `FanCall::get_preset_mode()` (`const char *` or
   `StringRef`) — used in Step 3.
5. Record the variant and the ESPHome version in the PR body.

Every code block below marked *(A)*, *(B)* or *(C)* is variant-specific;
everything else is shared. **Variant C is the expected outcome** and is what the
main text assumes.

### Phase 1 — Humidifier component

#### Step 1 — `/Users/luke/esphome-levoit/components/humidifier_oasismist1000s/__init__.py`

1. Imports: drop `select`, add `fan` →
   `from esphome.components import uart, sensor, fan, switch, number, binary_sensor`
   (keep the purifier's ordering habit: `uart, sensor, fan, switch, ...`).
2. `AUTO_LOAD = ["sensor", "fan", "switch", "number", "binary_sensor"]`.
3. Delete the `PowerSwitch`, `ModeSelect` and `MistLevelNumber` `ns.class_(...)`
   declarations; add
   ```python
   HumidifierFan = ns.class_("HumidifierFan", fan.Fan, cg.Parented.template(Humidifier))
   ```
   placed first in that block (mirrors the purifier's ordering).
4. Config keys: delete `CONF_POWER`, `CONF_MODE`, `CONF_MIST_LEVEL`; add
   `CONF_FAN = "fan"`.
5. Constants block: delete `MODE_OPTIONS`, `MIST_LEVEL_MIN`, `MIST_LEVEL_MAX`;
   keep `HUMIDITY_MIN`/`HUMIDITY_MAX` and reword the comment to
   `# These must stay in sync with types.h (HUMIDITY_MIN/MAX).`
6. Schema: delete the `CONF_POWER` switch entry, the `CONF_MODE` select entry and
   the `CONF_MIST_LEVEL` number entry. Add, in a `# Fan` section placed **before**
   the switches (matching the purifier's section order):
   ```python
   # Fan (restore_mode is included in fan_schema)
   cv.Optional(CONF_FAN): fan.fan_schema(
       HumidifierFan,
       icon="mdi:air-humidifier",
   ),
   ```
7. `to_code`: delete the `CONF_POWER`, `CONF_MODE` and `CONF_MIST_LEVEL` blocks;
   add (immediately after the binary-sensor blocks, before `CONF_DISPLAY`):
   ```python
   if c := config.get(CONF_FAN):
       f = cg.new_Pvariable(c[CONF_ID])
       await fan.register_fan(f, c)
       cg.add(var.set_fan(f))
   ```
8. Verify `ENTITY_CATEGORY_NONE` is still imported *and* used (it is — by
   `target_humidity`); leave the rest of the `esphome.const` imports alone.

#### Step 2 — `/Users/luke/esphome-levoit/components/humidifier_oasismist1000s/humidifier_oasismist1000s.h`

1. Includes: remove `#include "esphome/components/select/select.h"`, add
   `#include "esphome/components/fan/fan.h"`. Keep the existing order convention
   (`SortIncludes: false`); put `fan.h` where `select.h` was, i.e. after
   `switch.h`, so the block reads sensor → switch → fan → number → binary_sensor.
   Do not move `#include "types.h"` — it must stay last.
2. Delete `class PowerSwitch`, `class ModeSelect`, `class MistLevelNumber`.
3. Add, as the **first** entity class (mirrors the purifier's file layout):
   ```cpp
   class HumidifierFan : public fan::Fan, public Parented<Humidifier> {
    public:
     fan::FanTraits get_traits() override;
     void control(const fan::FanCall &call) override;

     // Not a Component override: called manually from Humidifier::setup().
     // Registers the preset modes only — the device is authoritative, so nothing
     // is restored or transmitted at boot (the MCU's status packets populate
     // this entity within one poll).
     void setup();

     // Called from the RX path: the MCU is the source of truth for the mode.
     void publish_mode(Mode mode);
   };
   ```
   ⚠️ `void setup();` must **not** be marked `override` — `fan::Fan` is not a
   `Component` (F2).
4. Setters: delete `set_power_switch`, `set_mode_select`, `set_mist_level_number`;
   add, as the first setter after the plain sensor setters:
   ```cpp
   void set_fan(HumidifierFan *f) {
     f->set_parent(this);
     fan_ = f;
   }
   ```
5. Members: delete `power_switch_`, `mode_select_`, `mist_level_number_`; add
   `HumidifierFan *fan_{nullptr};` as the first entity pointer after the binary
   sensors (matching the purifier's `PurifierFan *fan_{nullptr};` placement).
6. Leave `send_*` declarations, the private UART/parse helpers, `last_mode_` and
   `power_on_` untouched.

#### Step 3 — `/Users/luke/esphome-levoit/components/humidifier_oasismist1000s/humidifier_oasismist1000s.cpp`

1. Delete `PowerSwitch::write_state`, `ModeSelect::control` and
   `MistLevelNumber::control` (current lines 16, 20, 24). Keep
   `DisplaySwitch::write_state` and `TargetHumidityNumber::control` unchanged.
2. Add the fan implementation in the "Entity Implementations" section, **above**
   `DisplaySwitch::write_state` (mirrors the purifier, where the fan impl is in
   that same section).

   *(C) — expected:*
   ```cpp
   fan::FanTraits HumidifierFan::get_traits() {
     fan::FanTraits traits;
     traits.set_speed(true);
     // ESPHome fan speeds are 1..count, which maps 1:1 onto mist levels 1..9.
     traits.set_supported_speed_count(MIST_LEVEL_MAX);
     traits.set_direction(false);
     traits.set_oscillation(false);
     // Preset modes live on the entity since ESPHome 2026.4.0; wire them in here.
     this->wire_preset_modes_(traits);
     return traits;
   }

   void HumidifierFan::setup() { this->set_supported_preset_modes({MODE_AUTO, MODE_MANUAL, MODE_SLEEP}); }
   ```
   *(A/B) — only if Step 0 says so:* drop `HumidifierFan::setup()` entirely (and
   its declaration, and the `fan_->setup()` call in step 7 below) and put
   `traits.set_supported_preset_modes({MODE_AUTO, MODE_MANUAL, MODE_SLEEP});` in
   `get_traits()` instead of `wire_preset_modes_`.

3. `control()` — shared skeleton, with the preset branch depending on Step 0:

   ```cpp
   void HumidifierFan::control(const fan::FanCall &call) {
     if (call.get_state().has_value()) {
       parent_->send_power(*call.get_state());
     }

     // <<< preset branch, variant-specific — see below >>>

     if (call.get_speed().has_value()) {
       const int speed = *call.get_speed();
       if (speed >= MIST_LEVEL_MIN && speed <= MIST_LEVEL_MAX) {
         // Switches the device to Manual and powers it on if needed.
         parent_->send_mist_level(static_cast<uint8_t>(speed), true);
       }
     }
   }
   ```

   *(B/C) modern* preset branch (expected):
   ```cpp
     if (call.has_preset_mode()) {
       // send_mode() powers the device on first if it is off.
       parent_->send_mode(string_to_mode(call.get_preset_mode()));
     }
   ```
   If Step 0.4 showed `FanCall::get_preset_mode()` returns `StringRef`, write
   `string_to_mode(call.get_preset_mode().str())` instead.

   *(A) legacy* preset branch:
   ```cpp
     const std::string &preset = call.get_preset_mode();
     if (!preset.empty()) {
       parent_->send_mode(string_to_mode(preset));
     }
   ```

4. `publish_mode()` — the only other variant-specific code:

   *(B/C) modern:*
   ```cpp
   void HumidifierFan::publish_mode(Mode mode) {
     this->set_preset_mode_(mode_to_string(mode));
     this->publish_state();
   }
   ```
   *(A) legacy:*
   ```cpp
   void HumidifierFan::publish_mode(Mode mode) {
     this->preset_mode = mode_to_string(mode);
     this->publish_state();
   }
   ```

5. `dump_config()`: delete the `LOG_SWITCH("  ", "Power", ...)`,
   `LOG_SELECT(...)` and `LOG_NUMBER("  ", "Mist Level", ...)` lines; add, in the
   purifier's spelling, right after the binary-sensor lines:
   ```cpp
     if (fan_ != nullptr) {
       ESP_LOGCONFIG(TAG, "  Fan: %s", fan_->get_name().c_str());
     }
   ```
6. `handle_status_tlv_()` — rewrite exactly these three cases, leaving all others
   (RESERVOIR, WATER, DISPLAY, MISTING, TARGET_HUMIDITY, CURRENT_HUMIDITY,
   `default`) untouched:

   ```cpp
     case TLV::POWER:
       power_on_ = (v == VALUE_ON);
       if (fan_ != nullptr) {
         fan_->state = power_on_;
         fan_->publish_state();
       }
       if (!power_on_) invalidate_diagnostic_sensors_();
       break;
   ```
   ```cpp
     case TLV::MODE:
       last_mode_ = static_cast<Mode>(v);
       if (fan_ != nullptr) fan_->publish_mode(last_mode_);
       break;
   ```
   ```cpp
     case TLV::MIST_LEVEL:
       if (fan_ != nullptr && v >= MIST_LEVEL_MIN && v <= MIST_LEVEL_MAX) {
         fan_->speed = v;
         fan_->publish_state();
       }
       break;
   ```
   (`invalidate_diagnostic_sensors_()` must stay on the power-off path.)
7. `Humidifier::setup()` — add the fan hook, mirroring `AirPurifier::setup()`
   lines 78-81, **after** `invalidate_diagnostic_sensors_()`:
   ```cpp
     // Registers the fan's preset modes; nothing is sent to the MCU here.
     if (fan_ != nullptr) {
       fan_->setup();
     }
   ```
   Nothing else in `setup()` changes. *(Variant A/B: skip this entirely.)*
8. Do **not** touch the UART functions or any `send_*` body.
9. `types.h` needs **no changes at all**. Confirm `MIST_LEVEL_MIN/MAX`,
   `MODE_AUTO/MANUAL/SLEEP`, `mode_to_string`, `string_to_mode` are all already
   there (they are).

#### Step 4 — `/Users/luke/esphome-levoit/humidifier_oasismist1000s.yaml`

Replace the component block (currently lines 85-105) with:

```yaml
humidifier_oasismist1000s:
  id: humidifier
  uart_id: uart_bus
  fan:
    id: humidifier_fan
    name: "Humidifier"
  target_humidity:
    name: "Target Humidity"
  humidity:
    name: "Humidity"
  display:
    name: "Display"
  reservoir:
    name: "Reservoir"
  water:
    name: "Water Available"
  misting:
    name: "Misting"
```

Rules:
- Nothing else in the file changes. In particular **do not** add
  `fan.turn_on` to `on_boot` (D5) and do not touch the `update_wifi_led` script,
  `uart:`, `wifi:`, `api:` or the substitutions.
- Surviving entity `name:` values stay verbatim (`Target Humidity`, `Humidity`,
  `Display`, `Reservoir`, `Water Available`, `Misting`).
- The new fan is named `"Humidifier"` → `fan.humidifier` in HA, mirroring the
  purifier's `"Air Purifier"` → `fan.air_purifier`.
- `id: humidifier_fan` is new and only needed if someone wants to script it; keep
  it for parity with `air_purifier_fan`.

#### Step 5 — `/Users/luke/esphome-levoit/tests/humidifier_oasismist1000s.yaml`

**No edit.** It is an `!include` wrapper and picks the new entity block up
automatically. Its "expectations" are verified in §7 (the `esphome config` output
must contain a `fan:` platform entry for `humidifier_oasismist1000s` and must not
contain `select` or the removed switch/number).

#### Step 6 — `/Users/luke/esphome-levoit/README.md`

1. Replace the humidifier table rows (lines 74-84) so it reads:

   | Key | Entity type | Notes |
   |---|---|---|
   | `fan` | fan | power on/off, speed 1-9 (mist level), presets Auto / Manual / Sleep |
   | `target_humidity` | number | 40-80 %, only meaningful in Auto / Sleep |
   | `humidity` | sensor | current RH % |
   | `display` | switch | |
   | `reservoir` | binary sensor | tank attached |
   | `water` | binary sensor | water present |
   | `misting` | binary sensor | actively misting |

2. Add immediately below it:

   ```markdown
   #### Why a `fan` entity and not a Home Assistant `humidifier`?

   ESPHome has no `humidifier` platform, so an ESPHome device cannot publish
   entities in Home Assistant's `humidifier` domain — only the domains ESPHome
   implements (`fan`, `climate`, `switch`, `number`, `sensor`, ...). `climate`
   was rejected: it is modelled as a thermostat and would need temperature
   traits this device does not have. A `fan` maps cleanly onto what the
   humidifier actually is — on/off, nine mist levels and three modes — and gives
   one card in Home Assistant instead of three unrelated entities. If you want a
   `humidifier` card, wrap the fan, the target-humidity number and the humidity
   sensor in a Home Assistant template humidifier.

   #### Breaking changes

   The `power` (switch), `mode` (select) and `mist_level` (number) keys were
   removed in favour of the single `fan` entity:

   | Removed | Replacement |
   |---|---|
   | `switch.<device>_power` | `fan.<device>` on/off |
   | `select.<device>_mode` | `fan.<device>` preset mode |
   | `number.<device>_mist_level` | `fan.<device>` percentage / speed (1-9) |

   Automations and dashboards referencing the old entity IDs must be updated. If
   you specifically want a 1-9 slider back, add a template number that calls the
   component directly:

   ```yaml
   number:
     - platform: template
       name: "Mist Level"
       min_value: 1
       max_value: 9
       step: 1
       optimistic: true
       set_action:
         - lambda: id(humidifier).send_mist_level((uint8_t) x, true);
   ```
   ```

3. Leave the air-purifier table, the protocol/TODO section and the
   "why the duplication" section alone.

### Phase 2 — Packet-array formatting (both components)

Apply to all **seven** `uint8_t packet[] = {...}` initializers. Wrap each one in
`// clang-format off` … `// clang-format on` (markers on their own lines,
indented with the code) and use the layouts below verbatim. **Element order and
values must be identical to today's** — check against §6 before committing.

`/Users/luke/esphome-levoit/components/humidifier_oasismist1000s/humidifier_oasismist1000s.cpp`

`send_ping_()` (10 bytes):
```cpp
  // clang-format off
  uint8_t packet[] = {
      PACKET_HEADER, static_cast<uint8_t>(PacketType::PING),  // header, packet type
      seq_++, static_cast<uint8_t>(PayloadLen::PING),         // sequence, payload length
      0x00, 0x00,                                             // reserved, checksum placeholder
      ADDR_STATUS[0], ADDR_STATUS[1], ADDR_STATUS[2], ADDR_STATUS[3],
  };
  // clang-format on
```

`send_command_()` (13 bytes):
```cpp
  // clang-format off
  uint8_t packet[] = {
      PACKET_HEADER, static_cast<uint8_t>(PacketType::STATUS),  // header, packet type
      seq_++, static_cast<uint8_t>(PayloadLen::COMMAND),        // sequence, payload length
      0x00, 0x00,                                               // reserved, checksum placeholder
      addr[0], addr[1], addr[2], addr[3],                       // target address
      0x01, 0x01, value,                                        // TLV: type, length, value
  };
  // clang-format on
```

`send_wifi_status()` (24 bytes):
```cpp
  // clang-format off
  uint8_t packet[] = {
      PACKET_HEADER, static_cast<uint8_t>(PacketType::STATUS),  // header, packet type
      seq_++, static_cast<uint8_t>(PayloadLen::WIFI_LED),       // sequence, payload length
      0x00, 0x00,                                               // reserved, checksum placeholder
      ADDR_WIFI_LED[0], ADDR_WIFI_LED[1], ADDR_WIFI_LED[2], ADDR_WIFI_LED[3],
      static_cast<uint8_t>(WifiLedTLV::STATUS), 0x01, static_cast<uint8_t>(status),
      static_cast<uint8_t>(WifiLedTLV::BLINK_ON), 0x02, blink_lo, blink_hi,
      static_cast<uint8_t>(WifiLedTLV::BLINK_OFF), 0x02, blink_lo, blink_hi,
      static_cast<uint8_t>(WifiLedTLV::RESET_FLAG), 0x01, 0x00,
  };
  // clang-format on
```

`/Users/luke/esphome-levoit/components/air_purifier_vital200s/air_purifier_vital200s.cpp`

- `send_ping_()` — identical to the humidifier's ping block above.
- `send_command_()` — identical to the humidifier's command block above.
- `send_wifi_status()` — identical to the humidifier's Wi-Fi block above.
- `send_timer_cancel_()` (16 bytes):
```cpp
  // clang-format off
  uint8_t packet[] = {
      PACKET_HEADER, static_cast<uint8_t>(PacketType::STATUS),   // header, packet type
      seq_++, static_cast<uint8_t>(PayloadLen::TIMER_CANCEL),    // sequence, payload length
      0x00, 0x00,                                                // reserved, checksum placeholder
      ADDR_TIMER_SET[0], ADDR_TIMER_SET[1], ADDR_TIMER_SET[2], ADDR_TIMER_SET[3],
      0x01, 0x04, 0x00, 0x00, 0x00, 0x00,                        // TLV: type, length, 4-byte value
  };
  // clang-format on
```

Notes:
- Byte 4 is not named in either `Offset` enum and is never read by the RX parser;
  `// reserved` is the honest label — do not invent a meaning for it.
- Keep the blank line and `packet[static_cast<size_t>(Offset::CHECKSUM)] = ...`
  lines exactly as they are today.
- Trailing commas inside the initializer are intentional and legal.
- Re-run `clang-format --dry-run --Werror` afterwards: regions between the markers
  are skipped, so it must stay silent.

### Phase 3 — Migrate the air purifier's fan preset API (**fixes #1**)

Skip only if Step 0 reported Variant A. Otherwise this phase is required: part 3A
closes issue #1, part 3B is what makes the file compile at all (F1).

File: `/Users/luke/esphome-levoit/components/air_purifier_vital200s/air_purifier_vital200s.{h,cpp}`.
**Entity-layer only — no UART byte changes, so §6 is unaffected.**

#### 3A — preset-mode vector moves onto the entity (issue #1, Variant C only)

1. `PurifierFan::get_traits()` (cpp lines 22-30): delete
   `traits.set_supported_preset_modes({MODE_AUTO, MODE_SLEEP, MODE_MANUAL});`
   and replace it, in the same position, with
   ```cpp
     // Preset modes live on the entity since ESPHome 2026.4.0; wire them in here.
     this->wire_preset_modes_(traits);
   ```
2. `PurifierFan::setup()` (cpp lines 32-49): insert as the **first** statement,
   before `this->restore_state_();`
   ```cpp
     this->set_supported_preset_modes({MODE_AUTO, MODE_SLEEP, MODE_MANUAL});
   ```
   It must come first: `FanRestoreState` resolves the stored preset **index**
   against this list.
3. **Preserve the order `{MODE_AUTO, MODE_SLEEP, MODE_MANUAL}` verbatim** — it is
   the purifier's existing order and it defines the restore index mapping (D3).
   Do not "align" it with the humidifier's order.
4. No `.h` change is needed for 3A (`setup()` and `get_traits()` are already
   declared).

#### 3B — `preset_mode` member / getter migration (Variant B or C)

5. `PurifierFan::setup()` line 37: `this->preset_mode.c_str()` →
   `this->get_preset_mode().c_str()`; line 39
   `if (!this->preset_mode.empty())` → `if (this->has_preset_mode())`; line 40
   `string_to_mode(this->preset_mode)` →
   `string_to_mode(this->get_preset_mode().str())`.
   *(Leave the discarded-`restore_state_()` bug alone — out of scope, see Q1.)*
6. `PurifierFan::control()` lines 56-59 →
   ```cpp
     if (call.has_preset_mode()) {
       parent_->send_mode(string_to_mode(call.get_preset_mode()));
     }
   ```
   (add `.str()` if Step 0.4 showed `FanCall::get_preset_mode()` returns
   `StringRef`).
7. Add a public `void publish_mode(Mode mode);` to `class PurifierFan` in the
   header and implement it exactly like the humidifier's (Step 3.4), then replace
   the `TLV::MODE` body (cpp lines 406-413) with
   ```cpp
     case TLV::MODE:
       last_mode_ = static_cast<Mode>(v);
       if (fan_ != nullptr) fan_->publish_mode(last_mode_);
       ESP_LOGD(TAG, "Mode: %s", mode_to_string(last_mode_));
       break;
   ```
   This keeps the two components spelled identically (PLAN.md D1/D4).
8. Nothing else in the air purifier changes. Confirm with
   `git diff -- components/air_purifier_vital200s/` that no `send_*` body and no
   `types.h` line is touched.

#### 3C — verify the deprecation is actually gone

9. `esphome compile tests/air_purifier_vital200s.yaml` must build with **no**
   `set_supported_preset_modes ... is deprecated` warning
   (`... | grep -i "deprecat"` should return nothing for the fan traits).
10. Runtime/HA check for the maintainer: the purifier's fan card still lists
    Auto / Sleep / Manual. If it lists none, `wire_preset_modes_(traits)` was
    omitted from `get_traits()` (F4, R4).

### Phase 4 — Commits on the existing branch

Stay on `polish-components-and-ci` and **add** commits (do not `git commit --amend`,
do not rebase, do not force-push over existing history). Do not stage `PLAN.md`,
`PLAN-humidifier-fan.md` or `.claude/` unless the maintainer asks.

| # | Subject | Files |
|---|---|---|
| 1 | `feat(humidifier_oasismist1000s)!: expose a single fan entity for power, mist level and mode` | `components/humidifier_oasismist1000s/{__init__.py,humidifier_oasismist1000s.h,humidifier_oasismist1000s.cpp}`, `humidifier_oasismist1000s.yaml`, `README.md` |
| 2 | `style: hand-format TX packet byte arrays` | both `components/**/*.cpp` |
| 3 | `fix(air_purifier_vital200s): migrate fan preset modes to the entity API` | `components/air_purifier_vital200s/air_purifier_vital200s.{h,cpp}` |

Commit 3's body must explain that ESPHome 2026.4.0 moved preset-mode vectors from
`FanTraits` to `fan::Fan` (deprecated setter removed in 2026.11.0), that presets
are now registered in `setup()` and wired into the traits with
`wire_preset_modes_()`, and must contain the line:

```
Fixes #1
```

Every commit message ends with:

```
Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_012RUYiRNZCXJk84M5MsBbKL
```

PR body must call out:
- **Closes #1** (fan preset API migration; the new `HumidifierFan` is written
  against the entity-level API from the start, the existing `PurifierFan` is
  migrated in commit 3).
- The breaking change + migration table (D10).
- The fan API variant found in Step 0 and the ESPHome version it was verified
  against.
- The restore-on-boot decision (D5) with Q1.
- That no hardware test was possible, but every TX byte was verified unchanged
  (§6) and the preset migration is entity-layer only.

It ends with:

```
🤖 Generated with [Claude Code](https://claude.com/claude-code)

https://claude.ai/code/session_012RUYiRNZCXJk84M5MsBbKL
```

---

## 6. Byte-freeze verification (do this before Phase 4)

The protocol cannot be tested here. Guard it by diffing bytes, not behaviour.
Note that Phase 3 is entity-layer only and cannot affect any of these checks.

1. **`send_*` bodies are untouched.** `git diff -- components/humidifier_oasismist1000s/humidifier_oasismist1000s.cpp`
   must show **no** hunk inside `send_power`, `send_display`, `send_mode`,
   `send_mist_level`, `send_target_humidity` or `calc_checksum_`. The new fan code
   only *calls* them.
2. **Packet arrays: element-by-element check.** For each of the seven arrays,
   compare the new initializer against `git show HEAD:<file>` and confirm the
   ordered element list is identical. Expected element counts:

   | Function | Elements | `PayloadLen` | `6 + payload_len` |
   |---|---|---|---|
   | humidifier `send_ping_` | 10 | `PING = 4` | 10 ✓ |
   | humidifier `send_command_` | 13 | `COMMAND = 7` | 13 ✓ |
   | humidifier `send_wifi_status` | 24 | `WIFI_LED = 18` | 24 ✓ |
   | purifier `send_ping_` | 10 | `PING = 4` | 10 ✓ |
   | purifier `send_command_` | 13 | `COMMAND = 7` | 13 ✓ |
   | purifier `send_wifi_status` | 24 | `WIFI_LED = 18` | 24 ✓ |
   | purifier `send_timer_cancel_` | 16 | `TIMER_CANCEL = 10` | 16 ✓ |

3. **Position of `seq_++` is index 2 in every array** (it is the only
   side-effecting element; braced-init-list evaluation is left-to-right, so its
   position must not move).
4. **Checksum assignment untouched:**
   `packet[static_cast<size_t>(Offset::CHECKSUM)] = calc_checksum_(packet, sizeof(packet));`
   stays immediately after each initializer.
5. **No `types.h` changes** in either component (`git diff --stat` must not list
   either `types.h`).
6. **Mode mapping unchanged:** humidifier `Mode { AUTO=0, MANUAL=1, SLEEP=2 }` and
   its `mode_to_string`/`string_to_mode` are untouched; the preset list
   `{MODE_AUTO, MODE_MANUAL, MODE_SLEEP}` uses those same constants, so the strings
   HA sees (`Auto`, `Manual`, `Sleep`) are identical to the old select's options.
   The purifier's preset list keeps its own order `{MODE_AUTO, MODE_SLEEP, MODE_MANUAL}`.
7. **Speed mapping:** `send_mist_level` still clamps to `[MIST_LEVEL_MIN,
   MIST_LEVEL_MAX]` and the fan passes 1-9 through unscaled.

---

## 7. Verification

```bash
cd /Users/luke/esphome-levoit
cp secrets.yaml.example secrets.yaml
cp secrets.yaml.example tests/secrets.yaml

esphome config tests/humidifier_oasismist1000s.yaml
esphome config tests/air_purifier_vital200s.yaml
esphome compile tests/humidifier_oasismist1000s.yaml
esphome compile tests/air_purifier_vital200s.yaml

find components \( -name '*.cpp' -o -name '*.h' \) -print0 \
  | xargs -0 clang-format --dry-run --Werror     # must be silent
```

Check in the humidifier `esphome config` output:
- a `fan:` entity with `name: Humidifier` and `id: humidifier_fan`;
- **no** `select:` entity, **no** `Power` switch, **no** `Mist Level` number;
- `Target Humidity`, `Humidity`, `Display`, `Reservoir`, `Water Available`,
  `Misting` all still present with unchanged names;
- `external_components.source` resolved to the local `../components` path.

Check in both `esphome compile` logs: no `set_supported_preset_modes` deprecation
warning (Phase 3C).

Runtime smoke test (maintainer, on hardware):
- HA fan card shows the device's real power state within ~1 s of boot without any
  command being sent (D5: nothing is pushed at boot).
- The card lists exactly three presets (Auto / Manual / Sleep) — if it lists none,
  `wire_preset_modes_()` is missing (R4).
- Preset Auto/Manual/Sleep switches the device and the card follows.
- Dragging the speed slider sets mist level 1-9 and flips the device to Manual.
- Turning the fan on from HA while the device is off powers it on.
- The air purifier's card still lists Auto / Sleep / Manual after Phase 3.
- No `Checksum mismatch` spam in either log.

---

## 8. Risks and open questions

### Risks

- **R1 — Fan API drift (highest).** Handled by Step 0 + Phase 3. If Step 0 is
  skipped, the build fails or (worse) `std::string` is constructed from a null
  `const char *` at runtime. Do not skip it.
- **R2 — Breaking change for existing users.** `switch.*_power`,
  `select.*_mode`, `number.*_mist_level` disappear; automations break silently
  (HA logs "entity not found"). Mitigated by the README breaking-change section
  and the PR body (D10). Accepted deliberately per D1.
- **R3 — No hardware test.** All new logic is entity plumbing on top of unchanged
  `send_*` methods; §6 freezes the bytes. The only new *behaviour* is that
  nothing is transmitted at boot (D5), which is strictly fewer commands than
  today.
- **R4 — Forgetting `wire_preset_modes_(traits)`.** The single most likely defect
  in Phase 1/3: `set_supported_preset_modes()` alone stores the vector on the
  entity, but `FanTraits::preset_modes_` stays null unless `get_traits()` wires
  it, so Home Assistant would show a fan with **no** presets and no compile error.
  Verified by the runtime check in §7.
- **R5 — Auto-100% on plain turn-on.** `FanCall::validate_()` forces speed to max
  when a fan with no active preset and speed 0 is turned on. Reachable only in
  the <250 ms window after boot before the first status packet arrives; after
  that both preset and speed are always populated from RX. Not worth guarding.
- **R6 — Publish frequency.** Up to three `publish_state()` per 250 ms poll, each
  of which also runs `save_state_()`. This matches the purifier and today's
  number/select behaviour, so it is not a regression — see Q2.
- **R7 — clang-format markers.** Unbalanced `off`/`on` silently disables
  formatting for the rest of the file. Verify each `off` has a matching `on`
  (7 pairs total across the two `.cpp` files).
- **R8 — Preset list order is restore-index state.** Reordering either
  component's preset list changes what a previously saved `FanRestoreState`
  index maps to. Both lists are frozen as-is (D3).

### Open questions (flag to the maintainer, do not block)

- **Q1 — Restore on boot.** `PurifierFan::setup()` discards the result of
  `restore_state_()` (F2), so it always sends `send_power(false)` at boot, which
  is why the purifier sample needs `fan.turn_on` in `on_boot`. The humidifier
  deliberately does not restore at all (D5). Does the maintainer want (a) the
  humidifier left as designed here, (b) the purifier's restore fixed and
  `restore_mode: RESTORE_DEFAULT_OFF` in its sample, or (c) both components
  switched to "MCU is authoritative, never push at boot"? Option (c) would also
  let `- fan.turn_on: air_purifier_fan` be deleted from the purifier sample.
- **Q2 — Publish coalescing.** All three fan-relevant TLVs arrive in one status
  packet; publishing once per packet instead of up to three times would cut HA
  API traffic and preference writes by ~3x on both components. Out of scope here.
- **Q3 — ESPHome version floor.** Once the fan API variant is known, consider
  `esphome: min_version:` in the samples so users on older ESPHome get a clear
  error instead of a compile failure (PLAN.md Q3 covers the same ground). The
  entity-level preset API requires ≥ 2026.4.0.
- **Q4 — `CHANGELOG.md`.** The breaking change currently lives in the README and
  the PR body. Add a real changelog if the repo starts tagging releases.
