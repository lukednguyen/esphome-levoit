# Implementation Plan: Polish + Package `esphome-levoit` for ESPHome Builder

> Status: ready to implement. Every judgement call is resolved in the **Decisions**
> section — do not re-derive them. Follow the **Checklist** in order.

> **Baseline (re-verified 2026-09-09).** The working tree is **clean at commit
> `b9b97cf`** (= `master`), on branch `polish-components-and-ci`. Only `PLAN.md`
> and `.claude/` are untracked. **All prior direct code edits have been reverted.**
> Every line number and line count below was re-checked against these committed
> files. Nothing in this plan has been applied yet — treat every step as fresh.

Two findings materially changed the design, both verified against ESPHome `dev`
source rather than assumed:

1. **A shared helper component is provably impossible** for this repo's use case.
   `esphome/loader.py`'s `ComponentMetaFinder.find_spec()` contains
   `if (self._allowed_components is not None and component not in self._allowed_components): return None`,
   and `external_components` passes the user's `components:` list as
   `allowed_components`. So a `DEPENDENCIES = ["levoit_uart_common"]` would fail
   with "Component not found" for anyone writing `components: [air_purifier_vital200s]`.
   Duplication is not a preference here, it's forced.
2. **CI can build the *real* sample files** rather than copies.
   `cv.validate_source_shorthand` tries `{type: local, path: value}` first
   (validated with `cv.directory`) before the `github://` regex, so one `source:`
   field accepts either form; local paths resolve via `CORE.relative_config_path`
   against the **main** config dir; and main-config substitutions override package
   substitutions. That enables a 5-line `tests/` wrapper that `!include`s the
   sample and flips one substitution.

---

## 1. Task & Definition of Done

### Task
1. Beautify / tidy the two ESPHome external components
   (`components/air_purifier_vital200s/`, `components/humidifier_oasismist1000s/`,
   C++ and `__init__.py`).
2. Produce clean, first-class sample device YAMLs for **both** devices
   (ESP32-C3 + `esp-idf`). **Both currently need a full rewrite.**
3. Add a GitHub Actions workflow that compiles the sample configs on push/PR so
   they stay known-good.
4. Add a git-ignored / stub secrets mechanism so CI can compile.
5. Ship a repo consumable purely from ESPHome Builder / the Home Assistant add-on
   with **zero** local custom code.
6. Commit the work in reviewable chunks and open a PR against `master`.

### Definition of Done
A user with **no local checkout and no local files** can:

1. Open ESPHome Builder (or the HA ESPHome add-on), create a new config, paste
   the contents of `air_purifier_vital200s.yaml` (or
   `humidifier_oasismist1000s.yaml`).
2. Edit only: `substitutions` (name / friendly name / board / pins) and their
   `secrets.yaml` entries (`wifi_ssid`, `wifi_password`, `api_key_air_purifier` /
   `api_key_humidifier`).
3. Hit **Install** and get a working firmware.

Concretely, all of the following must be true:

- [ ] `external_components: - source: github://lukednguyen/esphome-levoit` +
  `components: [<one>]` is the only thing needed to pull in code. **No
  shared/helper component is required to be listed** (see Decision D1 — this is a
  hard constraint, not a preference).
- [ ] Both root sample YAMLs pass `esphome config` **and** `esphome compile` for
  `esp32-c3` + `esp-idf`.
- [ ] CI compiles both sample configs against the working tree (not the published
  repo) on every push and PR.
- [ ] `secrets.yaml` is git-ignored; `secrets.yaml.example` is committed and is
  what CI uses.
- [ ] No behavioural change to the UART protocol: every transmitted byte sequence
  is identical to today's. This code is known-working on real hardware and
  **cannot be hardware-tested here**. (One deliberate, clearly-flagged exception
  on the *receive* side — see Phase 1 step 5f and Risk R8.)
- [ ] No user-visible entity `name:` changes (would break existing Home Assistant
  entity IDs).
- [ ] `clang-format --dry-run --Werror` is clean over `components/`.
- [ ] Work is committed in logical chunks and a PR is open against `master`.

---

## 2. Current State Inventory

Repo root at `b9b97cf`: `README.md` (33 lines), `.gitignore` (4 lines),
`air_purifier_vital200s.yaml` (81 lines), `humidifier_oasismist1000s.yaml`
(85 lines), `components/`.
**There is no `.clang-format`, no `.github/`, no `tests/`, no
`secrets.yaml.example`.** Default branch is `master`; work happens on
`polish-components-and-ci`.

| File | State at `b9b97cf` | Needs |
|---|---|---|
| `components/air_purifier_vital200s/types.h` | **202 lines.** Good structure. `MODE_*` already `const char *`, free `mode_to_string`/`string_to_mode` present, `Offset : size_t` already. Includes only `<cstdint> <cstddef> <array>`. | `#include <string>` (uses `std::string` in `string_to_mode` without it — compiles only by include-order luck); drop `constexpr` on `string_to_mode`; add `RX_MIN_HEADER_LEN`; remove the 3 commented `FILTER_RESET` fragments → replace with a "protocol notes" block. |
| `components/air_purifier_vital200s/air_purifier_vital200s.h` | **136 lines.** Clean. No `<vector>` include despite `std::vector<uint8_t> rx_buffer_` (line 129). | Remove commented `button.h` include, `FilterResetButton` class, setter, `send_filter_reset()` decl, member. Add `#include <vector>`. |
| `components/air_purifier_vital200s/air_purifier_vital200s.cpp` | **495 lines.** **RX checksum validation is NOT present** (unlike the humidifier). Closing namespace comment on line 494 reads `// namespace air_purifier` (wrong). `TAG` is `static const char *const` (this is the canonical spelling). | Add RX checksum validation to match the humidifier; `send_ping_()` uses raw `0x02,0x00,0x55,0x00` → use `ADDR_STATUS[...]`; `parse_packet_(data, rx_buffer_.size())` → `expected_size`; magic `6` → `RX_MIN_HEADER_LEN`; delete commented filter-reset blocks (28-31, 268-288) and `// LOG_BUTTON` (115); wrap the 127-column `send_display` line; C-cast `(unsigned long)` → `static_cast`; fix `value[3] << 24` signed-overflow UB; fix namespace comment. |
| `components/air_purifier_vital200s/__init__.py` | **121 lines.** Modern schema. | Remove commented `FilterResetButton` / `CONF_FILTER_RESET` blocks (lines **25-26, 36-37, 75-80, 118-121** — all four verified correct). |
| `components/humidifier_oasismist1000s/types.h` | **129 lines.** `MODE_*` as `char[]`, no conversion fns. `Offset : uint8_t`. `WifiTLV` / `WifiStatus` naming. `WIFI_BLINK_MS` sits near the top. No `RX_MIN_PACKET_LEN`. **Includes only `<cstdint> <cstddef>` but uses `std::array` on line 53** — same include-order luck bug. Stray double blank line at 126-127. | Mirror the air-purifier layout: `Offset : size_t`, `#include <array>` **and** `<string>`, `MODE_*` as `const char *` + free `mode_to_string`/`string_to_mode`, rename Wi-Fi symbols, add `RX_MIN_PACKET_LEN`/`RX_MIN_HEADER_LEN`, move `WIFI_BLINK_MS` to the bottom, drop stray blank lines. |
| `components/humidifier_oasismist1000s/humidifier_oasismist1000s.h` | **135 lines.** Checksum fn is named `calculate_checksum_` (line 98) vs the purifier's `calc_checksum_`. `match_address_` sits in its own section (100-101). Closing namespace comment line 134 reads `// namespace levoit_humidifier` (wrong). Trailing whitespace on 86, 114, 125. No `<vector>` include. | Remove `// Helpers` + `static mode_to_string/string_to_mode` decls (lines **89-91**) and `handle_wifi_tlv_` decl (line **110**). Rename `calculate_checksum_` → `calc_checksum_`. Regroup `match_address_` under `// UART`. Fix namespace comment. Add `#include <vector>`. |
| `components/humidifier_oasismist1000s/humidifier_oasismist1000s.cpp` | **423 lines.** `handle_wifi_tlv_` (lines **377-414**) is **dead** — its only call site is the commented body of the `else if` branch at **291-296**. `// Mode Helpers` section at **262-278**. `TAG` is `static const char *` (missing the second `const`). Unused `#include <cstring>` (line 6). Broken 4-space indent on the three packet literals (130-138, 144-153, 242-254). Whitespace-only line at 258. Bare `if (ptr)` truth-tests throughout. Closing namespace comment line 422 reads `// namespace levoit_humidifier` (wrong). | Delete dead Wi-Fi RX path; move mode helpers to `types.h`; rename Wi-Fi symbols and `calc_checksum_`; `static_cast<uint8_t>(Offset::…)` → `size_t`; make `send_wifi_status` structurally match the air purifier's; fix TAG, includes, namespace comment, null checks. |
| `components/humidifier_oasismist1000s/__init__.py` | **144 lines.** Hardcoded `options=["Auto","Manual","Sleep"]` (135), `min_value=40,max_value=80` (139), `min_value=1,max_value=9` (143). | Hoist to named module constants with a comment pointing at `types.h`. |
| `air_purifier_vital200s.yaml` | **81 lines. Messy original state** — `script:` block first, no `substitutions`, no `esp32: board:`, hardcoded pins, `source: github://lukednguyen/esphome-levoit` with no `refresh:`, no header comment. | **Full rewrite** (Phase 4). Entity `name:` values must be preserved verbatim. |
| `humidifier_oasismist1000s.yaml` | **85 lines. Messy original state** — same shape as above (`script:` first, no `substitutions`, no `board:`, no `refresh:`). Trailing whitespace on line 42. | **Full rewrite** mirroring the air purifier sample. Entity `name:` values must be preserved verbatim. |
| `.gitignore` | 4 lines, anchored patterns (`/.esphome/`, `/secrets.yaml`, `/components/**/__pycache__/`) — won't cover `tests/`. | Un-anchor + add build dirs. |
| `README.md` | 33 lines. Stale: tells users to copy `components/` locally, `source: components/`. | Rewrite around the ESPHome Builder / `github://` flow. |

### Verified external facts (do not re-investigate)

- **`allowed_components` blocks imports.** `esphome/loader.py`'s
  `ComponentMetaFinder.find_spec()` contains
  `if (self._allowed_components is not None and component not in self._allowed_components): return None`.
  `external_components` passes the user's `components:` list as
  `allowed_components`. ⇒ A shared helper component would be **unimportable**
  whenever a user writes `components: [air_purifier_vital200s]`.
- **Source shorthand.** `cv.validate_source_shorthand` first tries
  `{type: local, path: value}` (validated with `cv.directory`, i.e. the dir must
  exist) and only falls back to the `github://user/repo[@ref]` regex. ⇒ **One
  `source:` field can hold either form**, and a bare relative path such as
  `../components` is a valid local source.
- **Local path resolution.**
  `components_dir = Path(CORE.relative_config_path(conf[CONF_PATH]))` ⇒ relative
  to the **main config file's** directory, not the file that declared it. A
  wrapper in `tests/` therefore uses `../components`.
- **`refresh:`** has a default of `1d`, is accepted for all source types, and is
  a no-op for local.
- **Package substitution precedence.** ESPHome merges `packages` *before*
  substitutions, with the main config winning: "Substitutions in your main
  configuration will override substitutions with the same name in a package."
- **`!secret` lookup order.** `yaml_util.construct_secret` first tries
  `secrets.yaml` next to the **including file**, then falls back to the main
  config dir. ⇒ copying the stub to **both** the repo root and `tests/` is
  correct and covers either path (D6).
- **⚠️ CORRECTED — `esp32: board:` is *optional*, not required.** Current ESPHome
  `dev` has `cv.Optional(CONF_BOARD)` plus
  `cv.has_at_least_one_key(CONF_BOARD, CONF_VARIANT)`; when omitted the board is
  derived from `variant` (`STANDARD_BOARDS` for PlatformIO, a synthesised string
  for ESP-IDF). Both sample YAMLs already set `variant: esp32c3`, so **neither
  one is currently a hard validation failure.** Adding `board: ${board}` is done
  for *explicitness, user-configurability and older-ESPHome compatibility* — not
  to fix a crash. (The previous revision of this plan claimed the humidifier
  YAML "fails validation"; that claim is wrong and has been removed.)
- **⚠️ UPDATED — `esphome/build-action`**: latest release is **`v8.1.0`**
  (2026-09-01); `v8.0.0` (2026-07-17) was the breaking release that added
  "support for all ESPHome target platforms and a minimum ESPHome version".
  Releases are immutable and **no floating major tag (`v8`) is published** — pin
  the full tag. Requires **ESPHome ≥ 2026.7.0**. Inputs: `yaml-file` (required),
  `version` (default `latest`), `release-summary`, `release-url`,
  `complete-manifest`, `substitutions`, `platform` (**deprecated, "no longer
  used" — do not set**). Outputs: `name` (device name with platform variant
  appended), `version`, `original-name`, `project-name`, `project-version`.
- **⚠️ UPDATED — GitHub-maintained actions are on v7.** `actions/checkout@v7`
  (v7.0.1), `actions/setup-python@v7` (v7.0.0), `actions/upload-artifact@v7`
  (v7.0.1). Unlike `build-action`, these **do** publish floating major tags, so
  `@v7` is fine. (The previous revision pinned `@v4`/`@v5`.)
- **`clang-format` on PyPI** currently publishes up to `23.1.0`. The exact
  version does not matter; **local and CI must match exactly** (Risk R2).

---

## 3. Decisions

*(Unchanged from the previous revision. Do not re-litigate D1–D8. The only edit
is a set of **additional rows** appended to D4's unification table, discovered
during the re-verification pass — they are applications of D4's existing
principle, not new decisions.)*

### D1 — Shared code: **keep the duplication. Do NOT create a shared component.**
`read_uart_()`, `send_command_()`, `match_address_()`, `calc_checksum_()` and the
`parse_tlvs_()` template are near-identical (~60 lines) across the two components.
Tempting to factor out. **Rejected**, because:

- A shared ESPHome component (`components/levoit_uart_common/`) declared via
  `DEPENDENCIES`/`AUTO_LOAD` is **provably broken** for our headline use case:
  `find_spec` returns `None` for any component not in the user's `components:`
  list, so the import fails with "Component not found". It would only work if
  users omitted `components:` entirely — we cannot rely on that, and every sample
  and every third-party doc uses the explicit list. This directly violates the
  Definition of Done.
- A plain shared header outside a component directory (e.g.
  `components/shared/levoit.h`) is not copied into the build tree — ESPHome only
  copies the directories of components actually in use. The `#include` would not
  resolve.
- The only remaining option, a single `levoit` component with sub-platforms, is a
  redesign and is explicitly out of scope.

**Therefore:** the two components stay standalone and self-contained. Pay for it
with *consistency* instead: the duplicated functions must be byte-for-byte
identical in spelling, ordering, naming and comments so a reader/diff tool can
see they are the same. Record the rationale in the README so a future contributor
doesn't "helpfully" factor it out.

### D2 — Dead code: **delete it, but preserve the protocol knowledge as comments in `types.h`.**
- Air purifier `FilterResetButton` (commented out across all four files): delete
  every commented fragment. In `types.h`, add a compact
  `// Protocol notes (observed, not implemented)` block recording
  `ADDR_FILTER_RESET = {0x02, 0x05, 0x55, 0x00}`,
  `PayloadLen::FILTER_RESET = 6`, `FILTER_RESET_ACTION = 0x03`, and the reason
  (filter life is not readable back from the MCU).
- Humidifier `handle_wifi_tlv_()` (~38 lines, never called): delete the function,
  its declaration, and the now-empty
  `else if (match_address_(data, ADDR_WIFI_STATUS))` branch in `parse_packet_()`.
  Keep the `WifiLedTLV` / `WifiLedStatus` enums and the address — they are used by
  the TX path (`send_wifi_status`). Add a one-line comment in `types.h` noting the
  MCU echoes these TLVs back and that we deliberately ignore them.

Rationale: commented-out code rots and is the single loudest "unpolished" signal
in the repo. Git history is the archive; `types.h` comments keep the
reverse-engineering findings discoverable.

### D3 — Humidifier option lists in `__init__.py`: **keep them in Python, hoisted to named constants.**
Python cannot read C++ constants, and adding schema options for a fixed,
single-model device is API surface nobody needs. Define module-level constants
near the top of the humidifier `__init__.py` and use them in `to_code()`:

```python
# These must stay in sync with types.h (MODE_*, HUMIDITY_MIN/MAX, MIST_LEVEL_MIN/MAX).
MODE_OPTIONS = ["Auto", "Manual", "Sleep"]
HUMIDITY_MIN = 40
HUMIDITY_MAX = 80
MIST_LEVEL_MIN = 1
MIST_LEVEL_MAX = 9
```

Zero behaviour change, self-documenting, and the sync requirement is stated
explicitly.

### D4 — How far to unify the two `types.h`: **unify structure, naming and idioms; never values.**
Both files must end up with the same section order and the same idioms:

`includes` → `RX constants` → `packet structure (PACKET_HEADER, PacketType,
PayloadLen, Offset)` → `command addresses` → `TLV types` → `mode` →
`values / limits` → `Wi-Fi LED`.

Specific unifications (all rename/format-only; the compiler catches every miss):

| Concern | Target for both files |
|---|---|
| `Offset` underlying type | `enum class Offset : size_t` (humidifier is `uint8_t` today) |
| All `Offset` casts | `static_cast<size_t>(Offset::X)` (humidifier uses `static_cast<uint8_t>` in 9 places) |
| Mode strings | `inline constexpr const char *MODE_AUTO = "Auto";` (etc.) |
| Mode conversion | free functions in `types.h`: `inline constexpr const char *mode_to_string(Mode)` and `inline Mode string_to_mode(const std::string &)` — **`string_to_mode` is `inline`, not `constexpr`** (it calls non-constexpr `std::string::operator==`; `constexpr` there is ill-formed-NDR and merely tolerated today) |
| `<string>` | `#include <string>` in both `types.h` |
| Wi-Fi LED enums | `WifiLedTLV { STATUS, BLINK_ON, BLINK_OFF, RESET_FLAG }` and `WifiLedStatus { OFF = 0x00, SOLID = 0x01, BLINKING = 0x02 }` in both. Humidifier's `WifiTLV` → `WifiLedTLV`, `WifiStatus{DISCONNECTED,CONNECTED,CONNECTING}` → `WifiLedStatus{OFF,SOLID,BLINKING}` (**same numeric values**). |
| Wi-Fi address / payload | Humidifier `ADDR_WIFI_STATUS` → `ADDR_WIFI_LED`, `PayloadLen::WIFI_STATUS` → `PayloadLen::WIFI_LED`. (Both devices use the identical address `{0x02, 0x18, 0x50, 0x00}` — the shared name makes that visible.) |
| Ping packet address | Both build the ping from `ADDR_STATUS[0..3]`; no inline byte literals. (Humidifier already does; air purifier does not.) |
| Null checks | `if (ptr != nullptr)` everywhere (humidifier currently uses bare `if (ptr)`). |
| **`<array>` include** | **[added]** Humidifier `types.h` uses `std::array` (line 53) with no `#include <array>`. Add it. |
| **`<vector>` include** | **[added]** Both component `.h` files declare `std::vector<uint8_t> rx_buffer_` with no `#include <vector>`. Add it to both. |
| **`TAG` spelling** | **[added]** `static const char *const TAG = "<component>";` in both `.cpp` files. Humidifier is missing the second `const`. |
| **Checksum fn name** | **[added]** `calc_checksum_` in both. Humidifier's `calculate_checksum_` is renamed (5 call sites + 1 decl + 1 definition). |
| **RX length constants** | **[added]** Both `types.h` get `RX_MIN_HEADER_LEN = 6` and `RX_MIN_PACKET_LEN = 10`; the magic `6` in both `read_uart_()` and the humidifier's `Offset::TLV_START` min-length guard use them. |
| **Closing namespace comments** | **[added]** Must match the real namespace. Three are wrong today: `air_purifier_vital200s.cpp:494` (`// namespace air_purifier`), `humidifier_oasismist1000s.h:134` and `humidifier_oasismist1000s.cpp:422` (both `// namespace levoit_humidifier`). |
| **Unused includes** | **[added]** Humidifier `.cpp` line 6 `#include <cstring>` is unused — remove. |

**Explicitly NOT unified:** `Mode` enum values differ per device (purifier
`MANUAL=0, SLEEP=1, AUTO=2`; humidifier `AUTO=0, MANUAL=1, SLEEP=2`) — leave them
alone. Addresses, TLV IDs and `PayloadLen` values are device-specific — leave
them alone. The purifier's `PacketType` switch framing vs the humidifier's
length-only framing, and the purifier's `parse_tlvs_(…, start, …)` extra
parameter, stay as they are (D8).

### D5 — CI must test the *actual* sample configs: **`component_source` substitution + `tests/` package wrappers.**
Building a hand-copied duplicate of the sample under `tests/` would let the real
sample rot silently, defeating the point of the workflow. Instead:

- Each root sample gains one substitution:
  `component_source: github://lukednguyen/esphome-levoit`, used as
  `source: ${component_source}`.
- `tests/<component>.yaml` is a 5-line wrapper that overrides that substitution to
  `../components` and `!include`s the real sample as a package.

This is safe because of the verified facts in §2: the `source:` shorthand
validator accepts both a `github://` URL and an existing relative directory;
local paths resolve against the *main* config dir (`tests/`), so `../components`
is correct; and main-config substitutions beat package substitutions.

Cost: one extra substitution line in each sample. Accepted — it is
self-documenting, and the sample still works verbatim when pasted into ESPHome
Builder.

### D6 — Secrets: **committed `secrets.yaml.example`, git-ignored `secrets.yaml`, CI copies the example.**
Real `!secret` references stay in the samples (so CI also validates that the
secret *key names* are consistent). CI copies `secrets.yaml.example` →
`secrets.yaml` in **both** the repo root and `tests/`, which covers either
`!secret` lookup location without needing to know which one ESPHome picks. The
stub API key must be valid base64 decoding to exactly 32 bytes.

### D7 — CI shape: **`esphome/build-action` matrix + a separate clang-format job.**
`build-action` is the official, maintained path and is exactly designed for "does
this config still build". Pin the full tag (no floating major tag exists) — as of
this revision that is **`v8.1.0`**. Use `version: latest` for ESPHome so upstream
regressions surface early — this is a component library, that is desirable. Two
matrix entries run in parallel; ESP-IDF C3 builds are ~5-10 min uncached, which is
acceptable. `clang-format` is a separate job so a formatting nit does not mask a
compile failure (and vice versa).

### D8 — Runtime logic: **freeze it.**
No changes to `read_uart_()` control flow, buffer handling, checksum maths,
packet layouts, polling intervals, or entity `name:` values. The air purifier's
`PacketType` switch and the humidifier's length-only framing are both correct for
their respective MCUs; unifying them would be an untestable behaviour change. See
§6 for the known nits we are deliberately *not* fixing.

> **One documented relaxation.** Phase 1 step 5f adds RX checksum validation to
> the air purifier so it matches the humidifier, which already does exactly this
> on real hardware. This is the **only** behaviour-affecting change in the plan.
> It is flagged inline, has a one-line rollback, and is tracked as Risk R8.

---

## 4. Implementation Checklist

Work in this order. **Commit after each phase** so a bisect is possible; Phase 6
turns those commits into a PR.

### Phase 0 — Tooling baseline

1. **Create `.clang-format` at the repo root** (it does **not** exist today):

   ```yaml
   ---
   BasedOnStyle: Google
   IndentWidth: 2
   ColumnLimit: 120
   DerivePointerAlignment: false
   PointerAlignment: Right
   AllowShortIfStatementsOnASingleLine: true
   AllowShortLoopsOnASingleLine: true
   SortIncludes: false
   ```

   Notes for whoever writes this file:
   - `AllowShortIfStatementsOnASingleLine: true` is the legacy boolean spelling;
     modern clang-format maps it to `WithoutElse`. Keep the boolean — it is what
     ESPHome upstream uses and it is still accepted.
   - `SortIncludes: false` is deliberate: the components rely on
     `#include "types.h"` coming *after* the ESPHome headers.
   - Do **not** add any other keys. Any layout you dislike is handled in Phase 3.

2. Install the formatter and **record the exact version**:
   `pip install clang-format==20.1.7`. If a different version is already
   installed, run `clang-format --version` and pin *that* string in both this
   step and the workflow in Phase 5 — the two must match or CI will fail on
   harmless version drift. (PyPI currently publishes up to `23.1.0`; any version
   works as long as local == CI.)
3. Do not run clang-format yet; it runs in Phase 3, after the edits.

### Phase 1 — Air purifier component

*(All line numbers below are against the committed files at `b9b97cf` and have
been re-verified. Apply edits within a file bottom-up, or re-read the file after
each edit, so earlier deletions don't shift later line numbers.)*

4. **`components/air_purifier_vital200s/types.h`** (202 lines)

   a. Add `#include <string>` to the include block (lines 3-5; keep `<cstdint>`,
      `<cstddef>`, `<array>` in their current order).

   b. In the "RX Constants" block (lines 14-16), add above `RX_MIN_PACKET_LEN`:
      ```cpp
      inline constexpr size_t RX_MIN_HEADER_LEN = 6;
      ```

   c. Delete **line 31**:
      `// FILTER_RESET = 6,  // Commented out - filter life not readable from MCU`

   d. Delete **line 59**:
      `// inline constexpr Address ADDR_FILTER_RESET = {0x02, 0x05, 0x55, 0x00};  // Commented out`

   e. Delete **lines 181-182** (the `// Filter (commented out …)` comment and the
      `// inline constexpr uint8_t FILTER_RESET_ACTION = 0x03;` line).

   f. At the **end of the "Command Addresses" section** (after `ADDR_DISPLAY_LOCK`,
      currently line 64), add the preserved-knowledge block:
      ```cpp

      // Protocol notes (observed, not implemented):
      //   ADDR_FILTER_RESET  = {0x02, 0x05, 0x55, 0x00}, PayloadLen 6, action byte 0x03.
      //   Filter reset is write-only: the MCU never reports filter life, so there is nothing
      //   sensible to expose in Home Assistant. Left unimplemented on purpose.
      ```

   g. In the "TLV Types - WiFi LED" section (line ~92), add:
      ```cpp
      // The MCU echoes these TLVs back on ADDR_WIFI_LED; we ignore the RX side.
      ```

   h. **Line 124**: `inline constexpr Mode string_to_mode(const std::string &str) {`
      → `inline Mode string_to_mode(const std::string &str) {`
      (drop `constexpr` only; body unchanged).

5. **`components/air_purifier_vital200s/air_purifier_vital200s.h`** (136 lines)

   a. Delete **line 9** (commented `button.h` include).
   b. Delete **lines 33-36** (commented `FilterResetButton` class + its
      `// Filter reset (filter life not readable from MCU)` header).
   c. Delete **lines 79-83** (commented `set_filter_reset_button`).
   d. Delete **lines 94-95** (commented `send_filter_reset()`).
   e. Delete **lines 125-126** (commented `filter_reset_button_` member).
   f. Add `#include <vector>` to the include block (before the ESPHome headers,
      since `SortIncludes: false` means order is preserved as written).

6. **`components/air_purifier_vital200s/air_purifier_vital200s.cpp`** (495 lines)

   a. Delete **lines 28-31** (commented `FilterResetButton::press_action`).

   b. Delete **line 115** (`  // LOG_BUTTON("  ", "Filter Reset", filter_reset_button_);`).

   c. `read_uart_()` — replace the magic `6`s with the new constant. **Line 143**
      `if (rx_buffer_.size() < 6) {` → `if (rx_buffer_.size() < RX_MIN_HEADER_LEN) {`
      and **line 154** `expected_size = 6 + payload_len;` →
      `expected_size = RX_MIN_HEADER_LEN + payload_len;`.
      **Leave line 157 (`expected_size = 10;`) as a literal** — it is the fixed
      PING frame length, semantically distinct from `RX_MIN_PACKET_LEN` even
      though the value coincides. Add a trailing comment
      `// PING frames are fixed-length` instead.

   d. **Line 171**: `parse_packet_(rx_buffer_.data(), rx_buffer_.size());` →
      `parse_packet_(rx_buffer_.data(), expected_size);`. The two are the same
      value at that point (the loop only reaches here when
      `rx_buffer_.size() >= expected_size`, and the buffer is cleared each frame);
      this matches the humidifier and reads better.

   e. `send_ping_()` (lines 183-199): replace the trailing literal
      `0x02, 0x00, 0x55, 0x00` (lines 191-194) with
      `ADDR_STATUS[0], ADDR_STATUS[1], ADDR_STATUS[2], ADDR_STATUS[3]`.
      **Verify by eye that `ADDR_STATUS == {0x02, 0x00, 0x55, 0x00}` in
      `types.h` (it is, line 55) — so the emitted bytes are unchanged.**

   f. ⚠️ **RX checksum validation — the one behaviour change in this plan.**
      Restructure the tail of the `while (available())` loop (currently lines
      169-172) to mirror the humidifier's lines 107-118 exactly:

      ```cpp
      // Validate and process
      if (type == PacketType::STATUS) {
        const uint8_t received_checksum = rx_buffer_[static_cast<size_t>(Offset::CHECKSUM)];
        const uint8_t calculated_checksum = calc_checksum_(rx_buffer_.data(), expected_size);

        if (received_checksum == calculated_checksum) {
          parse_packet_(rx_buffer_.data(), expected_size);
        } else {
          ESP_LOGW(TAG, "Checksum mismatch: 0x%02X != 0x%02X", received_checksum, calculated_checksum);
        }
      }
      ```

      **Why this is riskier than everything else in the plan:** if the Vital 200S
      MCU computes its *outbound* checksum differently from the OasisMist, every
      status packet is dropped and all sensors go stale. We have no hardware to
      prove it. The evidence for it being safe: identical header byte, identical
      packet types, identical offsets, identical TLV framing, and the humidifier
      uses this exact formula against real MCU frames today.
      **Rollback is one line** — change the `else` branch to log *and still call*
      `parse_packet_`. See Risk R8 and the §5 hardware check.

   g. `send_display()` — **line 255** is 127 columns. Split it:
      ```cpp
      void AirPurifier::send_display(bool on) {
        ESP_LOGI(TAG, "Display: %s", on ? "ON" : "OFF");
        const auto brightness = on ? DisplayBrightness::ON : DisplayBrightness::OFF;
        send_command_(ADDR_DISPLAY, static_cast<uint8_t>(brightness));
      }
      ```
      Emitted byte is `0x64` for on / `0x00` for off, exactly as before.

   h. Delete **lines 268-288** (the commented `send_filter_reset()` body,
      including its `// Filter reset (filter life not readable from MCU)` header
      on 268).

   i. `handle_timer_tlv_()` — **line 358** currently reads
      `const uint32_t seconds = value[0] | (value[1] << 8) | (value[2] << 16) | (value[3] << 24);`.
      `value[3] << 24` promotes `uint8_t` to `int` and is **signed-overflow UB**
      when `value[3] >= 0x80`. Replace with:
      ```cpp
      const uint32_t seconds = static_cast<uint32_t>(value[0]) | (static_cast<uint32_t>(value[1]) << 8) |
                               (static_cast<uint32_t>(value[2]) << 16) | (static_cast<uint32_t>(value[3]) << 24);
      ```
      Bit-for-bit identical on every compiler we target; this only removes the UB.

   j. **Lines 362 and 367**: `(unsigned long)seconds` →
      `static_cast<unsigned long>(seconds)`.

   k. **Line 494**: `}  // namespace air_purifier` →
      `}  // namespace air_purifier_vital200s`.

   l. *(Optional, consistency only)* **line 468**
      `const uint16_t pm25 = value[0] | (value[1] << 8);` — no UB here (max
      `0xFFFF` fits in `int`), so this may be left alone or given the same
      explicit casts. Either is fine; do not spend time on it.

7. **`components/air_purifier_vital200s/__init__.py`** (121 lines)
   - Delete the four commented `FilterResetButton` / `CONF_FILTER_RESET` blocks at
     **lines 25-26, 36-37, 75-80, 118-121** (all four ranges verified correct).
     Tidy the resulting blank lines. Leave everything else as-is.

### Phase 2 — Humidifier component (mirror the air purifier)

8. **`components/humidifier_oasismist1000s/types.h`** (129 lines)

   a. Include block (lines 3-4) → add **`#include <array>`** (currently missing
      despite `std::array` on line 53) and **`#include <string>`**. Final order:
      `<cstdint>`, `<cstddef>`, `<array>`, `<string>` — matching the air purifier
      plus `<string>`.

   b. "RX Constants" (lines 13-14): add `RX_MIN_HEADER_LEN = 6` and
      `RX_MIN_PACKET_LEN = 10` so the block matches the air purifier's:
      ```cpp
      inline constexpr size_t RX_BUFFER_MAX = 128;
      inline constexpr size_t RX_MIN_HEADER_LEN = 6;
      inline constexpr size_t RX_MIN_PACKET_LEN = 10;
      inline constexpr uint32_t RX_TIMEOUT_MS = 100;
      ```

   c. Delete the "WiFi LED Constants" section at **lines 16-20**; `WIFI_BLINK_MS`
      moves to the new Wi-Fi LED section at the bottom (step 8j).

   d. **Line 36**: `WIFI_STATUS = 18,` → `WIFI_LED = 18,`.

   e. **Line 39**: `enum class Offset : uint8_t {` → `enum class Offset : size_t {`.

   f. **Line 57**: `ADDR_WIFI_STATUS` → `ADDR_WIFI_LED` (bytes unchanged;
      keep the column alignment — clang-format will normalise it in Phase 3).

   g. **Lines 79-88**: `enum class WifiTLV` → `enum class WifiLedTLV`; retitle the
      section header to `TLV Types - WiFi LED (ADDR_WIFI_LED)` to match the air
      purifier, and add:
      ```cpp
      // The MCU echoes these TLVs back on ADDR_WIFI_LED; we ignore the RX side.
      ```

   h. **Lines 90-98**: replace
      ```cpp
      enum class WifiStatus : uint8_t {
        DISCONNECTED = 0x00,
        CONNECTED = 0x01,
        CONNECTING = 0x02,
      };
      ```
      with the air purifier's spelling **verbatim** (same numeric values):
      ```cpp
      enum class WifiLedStatus : uint8_t {
        OFF = 0x00,       // LED off (disconnected)
        SOLID = 0x01,     // LED solid on (HA connected)
        BLINKING = 0x02,  // LED blinking (WiFi only, connecting to HA)
      };
      ```

   i. **Lines 110-112**: `inline constexpr char MODE_AUTO[] = "Auto";` →
      `inline constexpr const char *MODE_AUTO = "Auto";` (and `MODE_MANUAL`,
      `MODE_SLEEP`). Then append the two free conversion functions, formatted
      exactly like the air purifier's but with **this device's** enum ordering:
      ```cpp
      inline constexpr const char *mode_to_string(Mode mode) {
        switch (mode) {
          case Mode::MANUAL: return MODE_MANUAL;
          case Mode::SLEEP:  return MODE_SLEEP;
          default:           return MODE_AUTO;
        }
      }

      inline Mode string_to_mode(const std::string &str) {
        if (str == MODE_MANUAL) return Mode::MANUAL;
        if (str == MODE_SLEEP)  return Mode::SLEEP;
        return Mode::AUTO;
      }
      ```

   j. After the "Values" section (ends line 125), add the Wi-Fi LED section as the
      **last** block before the namespace close, containing the `WifiLedStatus`
      enum (moved from step 8h if you prefer to relocate rather than edit
      in place) and:
      ```cpp
      inline constexpr uint16_t WIFI_BLINK_MS = 500;
      ```
      Target final section order, matching the air purifier:
      `includes → RX constants → packet structure → command addresses →
      TLV types (status) → TLV types (WiFi LED) → mode → values/limits → WiFi LED`.

   k. Delete the stray double blank line at **lines 126-127** (one blank line
      before `}  // namespace humidifier_oasismist1000s`).

9. **`components/humidifier_oasismist1000s/humidifier_oasismist1000s.h`** (135 lines)

   a. **Line 86**: strip the trailing whitespace after
      `send_target_humidity(uint8_t humidity, bool auto_switch_mode = false);`
      (clang-format also does this).
   b. Delete **lines 89-91** — the `  // Helpers` comment *and* both
      `static const char *mode_to_string(Mode mode);` /
      `static Mode string_to_mode(const std::string &str);` declarations. Also
      remove the now-redundant blank line at 88.
   c. **Line 98**: `uint8_t calculate_checksum_(const uint8_t *data, size_t len);`
      → `uint8_t calc_checksum_(const uint8_t *data, size_t len);`.
   d. **Lines 100-101**: delete the `  // Address matching` section header and move
      `bool match_address_(const uint8_t *data, const Address &addr);` up into the
      `// UART` group, placed **before** `calc_checksum_`, so the ordering matches
      the air purifier exactly: `read_uart_`, `send_ping_`, `send_command_`,
      `match_address_`, `calc_checksum_`.
   e. Delete **line 110** (`void handle_wifi_tlv_(uint8_t type, uint8_t len, const uint8_t *value);`).
   f. **Line 134**: `}  // namespace levoit_humidifier` →
      `}  // namespace humidifier_oasismist1000s`.
   g. Add `#include <vector>` to the include block (`std::vector<uint8_t> rx_buffer_`
      on line 127 has no include today).
   h. Leave `invalidate_diagnostic_sensors_()`, the entity setters and the state
      members untouched.

10. **`components/humidifier_oasismist1000s/humidifier_oasismist1000s.cpp`** (423 lines)

    a. **Line 6**: delete `#include <cstring>` (unused — nothing in this file calls
       a `<cstring>` function). If the ESP-IDF build then fails, put it back and
       note it in the PR.

    b. **Line 11**: `static const char *TAG = "humidifier_oasismist1000s";` →
       `static const char *const TAG = "humidifier_oasismist1000s";`.

    c. **Line 26**: `parent_->send_mode(Humidifier::string_to_mode(value));` →
       `parent_->send_mode(string_to_mode(value));` (now the free function from
       `types.h`).

    d. **Lines 94 and 100**: magic `6` → `RX_MIN_HEADER_LEN` (matching Phase 1
       step 6c).

    e. **Mechanical cast sweep:** replace every `static_cast<uint8_t>(Offset::`
       with `static_cast<size_t>(Offset::`. Occurrences: **lines 99, 108, 110,
       111, 139, 154, 162 (twice), 166, 256**. The indexed values are all ≤ 10 so
       nothing changes numerically; this just matches the new `Offset : size_t`
       and the air purifier. (Lines 170, 285 and 301 already use `size_t`.)

    f. **Rename `calculate_checksum_` → `calc_checksum_`** at the definition
       (**line 159**) and every call site (**lines 111, 139, 154, 256**).

    g. `send_ping_()` / `send_command_()` (lines 129-157): fix only the broken
       4-space indentation of the `uint8_t packet[] = {` literals (clang-format
       does this in Phase 3) and add the `// checksum placeholder` trailing
       comment on the second `0x00` (lines 136 and 150) to match the air
       purifier. **Do not touch any byte.**

    h. `send_wifi_status()` (lines 224-260) — rewrite to structurally match the
       air purifier's (lines 290-327), with **byte-identical output**:
       - `uint8_t status;` → `WifiLedStatus status;` and drop the
         `static_cast<uint8_t>` at assignment; cast at point of use inside the
         packet literal (`static_cast<uint8_t>(status)`).
       - `WifiStatus::CONNECTED` → `WifiLedStatus::SOLID` (both `0x01`),
         `WifiStatus::CONNECTING` → `WifiLedStatus::BLINKING` (both `0x02`),
         `WifiStatus::DISCONNECTED` → `WifiLedStatus::OFF` (both `0x00`).
       - `WifiTLV::` → `WifiLedTLV::` (4 sites, values `0x01`-`0x04` unchanged).
       - `ADDR_WIFI_STATUS` → `ADDR_WIFI_LED` (4 sites, bytes `02 18 50 00`).
       - `PayloadLen::WIFI_STATUS` → `PayloadLen::WIFI_LED` (value `18`).
       - Change the third `status_str` from `"off (no WiFi)"` to
         `"off (disconnected)"` so it matches the air purifier. **Log string
         only** — no protocol effect.
       - Move `ESP_LOGI(TAG, "WiFi LED: %s", status_str);` from **line 259** up to
         just before the `constexpr uint8_t blink_lo` declarations, matching the
         air purifier's ordering, and delete the whitespace-only line at 258.
       - **The emitted 24-byte frame must be identical — verify element-by-element
         against the current file before committing (see §5).**

    i. Delete the entire `// Mode Helpers` section, **lines 262-278** (the
       `=====` banner comment plus both `Humidifier::mode_to_string` and
       `Humidifier::string_to_mode` definitions), and the surrounding blank line.

    j. `parse_packet_()` (lines 284-297): change the guard on **line 285** from
       `if (len < static_cast<size_t>(Offset::TLV_START)) return;` to
       `if (len < RX_MIN_PACKET_LEN) return;` (same value, `10`), and delete the
       whole `else if (match_address_(data, ADDR_WIFI_STATUS)) { … }` branch at
       **lines 291-296** including its commented body. Result:
       ```cpp
       void Humidifier::parse_packet_(const uint8_t *data, size_t len) {
         if (len < RX_MIN_PACKET_LEN) return;

         if (match_address_(data, ADDR_STATUS)) {
           parse_tlvs_(data, len, [this](uint8_t t, uint8_t l, const uint8_t *v) {
             handle_status_tlv_(t, l, v);
           });
         }
       }
       ```
       Zero behaviour change — the deleted branch was already a no-op.

    k. Delete `handle_wifi_tlv_()` entirely, **lines 377-414**.

    l. Replace every bare pointer truth-test with an explicit comparison:
       **lines 326, 331, 339 (two on that line), 345, 349, 355, 359, 364, 368**
       in `handle_status_tlv_`, and **lines 417, 419** in
       `invalidate_diagnostic_sensors_()`. E.g.
       `if (power_switch_)` → `if (power_switch_ != nullptr)`,
       `if (power_on_ && water_sensor_ && reservoir_sensor_ && reservoir_sensor_->state)`
       → `if (power_on_ && water_sensor_ != nullptr && reservoir_sensor_ != nullptr && reservoir_sensor_->state)`.

    m. **Line 422**: `}  // namespace levoit_humidifier` →
       `}  // namespace humidifier_oasismist1000s`.

11. **`components/humidifier_oasismist1000s/__init__.py`** (144 lines)
    - Add the D3 constants immediately after the `CONF_*` block (which ends at
      line 39):
      ```python
      # These must stay in sync with types.h (MODE_*, HUMIDITY_MIN/MAX, MIST_LEVEL_MIN/MAX).
      MODE_OPTIONS = ["Auto", "Manual", "Sleep"]
      HUMIDITY_MIN = 40
      HUMIDITY_MAX = 80
      MIST_LEVEL_MIN = 1
      MIST_LEVEL_MAX = 9
      ```
    - **Line 135**: `options=["Auto", "Manual", "Sleep"]` → `options=MODE_OPTIONS`.
    - **Line 139**: `min_value=40, max_value=80` →
      `min_value=HUMIDITY_MIN, max_value=HUMIDITY_MAX`.
    - **Line 143**: `min_value=1, max_value=9` →
      `min_value=MIST_LEVEL_MIN, max_value=MIST_LEVEL_MAX`.
    - **The `MODE_OPTIONS` list order must stay `["Auto", "Manual", "Sleep"]`** —
      it is what Home Assistant shows in the select.

### Phase 3 — Formatting

12. Run the formatter over the components:
    ```bash
    cd /Users/luke/esphome-levoit
    find components \( -name '*.cpp' -o -name '*.h' \) -print0 | xargs -0 clang-format -i
    ```

13. **Review the diff, then commit it on its own** (a pure-formatting commit keeps
    Phase 1/2 reviewable). Expected and *acceptable* effects — do **not** fight
    them, and do **not** add `// clang-format off` blocks or extra `.clang-format`
    keys:
    - The manually column-aligned `inline constexpr Address ADDR_… = {…};` tables
      in both `types.h` collapse to single spaces (Google style sets
      `AlignConsecutiveAssignments: None`).
    - `case Mode::MANUAL: return MODE_MANUAL;` splits onto two lines
      (`AllowShortCaseLabelsOnASingleLine: false` in Google style).
    - Short one-statement function bodies such as
      `void DisplaySwitch::write_state(bool state) { parent_->send_display(state); }`
      may collapse onto one line (`AllowShortFunctionsOnASingleLine: All`).
    - The humidifier `.cpp`'s broken 4-space packet-literal indentation and all
      trailing whitespace get fixed automatically.
    - `AllowShortIfStatementsOnASingleLine: true` preserves the one-line guards;
      `SortIncludes: false` preserves the deliberate include order.
    - The 121-column `parse_tlvs_(…)` lambda call in the air purifier's
      `parse_packet_` gets reflowed.
14. Re-run `clang-format --dry-run --Werror` and confirm it is silent before
    committing.

### Phase 4 — Sample YAMLs, secrets, tests

> **Both root YAMLs are in their original messy state and both get a full
> rewrite.** They must end up structurally identical to each other, differing only
> in device-specific values and entity blocks. Preserve every entity `name:`
> verbatim — changing one breaks existing Home Assistant entity IDs.

15. **Rewrite `air_purifier_vital200s.yaml`** (currently 81 lines, `script:` first,
    no substitutions, no `board:`). Target file, in this exact block order:

    ```yaml
    # Levoit Vital 200S air purifier — ESPHome sample configuration.
    #
    # Paste this whole file into ESPHome Builder (or the Home Assistant ESPHome
    # add-on), adjust the substitutions below, add the secrets listed in
    # secrets.yaml.example, and hit Install. Nothing needs to be copied locally.
    #
    # Hardware: an ESP32-C3 replacing the stock Wi-Fi module. UART @ 115200,
    # TX GPIO19 / RX GPIO18.

    substitutions:
      name: air-purifier
      friendly_name: "Air Purifier"
      board: esp32-c3-devkitm-1
      tx_pin: GPIO19
      rx_pin: GPIO18
      # Where the component code comes from. CI overrides this with a local path.
      component_source: github://lukednguyen/esphome-levoit

    esphome:
      name: ${name}
      friendly_name: ${friendly_name}
      on_boot:
        priority: -100
        then:
          - script.execute: update_wifi_led
          # Forces the purifier on at every boot, overriding the fan's restored
          # state (this runs after PurifierFan::setup()). Delete this line if you
          # would rather honour the state restored from flash. See Q1.
          - fan.turn_on: air_purifier_fan

    external_components:
      - source: ${component_source}
        components: [air_purifier_vital200s]
        refresh: 1d

    esp32:
      board: ${board}
      variant: esp32c3
      framework:
        type: esp-idf

    logger:
      level: DEBUG

    api:
      encryption:
        key: !secret api_key_air_purifier

    ota:
      - platform: esphome

    wifi:
      ssid: !secret wifi_ssid
      password: !secret wifi_password
      on_connect:
        - script.execute: update_wifi_led
      on_disconnect:
        - script.execute: update_wifi_led

    captive_portal:

    web_server:
      port: 80

    script:
      - id: update_wifi_led
        mode: restart
        then:
          - lambda: |-
              id(air_purifier).send_wifi_status(
                id(api_connected).state,
                wifi::global_wifi_component->is_connected()
              );

    binary_sensor:
      - platform: status
        id: api_connected
        on_press:
          - script.execute: update_wifi_led
        on_release:
          - script.execute: update_wifi_led

    uart:
      id: uart_bus
      tx_pin: ${tx_pin}
      rx_pin: ${rx_pin}
      baud_rate: 115200

    air_purifier_vital200s:
      id: air_purifier
      uart_id: uart_bus
      fan:
        id: air_purifier_fan
        name: "Air Purifier"
      pm25:
        name: "PM2.5"
      air_quality:
        name: "Air Quality"
      display:
        name: "Display"
      display_lock:
        name: "Display Lock"
      light_detection:
        name: "Light Detection"
    ```

    Invariants for this file:
    - **Entity names preserved verbatim:** `Air Purifier` (fan), `PM2.5`,
      `Air Quality`, `Display`, `Display Lock`, `Light Detection`.
    - **IDs preserved verbatim:** `air_purifier`, `air_purifier_fan`, `uart_bus`,
      `api_connected`, `update_wifi_led`.
    - The `update_wifi_led` script lambda is copied **character-for-character**
      from the current file.
    - `esphome.name` stays `air-purifier` (hyphenated) via `${name}`.
    - The `on_boot` action list keeps `script.execute: update_wifi_led` **first**,
      then `fan.turn_on: air_purifier_fan`.
    - No OTA password is added (that would require a new secret).

16. **Rewrite `humidifier_oasismist1000s.yaml`** (currently 85 lines) using the
    **same block order and the same comment header style**, with these
    device-specific differences:

    - `substitutions`: `name: humidifier`, `friendly_name: "Humidifier"`,
      `board: esp32-c3-devkitm-1`, `tx_pin: GPIO19`, `rx_pin: GPIO18`,
      `component_source: github://lukednguyen/esphome-levoit`.
    - `esphome.on_boot` contains **only** `- script.execute: update_wifi_led`
      (no `fan.turn_on`).
    - `external_components.components: [humidifier_oasismist1000s]`.
    - `api.encryption.key: !secret api_key_humidifier`.
    - The `update_wifi_led` lambda targets `id(humidifier)`.
    - Final block:
      ```yaml
      humidifier_oasismist1000s:
        id: humidifier
        uart_id: uart_bus
        power:
          name: "Power"
        mode:
          name: "Mode"
        target_humidity:
          name: "Target Humidity"
        mist_level:
          name: "Mist Level"
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
    - **Entity names preserved verbatim:** `Humidity`, `Reservoir`,
      `Water Available`, `Misting`, `Power`, `Display`, `Mode`,
      `Target Humidity`, `Mist Level`. Reordering the keys as above is fine;
      renaming is not.
    - **IDs preserved verbatim:** `humidifier`, `uart_bus`, `api_connected`,
      `update_wifi_led`.

17. Create **`secrets.yaml.example`** at the repo root:
    ```yaml
    # Copy to secrets.yaml and fill in real values. secrets.yaml is git-ignored.
    # CI copies this file verbatim, so the placeholders must stay valid:
    # the API keys must be base64 that decodes to exactly 32 bytes.
    wifi_ssid: "my-wifi"
    wifi_password: "my-wifi-password"
    api_key_air_purifier: "MDEyMzQ1Njc4OWFiY2RlZjAxMjM0NTY3ODlhYmNkZWY="
    api_key_humidifier: "MDEyMzQ1Njc4OWFiY2RlZjAxMjM0NTY3ODlhYmNkZWY="
    ```
    (That value is base64 of `0123456789abcdef0123456789abcdef` = 32 bytes.
    Sanity-check with
    `python3 -c "import base64;print(len(base64.b64decode('MDEyMzQ1Njc4OWFiY2RlZjAxMjM0NTY3ODlhYmNkZWY=')))"`
    → must print `32`.)

18. Rewrite **`.gitignore`** — un-anchor so `tests/` is covered:
    ```
    .esphome/
    .vscode/
    .pio/
    secrets.yaml
    __pycache__/
    ```
    Note `secrets.yaml` does **not** match `secrets.yaml.example`, so the example
    stays tracked. Optionally also add `/.claude/` and `/PLAN.md` if the
    maintainer does not want the planning artefacts in the repo (see Phase 6).

19. Create **`tests/air_purifier_vital200s.yaml`**:
    ```yaml
    # CI-only wrapper. Builds the real sample config (../air_purifier_vital200s.yaml)
    # against the components in this working tree instead of the published GitHub repo.
    # Not intended for flashing to a device.
    substitutions:
      component_source: ../components

    packages:
      device: !include ../air_purifier_vital200s.yaml
    ```

20. Create **`tests/humidifier_oasismist1000s.yaml`** — identical, but
    `!include ../humidifier_oasismist1000s.yaml`.

### Phase 5 — CI and README

21. Create **`.github/workflows/ci.yaml`**:
    ```yaml
    name: CI

    on:
      push:
        branches: [master, main]
      pull_request:
      schedule:
        # Weekly, to catch upstream ESPHome regressions.
        - cron: "0 6 * * 1"
      workflow_dispatch:

    concurrency:
      group: ${{ github.workflow }}-${{ github.ref }}
      cancel-in-progress: true

    jobs:
      clang-format:
        name: clang-format
        runs-on: ubuntu-latest
        steps:
          - uses: actions/checkout@v7
          - uses: actions/setup-python@v7
            with:
              python-version: "3.12"
          - run: pip install clang-format==20.1.7   # keep in sync with Phase 0
          - name: Check C++ formatting
            run: |
              find components \( -name '*.cpp' -o -name '*.h' \) -print0 \
                | xargs -0 clang-format --dry-run --Werror

      build:
        name: build ${{ matrix.config }}
        runs-on: ubuntu-latest
        strategy:
          fail-fast: false
          matrix:
            config:
              - tests/air_purifier_vital200s.yaml
              - tests/humidifier_oasismist1000s.yaml
        steps:
          - uses: actions/checkout@v7
          - name: Provide CI secrets
            run: |
              cp secrets.yaml.example secrets.yaml
              cp secrets.yaml.example tests/secrets.yaml
          - name: Build firmware
            id: build
            uses: esphome/build-action@v8.1.0
            with:
              yaml-file: ${{ matrix.config }}
              version: latest
          - name: Upload firmware
            uses: actions/upload-artifact@v7
            with:
              name: ${{ steps.build.outputs.name }}
              path: ${{ steps.build.outputs.name }}
              if-no-files-found: warn
    ```
    Before committing, check <https://github.com/esphome/build-action/releases>
    for a tag newer than `v8.1.0` and use it (**full tag — there is no floating
    major tag for this action**). `actions/checkout`, `actions/setup-python` and
    `actions/upload-artifact` *do* publish floating majors, so `@v7` is fine;
    bump if a v8 has appeared.

22. Rewrite **`README.md`** (currently 33 stale lines):
    - **Quick start (ESPHome Builder / HA add-on):** create a new device → paste
      the sample YAML → add the four secrets → Install. Emphasise that nothing
      has to be copied locally.
    - `external_components` snippet using
      `source: github://lukednguyen/esphome-levoit` with
      `components: [air_purifier_vital200s]` / `[humidifier_oasismist1000s]`, and
      a note that `${component_source}` in the samples exists so CI can build them
      from a local path.
    - Hardware / wiring section: ESP32-C3 replacing the stock Wi-Fi module, UART
      `GPIO19` TX / `GPIO18` RX @ 115200.
    - Configuration reference tables: air purifier keys (`fan`, `pm25`,
      `air_quality`, `display`, `display_lock`, `light_detection`) and humidifier
      keys (`power`, `mode`, `target_humidity`, `mist_level`, `humidity`,
      `display`, `reservoir`, `water`, `misting`), plus `uart_id`,
      `update_interval` (default `250ms`).
    - Secrets: `secrets.yaml.example` → `secrets.yaml`.
    - Development: `tests/` wrappers, how CI works, `clang-format` + the pinned
      version, and a short **"Why the two components duplicate their UART
      plumbing"** note citing D1 so nobody refactors it into a shared component.
    - Drop the stale "copy the `components/` folder" instructions and the
      `source: components/` snippets.

### Phase 6 — Commit in logical chunks and open a PR against `master`

23. Confirm you are on branch `polish-components-and-ci` and that `master` is at
    `b9b97cf`. **Do not `git add` `PLAN.md` or `.claude/`** unless the maintainer
    asks for them (or you added them to `.gitignore` in step 18).

24. Suggested commit breakdown — **one commit per phase**, in order:

    | # | Phase | Suggested subject | Files |
    |---|---|---|---|
    | 1 | 0 | `chore: add .clang-format` | `.clang-format` |
    | 2 | 1 | `refactor(air_purifier_vital200s): drop dead filter-reset code, tidy UART helpers` | `components/air_purifier_vital200s/*` |
    | 3 | 2 | `refactor(humidifier_oasismist1000s): mirror air purifier structure, drop dead WiFi RX path` | `components/humidifier_oasismist1000s/*` |
    | 4 | 3 | `style: apply clang-format to components/` | `components/**/*.{h,cpp}` |
    | 5 | 4 | `feat: first-class sample configs, secrets example, CI test wrappers` | both root YAMLs, `secrets.yaml.example`, `.gitignore`, `tests/*` |
    | 6 | 5 | `ci: build both sample configs on push and PR; rewrite README` | `.github/workflows/ci.yaml`, `README.md` |

    Split #6 into two commits (`ci:` and `docs:`) if you prefer. Keep the
    formatting commit (#4) separate and mechanical so reviewers can skip it — and
    optionally record its SHA in a `.git-blame-ignore-revs` file.

    Every commit message must end with:
    ```
    Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>
    Claude-Session: https://claude.ai/code/session_012RUYiRNZCXJk84M5MsBbKL
    ```

25. Push the branch and open a PR against `master`. The PR body should cover:

    - **What changed, in three buckets:**
      1. *Component beautification* — dead code removed (air purifier
         `FilterResetButton` across 4 files; humidifier `handle_wifi_tlv_`),
         naming/idiom unification between the two components per D4
         (`calc_checksum_`, `WifiLed*`, `Offset : size_t`, `TAG`, namespace
         comments), missing `<string>`/`<array>`/`<vector>` includes added,
         a signed-overflow UB fixed in the air purifier's timer TLV parser,
         humidifier schema magic numbers hoisted to named constants,
         `.clang-format` added and applied.
      2. *Sample YAMLs* — both rewritten as first-class, paste-into-ESPHome-Builder
         configs with `substitutions`, an explicit `board:`, `refresh: 1d`, a
         `component_source` substitution, and a header comment. **All entity
         `name:` values and all IDs are preserved verbatim**, so existing Home
         Assistant entity IDs are unaffected.
      3. *CI + packaging* — `secrets.yaml.example`, un-anchored `.gitignore`,
         `tests/` package wrappers that build the *real* sample files against the
         working tree, and a GitHub Actions workflow (clang-format job + a
         two-config `esphome/build-action` build matrix).
    - **⚠️ No-hardware caveat, stated prominently:** none of this was tested on a
      real Vital 200S or OasisMist 1000S. Every TX byte sequence was verified
      unchanged element-by-element (see §5). **The single behavioural change is
      RX checksum validation added to the air purifier** (Phase 1 step 5f) to
      match what the humidifier already does; if the Vital 200S MCU computes its
      outbound checksum differently, status packets will be dropped and all
      sensors will go stale. Ask the maintainer to watch the log for
      `Checksum mismatch` after the first flash; rollback is one line.
    - **Explain D1** briefly (why the duplication between the two components is
      deliberate and must not be refactored into a shared component).
    - **List open questions Q1-Q5 verbatim** from §6 for the maintainer to answer:
      Q1 `on_boot: fan.turn_on` intent, Q2 `dashboard_import`, Q3
      `esphome: min_version:`, Q4 default branch naming, Q5 release artifacts.

    The PR body must end with:
    ```
    🤖 Generated with [Claude Code](https://claude.com/claude-code)

    https://claude.ai/code/session_012RUYiRNZCXJk84M5MsBbKL
    ```

26. Wait for CI. Both build matrix legs and the clang-format job must go green
    before requesting review.

---

## 5. Verification

### Local (requires `pip install esphome`)
```bash
cd /Users/luke/esphome-levoit
cp secrets.yaml.example secrets.yaml
cp secrets.yaml.example tests/secrets.yaml

# 1. Fast: validation only. Must succeed for both.
esphome config tests/air_purifier_vital200s.yaml
esphome config tests/humidifier_oasismist1000s.yaml

# 2. Full compile (slow first time; downloads the ESP-IDF toolchain).
esphome compile tests/air_purifier_vital200s.yaml
esphome compile tests/humidifier_oasismist1000s.yaml
```
Checks on the `esphome config` output:
- No network access / no git clone happens → confirms
  `component_source: ../components` resolved to the **local** directory, not
  GitHub.
- The rendered `external_components.source` is `{type: local, path: ...}`.
- `esp32.board` is `esp32-c3-devkitm-1` for **both** devices.
- The humidifier entity names in the output still read `Water Available`,
  `Target Humidity`, etc.; the air purifier's still read `PM2.5`, `Air Quality`,
  `Display Lock`, `Light Detection`.

Confirm the working tree is really what got compiled:
```bash
ls tests/.esphome/build/humidifier/src/esphome/components/humidifier_oasismist1000s/
ls tests/.esphome/build/air-purifier/src/esphome/components/air_purifier_vital200s/
```
should contain the edited `types.h` / `.cpp` / `.h`.

Formatting:
```bash
find components \( -name '*.cpp' -o -name '*.h' \) -print0 \
  | xargs -0 clang-format --dry-run --Werror   # must be silent
```

### ⚠️ Protocol freeze: byte-level sanity checks without hardware
**This is the main safety net. Do not skip it, and do it before Phase 6.**
The protocol is untestable here, so guard it by diffing bytes, not behaviour:

1. `git diff b9b97cf -- components/` each of the **five** TX builders —
   air purifier `send_ping_`, `send_command_`, `send_wifi_status`,
   `send_timer_cancel_`, and humidifier `send_ping_` / `send_command_` /
   `send_wifi_status` — and confirm every array element is either unchanged or a
   *named constant whose value you have visually confirmed equals the literal it
   replaced*:
   - `ADDR_STATUS == {0x02, 0x00, 0x55, 0x00}` (air purifier `types.h` line 55).
   - `ADDR_WIFI_LED == ADDR_WIFI_STATUS == {0x02, 0x18, 0x50, 0x00}` on both.
   - `PayloadLen::WIFI_LED == PayloadLen::WIFI_STATUS == 18`.
   - `WifiLedStatus::OFF/SOLID/BLINKING == WifiStatus::DISCONNECTED/CONNECTED/CONNECTING == 0x00/0x01/0x02`.
   - `WifiLedTLV::* == WifiTLV::* == 0x01/0x02/0x03/0x04`.
   - `DisplayBrightness::ON == 0x64`, `OFF == 0x00`.
2. Confirm `calc_checksum_`'s **body** is untouched in both components (only its
   *name* changes, and only in the humidifier).
3. Confirm the only changes inside `read_uart_()` are: the `6` →
   `RX_MIN_HEADER_LEN` substitution (both), the air purifier's
   `rx_buffer_.size()` → `expected_size`, and the air purifier's new checksum
   guard (Phase 1 step 5f). Nothing else.
4. Confirm `RX_MIN_HEADER_LEN == 6` and `RX_MIN_PACKET_LEN == 10` in **both**
   `types.h`, and that every place they replaced a literal used that same value.
5. Confirm the `Mode` enum numeric values were not swapped between the two
   devices (they differ on purpose — purifier `MANUAL=0, SLEEP=1, AUTO=2`;
   humidifier `AUTO=0, MANUAL=1, SLEEP=2`), and that each device's
   `mode_to_string`/`string_to_mode` still maps to *its own* ordering.
6. Confirm the humidifier `MODE_OPTIONS` list order (`Auto`, `Manual`, `Sleep`)
   is unchanged — it is what Home Assistant shows in the select.
7. Confirm no entity `name:` string changed in either root YAML (compare against
   `git show b9b97cf:air_purifier_vital200s.yaml` and the humidifier equivalent).
8. Belt-and-braces: `grep -n 'send_wifi_status' -A 30` on both `.cpp` files, and
   the same on `git show b9b97cf:…`, then compare side by side.
9. **Hardware check to hand to the maintainer (post-merge):** after flashing the
   air purifier, watch the log for `Checksum mismatch: 0xXX != 0xYY`. A steady
   stream of these means the Vital 200S MCU's outbound checksum differs from the
   OasisMist's — revert the Phase 1 step 5f `else` branch to log and still parse.

### CI
Push the branch and open the PR (Phase 6). Both matrix legs and the clang-format
job must go green. The first run has no toolchain cache and will take ~5-10
minutes per config.

---

## 6. Risks & Open Questions

### Risks
- **R1 — Untestable protocol changes.** Highest risk. Mitigated by D8 (freeze
  runtime logic) and by the byte-diff verification in §5. If in doubt about any
  single byte, leave the code alone.
- **R2 — clang-format version drift.** Different clang-format majors produce
  different output; a mismatch between the local run and the pinned CI version
  makes CI red on a no-op. Mitigation: pin the exact version you ran locally in
  both Phase 0 and `.github/workflows/ci.yaml`.
- **R3 — `esphome/build-action@v8.1.0` requires ESPHome ≥ 2026.7.0** and
  `version: latest` floats. An upstream ESPHome change can turn CI red without
  any change here. That is intentional (early warning), but if it becomes noisy,
  pin `version:` to a known-good ESPHome release.
- **R4 — The `tests/` wrapper depends on three ESPHome behaviours** (shorthand
  local source, `CORE.relative_config_path` resolution, main-config substitutions
  winning over package substitutions). All three are verified against current
  `dev` (§2), but they are internal-ish behaviours. If
  `esphome config tests/*.yaml` fails at Phase 4, the fallback is to drop the
  `packages:` indirection and make `tests/*.yaml` standalone near-copies of the
  samples with `source: {type: local, path: ../components}` — and add a README
  note that they must be kept in sync manually.
- **R5 — Renaming misses.** All D4 renames are compiler-checked, so a missed spot
  is a build error, not a silent bug. Phase 5 CI catches it.
- **R6 — Repo visibility.** `github://lukednguyen/esphome-levoit` only works if
  the repo is public.
- **R7 — Deliberately unfixed nit:** in both `read_uart_()` the `RX_BUFFER_MAX`
  overflow guard sits *after* the `while (available())` loop, so a malformed
  `payload_len` (up to 255) can grow `rx_buffer_` past the reserved 128 bytes
  within a single `loop()` — a vector realloc, not a crash or an overflow.
  Fixing it would change drop semantics for large packets, which is untestable
  here. Leave it. Revisit only with hardware in hand.
- **R8 — NEW: air purifier RX checksum validation (Phase 1 step 5f).** The only
  behaviour-affecting change in this plan. If the Vital 200S MCU's outbound
  checksum differs from the formula in `calc_checksum_`, *every* status packet is
  rejected and PM2.5 / air quality / fan state / switches all stop updating —
  with only an `ESP_LOGW` to show for it. Evidence it is safe: identical framing,
  offsets and checksum formula to the OasisMist, whose RX path works on real
  hardware. Mitigations: the one-line rollback documented in step 5f, the
  post-flash log check in §5.9, and calling it out explicitly in the PR body.
- **R9 — Cosmetic regression from clang-format.** The hand-aligned `ADDR_*` and
  `case …: return …;` columns in both `types.h` will be reflowed. This is
  accepted (Phase 3 step 13); do not add `// clang-format off` or extra
  `.clang-format` keys to preserve them.
- **R10 — `esp32: board:` was mis-diagnosed in the previous revision.** The
  humidifier YAML is **not** currently a validation failure (`board` is optional
  when `variant` is set). Adding `board: ${board}` is a usability/explicitness
  improvement, not a bug fix. Do not describe it as a fix in commit messages or
  the PR.

### Open questions (flag to the maintainer; do not block on them)
- **Q1 — `on_boot: fan.turn_on: air_purifier_fan` in the air purifier sample.**
  It runs at priority `-100`, i.e. *after* `PurifierFan::setup()`'s flash-restore,
  so it unconditionally forces the purifier on at every boot and defeats the
  restore logic. Kept as-is because it looks deliberate; Phase 4 step 15 adds an
  inline comment saying so and noting it can be deleted to honour the restored
  state instead. Confirm intent before removing it.
- **Q2 — `dashboard_import`.** Adding
  `dashboard_import: package_import_url: github://...` would let ESPHome Builder
  "adopt" these devices, which fits the no-custom-code goal nicely — but it
  requires restructuring the samples as importable packages. Out of scope; worth
  a follow-up issue.
- **Q3 — `esphome: min_version:`.** Would protect users on old ESPHome releases,
  but picking a wrong floor breaks people. Skipped; the README states CI builds
  against ESPHome `latest`. Note that `esphome/build-action@v8` already enforces
  ESPHome ≥ 2026.7.0 for CI.
- **Q4 — Default branch.** Local refs show `master`; the workflow lists both
  `master` and `main`, so no action needed unless the branch is later renamed.
- **Q5 — Release artifacts.** Should tagged releases publish factory binaries /
  an update manifest (`complete-manifest: true`)? Not required by this task; easy
  to add later on top of this workflow.

---

## Files referenced

Existing (all read and line-verified at `b9b97cf`):
- `components/air_purifier_vital200s/types.h` (202 lines)
- `components/air_purifier_vital200s/air_purifier_vital200s.h` (136 lines)
- `components/air_purifier_vital200s/air_purifier_vital200s.cpp` (495 lines)
- `components/air_purifier_vital200s/__init__.py` (121 lines)
- `components/humidifier_oasismist1000s/types.h` (129 lines)
- `components/humidifier_oasismist1000s/humidifier_oasismist1000s.h` (135 lines)
- `components/humidifier_oasismist1000s/humidifier_oasismist1000s.cpp` (423 lines)
- `components/humidifier_oasismist1000s/__init__.py` (144 lines)
- `air_purifier_vital200s.yaml` (81 lines — full rewrite)
- `humidifier_oasismist1000s.yaml` (85 lines — full rewrite)
- `.gitignore` (4 lines)
- `README.md` (33 lines)

New files to create:
- `.clang-format` (Phase 0 — does **not** exist yet)
- `secrets.yaml.example`
- `tests/air_purifier_vital200s.yaml`
- `tests/humidifier_oasismist1000s.yaml`
- `.github/workflows/ci.yaml`
- *(optional)* `.git-blame-ignore-revs`

### Load-bearing correctness notes

1. `components/air_purifier_vital200s/types.h` declares
   `inline constexpr Mode string_to_mode(const std::string &str)` (line 124) but
   only includes `<cstdint>`, `<cstddef>` and `<array>` — `std::string` is
   available purely by include-order luck from `air_purifier_vital200s.h`. The
   plan adds `#include <string>` and drops the `constexpr` (it can never be
   constant-evaluated, since it calls non-constexpr `std::string::operator==`).
2. `components/humidifier_oasismist1000s/types.h` has the **same class of bug,
   one worse**: it uses `std::array` (line 53, `using Address = std::array<uint8_t, 4>;`)
   with no `#include <array>` at all. Add it.
3. Both component headers declare `std::vector<uint8_t> rx_buffer_` with no
   `#include <vector>`. Add it to both.
4. Three closing-namespace comments name the wrong namespace:
   `air_purifier_vital200s.cpp:494`, `humidifier_oasismist1000s.h:134`,
   `humidifier_oasismist1000s.cpp:422`.
