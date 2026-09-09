# ESPHome components for Levoit devices

Native ESPHome (UART) integrations for:

- **Levoit Vital 200S air purifier** — component `air_purifier_vital200s`
- **Levoit OasisMist 1000S humidifier** — component `humidifier_oasismist1000s`

Both talk to the device's own MCU over the serial link that the built-in ESP32
module used, so all control and sensor data stays local — no cloud, no Levoit
app.

## ⚠️ Before you flash: back up the stock firmware

The built-in ESP32 module inside these devices is an ESP32-C3. This project
reflashes that chip **in place** with ESPHome — it does not add a second board.
Flashing
ESPHome **overwrites Levoit's original firmware**, and Levoit does not publish
it anywhere. If you do not save a copy first, there is **no way back** — the
Levoit app, cloud, and Levoit OTA updates are gone permanently.

> [!WARNING]
> Read out and save the **entire flash** before you write anything to the chip.
> Do this once, while the device still has stock firmware:
>
> ```bash
> # ESP32-C3 is typically 4 MB (0x400000). Adjust if esptool reports otherwise.
> esptool.py --port /dev/ttyUSB0 --baud 460800 read_flash 0x0 0x400000 \
>   levoit-<model>-stock-firmware.bin
> ```
>
> Verify the file is the expected size, then store it somewhere **off the
> device** (not just on the machine you flashed from). To return to stock later:
>
> ```bash
> esptool.py --port /dev/ttyUSB0 write_flash 0x0 levoit-<model>-stock-firmware.bin
> ```
>
> Each unit's backup is unique (it contains that unit's Wi-Fi MAC and factory
> calibration) — keep one `.bin` per device and label it.

## Quick start (ESPHome Builder / Home Assistant add-on)

Nothing has to be checked out or copied locally.

1. Open ESPHome Builder (or the Home Assistant ESPHome add-on) and create a new
   device.
2. Replace the generated YAML with the contents of
   [`air_purifier_vital200s.yaml`](air_purifier_vital200s.yaml) or
   [`humidifier_oasismist1000s.yaml`](humidifier_oasismist1000s.yaml).
3. Edit only the `substitutions:` block (name, friendly name, board, pins).
4. Add these keys to your `secrets.yaml` (see
   [`secrets.yaml.example`](secrets.yaml.example)):
   - `wifi_ssid`, `wifi_password`
   - `api_key_air_purifier` **or** `api_key_humidifier` (base64, decodes to 32
     bytes)
5. Hit **Install**.

## Using the component in your own config

```yaml
external_components:
  - source: github://lukednguyen/esphome-levoit
    components: [air_purifier_vital200s]   # or [humidifier_oasismist1000s]
    refresh: 1d
```

The sample YAMLs wrap that URL in a `component_source` substitution so CI can
rebuild them from a local checkout; when you paste the sample as-is it still
pulls straight from GitHub.

## Hardware / wiring

The built-in ESP32 module inside the device is an ESP32-C3, reflashed in place
with ESPHome (see the backup warning above). It reaches the main MCU over this
serial link:

| Signal | ESP32-C3 pin | Notes |
|---|---|---|
| TX (to MCU) | `GPIO19` | `uart.tx_pin` |
| RX (from MCU) | `GPIO18` | `uart.rx_pin` |
| Baud | 115200 | 8N1 |

## Configuration reference

Common to both components:

| Key | Default | Description |
|---|---|---|
| `uart_id` | — | ID of the `uart:` bus |
| `update_interval` | `250ms` | MCU poll interval |

### `air_purifier_vital200s`

| Key | Entity type | Notes |
|---|---|---|
| `fan` | fan | speed 1-4, presets Auto / Sleep / Manual |
| `pm25` | sensor | µg/m³ |
| `air_quality` | text sensor | Very Good / Good / Moderate / Bad / Unknown |
| `display` | switch | panel brightness on/off |
| `display_lock` | switch | child lock |
| `light_detection` | switch | auto-dim in the dark |

### `humidifier_oasismist1000s`

| Key | Entity type | Notes |
|---|---|---|
| `fan` | fan | power on/off, speed 1-9 (mist level), presets Auto / Manual / Sleep |
| `target_humidity` | number | 40-80 %, only meaningful in Auto / Sleep |
| `humidity` | sensor | current RH % |
| `display` | switch | |
| `reservoir` | binary sensor | tank attached |
| `water` | binary sensor | water present |
| `misting` | binary sensor | actively misting |

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

#### Prefer a plain 1-9 mist-level slider?

The fan exposes mist level as a percentage. If you would rather have an exact
1-9 slider alongside the fan, add a template number that calls the component
directly:

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

## Protocol coverage / TODO

The UART protocol was reverse-engineered by capturing traffic between the
built-in ESP32 module and the device MCU. **The capture is incomplete — not
every device function has been decoded.** Unknown frames are logged at
`VERBOSE` as
`Unknown TLV 0x..` / `Unknown Status TLV 0x..`.

### Air purifier (`vital200s`) — not yet captured / incomplete

- **Timer:** only "cancel timer" is transmitted. There is no "set timer to N
  hours" TX, and the timer-remaining value reported by the MCU is logged only,
  not exposed as an entity.
- **Filter life:** not readable back from the MCU with current knowledge. The
  filter-reset command was partially reverse-engineered but is left
  unimplemented (see the protocol-notes comment in `types.h`).
- **Auto-mode sub-preferences** (the app's finer "Auto" tuning), if any.
- **Air-quality levels:** only 4 are mapped (plus Unknown) — confirm the device
  never emits a 5th.

### Humidifier (`oasismist1000s`) — not yet captured / incomplete

- **Warm mist / heating element:** the OasisMist 1000S supports warm mist; no
  address or TLV for it has been captured. Biggest gap.
- **Sleep timer:** no address captured.
- **Filter-replacement reminder / filter life:** not captured.
- **Nightlight** (if this SKU has one): not captured.
- **Unknown status TLV IDs:** there are gaps in the observed sequence at `0x01`,
  `0x06`, `0x0A` and `>= 0x0D`.
- The Wi-Fi-LED TLVs the MCU echoes back are intentionally ignored (TX only).

### How to capture more

Set `logger: level: VERY_VERBOSE`, trigger each unmapped function from the stock
Levoit app (or the physical buttons) while watching the logs, and note the
header / address / TLV type + value. Then add the address to `types.h` and a
`case` to the relevant `handle_*_tlv_` (or add a `send_*` method for a new
command). PRs welcome.

## Development

- `tests/*.yaml` are CI-only wrappers: they `!include` the real sample configs
  as a package and override `component_source` to `../components`, so CI builds
  the actual sample files against the working tree rather than the published
  repo.
- `secrets.yaml.example` → copy to `secrets.yaml` (git-ignored) for local builds.
- GitHub Actions (`.github/workflows/ci.yaml`) runs two jobs on every push / PR:
  a `clang-format` check over `components/` and an `esphome/build-action` matrix
  that compiles both sample configs for ESP32-C3 + ESP-IDF.
- C++ formatting is pinned to `clang-format==20.1.7` (`.clang-format` at the repo
  root). Keep the local version and the CI pin in sync.

### Why the two components duplicate their UART plumbing

`read_uart_`, `send_command_`, `match_address_`, `calc_checksum_` and
`parse_tlvs_` are near-identical across the two components and are deliberately
**not** factored into a shared component. ESPHome's `external_components` loader
only makes the components listed in the user's `components:` array importable, so
a shared `DEPENDENCIES` component would fail to load for anyone using the normal
explicit list. The duplication is kept spelled identically instead. Please do not
"helpfully" merge it.

## AI assistance

Parts of this repository — the component refactor, the sample configs, the CI
workflow, and this README — were written with help from an AI coding assistant
(Anthropic's Claude, via Claude Code). All changes were reviewed by a human
before merging. The UART protocol details were reverse-engineered from real
device captures, not generated.

## License

MIT
