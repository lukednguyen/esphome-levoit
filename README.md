# ESPHome Levoit Custom Components

This repository contains custom ESPHome components for:
- **Levoit Air Purifier Vital 200S** (`air_purifier_vital200s`)
- **Levoit Humidifier OasisMist 1000S** (`humidifier_oasismist1000s`)

## Features
- Native UART protocol decoding
- Full control and sensor support
- WiFi LED/status integration

## Usage
1. Copy the `components/` folder into your ESPHome project, or query it direct from GitHub.
2. Reference the component in your YAML:
   ```yaml
   external_components:
     - source: components/
       components: [air_purifier_vital200s]
   ```
   or
   ```yaml
   external_components:
     - source: components/
       components: [humidifier_oasismist1000s]
   ```
3. See the provided example YAMLs for configuration.

## Development
- Contributions and issues welcome!

## License
MIT
