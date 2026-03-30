# ESPHome RF433 Gateway

Multi-protocol 433 MHz RF gateway for ESP32-C3 + CC1101, designed as a universal
ESPHome external component for Home Assistant.

## Supported Protocols

| Protocol | TX | RX | Use Case |
|---|---|---|---|
| **A-OK** | ✅ | ✅ | Motorised blinds, pergolas, RF lights (AC114/AC226 series) |
| **Dooya** | ✅ | ✅ | Roller blinds (DC2702 remotes) — uses ESPHome built-in support |
| **Nexus / Solight TE81** | — | ✅ | Temperature/humidity sensors (TE81, TE82S, NC-7345, NX-3980) |

## Hardware

### Wiring (ESP32-C3 + CC1101 single-pin GDO0)

```
ESP32-C3-DevKitM-1       CC1101
──────────────────       ──────
3V3    ────────────────  VCC
GND    ────────────────  GND
GPIO4  ────────────────  SCK    (SPI CLK)
GPIO6  ────────────────  MOSI   (SPI MOSI)
GPIO5  ────────────────  MISO   (SPI MISO)
GPIO7  ────────────────  CSN    (SPI Chip Select)
GPIO3  ────────────────  GDO0   (Shared RX/TX data)
GPIO8  ────────────────  —      (Status LED, active low)
                         ANT  ← solder 17.3 cm wire for 433 MHz
```

> ⚠️ CC1101 is a 3.3V device. Never connect VCC to 5V.

The single-pin (GDO0) configuration uses the built-in `cc1101` component's `gdo0_pin`
option for automatic TX/RX mode switching. No manual pin manipulation needed.

## Installation

### As local component (recommended for development)

Place the `components/rf433_gw/` folder next to your YAML:

```
your-esphome-config/
├── components/
│   └── rf433_gw/
│       ├── __init__.py
│       ├── rf433_gw.h
│       └── protocols/
│           ├── aok.h
│           ├── aok.cpp
│           ├── nexus.h
│           └── nexus.cpp
├── my_home.yaml
└── secrets.yaml
```

```yaml
external_components:
  - source:
      type: local
      path: components
    components: [rf433_gw]
```

### From GitHub

```yaml
external_components:
  - source:
      type: git
      url: https://github.com/YOUR_USER/esphome-cc1101
      ref: main
    components: [rf433_gw]
    refresh: 1d
```

## Quick Start

### 1. CC1101 + SPI setup

```yaml
spi:
  clk_pin: GPIO4
  mosi_pin: GPIO6
  miso_pin: GPIO5

cc1101:
  cs_pin: GPIO7
  gdo0_pin:
    number: GPIO3
    allow_other_uses: true
  frequency: 433.92MHz
  modulation_type: ASK/OOK
  output_power: 1
```

### 2. Remote receiver/transmitter (shared GDO0 pin)

```yaml
remote_receiver:
  - id: rf_rx
    pin:
      number: GPIO3
      allow_other_uses: true
    dump: [raw, dooya]
    idle: 10ms
    filter: 100us
    tolerance: 40%
    buffer_size: 4kb

remote_transmitter:
  - id: rf_tx
    pin:
      number: GPIO3
      allow_other_uses: true
    carrier_duty_percent: 100%
    on_transmit:
      then:
        - cc1101.begin_tx
    on_complete:
      then:
        - cc1101.begin_rx
```

### 3. RF433 Gateway component

```yaml
rf433_gw:
  receiver_id: rf_rx
  debounce: 500ms

  on_aok:
    - remote_id: 0xABCDEF
      command: DOWN
      then:
        - cover.close: my_blind

  on_nexus:
    - sensor_id: 42
      channel: 1
      then:
        - lambda: |-
            ESP_LOGI("home", "Temp: %.1f°C Humidity: %d%%",
                     x.temperature, x.humidity);
```

### 4. Transmit A-OK commands

```yaml
button:
  - platform: template
    name: "Blind Down"
    on_press:
      - rf433_gw.transmit_aok:
          transmitter_id: rf_tx
          remote_id: 0xABCDEF
          address: 0x0001       # Channel 1
          command: DOWN
          repeat:
            times: 3
            wait_time: 25ms
```

### 5. Dooya (built-in ESPHome)

Dooya is natively supported by ESPHome — **no need for a wrapper**. Use the
built-in `remote_transmitter.transmit_dooya` action and `dooya:` binary sensor:

```yaml
# TX
- remote_transmitter.transmit_dooya:
    transmitter_id: rf_tx
    id: 0x00BB6A83
    channel: 1
    button: 1    # UP
    check: 1

# RX
binary_sensor:
  - platform: remote_receiver
    name: "Dooya CH1 UP"
    dooya:
      id: 0x00BB6A83
      channel: 1
      button: 1
      check: 1
```

## Component Reference

### `rf433_gw:` (top-level)

| Variable | Required | Default | Description |
|---|---|---|---|
| `receiver_id` | **yes** | — | ID of the `remote_receiver` component |
| `debounce` | no | `500ms` | Debounce window for duplicate packet suppression |
| `on_aok` | no | — | Automation triggers for received A-OK packets |
| `on_nexus` | no | — | Automation triggers for received Nexus packets |

### `on_aok:` trigger filters (all optional)

| Filter | Type | Description |
|---|---|---|
| `remote_id` | hex (0–0xFFFFFF) | 24-bit remote identifier |
| `address` | hex (0–0xFFFF) | 16-bit channel bitmask |
| `command` | enum | `UP`, `DOWN`, `STOP`, `PROGRAM`, `LIGHT_ON`, `LIGHT_OFF` |

The automation receives an `x` variable of type `AOKData` with fields:
`x.remote_id`, `x.address`, `x.command`.

### `on_nexus:` trigger filters (all optional)

| Filter | Type | Description |
|---|---|---|
| `sensor_id` | int (0–255) | 8-bit sensor ID (changes on battery swap) |
| `channel` | int (1–4) | Sensor channel |

The automation receives an `x` variable of type `NexusData` with fields:
`x.id`, `x.channel`, `x.battery_ok`, `x.test_mode`, `x.temperature`, `x.humidity`.

### `rf433_gw.transmit_aok` action

| Variable | Required | Default | Description |
|---|---|---|---|
| `transmitter_id` | **yes** | — | ID of the `remote_transmitter` component |
| `receiver_id` | no | — | ID of `rf433_gw` (enables self-reception protection) |
| `remote_id` | **yes** | — | 24-bit remote ID (hex, templatable) |
| `address` | **yes** | — | 16-bit channel bitmask (hex, templatable) |
| `command` | **yes** | — | `UP`, `DOWN`, `STOP`, `PROGRAM`, `LIGHT_ON`, `LIGHT_OFF` (templatable) |
| `repeat.times` | no | — | Number of retransmissions |
| `repeat.wait_time` | no | `25ms` | Delay between retransmissions |

### A-OK Channel Mapping

The `address` field is a **bitmask** — one bit per channel:

| Channel | Address | Description |
|---|---|---|
| 1 | `0x0001` | Bit 0 |
| 2 | `0x0002` | Bit 1 |
| 3 | `0x0004` | Bit 2 |
| 9 | `0x0100` | Bit 8 (used for pergola in example) |
| 1+2+3 | `0x0007` | Channels 1, 2, 3 simultaneously |
| All 16 | `0xFFFF` | All channels |

## Project Structure

```
components/rf433_gw/
├── __init__.py              ← ESPHome Python schema & code generation
├── rf433_gw.h               ← Receiver, triggers, transmit action (C++)
└── protocols/
    ├── aok.h                ← AOKData, AOKCommand enum, timing, AOKProtocol class
    ├── aok.cpp              ← A-OK encoder (3-frame TX) & decoder
    ├── nexus.h              ← NexusData, timing, NexusProtocol class
    └── nexus.cpp            ← Nexus PPM decoder (36-bit, temp/humidity)
```

Each protocol is self-contained — its data struct, constants, and implementation
are all in the same `.h`/`.cpp` pair. No shared base class needed.

### Adding a New Protocol

1. Create `protocols/myproto.h` — define `MyProtoData` struct + `MyProtoProtocol` class
2. Create `protocols/myproto.cpp` — implement decode (and optionally encode)
3. Add decode attempt in `RF433GWReceiver::on_receive()` in `rf433_gw.h`
4. Add trigger class (`MyProtoTrigger`) in `rf433_gw.h`
5. Add schema + code generation in `__init__.py`
6. If TX needed, add a transmit action class

## Design Decisions

### Why not a custom CC1101 SPI driver?

ESPHome has a **built-in `cc1101` component** that handles all SPI communication,
register configuration, frequency setting, AGC tuning, and automatic GDO0 pin mode
switching. Writing a custom driver would be reinventing the wheel with no benefit.

### Why is Dooya not part of the component?

ESPHome already includes full Dooya support in `remote_base`:
- `remote_transmitter.transmit_dooya` for TX
- `dooya:` binary sensor platform for RX with filtering

There is no value in wrapping this. The `my_home.yaml` uses both the built-in Dooya
and the `rf433_gw` component side-by-side.

### A-OK protocol: component vs lambda

The original YAML used ~60 lines of lambda C++ in `on_raw:` for A-OK decoding and
a `transmit_aok` script with manual packet construction. The `rf433_gw` component
replaces all of this with:
- Proper `RemoteProtocol<AOKData>` encoder/decoder with 3-frame transmission
- Named commands (`UP`, `DOWN`, `STOP`, `LIGHT_ON`, `LIGHT_OFF`) instead of hex values
- `on_aok:` triggers with optional filtering (no manual `if/switch` in lambdas)
- Checksum validation, debounce, and self-reception protection built-in

## Example Files

| File | Description |
|---|---|
| `example.yaml` | Generic example demonstrating all features |
| `my_home.yaml` | Production config — drop-in replacement for the original YAML |

## Things to Verify with Real Hardware

Before considering everything 100% functional, test these with actual devices:

### Must verify

1. **CC1101 `gdo0_pin` auto-switching** — The built-in `cc1101` component handles
   pin mode switching automatically when `gdo0_pin` is configured. Verify TX/RX
   transitions work reliably (the original YAML used manual open-drain mode).

2. **A-OK timing tolerance** — The component uses base timing of 300µs (matching the
   `nrambaud/ESPHOME_A-OK` component). The original YAML used 270/565µs. Both are
   within the 40% tolerance, but verify reception of your specific remote.

3. **A-OK 3-frame encoding** — The original YAML sent a single frame. The component
   sends 3 frames (matching the real A-OK remote behavior per the protocol spec).
   This should be more reliable, but verify it doesn't cause double-triggering on
   the motor.

4. **Dooya check values** — The `check: 14` and `check: 12` values in the Dooya TX
   scripts were captured from your specific DC2702 remote. These are preserved as-is.

### Nexus / Solight TE81

5. **Nexus timing values** — The decoder uses timing from `rtl_433` (pulse: 500µs,
   short gap: 1000µs, long gap: 2000µs, sync gap: 4000µs). These are well-documented
   but ESPHome's `remote_receiver` tolerance of 40% should handle natural variation.
   If the sensor is not detected, try:
   - Increasing `buffer_size` to `8kb`
   - Decreasing `idle` to `5ms`
   - Adding `- raw` to `dump:` and checking the pulse timings manually

6. **Nexus sensor ID** — The 8-bit ID changes when you replace the battery. After
   first successful decode, note the ID from the log and add a filtered `on_nexus:`
   trigger.

7. **Filter bandwidth** — The config uses `filter_bandwidth: 540kHz` (matching the
   original). For Nexus sensors that send at a lower rate, the default 203kHz might
   work better. If you have issues receiving Nexus while Dooya/A-OK works fine,
   try removing the `filter_bandwidth` setting.

### Nice to have

8. **BLE Proxy coexistence** — BLE scanning can occasionally interfere with RF
   timing on ESP32-C3 (single-core). Monitor for missed packets under heavy BLE
   traffic.

9. **RSSI monitoring** — The built-in `cc1101` component does not expose RSSI in
   async OOK mode. If you need signal strength monitoring, you would need to switch
   to the `radiolib_cc1101` external component or use `cc1101` in packet mode.

## License

MIT
