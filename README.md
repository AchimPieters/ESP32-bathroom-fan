# ESP32 Bathroom Fan

HomeKit-enabled bathroom ventilation controller for ESP32 and ESP32-C3. The firmware reads an SHT3x temperature/humidity sensor, controls a multi-speed bathroom fan through open-drain GPIO pulses, and exposes the fan plus sensor readings to Apple HomeKit.

This README reflects the current implementation in `main/main.c`, `main/Kconfig.projbuild`, and `main/idf_component.yml`.

## Features

- HomeKit fan accessory named **Bathroom Fan**.
- HomeKit temperature sensor named **Bathroom Temperature**.
- HomeKit humidity sensor named **Bathroom Humidity**.
- Manual fan control from HomeKit using `On` and `Rotation Speed`.
- Automatic fan control based on humidity, humidity rise, and temperature.
- Quiet-first control strategy with low, medium, and high fan modes.
- Night quiet mode from 22:00 until 07:00.
- Emergency humidity override at high humidity levels.
- SHT3x temperature/humidity sensor over I2C.
- Exponential moving average filtering for smoother sensor readings.
- HomeKit notification rate limiting to avoid excessive HAP updates.
- Lifecycle Manager support for NVS initialization, firmware revision, OTA trigger, Wi-Fi startup, reset handling, and restart counter protection.
- Hardware button actions for OTA, HomeKit reset, and factory reset.

## Hardware overview

The controller is designed to simulate button presses on an existing fan remote or fan control board. The LOW, MED, and HIGH outputs are configured as open-drain GPIOs and pulse active-low for 150 ms.

| Function | ESP32 default | ESP32-C3 default | Description |
|---|---:|---:|---|
| Status LED | GPIO 2 | GPIO 8 | Blinks during HomeKit identify |
| Button | GPIO 32 | GPIO 21 | Active-low control button |
| Fan LOW pad | GPIO 25 | GPIO 4 | Open-drain active-low pulse |
| Fan MED pad | GPIO 26 | GPIO 5 | Open-drain active-low pulse |
| Fan HIGH pad | GPIO 27 | GPIO 10 | Open-drain active-low pulse |
| I2C SCL | GPIO 22 | GPIO 7 | SHT3x clock |
| I2C SDA | GPIO 21 | GPIO 6 | SHT3x data |
| SHT3x address | `0x44` | `0x44` | Configurable, sometimes `0x45` |

> Note: the current code maps both `FAN_OFF` and `FAN_LOW` to a pulse on the LOW GPIO. Keep this in mind when wiring the remote/control board.

## HomeKit services

The firmware exposes one HomeKit accessory with these services:

| Service | Name | Characteristics |
|---|---|---|
| Accessory Information | Bathroom Fan | Name, manufacturer, serial number, model, firmware revision, identify |
| Fan | Bathroom Fan | On, Rotation Speed, OTA trigger |
| Temperature Sensor | Bathroom Temperature | Current Temperature |
| Humidity Sensor | Bathroom Humidity | Current Relative Humidity |

Default accessory metadata in the code:

| Field | Value |
|---|---|
| Manufacturer | `StudioPieters®` |
| Serial number | `R1TFL8J965HE` |
| Model | `TY3V0LC/Q` |
| HomeKit setup code | `374-29-730` |
| HomeKit setup ID | `F5Z1` |

## Fan control logic

The firmware continuously reads the SHT3x sensor in `sensor_task` and applies offsets, EMA filtering, and baseline tracking.

### Sensor calibration

The current calibration constants are:

| Constant | Value |
|---|---:|
| Temperature offset | `+0.8 °C` |
| Humidity offset | `-10.0 %RH` |
| EMA alpha | `0.2` |

Humidity is clamped to `0..100 %RH` after applying the offset.

### Automatic mode thresholds

| Constant | Value | Meaning |
|---|---:|---|
| `HUM_OFF` | `58 %RH` | Fan can stop after minimum runtime when below this humidity |
| `HUM_LOW_ON` | `60 %RH` | Low fan trigger |
| `HUM_MID_ON` | `70 %RH` | Shower / medium trigger |
| `HUM_HIGH_ON` | `80 %RH` | High fan trigger |
| `HUM_EMERGENCY` | `85 %RH` | Emergency high fan trigger |
| `HUM_RISE_MID` | `10 %RH` | Humidity rise trigger for shower detection |
| `HUM_RISE_HIGH` | `18 %RH` | Humidity rise trigger for high fan |
| `SHOWER_MIN_HUM` | `55 %RH` | Minimum humidity for rise-based shower detection |
| `SHOWER_TEMP_RISE_HUM` | `5 %RH` | Humidity rise used together with temperature trigger |
| `TEMP_ON` | `30 °C` | Temperature trigger for low fan |
| `TEMP_MID_ON` | `32 °C` | Temperature trigger for medium fan |
| `TEMP_OFF` | `28 °C` | Fan can stop after minimum runtime below this temperature |
| `MIN_RUNTIME_MIN` | `10 min` | Minimum automatic runtime before stopping |
| `MANUAL_TIMEOUT_MIN` | `20 min` | Manual HomeKit override timeout |

### Mode selection

In automatic mode the firmware chooses a target mode:

- **High** when emergency humidity is reached, humidity is at least `80 %RH`, or humidity rise is at least `18 %RH`.
- **Medium** when shower detection is true or temperature is at least `32 °C`.
- **Low** when humidity is at least `60 %RH` or temperature is at least `30 °C`.
- **Off** when no trigger is active and the minimum runtime / off thresholds allow it.

When the fan is already running, the firmware keeps it running for at least 10 minutes. After that, it can stop when humidity is below `58 %RH` and temperature is below `28 °C`.

### Night quiet mode

Night quiet mode is active from **22:00** until **07:00** using SNTP time from `pool.ntp.org` and the configured CET/CEST timezone.

During night mode the fan target is capped to **LOW**, unless emergency humidity is active.

## Manual HomeKit control

HomeKit writes to `On` or `Rotation Speed` switch the controller to manual mode for 20 minutes.

| HomeKit value | Fan mode |
|---:|---|
| `On = false` or speed `<= 0` | Off |
| Speed `< 40` | Low |
| Speed `< 80` | Medium |
| Speed `>= 80` | High |

After the manual timeout, automatic mode is enabled again.

## HomeKit notification safety

Sensor notifications are only sent when values change enough and rate limits allow it.

| Setting | Value |
|---|---:|
| Temperature delta | `0.2 °C` |
| Humidity delta | `0.5 %RH` |
| Temperature minimum interval | `30 s` |
| Humidity normal interval | `15 s` |
| Humidity spike interval | `5 s` |
| Max events/min normal | `20` |
| Max events/min spike | `35` |
| Spike level | `70 %RH` or humidity rise `>= 10 %RH` |

The sensor task runs every 10 seconds normally and every 3 seconds during spike mode.

## Button actions

The button uses the `achimpieters/esp32-button` component.

| Button event | Action |
|---|---|
| Single press | Request Lifecycle Manager update and reboot |
| Double press | Reset HomeKit pairing and reboot |
| Long press | Factory reset and reboot |

## Lifecycle Manager and Wi-Fi

On boot the firmware:

1. Initializes NVS with `lifecycle_nvs_init()`.
2. Logs post-reset state and increments the restart counter.
3. Configures the HomeKit firmware revision and OTA trigger.
4. Initializes GPIO, button, I2C, and SHT3x.
5. Starts the sensor task.
6. Starts Wi-Fi with `wifi_start(on_wifi_ready)`.
7. Starts SNTP and HomeKit after Wi-Fi is ready.

Wi-Fi credentials are read from NVS namespace `wifi_cfg` using keys `wifi_ssid` and `wifi_password`. If credentials are missing, `wifi_start()` returns `ESP_ERR_NVS_NOT_FOUND` and logs that provisioning is required.

The Lifecycle Manager also protects against boot loops: after 10 consecutive restarts within the configured timeout window it performs a factory reset countdown.

## Configuration

Run:

```sh
idf.py menuconfig
```

Then open the **StudioPieters** menu to configure:

- restart counter timeout
- LED GPIO
- button GPIO
- fan LOW/MED/HIGH GPIOs
- SHT3x I2C SCL/SDA pins
- SHT3x I2C address
- HomeKit setup code
- HomeKit setup ID

If you change the HomeKit setup code or setup ID, generate a new HomeKit QR code.

## Requirements

Dependencies are declared in `main/idf_component.yml`:

| Dependency | Version |
|---|---|
| ESP-IDF | `>=5.0,<7.0` |
| `achimpieters/esp32-homekit` | `>=3.0.0` |
| `achimpieters/esp32-button` | `>=1.2.3` |
| `achimpieters/esp32-sht3x` | `>=1.0.7` |

The main component builds from:

- `main/main.c`
- `main/esp32-lcm.c`

## Build and flash

Set the target once:

```sh
idf.py set-target esp32
# or
idf.py set-target esp32c3
```

Configure the project:

```sh
idf.py menuconfig
```

Build, flash, and monitor:

```sh
idf.py build
idf.py -p /dev/ttyUSB0 flash monitor
```

Adjust the serial port for your system.

## Troubleshooting `set-target`

If `idf.py set-target <chip>` prints:

```text
Directory ".../build" doesn't seem to be a CMake build directory.
Refusing to automatically delete files in this directory.
```

remove the stale build folder and run the command again:

```sh
rm -rf build
idf.py set-target esp32c3
```

For ESP32-C5/C6 on ESP-IDF 6.0, preview mode may be required:

```sh
idf.py --preview set-target esp32c5
```
