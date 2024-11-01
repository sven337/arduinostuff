# OpenBopi

OpenBopi is an open-source replacement firmware for the [BoPi pool controller hardware](https://meetbopi.com). It provides reliable pool automation with pH and ORP (Redox) monitoring and regulation.

## Features

### Sensor Monitoring
- Real-time reliable pH measurement
- Real-time reliable ORP (Redox) measurement
- Dual temperature monitoring (water and air) using DS18B20 sensors

### Chemical Regulation
- Automated ORP regulation through chlorine injection
- Configurable ORP target value (default 750mV)
- Safety cutoff when pH > 7.5 to prevent chlorine injection  if pump is not running
- Minimum 5-minute interval between injections
- Regulation can be enabled/disabled via MQTT or web interface

### Control Interface
- Built-in web interface for monitoring and control
- MQTT integration for home automation systems
- OTA (Over-The-Air) firmware updates
- Manual pump control (10-second activation)

### Connectivity
- WiFi connectivity
- MQTT support
- Integration with Home Assistant

### Missing features

- MDNS
- pH regulation (easy to add)
- onboard RTC (useless?)
- relay control (easy to add)
- read/write preferences in SPIFFS
- add more HW features by hacking on the board, but honestly it's probably better to build this from scratch as the board is not an improvement over the piecewise BOM from [ESP32-PoolMaster](https://github.com/Gixy31/ESP32-PoolMaster/blob/main/BOM.xlsx) as long as you use the same cheap pumps and probes as Bopi does

## Hardware Details

### Board Components
- MCU: ESP32-WROOM-32D w/ 8MB flash (but this project fits in 4MB)
- DHT11 temperature sensor (limited usefulness)
- External DS18B20 temperature sensors on jack connectors (wired incorrectly in original design, causing short circuit on connection, not a big deal but one of the various HW bugs that make me advise not buying this board)
- Peristaltic pumps: [NKP-DC-B08D (12V, 5W)](https://www.aliexpress.com/i/1005005061256347.html)
- pH/ORP reading inspired by [pH_Orp_Board](https://github.com/Loic74650/pH_Orp_Board/tree/main)
- RTC (not used in this firmware)

### GPIO Mapping

- GPIO32: Peristaltic pump 1 relay
- GPIO33: Peristaltic pump 2 relay
- GPIO14: Relay 1
- GPIO27: Relay 2
- GPIO26: Relay 3
- GPIO25: Relay 4
- GPIO19: DS18B20 temperature sensors
- GPIO23: DHT11
- I2C on the default Wire0 pins

Note: Relays are inverted (1 = OFF)

### ADC Configuration
- ORP measurement: Differential reading on channels 0-1
- pH measurement: Reading on channel 2 (differential seems not wired properly?)

### UART/programming connector

Pinout is:
```
EN     TX0ESP Vcc
GPIO0  RX0ESP GND
```

Hook this up to your USB-UART adapter, remember that Vcc requires a stronger 3.3V supply than your adapter can provide, so use a separate power supply.
Ideally, do not do it with the AC input connected, as the HW design in unsafe with 230V AC switch very close to low-voltage parts.

## MQTT Topics

### Status Topics (Read-only)
- `openbopi/pH` - Current pH value
- `openbopi/ORP` - Current ORP value (mV)
- `openbopi/orp_target` - Target ORP value (mV)
- `openbopi/orp_regulation` - Regulation status (0/1)
- `openbopi/pump1` - Pump 1 status (0/1)
- `openbopi/pump2` - Pump 2 status (0/1)
- `openbopi/watertemp` - Water temperature (°C)
- `openbopi/airtemp` - Air temperature (°C)

### Control Topics
- `openbopi/force_pump_10sec` - Force pump activation (0=pump1, 1=pump2)
- `openbopi/set_orp_target` - Set ORP target value
- `openbopi/enable_orp_regulation` - Enable/disable ORP regulation (0/1)

## Installation

1. Install PlatformIO
2. Clone this repository
3. Create `wifi.params.h` containing: 
```C
const char *ssid     = "your_ssid";
const char *password = "your_password";
```
4. Create `mqtt_login.h` containing:
```C
#define MQTT_USER "your_user"
#define MQTT_PASS "your_pass"
#define MQTT_IP "your_ip"
```
5. Connect to your BoPi hardware using the onboard programming connector and a USB-UART adapter, change platformio.ini to remove `upload_protocol` and update your `upload_port`. This is only for the first flash, you can use OTAs afterwards.

## Home Assistant Integration

Add the following to your Home Assistant configuration:
```yaml
mqtt:
  sensor:
    - name: pool_pH
      unique_id: "openbopi_ph"
      state_topic: "openbopi/pH"
      unit_of_measurement: "pH"
      device_class: "ph"
      state_class: "measurement"
      device:
        identifiers: ["openbopi_pool"]
        name: "OpenBopi"

    - name: pool_ORP
      unique_id: "openbopi_orp"
      state_topic: "openbopi/ORP"
      unit_of_measurement: "mV"
      device_class: "voltage"
      state_class: "measurement"
      device:
        identifiers: ["openbopi_pool"]

    - name: pool_filter_water_temp
      unique_id: "openbopi_water_temp"
      state_topic: "openbopi/watertemp"
      unit_of_measurement: "°C"
      device_class: "temperature"
      state_class: "measurement"
      device:
        identifiers: ["openbopi_pool"]

    - name: pool_air_temperature
      unique_id: "openbopi_air_temp"
      state_topic: "openbopi/airtemp"
      unit_of_measurement: "°C"
      device_class: "temperature"
      state_class: "measurement"
      device:
        identifiers: ["openbopi_pool"]

  binary_sensor:
    - name: pool_bopi_peripump1_status
      unique_id: "openbopi_pump1"
      state_topic: "openbopi/pump1"
      device_class: "running"
      payload_on: "1"
      payload_off: "0"
      device:
        identifiers: ["openbopi_pool"]

    - name: pool_bopi_chlorinepump_status
      unique_id: "openbopi_pump2"
      state_topic: "openbopi/pump2"
      payload_on: "1"
      payload_off: "0"
      device_class: "running"
      device:
        identifiers: ["openbopi_pool"]

  switch:
    - name: pool_orp_regulation_control
      unique_id: "openbopi_orp_regulation_control"
      command_topic: "openbopi/enable_orp_regulation"
      state_topic: "openbopi/orp_regulation"
      payload_on: "1"
      payload_off: "0"
      device:
        identifiers: ["openbopi_pool"]

  number:
    - name: pool_ORP_target
      unique_id: "openbopi_orp_target"
      state_topic: "openbopi/orp_target"
      command_topic: "openbopi/set_orp_target"
      min: 500
      max: 800
      step: 10
      unit_of_measurement: "mV"
      device_class: "voltage"
      device:
        identifiers: ["openbopi_pool"]


```

## License

[GNU Affero General Public License v3.0](https://www.gnu.org/licenses/agpl-3.0.en.html)

## Acknowledgments

- Using the [BoPi hardware](https://meetbopi.com)
- [PoolMaster](https://github.com/Gixy31/ESP32-PoolMaster/tree/main) is a better basis for projects
- pH/ORP measurement inspired by [pH_Orp_Board](https://github.com/Loic74650/pH_Orp_Board)

## Why OpenBopi?

This project aims to provide a more reliable and maintainable alternative to the original BoPi firmware, addressing various issues and adding features like:
- Proper pH and ORP measurement using differential ADC inputs
- Reliable MQTT integration
- Clean web interface
- Safety checks for chemical injection
- Home Assistant integration
- Full open-source codebase
- Hardware design improvements documentation

