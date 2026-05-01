# Seplos BMS 10E Bluetooth Reader for Raspberry Pi

Python script to read Seplos 10E BMS data via Bluetooth Low Energy (BLE) on Raspberry Pi.

## Credits

This project is based on the excellent reverse engineering work by **syssi** and the [esphome-seplos-bms](https://github.com/syssi/esphome-seplos-bms) project. They did the heavy lifting of decoding the Seplos BMS protocol, which this script ports to Python for Raspberry Pi use.

## Features

- Read cell voltages, temperatures, current, SOC, SOH and more
- Decode switch states (charge/discharge/current limit/heating)
- Decode system status (charging/discharging/float/standby/shutdown)
- Decode alarm events with human-readable messages
- Support for continuous monitoring mode
- MQTT publishing with configurable topic prefix
- JSON output for integration with other systems
- Auto-reconnect on connection loss

## Requirements

- Raspberry Pi with Bluetooth (3B+, 4, 5, Zero 2 W, etc.)
- Seplos 10E BMS with Bluetooth module
- Python 3.8+

## Installation

```bash
# Install dependencies
sudo apt-get update
sudo apt-get install python3-pip
pip3 install bleak paho-mqtt

# Download script
wget https://raw.githubusercontent.com/YOUR_USERNAME/YOUR_REPO/main/seplos_bms_ble.py
chmod +x seplos_bms_ble.py
```

## Configuration

Edit the MAC address in the script:

```python
MAC_ADDRESS = "60:6E:41:16:73:DC"  # Your BMS MAC address
```

Find your BMS MAC address:
```bash
sudo bluetoothctl
[bluetooth]# scan on
# Look for device starting with 60:6E:41 (Seplos OUI)
```

## Usage

### One-shot reading (default)

```bash
sudo python3 seplos_bms_ble.py
```

Outputs full formatted data to console.

### Monitor mode (continuous)

```bash
# Every 20 seconds (default)
sudo python3 seplos_bms_ble.py monitor

# Every 5 seconds
sudo python3 seplos_bms_ble.py monitor 5

# Every minute
sudo python3 seplos_bms_ble.py monitor 1m
```

### JSON output

```bash
# One-shot
sudo python3 seplos_bms_ble.py json

# Continuous every 10 seconds
sudo python3 seplos_bms_ble.py json 10
```

### MQTT mode

```bash
# Default: localhost:1883, topic: seplosbms, interval: 20s
sudo python3 seplos_bms_ble.py mqtt

# Custom interval
sudo python3 seplos_bms_ble.py mqtt 10

# Custom MQTT broker
sudo python3 seplos_bms_ble.py mqtt --host 192.168.1.50 --port 1883

# Custom topic prefix
sudo python3 seplos_bms_ble.py mqtt --topic mybms

# Combined
sudo python3 seplos_bms_ble.py mqtt 30 --host 192.168.1.50 --port 1883 --topic seplosbms
```

### Help

```bash
sudo python3 seplos_bms_ble.py --help
```

## MQTT Topics

With default topic prefix `seplosbms`:

| Topic | Description | Unit |
|-------|-------------|------|
| `seplosbms/voltage` | Total battery voltage | V |
| `seplosbms/current` | Battery current | A |
| `seplosbms/power` | Calculated power | W |
| `seplosbms/soc` | State of charge | % |
| `seplosbms/soh` | State of health | % |
| `seplosbms/capacity_remaining` | Remaining capacity | Ah |
| `seplosbms/capacity_total` | Total capacity | Ah |
| `seplosbms/cycles` | Charge cycles | count |
| `seplosbms/port_voltage` | Port voltage | V |
| `seplosbms/temp/ambient` | Ambient temperature | °C |
| `seplosbms/temp/mosfet` | MOSFET temperature | °C |
| `seplosbms/temp/average_cell` | Average cell temperature | °C |
| `seplosbms/temp/cell_1` ... `cell_8` | Individual cell temperatures | °C |
| `seplosbms/cell/1/voltage` ... `cell_24/voltage` | Individual cell voltages | V |
| `seplosbms/cell/1/balancing` ... | Balancing state | ON/OFF |
| `seplosbms/cell/1/disconnected` ... | Disconnected state | ON/OFF |
| `seplosbms/cell/min_voltage` | Minimum cell voltage | V |
| `seplosbms/cell/max_voltage` | Maximum cell voltage | V |
| `seplosbms/cell/delta` | Cell voltage delta | V |
| `seplosbms/cell/average` | Average cell voltage | V |
| `seplosbms/switch/discharge` | Discharge switch | ON/OFF |
| `seplosbms/switch/charge` | Charge switch | ON/OFF |
| `seplosbms/switch/current_limit` | Current limit switch | ON/OFF |
| `seplosbms/switch/heating` | Heating switch | ON/OFF |
| `seplosbms/system/discharging` | System discharging | ON/OFF |
| `seplosbms/system/charging` | System charging | ON/OFF |
| `seplosbms/system/float_charge` | Float charging | ON/OFF |
| `seplosbms/system/standby` | Standby mode | ON/OFF |
| `seplosbms/system/shutdown` | Shutdown mode | ON/OFF |
| `seplosbms/alarms/count` | Number of active alarms | count |
| `seplosbms/alarms/list` | Semicolon-separated alarm list | text |
| `seplosbms/alarms/event_1` ... `event_8` | Alarm bitmask hex | hex |
| `seplosbms/info/model` | Device model | text |
| `seplosbms/info/hardware` | Hardware version | text |
| `seplosbms/info/software` | Software version | text |
| `seplosbms/info/battery_type` | Battery chemistry | text |
| `seplosbms/info/can_protocol` | CAN protocol | text |
| `seplosbms/info/rs485_protocol` | RS485 protocol | text |
| `seplosbms/info/timestamp` | Reading timestamp | ISO8601 |
| `seplosbms/json` | Complete data as JSON | JSON |
| `seplosbms/status` | Online status (retained) | online |

## Example Output

```
============================================================
SEPLOS BMS 10E
============================================================

Model: CAN:Victron | HW: 1101-SP76 | SW: 16.6
Type: LFP | CAN: Victron | RS485: Pylontech

--- Cells (16) ---
   1: 3.446V
   2: 3.449V
   3: 3.444V
   4: 3.445V
  ...
  Min: 3.444V (Cell 3)
  Max: 3.449V (Cell 2)
  Delta: 0.005V | Avg: 3.447V

--- Temps ---
  T1: 17.4C
  T2: 17.1C
  T3: 16.9C
  T4: 17.6C
  Ambient: 22.5C | MOSFET: 18.1C
  Avg Cell: 17.2C

--- Main ---
  Voltage: 55.15V | Current: -0.90A | Power: -49.6W
  SOC: 98.2% | Remaining: 275.0Ah / 280.0Ah
  Cycles: 550 | SOH: 100.0% | Port: 55.17V

--- Switches ---
  Discharge: ON | Charge: ON
  CurrentLimit: OFF | Heat: OFF

--- System ---
  Discharging: False | Charging: False | Float: False
  Standby: True | Shutdown: False

--- Alarms ---
  ! Total high voltage alarm

============================================================
```

## Protocol Details

The Seplos BMS uses a proprietary protocol over BLE with the following characteristics:

- **Service UUID**: `0000ff00-0000-1000-8000-00805f9b34fb`
- **Notify Characteristic**: `0000ff01-0000-1000-8000-00805f9b34fb` (handle 0x12)
- **Control Characteristic**: `0000ff02-0000-1000-8000-00805f9b34fb` (handle 0x14)
- **Frame format**: `0x7E [VER ADDR CID1 CID2 LEN payload CRC] 0x0D`
- **CRC**: XMODEM (same as ESPHome implementation)

Resolution of BMS values:

| Value | Resolution | Format |
|-------|-----------|--------|
| Cell voltage | 1 mV | uint16 * 0.001 |
| Temperature | 0.1 °C | (uint16 * 0.1) - 273.15 K |
| Current | 10 mA | int16 * 0.01 |
| Total voltage | 10 mV | uint16 * 0.01 |
| SOC/SOH | 0.1 % | uint16 * 0.1 |
| Capacity | 10 mAh | uint16 * 0.01 |

## Troubleshooting

### Permission denied
```bash
sudo usermod -a -G bluetooth $USER
# Logout and login again
```

### Connection timeout
- Move Raspberry Pi closer to BMS
- Check if BMS Bluetooth LED is blinking (pairing mode)
- Verify MAC address with `bluetoothctl scan on`

### MQTT connection refused
- Check if MQTT broker is running: `systemctl status mosquitto`
- Verify firewall rules for port 1883
- Test with `mosquitto_pub -t test -m "hello"`

### Wrong switch/alarm states
Make sure you are using the latest version of this script. Earlier versions had incorrect offset calculations for switch and alarm data.

## License

MIT License - See LICENSE file

## Acknowledgments

- **[syssi](https://github.com/syssi)** and the [esphome-seplos-bms](https://github.com/syssi/esphome-seplos-bms) project for decoding the Seplos BMS protocol
- **[esphome](https://esphome.io/)** community for the BLE implementation reference
