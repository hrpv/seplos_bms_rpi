#!/usr/bin/env python3
"""
Seplos BMS 10E Bluetooth Reader für Raspberry Pi
Basierend auf: https://github.com/syssi/esphome-seplos-bms

Usage:
    python3 seplos_bms_ble.py                           # Einmalige Abfrage
    python3 seplos_bms_ble.py monitor                   # Monitor alle 20s
    python3 seplos_bms_ble.py monitor 5                # Monitor alle 5s
    python3 seplos_bms_ble.py json                      # Einmalig JSON
    python3 seplos_bms_ble.py mqtt                     # MQTT mit Topic "seplosbms"
    python3 seplos_bms_ble.py mqtt 10                   # MQTT alle 10s
    python3 seplos_bms_ble.py mqtt --topic mybms        # MQTT mit Topic "mybms"
    python3 seplos_bms_ble.py mqtt --host 192.168.1.50  # MQTT zu anderem Broker
    python3 seplos_bms_ble.py mqtt --host 192.168.1.50 --port 1883 --topic seplosbms
"""

import asyncio
import struct
import sys
import json
import time
import signal
import argparse
from dataclasses import dataclass, field
from typing import Optional, List, Dict, Any
from bleak import BleakClient, BleakError

# --- Konfiguration ---
MAC_ADDRESS = "60:6E:41:16:73:DC"
DEFAULT_INTERVAL = 20
DEFAULT_MQTT_HOST = "localhost"
DEFAULT_MQTT_PORT = 1883
DEFAULT_MQTT_TOPIC = "seplosbms"

# BLE UUIDs
SEPLOS_BMS_NOTIFY_CHAR_UUID = "0000ff01-0000-1000-8000-00805f9b34fb"
SEPLOS_BMS_CONTROL_CHAR_UUID = "0000ff02-0000-1000-8000-00805f9b34fb"

# Protokoll-Konstanten
SEPLOS_PKT_START = 0x7E
SEPLOS_PKT_END = 0x0D

# Befehle
SEPLOS_CMD_GET_SETTINGS = 0x47
SEPLOS_CMD_GET_MANUFACTURER_INFO = 0x51
SEPLOS_CMD_GET_SINGLE_MACHINE_DATA = 0x61
SEPLOS_CMD_GET_PARALLEL_DATA = 0x62

COMMAND_QUEUE = [
    (SEPLOS_CMD_GET_SETTINGS, bytes([0x00])),
    (SEPLOS_CMD_GET_MANUFACTURER_INFO, bytes()),
    (SEPLOS_CMD_GET_SINGLE_MACHINE_DATA, bytes([0x00])),
    (SEPLOS_CMD_GET_PARALLEL_DATA, bytes()),
]

MAX_RESPONSE_SIZE = 200


def crc_xmodem(data: bytes) -> int:
    crc = 0x0000
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = (crc << 1) ^ 0x1021
            else:
                crc <<= 1
        crc &= 0xFFFF
    return crc


ALARM_MESSAGES = [
    ["Voltage sensing failure", "Temperature sensing failure", "Current sensing failure",
     "Key switch failure", "Cell voltage diff failure", "Charging switch failure",
     "Discharge switch failure", "Current limit switch failure"],
    ["Single high voltage alarm", "Single overvoltage protection", "Single low voltage alarm",
     "Single undervoltage protection", "Total high voltage alarm", "Total overvoltage protection",
     "Total low voltage alarm", "Total undervoltage protection"],
    ["Charging high temp alarm", "Charging overtemp protection", "Charging low temp alarm",
     "Charging undertemp protection", "Discharge high temp alarm", "Discharge overtemp protection",
     "Discharge low temp alarm", "Discharge undertemp protection"],
    ["Ambient high temp alarm", "Ambient overtemp protection", "Ambient low temp alarm",
     "Ambient undertemp protection", "Power overtemp protection", "Power high temp alarm",
     "Battery low temp heating", "Secondary trip protection"],
    ["Charging overcurrent alarm", "Charging overcurrent protection", "Discharge overcurrent alarm",
     "Discharge overcurrent protection", "Transient overcurrent protection", "Output short circuit protection",
     "Transient overcurrent lockout", "Output short circuit lockout"],
    ["Charging high voltage protection", "Intermittent power replenishment", "Remaining capacity alarm",
     "Remaining capacity protection", "Low voltage charging prohibited", "Output reverse polarity protection",
     "Output connection failure", "Internal alarm"],
    ["Internal alarm 1", "Internal alarm 2", "Internal alarm 3", "Internal alarm 4",
     "Automatic charging waiting", "Manual charging waiting", "Internal alarm 6", "Internal alarm 7"],
    ["EEP storage failure", "RTC clock failure", "Voltage calibration not done",
     "Current calibration not done", "Zero point calibration not done", "Calendar not synchronized",
     "Internal system error 6", "Internal system error 7"],
]


@dataclass
class CellData:
    voltage: float = 0.0
    balancing: bool = False
    disconnected: bool = False

    def to_dict(self):
        return {
            "voltage": round(self.voltage, 3),
            "balancing": self.balancing,
            "disconnected": self.disconnected
        }


@dataclass
class BmsData:
    connected: bool = False
    timestamp: str = ""
    device_model: str = ""
    hardware_version: str = ""
    software_version: str = ""
    battery_type: str = ""
    can_protocol: str = ""
    rs485_protocol: str = ""
    cells: List[CellData] = field(default_factory=list)
    temperatures: List[float] = field(default_factory=list)
    ambient_temperature: float = 0.0
    mosfet_temperature: float = 0.0
    current: float = 0.0
    total_voltage: float = 0.0
    power: float = 0.0
    capacity_remaining: float = 0.0
    battery_capacity: float = 0.0
    state_of_charge: float = 0.0
    nominal_capacity: float = 0.0
    charging_cycles: int = 0
    state_of_health: float = 0.0
    port_voltage: float = 0.0
    discharge_switch: bool = False
    charge_switch: bool = False
    current_limit_switch: bool = False
    heating_switch: bool = False
    system_discharge: bool = False
    system_charge: bool = False
    system_float_charge: bool = False
    system_standby: bool = False
    system_shutdown: bool = False
    alarms: List[str] = field(default_factory=list)
    alarm_bitmasks: List[int] = field(default_factory=list)
    min_cell_voltage: float = 0.0
    max_cell_voltage: float = 0.0
    min_voltage_cell: int = 0
    max_voltage_cell: int = 0
    delta_cell_voltage: float = 0.0
    average_cell_voltage: float = 0.0
    average_cell_temperature: float = 0.0

    def to_dict(self) -> Dict[str, Any]:
        return {
            "timestamp": self.timestamp,
            "connected": self.connected,
            "device_model": self.device_model,
            "hardware_version": self.hardware_version,
            "software_version": self.software_version,
            "battery_type": self.battery_type,
            "can_protocol": self.can_protocol,
            "rs485_protocol": self.rs485_protocol,
            "cells": [c.to_dict() for c in self.cells],
            "temperatures": [round(t, 1) for t in self.temperatures],
            "ambient_temperature": round(self.ambient_temperature, 1),
            "mosfet_temperature": round(self.mosfet_temperature, 1),
            "current": round(self.current, 2),
            "total_voltage": round(self.total_voltage, 2),
            "power": round(self.power, 1),
            "capacity_remaining": round(self.capacity_remaining, 1),
            "battery_capacity": round(self.battery_capacity, 1),
            "state_of_charge": round(self.state_of_charge, 1),
            "nominal_capacity": round(self.nominal_capacity, 1),
            "charging_cycles": self.charging_cycles,
            "state_of_health": round(self.state_of_health, 1),
            "port_voltage": round(self.port_voltage, 2),
            "switches": {
                "discharge": self.discharge_switch,
                "charge": self.charge_switch,
                "current_limit": self.current_limit_switch,
                "heating": self.heating_switch,
            },
            "system_status": {
                "discharging": self.system_discharge,
                "charging": self.system_charge,
                "float_charge": self.system_float_charge,
                "standby": self.system_standby,
                "shutdown": self.system_shutdown,
            },
            "alarms": self.alarms,
            "alarm_bitmasks": [f"0x{b:02X}" for b in self.alarm_bitmasks],
            "cell_stats": {
                "min_voltage": round(self.min_cell_voltage, 3),
                "max_voltage": round(self.max_cell_voltage, 3),
                "min_cell": self.min_voltage_cell,
                "max_cell": self.max_voltage_cell,
                "delta": round(self.delta_cell_voltage, 3),
                "average": round(self.average_cell_voltage, 3),
            },
            "average_cell_temperature": round(self.average_cell_temperature, 1),
        }


def seplos_get_16bit(data: bytes, offset: int) -> int:
    return (data[offset] << 8) | data[offset + 1]


def kelvin_to_celsius(val: int) -> float:
    return round(val * 0.1 - 273.15, 1)


def interpret_can_protocol(value: int) -> str:
    return {0x00: "Unset", 0x01: "Pylontech", 0x02: "Growatt", 0x03: "Victron",
            0x04: "SMA", 0x05: "GINL", 0x06: "Studer"}.get(value, f"Unknown(0x{value:02X})")


def interpret_rs485_protocol(value: int) -> str:
    return {0x00: "Unset", 0x01: "Pylontech", 0x02: "Growatt", 0x03: "Voltronic",
            0x04: "Sofar", 0x05: "Luxpowertek", 0x06: "Studer"}.get(value, f"Unknown(0x{value:02X})")


def interpret_battery_type(value: int) -> str:
    return {0x46: "LFP", 0x47: "NCM", 0x48: "LCO", 0x49: "LTO", 0x4A: "Reserved"}.get(value, f"Unknown(0x{value:02X})")


def decode_alarms(alarm_bytes: List[int]) -> List[str]:
    alarms = []
    for event_idx, byte_val in enumerate(alarm_bytes):
        if byte_val == 0:
            continue
        for bit in range(8):
            if byte_val & (1 << bit):
                alarms.append(ALARM_MESSAGES[event_idx][bit])
    return alarms


class SeplosBmsBle:
    def __init__(self, mac_address: str):
        self.mac_address = mac_address
        self.client: Optional[BleakClient] = None
        self.frame_buffer = bytearray()
        self.data = BmsData()
        self._notification_event = asyncio.Event()
        self._command_lock = asyncio.Lock()
        self._connected = False
        self._running = True

    def _build_command(self, function: int, payload: bytes = b"") -> bytes:
        data = bytearray()
        data.extend([0x10, 0x00, 0x46, function])
        payload_length = len(payload)
        data.extend([payload_length >> 8, payload_length & 0xFF])
        data.extend(payload)
        crc = crc_xmodem(bytes(data))
        data.extend([crc >> 8, crc & 0xFF])
        data.insert(0, SEPLOS_PKT_START)
        data.append(SEPLOS_PKT_END)
        return bytes(data)

    def _notification_handler(self, sender, data: bytearray):
        if len(data) >= 1 and data[0] == SEPLOS_PKT_START:
            self.frame_buffer.clear()
        self.frame_buffer.extend(data)

        if len(self.frame_buffer) >= 7:
            data_len = (self.frame_buffer[5] << 8) | self.frame_buffer[6]
            frame_len = 7 + data_len + 2 + 1

            if frame_len > MAX_RESPONSE_SIZE:
                self.frame_buffer.clear()
                return

            if len(self.frame_buffer) >= frame_len:
                if self.frame_buffer[frame_len - 1] == SEPLOS_PKT_END:
                    computed_crc = crc_xmodem(self.frame_buffer[1:frame_len - 3])
                    remote_crc = (self.frame_buffer[frame_len - 3] << 8) | self.frame_buffer[frame_len - 2]
                    if computed_crc == remote_crc:
                        self._decode_frame(bytes(self.frame_buffer[:frame_len]))
                    self.frame_buffer.clear()
                    self._notification_event.set()

    def _decode_frame(self, data: bytes):
        function = data[3]
        if function == SEPLOS_CMD_GET_SINGLE_MACHINE_DATA:
            self._decode_single_machine_data(data)
        elif function == SEPLOS_CMD_GET_MANUFACTURER_INFO:
            self._decode_manufacturer_info(data)
        elif function == SEPLOS_CMD_GET_SETTINGS:
            self._decode_settings(data)
        elif function == SEPLOS_CMD_GET_PARALLEL_DATA:
            self._decode_parallel_data(data)

    def _decode_single_machine_data(self, data: bytes):
        if len(data) < 60:
            return

        cells = data[9]
        offset_cells = 10

        self.data.cells = []
        total_v = 0.0
        min_v, max_v = 100.0, -100.0
        min_c = max_c = 0

        for i in range(min(cells, 24)):
            v_raw = seplos_get_16bit(data, offset_cells + i * 2)
            v = round(v_raw * 0.001, 3)
            self.data.cells.append(CellData(voltage=v))
            total_v += v
            if v < min_v:
                min_v, min_c = v, i + 1
            if v > max_v:
                max_v, max_c = v, i + 1

        self.data.min_cell_voltage = min_v
        self.data.max_cell_voltage = max_v
        self.data.min_voltage_cell = min_c
        self.data.max_voltage_cell = max_c
        self.data.delta_cell_voltage = round(max_v - min_v, 3)
        self.data.average_cell_voltage = round(total_v / cells, 3) if cells else 0

        offset_temps = offset_cells + cells * 2
        temps_count = data[offset_temps]
        cell_temps = max(0, temps_count - 2)

        self.data.temperatures = []
        total_temp = 0.0
        for i in range(min(cell_temps, 8)):
            t = kelvin_to_celsius(seplos_get_16bit(data, offset_temps + 1 + i * 2))
            self.data.temperatures.append(t)
            total_temp += t

        self.data.ambient_temperature = kelvin_to_celsius(seplos_get_16bit(data, offset_temps + 1 + cell_temps * 2))
        self.data.mosfet_temperature = kelvin_to_celsius(seplos_get_16bit(data, offset_temps + 3 + cell_temps * 2))
        self.data.average_cell_temperature = round(total_temp / cell_temps, 1) if cell_temps else 0

        offset_main = 7 + 3 + (cells * 2) + 1 + (temps_count * 2)

        self.data.current = round(struct.unpack(">h", data[offset_main:offset_main + 2])[0] * 0.01, 2)
        self.data.total_voltage = round(seplos_get_16bit(data, offset_main + 2) * 0.01, 2)
        self.data.power = round(self.data.total_voltage * self.data.current, 1)
        self.data.capacity_remaining = round(seplos_get_16bit(data, offset_main + 4) * 0.01, 1)
        self.data.battery_capacity = round(seplos_get_16bit(data, offset_main + 7) * 0.01, 1)
        self.data.state_of_charge = round(seplos_get_16bit(data, offset_main + 9) * 0.1, 1)
        self.data.nominal_capacity = round(seplos_get_16bit(data, offset_main + 11) * 0.01, 1)
        self.data.charging_cycles = seplos_get_16bit(data, offset_main + 13)
        self.data.state_of_health = round(seplos_get_16bit(data, offset_main + 15) * 0.1, 1)
        self.data.port_voltage = round(seplos_get_16bit(data, offset_main + 17) * 0.01, 2)

        protection_offset = offset_main + 19
        alarm_status_offset = protection_offset + cells + temps_count + 2

        system_status = data[alarm_status_offset]
        self.data.system_discharge = bool(system_status & 0x01)
        self.data.system_charge = bool(system_status & 0x02)
        self.data.system_float_charge = bool(system_status & 0x04)
        self.data.system_standby = bool(system_status & 0x10)
        self.data.system_shutdown = bool(system_status & 0x20)

        switch_status = data[alarm_status_offset + 1]
        self.data.discharge_switch = bool(switch_status & 0x01)
        self.data.charge_switch = bool(switch_status & 0x02)
        self.data.current_limit_switch = bool(switch_status & 0x04)
        self.data.heating_switch = bool(switch_status & 0x08)

        custom_alarms = data[alarm_status_offset + 2]
        alarm_offset = alarm_status_offset + 3

        alarm_bytes = []
        for i in range(min(custom_alarms, 8)):
            if alarm_offset + i < len(data) - 3:
                alarm_bytes.append(data[alarm_offset + i])

        self.data.alarm_bitmasks = alarm_bytes
        self.data.alarms = decode_alarms(alarm_bytes)

        balancing_offset = alarm_offset + custom_alarms
        for i in range(cells):
            if balancing_offset + i // 8 < len(data):
                self.data.cells[i].balancing = bool(data[balancing_offset + i // 8] & (1 << (i % 8)))

        disc_offset = balancing_offset + (cells + 7) // 8
        for i in range(cells):
            if disc_offset + i // 8 < len(data):
                self.data.cells[i].disconnected = bool(data[disc_offset + i // 8] & (1 << (i % 8)))

    def _decode_manufacturer_info(self, data: bytes):
        if len(data) < 45:
            return
        self.data.device_model = data[7:27].decode('ascii', errors='ignore').rstrip()
        self.data.hardware_version = data[27:37].decode('ascii', errors='ignore').rstrip()
        self.data.software_version = f"{data[37]}.{data[38]}"
        self.data.can_protocol = interpret_can_protocol(data[39])
        self.data.rs485_protocol = interpret_rs485_protocol(data[40])
        self.data.battery_type = interpret_battery_type(data[41])

    def _decode_settings(self, data: bytes):
        pass

    def _decode_parallel_data(self, data: bytes):
        pass

    async def _send_command(self, function: int, payload: bytes = b"") -> bool:
        if not self._connected:
            return False
        try:
            cmd = self._build_command(function, payload)
            print(f"[SEND] 0x{function:02X}: {cmd.hex()}")
            await self.client.write_gatt_char(SEPLOS_BMS_CONTROL_CHAR_UUID, cmd, response=False)
            return True
        except Exception as e:
            print(f"[ERROR] Send failed: {e}")
            return False

    async def _wait_for_response(self, timeout: float = 5.0) -> bool:
        try:
            await asyncio.wait_for(self._notification_event.wait(), timeout=timeout)
            self._notification_event.clear()
            return True
        except asyncio.TimeoutError:
            print("[WARN] Timeout")
            return False

    async def connect(self) -> bool:
        print(f"[INFO] Connecting to {self.mac_address}...")
        try:
            self.client = BleakClient(self.mac_address, timeout=10.0)
            await self.client.connect()
            if not self.client.is_connected:
                return False
            self._connected = True
            self.data.connected = True
            print("[OK] Connected")

            await self.client.start_notify(SEPLOS_BMS_NOTIFY_CHAR_UUID, self._notification_handler)
            print("[OK] Notifications active")
            await asyncio.sleep(0.5)
            return True
        except Exception as e:
            print(f"[ERROR] {e}")
            return False

    async def disconnect(self):
        if not self._connected or not self.client:
            return
        self._connected = False
        try:
            if self.client.is_connected:
                await self.client.stop_notify(SEPLOS_BMS_NOTIFY_CHAR_UUID)
        except Exception:
            pass
        try:
            if self.client.is_connected:
                await self.client.disconnect()
        except Exception:
            pass
        print("[INFO] Disconnected")

    async def read_all_data(self) -> bool:
        if not self._connected:
            return False
        async with self._command_lock:
            for function, payload in COMMAND_QUEUE:
                if await self._send_command(function, payload):
                    await self._wait_for_response(timeout=5.0)
                await asyncio.sleep(0.3)
        return True

    def print_compact(self):
        d = self.data
        t = time.strftime("%H:%M:%S")
        print(f"[{t}] {d.total_voltage:.2f}V {d.current:+.2f}A {d.power:+.1f}W SOC:{d.state_of_charge:.1f}% "
              f"Cells:{d.min_cell_voltage:.3f}-{d.max_cell_voltage:.3f}V Δ{d.delta_cell_voltage:.3f}V "
              f"T:{d.average_cell_temperature:.1f}C "
              f"D:{"ON" if d.discharge_switch else "off"}|C:{"ON" if d.charge_switch else "off"} "
              f"Sys:{"D" if d.system_discharge else ""}{"C" if d.system_charge else ""}{"F" if d.system_float_charge else ""}{"S" if d.system_standby else ""} "
              f"A:{len(d.alarms)}")

    def print_full(self):
        d = self.data
        print("\n" + "=" * 60)
        print("SEPLOS BMS 10E")
        print("=" * 60)
        print(f"\nModel: {d.device_model} | HW: {d.hardware_version} | SW: {d.software_version}")
        print(f"Type: {d.battery_type} | CAN: {d.can_protocol} | RS485: {d.rs485_protocol}")
        print(f"\n--- Cells ({len(d.cells)}) ---")
        for i, c in enumerate(d.cells):
            flags = ""
            if c.balancing: flags += " [BAL]"
            if c.disconnected: flags += " [DISC]"
            print(f"  {i+1:2d}: {c.voltage:.3f}V{flags}")
        if d.cells:
            print(f"  Min: {d.min_cell_voltage:.3f}V (Cell {d.min_voltage_cell})")
            print(f"  Max: {d.max_cell_voltage:.3f}V (Cell {d.max_voltage_cell})")
            print(f"  Delta: {d.delta_cell_voltage:.3f}V | Avg: {d.average_cell_voltage:.3f}V")
        print(f"\n--- Temps ---")
        for i, t in enumerate(d.temperatures):
            print(f"  T{i+1}: {t:.1f}C")
        print(f"  Ambient: {d.ambient_temperature:.1f}C | MOSFET: {d.mosfet_temperature:.1f}C")
        print(f"  Avg Cell: {d.average_cell_temperature:.1f}C")
        print(f"\n--- Main ---")
        print(f"  Voltage: {d.total_voltage:.2f}V | Current: {d.current:.2f}A | Power: {d.power:.1f}W")
        print(f"  SOC: {d.state_of_charge:.1f}% | Remaining: {d.capacity_remaining:.1f}Ah / {d.battery_capacity:.1f}Ah")
        print(f"  Cycles: {d.charging_cycles} | SOH: {d.state_of_health:.1f}% | Port: {d.port_voltage:.2f}V")
        print(f"\n--- Switches ---")
        print(f"  Discharge: {'ON' if d.discharge_switch else 'OFF'} | Charge: {'ON' if d.charge_switch else 'OFF'}")
        print(f"  CurrentLimit: {'ON' if d.current_limit_switch else 'OFF'} | Heat: {'ON' if d.heating_switch else 'OFF'}")
        print(f"\n--- System ---")
        print(f"  Discharging: {d.system_discharge} | Charging: {d.system_charge} | Float: {d.system_float_charge}")
        print(f"  Standby: {d.system_standby} | Shutdown: {d.system_shutdown}")
        print(f"\n--- Alarms ---")
        if d.alarms:
            for a in d.alarms: print(f"  ! {a}")
        else:
            print("  None")
        print("\n" + "=" * 60)

    def print_json(self):
        print(json.dumps(self.data.to_dict(), indent=2))

    def stop(self):
        self._running = False


# --- MQTT Publisher ---
class MqttPublisher:
    def __init__(self, host: str, port: int, topic: str):
        self.host = host
        self.port = port
        self.topic = topic
        self.client = None
        self._connected = False

    async def connect(self) -> bool:
        try:
            import paho.mqtt.publish as publish
            # Test-Publish um Verbindung zu prüfen
            publish.single(
                f"{self.topic}/status",
                "online",
                hostname=self.host,
                port=self.port,
                retain=True
            )
            self._connected = True
            print(f"[OK] MQTT connected to {self.host}:{self.port}")
            return True
        except ImportError:
            print("[ERROR] paho-mqtt not installed. Run: pip3 install paho-mqtt")
            return False
        except Exception as e:
            print(f"[ERROR] MQTT connection failed: {e}")
            return False

    def publish(self, data: BmsData):
        if not self._connected:
            return
        try:
            import paho.mqtt.publish as publish

            d = data
            msgs = []

            # Hauptwerte
            msgs.append((f"{self.topic}/voltage", str(d.total_voltage)))
            msgs.append((f"{self.topic}/current", str(d.current)))
            msgs.append((f"{self.topic}/power", str(d.power)))
            msgs.append((f"{self.topic}/soc", str(d.state_of_charge)))
            msgs.append((f"{self.topic}/soh", str(d.state_of_health)))
            msgs.append((f"{self.topic}/capacity_remaining", str(d.capacity_remaining)))
            msgs.append((f"{self.topic}/capacity_total", str(d.battery_capacity)))
            msgs.append((f"{self.topic}/cycles", str(d.charging_cycles)))
            msgs.append((f"{self.topic}/port_voltage", str(d.port_voltage)))

            # Temperaturen
            msgs.append((f"{self.topic}/temp/ambient", str(d.ambient_temperature)))
            msgs.append((f"{self.topic}/temp/mosfet", str(d.mosfet_temperature)))
            msgs.append((f"{self.topic}/temp/average_cell", str(d.average_cell_temperature)))
            for i, t in enumerate(d.temperatures):
                msgs.append((f"{self.topic}/temp/cell_{i+1}", str(t)))

            # Zellen
            for i, c in enumerate(d.cells):
                msgs.append((f"{self.topic}/cell/{i+1}/voltage", str(c.voltage)))
                msgs.append((f"{self.topic}/cell/{i+1}/balancing", "ON" if c.balancing else "OFF"))
                msgs.append((f"{self.topic}/cell/{i+1}/disconnected", "ON" if c.disconnected else "OFF"))

            # Cell Stats
            msgs.append((f"{self.topic}/cell/min_voltage", str(d.min_cell_voltage)))
            msgs.append((f"{self.topic}/cell/max_voltage", str(d.max_cell_voltage)))
            msgs.append((f"{self.topic}/cell/delta", str(d.delta_cell_voltage)))
            msgs.append((f"{self.topic}/cell/average", str(d.average_cell_voltage)))
            msgs.append((f"{self.topic}/cell/min_cell_num", str(d.min_voltage_cell)))
            msgs.append((f"{self.topic}/cell/max_cell_num", str(d.max_voltage_cell)))

            # Schalter
            msgs.append((f"{self.topic}/switch/discharge", "ON" if d.discharge_switch else "OFF"))
            msgs.append((f"{self.topic}/switch/charge", "ON" if d.charge_switch else "OFF"))
            msgs.append((f"{self.topic}/switch/current_limit", "ON" if d.current_limit_switch else "OFF"))
            msgs.append((f"{self.topic}/switch/heating", "ON" if d.heating_switch else "OFF"))

            # System
            msgs.append((f"{self.topic}/system/discharging", "ON" if d.system_discharge else "OFF"))
            msgs.append((f"{self.topic}/system/charging", "ON" if d.system_charge else "OFF"))
            msgs.append((f"{self.topic}/system/float_charge", "ON" if d.system_float_charge else "OFF"))
            msgs.append((f"{self.topic}/system/standby", "ON" if d.system_standby else "OFF"))
            msgs.append((f"{self.topic}/system/shutdown", "ON" if d.system_shutdown else "OFF"))

            # Alarme
            msgs.append((f"{self.topic}/alarms/count", str(len(d.alarms))))
            msgs.append((f"{self.topic}/alarms/list", ";".join(d.alarms) if d.alarms else "none"))
            for i, mask in enumerate(d.alarm_bitmasks):
                msgs.append((f"{self.topic}/alarms/event_{i+1}", f"0x{mask:02X}"))

            # Info
            msgs.append((f"{self.topic}/info/model", d.device_model))
            msgs.append((f"{self.topic}/info/hardware", d.hardware_version))
            msgs.append((f"{self.topic}/info/software", d.software_version))
            msgs.append((f"{self.topic}/info/battery_type", d.battery_type))
            msgs.append((f"{self.topic}/info/can_protocol", d.can_protocol))
            msgs.append((f"{self.topic}/info/rs485_protocol", d.rs485_protocol))
            msgs.append((f"{self.topic}/info/timestamp", d.timestamp))

            # JSON komplett
            msgs.append((f"{self.topic}/json", json.dumps(d.to_dict())))

            # Mehrfach-Publish
            publish.multiple(
                [(topic, payload, 0, False) for topic, payload in msgs],
                hostname=self.host,
                port=self.port
            )

        except Exception as e:
            print(f"[ERROR] MQTT publish failed: {e}")


async def run_once(bms: SeplosBmsBle, output_format: str = "full", mqtt: Optional[MqttPublisher] = None):
    try:
        if await bms.connect():
            if await bms.read_all_data():
                bms.data.timestamp = time.strftime("%Y-%m-%dT%H:%M:%S")
                if mqtt:
                    mqtt.publish(bms.data)
                if output_format == "json":
                    bms.print_json()
                elif output_format == "compact":
                    bms.print_compact()
                else:
                    bms.print_full()
    except KeyboardInterrupt:
        pass
    finally:
        await bms.disconnect()


async def run_monitor(bms: SeplosBmsBle, interval: int, output_format: str = "compact", mqtt: Optional[MqttPublisher] = None):
    loop = asyncio.get_event_loop()

    def signal_handler(sig, frame):
        print("\n[INFO] Stopping monitor...")
        bms.stop()

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    try:
        if not await bms.connect():
            print("[ERROR] Could not connect")
            return

        first = True
        while bms._running:
            if await bms.read_all_data():
                bms.data.timestamp = time.strftime("%Y-%m-%dT%H:%M:%S")

                if mqtt:
                    mqtt.publish(bms.data)

                if output_format == "json":
                    bms.print_json()
                elif output_format == "compact" and not first:
                    bms.print_compact()
                elif output_format == "mqtt":
                    if first:
                        print(f"[OK] MQTT publishing to {mqtt.host}:{mqtt.port}/{mqtt.topic}")
                        first = False
                    print(f"[{bms.data.timestamp}] Published: {bms.data.total_voltage:.2f}V {bms.data.current:+.2f}A SOC:{bms.data.state_of_charge:.1f}%")
                else:
                    bms.print_full()
                    first = False
            else:
                print("[ERROR] Read failed, reconnecting...")
                await bms.disconnect()
                await asyncio.sleep(2)
                if not await bms.connect():
                    print("[ERROR] Reconnect failed, exiting")
                    break

            for _ in range(interval * 10):
                if not bms._running:
                    break
                await asyncio.sleep(0.1)

    except KeyboardInterrupt:
        pass
    finally:
        await bms.disconnect()


def parse_interval(arg: str) -> int:
    arg = arg.lower().strip()
    if arg.endswith('s'):
        return int(arg[:-1])
    elif arg.endswith('m'):
        return int(arg[:-1]) * 60
    else:
        return int(arg)


def main():
    parser = argparse.ArgumentParser(
        description="Seplos BMS 10E Bluetooth Reader",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python3 seplos_bms_ble.py                           # Einmalige Abfrage
  python3 seplos_bms_ble.py monitor                   # Monitor alle 20s
  python3 seplos_bms_ble.py monitor 5                 # Monitor alle 5s
  python3 seplos_bms_ble.py json                      # Einmalig JSON
  python3 seplos_bms_ble.py mqtt                      # MQTT alle 20s, Topic: seplosbms
  python3 seplos_bms_ble.py mqtt 10                   # MQTT alle 10s
  python3 seplos_bms_ble.py mqtt --topic mybms         # MQTT mit Topic "mybms"
  python3 seplos_bms_ble.py mqtt --host 192.168.1.50  # MQTT zu anderem Broker
        """
    )

    parser.add_argument("mode", nargs="?", default="full", 
                        choices=["full", "compact", "json", "monitor", "mqtt"],
                        help="Betriebsmodus (default: full)")
    parser.add_argument("interval", nargs="?", type=str, default=None,
                        help="Abfrageintervall in Sekunden (z.B. 5, 10, 20s, 1m)")
    parser.add_argument("--host", default=DEFAULT_MQTT_HOST,
                        help=f"MQTT Broker Host (default: {DEFAULT_MQTT_HOST})")
    parser.add_argument("--port", type=int, default=DEFAULT_MQTT_PORT,
                        help=f"MQTT Broker Port (default: {DEFAULT_MQTT_PORT})")
    parser.add_argument("--topic", default=DEFAULT_MQTT_TOPIC,
                        help=f"MQTT Topic Prefix (default: {DEFAULT_MQTT_TOPIC})")

    args = parser.parse_args()

    interval = DEFAULT_INTERVAL
    if args.interval:
        try:
            interval = parse_interval(args.interval)
        except ValueError:
            print(f"[ERROR] Invalid interval: {args.interval}")
            sys.exit(1)

    bms = SeplosBmsBle(MAC_ADDRESS)
    mqtt = None

    if args.mode == "mqtt":
        mqtt = MqttPublisher(args.host, args.port, args.topic)
        if not asyncio.run(mqtt.connect()):
            print("[ERROR] MQTT connection failed, exiting")
            sys.exit(1)
        asyncio.run(run_monitor(bms, interval, "mqtt", mqtt))
    elif args.mode == "monitor":
        asyncio.run(run_monitor(bms, interval, "compact"))
    elif args.mode == "json":
        if args.interval:
            asyncio.run(run_monitor(bms, interval, "json"))
        else:
            asyncio.run(run_once(bms, "json"))
    elif args.mode == "compact":
        asyncio.run(run_once(bms, "compact"))
    else:
        asyncio.run(run_once(bms, "full"))


if __name__ == "__main__":
    main()
