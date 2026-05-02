#!/usr/bin/env python3
"""
Seplos BMS 10E Bluetooth Reader für Raspberry Pi
Verbesserte Version mit MQTT v2.0 Support, Reconnection-Logik und robustem Frame-Handling

Basierend auf: https://github.com/syssi/esphome-seplos-bms
"""

import asyncio
import struct
import sys
import json
import time
import signal
import argparse
import logging
import os
from dataclasses import dataclass, field, asdict
from typing import Optional, List, Dict, Any, Tuple
from pathlib import Path
from bleak import BleakClient, BleakError

# --- Logging Setup ---
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s [%(levelname)s] %(message)s',
    datefmt='%H:%M:%S'
)
logger = logging.getLogger("seplos_bms")

# --- Konfiguration ---
CONFIG_FILE = Path(__file__).parent / "seplos_config.json"

def load_config() -> Dict[str, Any]:
    """Lädt Konfiguration aus Datei oder Umgebungsvariablen."""
    config = {
        "mac_address": "60:6E:41:16:73:DC",
        "default_interval": 20,
        "mqtt_host": "localhost",
        "mqtt_port": 1883,
        "mqtt_topic": "seplosbms",
        "mqtt_user": None,
        "mqtt_pass": None,
        "ble_timeout": 15.0,
        "response_timeout": 5.0,
        "max_reconnect_attempts": 5,
    }
    
    # Aus Datei laden
    if CONFIG_FILE.exists():
        try:
            with open(CONFIG_FILE, 'r') as f:
                file_config = json.load(f)
                config.update(file_config)
                logger.info(f"Konfiguration geladen aus {CONFIG_FILE}")
        except Exception as e:
            logger.warning(f"Konfigurationsdatei konnte nicht geladen werden: {e}")
    
    # Umgebungsvariablen überschreiben
    env_mappings = {
        "SEPLOS_MAC": "mac_address",
        "SEPLOS_MQTT_HOST": "mqtt_host",
        "SEPLOS_MQTT_PORT": "mqtt_port",
        "SEPLOS_MQTT_TOPIC": "mqtt_topic",
        "SEPLOS_MQTT_USER": "mqtt_user",
        "SEPLOS_MQTT_PASS": "mqtt_pass",
    }
    for env_var, config_key in env_mappings.items():
        if os.environ.get(env_var):
            if config_key in ["mqtt_port"]:
                config[config_key] = int(os.environ[env_var])
            else:
                config[config_key] = os.environ[env_var]
    
    return config

CONFIG = load_config()

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

COMMAND_QUEUE: List[Tuple[int, bytes]] = [
    (SEPLOS_CMD_GET_SETTINGS, bytes([0x00])),
    (SEPLOS_CMD_GET_MANUFACTURER_INFO, bytes()),
    (SEPLOS_CMD_GET_SINGLE_MACHINE_DATA, bytes([0x00])),
    (SEPLOS_CMD_GET_PARALLEL_DATA, bytes()),
]

MAX_RESPONSE_SIZE = 300
MAX_RECONNECT_ATTEMPTS = CONFIG["max_reconnect_attempts"]


def crc_xmodem(data: bytes) -> int:
    """XMODEM CRC16 Berechnung."""
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

    def to_dict(self) -> Dict[str, Any]:
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
    """Liest 16-Bit Big-Endian aus Bytes."""
    if offset + 1 >= len(data):
        return 0
    return (data[offset] << 8) | data[offset + 1]


def kelvin_to_celsius(val: int) -> float:
    """Konvertiert Kelvin * 10 zu Celsius."""
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
    """Dekodiert Alarm-Bitmasken zu Textnachrichten."""
    alarms = []
    for event_idx, byte_val in enumerate(alarm_bytes):
        if event_idx >= len(ALARM_MESSAGES):
            break
        if byte_val == 0:
            continue
        for bit in range(8):
            if bit >= len(ALARM_MESSAGES[event_idx]):
                break
            if byte_val & (1 << bit):
                alarms.append(ALARM_MESSAGES[event_idx][bit])
    return alarms


class SeplosBmsBle:
    """Hauptklasse für BLE-Kommunikation mit dem Seplos BMS."""
    
    def __init__(self, mac_address: str):
        self.mac_address = mac_address
        self.client: Optional[BleakClient] = None
        self.frame_buffer = bytearray()
        self.data = BmsData()
        self._notification_event = asyncio.Event()
        self._command_lock = asyncio.Lock()
        self._connected = False
        self._running = True
        self._reconnect_count = 0

    def _build_command(self, function: int, payload: bytes = b"") -> bytes:
        """Baut ein Seplos-Protokoll-Kommando."""
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

    def _notification_handler(self, sender: Any, data: bytearray) -> None:
        """
        Verbesserter Notification Handler mit korrektem Frame-Reassembly.
        BLE kann Pakete fragmentieren - wir müssen auf das End-Byte warten.
        """
        if not data:
            return

        # Wenn neues Frame startet, Buffer zurücksetzen
        if data[0] == SEPLOS_PKT_START:
            self.frame_buffer.clear()
            self.frame_buffer.extend(data)
        else:
            # Fortsetzung eines bestehenden Frames
            self.frame_buffer.extend(data)

        # Prüfe ob wir genug Daten für Header haben
        if len(self.frame_buffer) < 7:
            return

        # Lese Payload-Länge aus Header (Bytes 5-6)
        data_len = (self.frame_buffer[5] << 8) | self.frame_buffer[6]
        frame_len = 7 + data_len + 2 + 1  # Header + Payload + CRC + End

        if frame_len > MAX_RESPONSE_SIZE:
            logger.warning(f"Frame zu groß ({frame_len}), verworfen")
            self.frame_buffer.clear()
            return

        # Warte auf vollständiges Frame
        if len(self.frame_buffer) < frame_len:
            return

        # Frame vollständig - prüfe End-Byte
        if self.frame_buffer[frame_len - 1] != SEPLOS_PKT_END:
            logger.warning("Ungültiges Frame-Ende, verworfen")
            self.frame_buffer.clear()
            return

        # CRC prüfen
        computed_crc = crc_xmodem(self.frame_buffer[1:frame_len - 3])
        remote_crc = (self.frame_buffer[frame_len - 3] << 8) | self.frame_buffer[frame_len - 2]

        if computed_crc != remote_crc:
            logger.warning(f"CRC-Fehler: berechnet=0x{computed_crc:04X}, empfangen=0x{remote_crc:04X}")
            self.frame_buffer.clear()
            return

        # Frame erfolgreich dekodiert
        self._decode_frame(bytes(self.frame_buffer[:frame_len]))
        self.frame_buffer.clear()
        self._notification_event.set()

    def _decode_frame(self, data: bytes) -> None:
        """Verteilt Frames an die entsprechenden Decoder."""
        if len(data) < 4:
            return
        
        function = data[3]
        logger.debug(f"Frame empfangen: Function=0x{function:02X}, Länge={len(data)}")

        if function == SEPLOS_CMD_GET_SINGLE_MACHINE_DATA:
            self._decode_single_machine_data(data)
        elif function == SEPLOS_CMD_GET_MANUFACTURER_INFO:
            self._decode_manufacturer_info(data)
        elif function == SEPLOS_CMD_GET_SETTINGS:
            self._decode_settings(data)
        elif function == SEPLOS_CMD_GET_PARALLEL_DATA:
            self._decode_parallel_data(data)

    def _decode_single_machine_data(self, data: bytes) -> None:
        """Dekodiert die Haupt-BMS-Daten."""
        if len(data) < 60:
            logger.warning(f"Single machine data zu kurz: {len(data)} bytes")
            return

        try:
            cells = data[9]
            if cells > 24 or cells == 0:
                logger.warning(f"Ungültige Zellenzahl: {cells}")
                return

            offset_cells = 10

            self.data.cells = []
            total_v = 0.0
            min_v, max_v = float('inf'), float('-inf')
            min_c = max_c = 0

            for i in range(cells):
                v_raw = seplos_get_16bit(data, offset_cells + i * 2)
                v = round(v_raw * 0.001, 3)
                self.data.cells.append(CellData(voltage=v))
                total_v += v
                if v < min_v:
                    min_v, min_c = v, i + 1
                if v > max_v:
                    max_v, max_c = v, i + 1

            self.data.min_cell_voltage = min_v if min_v != float('inf') else 0.0
            self.data.max_cell_voltage = max_v if max_v != float('-inf') else 0.0
            self.data.min_voltage_cell = min_c
            self.data.max_voltage_cell = max_c
            self.data.delta_cell_voltage = round(max_v - min_v, 3) if max_v != float('-inf') else 0.0
            self.data.average_cell_voltage = round(total_v / cells, 3) if cells else 0.0

            offset_temps = offset_cells + cells * 2
            if offset_temps >= len(data):
                logger.warning("Temperatur-Offset außerhalb der Daten")
                return

            temps_count = data[offset_temps]
            cell_temps = max(0, temps_count - 2)

            self.data.temperatures = []
            total_temp = 0.0
            for i in range(min(cell_temps, 8)):
                temp_offset = offset_temps + 1 + i * 2
                if temp_offset + 1 >= len(data):
                    break
                t = kelvin_to_celsius(seplos_get_16bit(data, temp_offset))
                self.data.temperatures.append(t)
                total_temp += t

            # Ambient und MOSFET Temperatur
            ambient_offset = offset_temps + 1 + cell_temps * 2
            mosfet_offset = offset_temps + 3 + cell_temps * 2
            
            if ambient_offset + 1 < len(data):
                self.data.ambient_temperature = kelvin_to_celsius(seplos_get_16bit(data, ambient_offset))
            if mosfet_offset + 1 < len(data):
                self.data.mosfet_temperature = kelvin_to_celsius(seplos_get_16bit(data, mosfet_offset))
            
            self.data.average_cell_temperature = round(total_temp / cell_temps, 1) if cell_temps else 0.0

            # Hauptdaten - Offset berechnen
            offset_main = 7 + 3 + (cells * 2) + 1 + (temps_count * 2)
            
            if offset_main + 19 >= len(data):
                logger.warning("Hauptdaten-Offset außerhalb der Daten")
                return

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

            # Schalter und Systemstatus
            protection_offset = offset_main + 19
            alarm_status_offset = protection_offset + cells + temps_count + 2

            if alarm_status_offset + 2 >= len(data):
                logger.warning("Alarm-Status außerhalb der Daten")
                return

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

            # Alarme
            custom_alarms = data[alarm_status_offset + 2]
            alarm_offset = alarm_status_offset + 3

            alarm_bytes = []
            for i in range(min(custom_alarms, 8)):
                if alarm_offset + i < len(data) - 3:
                    alarm_bytes.append(data[alarm_offset + i])

            self.data.alarm_bitmasks = alarm_bytes
            self.data.alarms = decode_alarms(alarm_bytes)

            # Balancing-Status
            balancing_offset = alarm_offset + custom_alarms
            for i in range(cells):
                byte_idx = balancing_offset + i // 8
                if byte_idx < len(data):
                    self.data.cells[i].balancing = bool(data[byte_idx] & (1 << (i % 8)))

            # Disconnected-Status
            disc_offset = balancing_offset + (cells + 7) // 8
            for i in range(cells):
                byte_idx = disc_offset + i // 8
                if byte_idx < len(data):
                    self.data.cells[i].disconnected = bool(data[byte_idx] & (1 << (i % 8)))

        except Exception as e:
            logger.error(f"Fehler beim Dekodieren der Single Machine Data: {e}")

    def _decode_manufacturer_info(self, data: bytes) -> None:
        """Dekodiert Herstellerinformationen."""
        if len(data) < 45:
            logger.warning(f"Manufacturer info zu kurz: {len(data)} bytes")
            return
        
        try:
            self.data.device_model = data[7:27].decode('ascii', errors='ignore').strip('\x00').strip()
            self.data.hardware_version = data[27:37].decode('ascii', errors='ignore').strip('\x00').strip()
            self.data.software_version = f"{data[37]}.{data[38]}"
            self.data.can_protocol = interpret_can_protocol(data[39])
            self.data.rs485_protocol = interpret_rs485_protocol(data[40])
            self.data.battery_type = interpret_battery_type(data[41])
        except Exception as e:
            logger.error(f"Fehler beim Dekodieren der Manufacturer Info: {e}")

    def _decode_settings(self, data: bytes) -> None:
        """Platzhalter für Settings-Dekodierung."""
        pass

    def _decode_parallel_data(self, data: bytes) -> None:
        """Platzhalter für Parallel-Daten-Dekodierung."""
        pass

    async def _send_command(self, function: int, payload: bytes = b"") -> bool:
        """Sendet ein Kommando an das BMS."""
        if not self._connected or not self.client:
            return False
        
        try:
            cmd = self._build_command(function, payload)
            logger.debug(f"[SEND] 0x{function:02X}: {cmd.hex()}")
            await self.client.write_gatt_char(SEPLOS_BMS_CONTROL_CHAR_UUID, cmd, response=False)
            return True
        except BleakError as e:
            logger.error(f"BLE Fehler beim Senden: {e}")
            self._connected = False
            return False
        except Exception as e:
            logger.error(f"Senden fehlgeschlagen: {e}")
            return False

    async def _wait_for_response(self, timeout: float = None) -> bool:
        """Wartet auf eine BLE-Notification."""
        if timeout is None:
            timeout = CONFIG["response_timeout"]
            
        try:
            await asyncio.wait_for(self._notification_event.wait(), timeout=timeout)
            self._notification_event.clear()
            return True
        except asyncio.TimeoutError:
            logger.warning(f"Timeout nach {timeout}s")
            return False

    async def connect(self) -> bool:
        """Stellt eine BLE-Verbindung her."""
        logger.info(f"Verbinde mit {self.mac_address}...")
        
        try:
            self.client = BleakClient(
                self.mac_address, 
                timeout=CONFIG["ble_timeout"],
                disconnected_callback=self._on_disconnect
            )
            await self.client.connect()
            
            if not self.client.is_connected:
                logger.error("Verbindung konnte nicht hergestellt werden")
                return False
                
            self._connected = True
            self.data.connected = True
            self._reconnect_count = 0
            logger.info("Verbunden")

            await self.client.start_notify(SEPLOS_BMS_NOTIFY_CHAR_UUID, self._notification_handler)
            logger.info("Notifications aktiviert")
            await asyncio.sleep(0.5)
            return True
            
        except BleakError as e:
            logger.error(f"BLE Fehler: {e}")
            return False
        except Exception as e:
            logger.error(f"Verbindungsfehler: {e}")
            return False

    def _on_disconnect(self, client: BleakClient) -> None:
        """Callback bei unerwarteter Trennung."""
        logger.warning("Verbindung unerwartet getrennt")
        self._connected = False
        self.data.connected = False

    async def disconnect(self) -> None:
        """Trennt die BLE-Verbindung sicher."""
        if not self.client:
            return
            
        self._connected = False
        self.data.connected = False
        
        try:
            if self.client.is_connected:
                await self.client.stop_notify(SEPLOS_BMS_NOTIFY_CHAR_UUID)
        except Exception as e:
            logger.debug(f"Fehler beim Stoppen der Notifications: {e}")
            
        try:
            if self.client.is_connected:
                await self.client.disconnect()
        except Exception as e:
            logger.debug(f"Fehler beim Disconnect: {e}")
            
        logger.info("Getrennt")

    async def read_all_data(self) -> bool:
        """Liest alle BMS-Daten sequentiell."""
        if not self._connected:
            return False
            
        async with self._command_lock:
            for function, payload in COMMAND_QUEUE:
                if await self._send_command(function, payload):
                    if not await self._wait_for_response():
                        logger.warning(f"Keine Antwort für Command 0x{function:02X}")
                else:
                    return False
                await asyncio.sleep(0.3)
            return True

    def print_compact(self) -> None:
        """Kompakte Ausgabe."""
        d = self.data
        t = time.strftime("%H:%M:%S")
        status = []
        if d.system_discharge: status.append("D")
        if d.system_charge: status.append("C")
        if d.system_float_charge: status.append("F")
        if d.system_standby: status.append("S")
        
        print(f"[{t}] {d.total_voltage:.2f}V {d.current:+.2f}A {d.power:+.1f}W "
              f"SOC:{d.state_of_charge:.1f}% "
              f"Cells:{d.min_cell_voltage:.3f}-{d.max_cell_voltage:.3f}V "
              f"Δ{d.delta_cell_voltage:.3f}V "
              f"T:{d.average_cell_temperature:.1f}°C "
              f"D:{'ON' if d.discharge_switch else 'off'}|"
              f"C:{'ON' if d.charge_switch else 'off'} "
              f"Sys:{''.join(status) or '-'} "
              f"A:{len(d.alarms)}")

    def print_full(self) -> None:
        """Detaillierte Ausgabe."""
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
            print(f" {i+1:2d}: {c.voltage:.3f}V{flags}")
            
        if d.cells:
            print(f" Min: {d.min_cell_voltage:.3f}V (Cell {d.min_voltage_cell})")
            print(f" Max: {d.max_cell_voltage:.3f}V (Cell {d.max_voltage_cell})")
            print(f" Delta: {d.delta_cell_voltage:.3f}V | Avg: {d.average_cell_voltage:.3f}V")
            
        print(f"\n--- Temps ---")
        for i, t in enumerate(d.temperatures):
            print(f" T{i+1}: {t:.1f}°C")
        print(f" Ambient: {d.ambient_temperature:.1f}°C | MOSFET: {d.mosfet_temperature:.1f}°C")
        print(f" Avg Cell: {d.average_cell_temperature:.1f}°C")
        
        print(f"\n--- Main ---")
        print(f" Voltage: {d.total_voltage:.2f}V | Current: {d.current:.2f}A | Power: {d.power:.1f}W")
        print(f" SOC: {d.state_of_charge:.1f}% | Remaining: {d.capacity_remaining:.1f}Ah / {d.battery_capacity:.1f}Ah")
        print(f" Cycles: {d.charging_cycles} | SOH: {d.state_of_health:.1f}% | Port: {d.port_voltage:.2f}V")
        
        print(f"\n--- Switches ---")
        print(f" Discharge: {'ON' if d.discharge_switch else 'OFF'} | Charge: {'ON' if d.charge_switch else 'OFF'}")
        print(f" CurrentLimit: {'ON' if d.current_limit_switch else 'OFF'} | Heat: {'ON' if d.heating_switch else 'OFF'}")
        
        print(f"\n--- System ---")
        print(f" Discharging: {d.system_discharge} | Charging: {d.system_charge} | Float: {d.system_float_charge}")
        print(f" Standby: {d.system_standby} | Shutdown: {d.system_shutdown}")
        
        print(f"\n--- Alarms ---")
        if d.alarms:
            for a in d.alarms:
                print(f" ! {a}")
        else:
            print(" None")
        print("\n" + "=" * 60)

    def print_json(self) -> None:
        """JSON-Ausgabe."""
        print(json.dumps(self.data.to_dict(), indent=2, ensure_ascii=False))

    def stop(self) -> None:
        """Stoppt den Monitor-Modus."""
        self._running = False


class MqttPublisher:
    """MQTT Publisher mit Paho v2.0 Kompatibilität."""
    
    def __init__(self, host: str, port: int, topic: str, 
                 username: Optional[str] = None, password: Optional[str] = None):
        self.host = host
        self.port = port
        self.topic = topic
        self.username = username
        self.password = password
        self.client = None
        self._connected = False

    async def connect(self) -> bool:
        """Verbindet mit dem MQTT Broker."""
        try:
            import paho.mqtt.client as mqtt
            
            self.client = mqtt.Client(
                callback_api_version=mqtt.CallbackAPIVersion.VERSION2
            )
            
            if self.username and self.password:
                self.client.username_pw_set(self.username, self.password)
            
            # Synchrone Verbindung (MQTT ist nicht async)
            self.client.connect(self.host, self.port, keepalive=60)
            self.client.loop_start()
            
            # Test-Publish
            self.client.publish(
                f"{self.topic}/status", 
                "online", 
                qos=0, 
                retain=True
            )
            
            self._connected = True
            logger.info(f"MQTT verbunden mit {self.host}:{self.port}")
            return True
            
        except ImportError:
            logger.error("paho-mqtt nicht installiert. Installieren mit: pip3 install paho-mqtt>=2.0")
            return False
        except Exception as e:
            logger.error(f"MQTT Verbindungsfehler: {e}")
            return False

    def publish(self, data: BmsData) -> None:
        """Publiziert BMS-Daten als MQTT-Nachrichten."""
        if not self._connected or not self.client:
            return
            
        try:
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

            # Batch-Publish
            for topic, payload in msgs:
                self.client.publish(topic, payload, qos=0)
                
        except Exception as e:
            logger.error(f"MQTT Publish Fehler: {e}")
            self._connected = False

    def disconnect(self) -> None:
        """Trennt die MQTT-Verbindung."""
        if self.client:
            try:
                self.client.publish(f"{self.topic}/status", "offline", retain=True)
                self.client.loop_stop()
                self.client.disconnect()
            except Exception:
                pass


async def run_once(bms: SeplosBmsBle, output_format: str = "full", 
                   mqtt: Optional[MqttPublisher] = None) -> bool:
    """Einmalige Abfrage der BMS-Daten."""
    success = False
    try:
        if await bms.connect():
            if await bms.read_all_data():
                bms.data.timestamp = time.strftime("%Y-%m-%dT%H:%M:%S")
                success = True
                
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
        
    return success


async def run_monitor(bms: SeplosBmsBle, interval: int, 
                      output_format: str = "compact",
                      mqtt: Optional[MqttPublisher] = None) -> None:
    """Kontinuierlicher Monitor-Modus mit Reconnection."""
    
    def signal_handler() -> None:
        logger.info("Monitor wird beendet...")
        bms.stop()

    # Async Signal-Handler
    loop = asyncio.get_running_loop()
    for sig in (signal.SIGINT, signal.SIGTERM):
        loop.add_signal_handler(sig, signal_handler)

    reconnect_delay = 2
    first = True
    
    try:
        while bms._running:
            if not bms._connected:
                if bms._reconnect_count >= MAX_RECONNECT_ATTEMPTS:
                    logger.error(f"Maximale Reconnect-Versuche ({MAX_RECONNECT_ATTEMPTS}) erreicht")
                    break
                    
                logger.info(f"Verbindungsversuch {bms._reconnect_count + 1}/{MAX_RECONNECT_ATTEMPTS}")
                if not await bms.connect():
                    bms._reconnect_count += 1
                    await asyncio.sleep(min(reconnect_delay * bms._reconnect_count, 30))
                    continue
                reconnect_delay = 2  # Reset nach erfolgreicher Verbindung

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
                        logger.info(f"MQTT Publishes an {mqtt.host}:{mqtt.port}/{mqtt.topic}")
                        first = False
                    logger.info(f"[{bms.data.timestamp}] Published: "
                               f"{bms.data.total_voltage:.2f}V "
                               f"{bms.data.current:+.2f}A "
                               f"SOC:{bms.data.state_of_charge:.1f}%")
                else:
                    bms.print_full()
                    first = False
            else:
                logger.warning("Lesen fehlgeschlagen, reconnect...")
                await bms.disconnect()
                await asyncio.sleep(2)

            # Intervall warten
            for _ in range(interval * 10):
                if not bms._running:
                    break
                await asyncio.sleep(0.1)

    except KeyboardInterrupt:
        pass
    finally:
        await bms.disconnect()
        if mqtt:
            mqtt.disconnect()


def parse_interval(arg: str) -> int:
    """Parst Intervall-String (z.B. '5s', '1m', '30')."""
    arg = arg.lower().strip()
    if arg.endswith('s'):
        return int(arg[:-1])
    elif arg.endswith('m'):
        return int(arg[:-1]) * 60
    else:
        return int(arg)


def create_config_template() -> None:
    """Erstellt eine Beispiel-Konfigurationsdatei."""
    template = {
        "mac_address": "60:6E:41:16:73:DC",
        "default_interval": 20,
        "mqtt_host": "localhost",
        "mqtt_port": 1883,
        "mqtt_topic": "seplosbms",
        "mqtt_user": None,
        "mqtt_pass": None,
        "ble_timeout": 15.0,
        "response_timeout": 5.0,
        "max_reconnect_attempts": 5
    }
    
    path = Path(__file__).parent / "seplos_config.json"
    with open(path, 'w') as f:
        json.dump(template, f, indent=2)
    print(f"Konfigurationsvorlage erstellt: {path}")


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Seplos BMS 10E Bluetooth Reader (Verbesserte Version)",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Beispiele:
  python3 seplos_bms_ble.py                    # Einmalige Abfrage
  python3 seplos_bms_ble.py monitor            # Monitor alle 20s
  python3 seplos_bms_ble.py monitor 5          # Monitor alle 5s
  python3 seplos_bms_ble.py monitor 1m         # Monitor alle 60s
  python3 seplos_bms_ble.py json               # Einmalig JSON
  python3 seplos_bms_ble.py mqtt               # MQTT alle 20s
  python3 seplos_bms_ble.py mqtt 10            # MQTT alle 10s
  python3 seplos_bms_ble.py mqtt --topic mybms # Custom Topic
  python3 seplos_bms_ble.py --create-config    # Erstellt config.json

Umgebungsvariablen:
  SEPLOS_MAC, SEPLOS_MQTT_HOST, SEPLOS_MQTT_PORT, 
  SEPLOS_MQTT_TOPIC, SEPLOS_MQTT_USER, SEPLOS_MQTT_PASS
        """
    )

    parser.add_argument("mode", nargs="?", default="full",
                       choices=["full", "compact", "json", "monitor", "mqtt"],
                       help="Betriebsmodus (default: full)")
    parser.add_argument("interval", nargs="?", type=str, default=None,
                       help="Abfrageintervall (z.B. 5, 10s, 1m)")
    parser.add_argument("--host", default=CONFIG["mqtt_host"],
                       help=f"MQTT Broker Host (default: {CONFIG['mqtt_host']})")
    parser.add_argument("--port", type=int, default=CONFIG["mqtt_port"],
                       help=f"MQTT Broker Port (default: {CONFIG['mqtt_port']})")
    parser.add_argument("--topic", default=CONFIG["mqtt_topic"],
                       help=f"MQTT Topic Prefix (default: {CONFIG['mqtt_topic']})")
    parser.add_argument("--user", default=CONFIG["mqtt_user"],
                       help="MQTT Benutzername")
    parser.add_argument("--pass", dest="password", default=CONFIG["mqtt_pass"],
                       help="MQTT Passwort")
    parser.add_argument("--mac", default=CONFIG["mac_address"],
                       help=f"BLE MAC-Adresse (default: {CONFIG['mac_address']})")
    parser.add_argument("--debug", action="store_true",
                       help="Debug-Logging aktivieren")
    parser.add_argument("--create-config", action="store_true",
                       help="Erstellt eine Beispiel-Konfigurationsdatei")

    args = parser.parse_args()

    if args.create_config:
        create_config_template()
        sys.exit(0)

    if args.debug:
        logging.getLogger().setLevel(logging.DEBUG)

    interval = CONFIG["default_interval"]
    if args.interval:
        try:
            interval = parse_interval(args.interval)
        except ValueError:
            logger.error(f"Ungültiges Intervall: {args.interval}")
            sys.exit(1)

    bms = SeplosBmsBle(args.mac)
    mqtt = None

    if args.mode == "mqtt":
        mqtt = MqttPublisher(args.host, args.port, args.topic, args.user, args.password)
        if not asyncio.run(mqtt.connect()):
            logger.error("MQTT Verbindung fehlgeschlagen")
            sys.exit(1)
        try:
            asyncio.run(run_monitor(bms, interval, "mqtt", mqtt))
        finally:
            mqtt.disconnect()
    elif args.mode == "monitor":
        asyncio.run(run_monitor(bms, interval, "compact"))
    elif args
	.mode == "json":
        if args.interval:
            asyncio.run(run_monitor(bms, interval, "json"))
        else:
            success = asyncio.run(run_once(bms, "json"))
            if not success:
                sys.exit(1)
    elif args.mode == "compact":
        success = asyncio.run(run_once(bms, "compact"))
        if not success:
            sys.exit(1)
    else:
        success = asyncio.run(run_once(bms, "full"))
        if not success:
            sys.exit(1)


if __name__ == "__main__":
    main()
	