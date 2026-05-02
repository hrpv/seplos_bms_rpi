#!/usr/bin/env python3
"""
Seplos BMS 10E Bluetooth Reader für Raspberry Pi
Basierend auf: https://github.com/syssi/esphome-seplos-bms

Usage:
    python3 seplos_bms_ble.py                           # Einmalige Abfrage
    python3 seplos_bms_ble.py monitor                   # Monitor alle 20s
    python3 seplos_bms_ble.py monitor 5                 # Monitor alle 5s
    python3 seplos_bms_ble.py json                      # Einmalig JSON
    python3 seplos_bms_ble.py mqtt                      # MQTT mit Topic "seplosbms"
    python3 seplos_bms_ble.py mqtt 10                   # MQTT alle 10s
    python3 seplos_bms_ble.py mqtt --topic mybms        # MQTT mit Topic "mybms"
    python3 seplos_bms_ble.py mqtt --host 192.168.1.50  # MQTT zu anderem Broker
    python3 seplos_bms_ble.py mqtt --host 192.168.1.50 --port 1883 --topic seplosbms
"""

import asyncio
import logging
import struct
import sys
import json
import time
import signal
import argparse
from dataclasses import dataclass, field
from typing import Optional, List, Dict, Any, Tuple
from bleak import BleakClient, BleakError

# --- Logging ---
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger("seplos_bms")

# --- Konfiguration ---
MAC_ADDRESS = "60:6E:41:16:73:DC"
DEFAULT_INTERVAL = 20
DEFAULT_MQTT_HOST = "localhost"
DEFAULT_MQTT_PORT = 1883
DEFAULT_MQTT_TOPIC = "seplosbms"
MAX_RECONNECT_ATTEMPTS = 3
RECONNECT_DELAY = 5  # Sekunden zwischen Reconnect-Versuchen

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

MAX_RESPONSE_SIZE = 200


# ---------------------------------------------------------------------------
# CRC
# ---------------------------------------------------------------------------

def crc_xmodem(data: bytes) -> int:
    """Berechnet CRC-CCITT (XModem) über die gegebenen Bytes."""
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


# ---------------------------------------------------------------------------
# Alarm-Definitionen
# ---------------------------------------------------------------------------

ALARM_MESSAGES: List[List[str]] = [
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

# ---------------------------------------------------------------------------
# Lookup-Tabellen (zentralisiert)
# ---------------------------------------------------------------------------

_CAN_PROTOCOL_MAP: Dict[int, str] = {
    0x00: "Unset", 0x01: "Pylontech", 0x02: "Growatt",
    0x03: "Victron", 0x04: "SMA", 0x05: "GINL", 0x06: "Studer",
}

_RS485_PROTOCOL_MAP: Dict[int, str] = {
    0x00: "Unset", 0x01: "Pylontech", 0x02: "Growatt",
    0x03: "Voltronic", 0x04: "Sofar", 0x05: "Luxpowertek", 0x06: "Studer",
}

_BATTERY_TYPE_MAP: Dict[int, str] = {
    0x46: "LFP", 0x47: "NCM", 0x48: "LCO", 0x49: "LTO", 0x4A: "Reserved",
}


def _lookup(mapping: Dict[int, str], value: int) -> str:
    return mapping.get(value, f"Unknown(0x{value:02X})")


def interpret_can_protocol(value: int) -> str:
    return _lookup(_CAN_PROTOCOL_MAP, value)


def interpret_rs485_protocol(value: int) -> str:
    return _lookup(_RS485_PROTOCOL_MAP, value)


def interpret_battery_type(value: int) -> str:
    return _lookup(_BATTERY_TYPE_MAP, value)


# ---------------------------------------------------------------------------
# Hilfsfunktionen
# ---------------------------------------------------------------------------

def seplos_get_16bit(data: bytes, offset: int) -> int:
    """Liest zwei Bytes big-endian aus `data` an `offset`."""
    if offset + 1 >= len(data):
        raise IndexError(f"seplos_get_16bit: offset {offset} out of range for data of length {len(data)}")
    return (data[offset] << 8) | data[offset + 1]


def kelvin_to_celsius(val: int) -> float:
    return round(val * 0.1 - 273.15, 1)


def decode_alarms(alarm_bytes: List[int]) -> List[str]:
    alarms: List[str] = []
    for event_idx, byte_val in enumerate(alarm_bytes):
        if byte_val == 0 or event_idx >= len(ALARM_MESSAGES):
            continue
        for bit in range(8):
            if byte_val & (1 << bit):
                alarms.append(ALARM_MESSAGES[event_idx][bit])
    return alarms


# ---------------------------------------------------------------------------
# Datenmodelle
# ---------------------------------------------------------------------------

@dataclass
class CellData:
    voltage: float = 0.0
    balancing: bool = False
    disconnected: bool = False

    def to_dict(self) -> Dict[str, Any]:
        return {
            "voltage": round(self.voltage, 3),
            "balancing": self.balancing,
            "disconnected": self.disconnected,
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


# ---------------------------------------------------------------------------
# BMS Klasse
# ---------------------------------------------------------------------------

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

    # ------------------------------------------------------------------
    # Protokoll
    # ------------------------------------------------------------------

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

    def _notification_handler(self, sender: Any, data: bytearray) -> None:
        # Neues Frame beginnt mit Start-Byte → Puffer zurücksetzen
        if len(data) >= 1 and data[0] == SEPLOS_PKT_START:
            self.frame_buffer.clear()

        self.frame_buffer.extend(data)

        if len(self.frame_buffer) < 7:
            return

        data_len = (self.frame_buffer[5] << 8) | self.frame_buffer[6]
        frame_len = 7 + data_len + 2 + 1  # header + payload + CRC + end-byte

        if frame_len > MAX_RESPONSE_SIZE:
            log.warning("Frame zu groß (%d Bytes) – Puffer verworfen", frame_len)
            self.frame_buffer.clear()
            return

        if len(self.frame_buffer) < frame_len:
            return  # Noch nicht vollständig

        if self.frame_buffer[frame_len - 1] != SEPLOS_PKT_END:
            log.debug("Ungültiges End-Byte – Puffer verworfen")
            self.frame_buffer.clear()
            return

        computed_crc = crc_xmodem(self.frame_buffer[1:frame_len - 3])
        remote_crc = (self.frame_buffer[frame_len - 3] << 8) | self.frame_buffer[frame_len - 2]

        if computed_crc != remote_crc:
            log.warning("CRC-Fehler (berechnet 0x%04X, empfangen 0x%04X)", computed_crc, remote_crc)
        else:
            try:
                self._decode_frame(bytes(self.frame_buffer[:frame_len]))
            except Exception:
                log.exception("Fehler beim Dekodieren des Frames")

        self.frame_buffer.clear()
        self._notification_event.set()

    def _decode_frame(self, data: bytes) -> None:
        function = data[3]
        decoder = {
            SEPLOS_CMD_GET_SINGLE_MACHINE_DATA: self._decode_single_machine_data,
            SEPLOS_CMD_GET_MANUFACTURER_INFO: self._decode_manufacturer_info,
            SEPLOS_CMD_GET_SETTINGS: self._decode_settings,
            SEPLOS_CMD_GET_PARALLEL_DATA: self._decode_parallel_data,
        }.get(function)

        if decoder:
            decoder(data)
        else:
            log.debug("Unbekannte Funktion: 0x%02X", function)

    # ------------------------------------------------------------------
    # Decoder
    # ------------------------------------------------------------------

    def _decode_single_machine_data(self, data: bytes) -> None:
        MIN_LENGTH = 60
        if len(data) < MIN_LENGTH:
            log.warning("_decode_single_machine_data: Frame zu kurz (%d < %d)", len(data), MIN_LENGTH)
            return

        try:
            cells = data[9]
            offset_cells = 10

            self.data.cells = []
            total_v = 0.0
            min_v, max_v = float("inf"), float("-inf")
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

            # Zell-Statistiken
            self.data.min_cell_voltage = min_v if cells > 0 else 0.0
            self.data.max_cell_voltage = max_v if cells > 0 else 0.0
            self.data.min_voltage_cell = min_c
            self.data.max_voltage_cell = max_c
            self.data.delta_cell_voltage = round(max_v - min_v, 3) if cells > 0 else 0.0
            self.data.average_cell_voltage = round(total_v / cells, 3) if cells > 0 else 0.0

            # Temperaturen
            offset_temps = offset_cells + cells * 2
            temps_count = data[offset_temps]
            cell_temps = max(0, temps_count - 2)

            self.data.temperatures = []
            total_temp = 0.0
            for i in range(min(cell_temps, 8)):
                t = kelvin_to_celsius(seplos_get_16bit(data, offset_temps + 1 + i * 2))
                self.data.temperatures.append(t)
                total_temp += t

            self.data.ambient_temperature = kelvin_to_celsius(
                seplos_get_16bit(data, offset_temps + 1 + cell_temps * 2)
            )
            self.data.mosfet_temperature = kelvin_to_celsius(
                seplos_get_16bit(data, offset_temps + 3 + cell_temps * 2)
            )
            self.data.average_cell_temperature = round(total_temp / cell_temps, 1) if cell_temps > 0 else 0.0

            # Hauptdaten
            offset_main = 7 + 3 + (cells * 2) + 1 + (temps_count * 2)

            self.data.current = round(
                struct.unpack(">h", data[offset_main: offset_main + 2])[0] * 0.01, 2
            )
            self.data.total_voltage = round(seplos_get_16bit(data, offset_main + 2) * 0.01, 2)
            self.data.power = round(self.data.total_voltage * self.data.current, 1)
            self.data.capacity_remaining = round(seplos_get_16bit(data, offset_main + 4) * 0.01, 1)
            self.data.battery_capacity = round(seplos_get_16bit(data, offset_main + 7) * 0.01, 1)
            self.data.state_of_charge = round(seplos_get_16bit(data, offset_main + 9) * 0.1, 1)
            self.data.nominal_capacity = round(seplos_get_16bit(data, offset_main + 11) * 0.01, 1)
            self.data.charging_cycles = seplos_get_16bit(data, offset_main + 13)
            self.data.state_of_health = round(seplos_get_16bit(data, offset_main + 15) * 0.1, 1)
            self.data.port_voltage = round(seplos_get_16bit(data, offset_main + 17) * 0.01, 2)

            # Status-Bytes
            alarm_status_offset = offset_main + 19 + cells + temps_count + 2

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
            alarm_bytes = [
                data[alarm_offset + i]
                for i in range(min(custom_alarms, 8))
                if alarm_offset + i < len(data) - 3
            ]
            self.data.alarm_bitmasks = alarm_bytes
            self.data.alarms = decode_alarms(alarm_bytes)

            # Balancing-/Disconnect-Flags pro Zelle
            balancing_offset = alarm_offset + custom_alarms
            disc_offset = balancing_offset + (cells + 7) // 8

            for i in range(cells):
                bal_byte_idx = balancing_offset + i // 8
                if bal_byte_idx < len(data):
                    self.data.cells[i].balancing = bool(data[bal_byte_idx] & (1 << (i % 8)))

                disc_byte_idx = disc_offset + i // 8
                if disc_byte_idx < len(data):
                    self.data.cells[i].disconnected = bool(data[disc_byte_idx] & (1 << (i % 8)))

        except (IndexError, struct.error) as e:
            log.error("Fehler beim Dekodieren der Maschinen-Daten: %s", e)

    def _decode_manufacturer_info(self, data: bytes) -> None:
        MIN_LENGTH = 45
        if len(data) < MIN_LENGTH:
            log.warning("_decode_manufacturer_info: Frame zu kurz (%d < %d)", len(data), MIN_LENGTH)
            return
        try:
            self.data.device_model = data[7:27].decode("ascii", errors="ignore").strip()
            self.data.hardware_version = data[27:37].decode("ascii", errors="ignore").strip()
            self.data.software_version = f"{data[37]}.{data[38]}"
            self.data.can_protocol = interpret_can_protocol(data[39])
            self.data.rs485_protocol = interpret_rs485_protocol(data[40])
            self.data.battery_type = interpret_battery_type(data[41])
        except IndexError as e:
            log.error("Fehler beim Dekodieren der Hersteller-Info: %s", e)

    def _decode_settings(self, data: bytes) -> None:
        """Platzhalter – BMS-Einstellungen werden noch nicht ausgewertet."""
        pass

    def _decode_parallel_data(self, data: bytes) -> None:
        """Platzhalter – Parallel-Daten werden noch nicht ausgewertet."""
        pass

    # ------------------------------------------------------------------
    # BLE-Kommunikation
    # ------------------------------------------------------------------

    async def _send_command(self, function: int, payload: bytes = b"") -> bool:
        if not self._connected or self.client is None:
            return False
        try:
            cmd = self._build_command(function, payload)
            log.debug("SEND 0x%02X: %s", function, cmd.hex())
            await self.client.write_gatt_char(SEPLOS_BMS_CONTROL_CHAR_UUID, cmd, response=False)
            return True
        except (BleakError, Exception) as e:
            log.error("Sendefehler: %s", e)
            return False

    async def _wait_for_response(self, timeout: float = 5.0) -> bool:
        try:
            await asyncio.wait_for(self._notification_event.wait(), timeout=timeout)
            self._notification_event.clear()
            return True
        except asyncio.TimeoutError:
            log.warning("Timeout beim Warten auf Antwort")
            return False

    async def connect(self) -> bool:
        log.info("Verbinde mit %s ...", self.mac_address)
        try:
            self.client = BleakClient(self.mac_address, timeout=10.0)
            await self.client.connect()
            if not self.client.is_connected:
                log.error("Gerät nicht verbunden")
                return False
            self._connected = True
            self.data.connected = True
            log.info("Verbunden")
            await self.client.start_notify(SEPLOS_BMS_NOTIFY_CHAR_UUID, self._notification_handler)
            log.info("Benachrichtigungen aktiv")
            await asyncio.sleep(0.5)
            return True
        except (BleakError, Exception) as e:
            log.error("Verbindungsfehler: %s", e)
            return False

    async def disconnect(self) -> None:
        if not self._connected or self.client is None:
            return
        self._connected = False
        self.data.connected = False
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
        log.info("Getrennt")

    async def read_all_data(self) -> bool:
        if not self._connected:
            return False
        async with self._command_lock:
            for function, payload in COMMAND_QUEUE:
                sent = await self._send_command(function, payload)
                if sent:
                    await self._wait_for_response(timeout=5.0)
                await asyncio.sleep(0.3)
        return True

    # ------------------------------------------------------------------
    # Ausgabe
    # ------------------------------------------------------------------

    def print_compact(self) -> None:
        d = self.data
        sys_flags = (
            ("D" if d.system_discharge else "")
            + ("C" if d.system_charge else "")
            + ("F" if d.system_float_charge else "")
            + ("S" if d.system_standby else "")
        )
        print(
            f"[{d.timestamp[11:19]}] "
            f"{d.total_voltage:.2f}V {d.current:+.2f}A {d.power:+.1f}W "
            f"SOC:{d.state_of_charge:.1f}% "
            f"Cells:{d.min_cell_voltage:.3f}-{d.max_cell_voltage:.3f}V "
            f"Δ{d.delta_cell_voltage:.3f}V "
            f"T:{d.average_cell_temperature:.1f}°C "
            f"D:{'ON' if d.discharge_switch else 'off'}|C:{'ON' if d.charge_switch else 'off'} "
            f"Sys:{sys_flags or '-'} "
            f"Alarms:{len(d.alarms)}"
        )

    def print_full(self) -> None:
        d = self.data
        sep = "=" * 60
        print(f"\n{sep}")
        print("SEPLOS BMS 10E")
        print(sep)

        print(f"\nModel  : {d.device_model}")
        print(f"HW/SW  : {d.hardware_version} / {d.software_version}")
        print(f"Typ    : {d.battery_type} | CAN: {d.can_protocol} | RS485: {d.rs485_protocol}")

        print(f"\n--- Zellen ({len(d.cells)}) ---")
        for i, c in enumerate(d.cells):
            flags = ""
            if c.balancing:
                flags += " [BAL]"
            if c.disconnected:
                flags += " [DISC]"
            print(f"  {i + 1:2d}: {c.voltage:.3f} V{flags}")
        if d.cells:
            print(f"  Min  : {d.min_cell_voltage:.3f} V  (Zelle {d.min_voltage_cell})")
            print(f"  Max  : {d.max_cell_voltage:.3f} V  (Zelle {d.max_voltage_cell})")
            print(f"  Delta: {d.delta_cell_voltage:.3f} V | Ø {d.average_cell_voltage:.3f} V")

        print("\n--- Temperaturen ---")
        for i, t in enumerate(d.temperatures):
            print(f"  T{i + 1}: {t:.1f} °C")
        print(f"  Umgebung: {d.ambient_temperature:.1f} °C | MOSFET: {d.mosfet_temperature:.1f} °C")
        print(f"  Ø Zelle : {d.average_cell_temperature:.1f} °C")

        print("\n--- Hauptwerte ---")
        print(f"  Spannung : {d.total_voltage:.2f} V")
        print(f"  Strom    : {d.current:.2f} A")
        print(f"  Leistung : {d.power:.1f} W")
        print(f"  SOC      : {d.state_of_charge:.1f} %")
        print(f"  Kapazität: {d.capacity_remaining:.1f} Ah / {d.battery_capacity:.1f} Ah")
        print(f"  Zyklen   : {d.charging_cycles} | SOH: {d.state_of_health:.1f} % | Port: {d.port_voltage:.2f} V")

        print("\n--- Schalter ---")
        print(f"  Entladen : {'EIN' if d.discharge_switch else 'AUS'} | "
              f"Laden: {'EIN' if d.charge_switch else 'AUS'}")
        print(f"  Strom-Limit: {'EIN' if d.current_limit_switch else 'AUS'} | "
              f"Heizung: {'EIN' if d.heating_switch else 'AUS'}")

        print("\n--- Systemstatus ---")
        print(f"  Entladen: {d.system_discharge} | Laden: {d.system_charge} | Float: {d.system_float_charge}")
        print(f"  Standby : {d.system_standby} | Abgeschaltet: {d.system_shutdown}")

        print("\n--- Alarme ---")
        if d.alarms:
            for a in d.alarms:
                print(f"  ! {a}")
        else:
            print("  Keine")

        print(f"\n{sep}\n")

    def print_json(self) -> None:
        print(json.dumps(self.data.to_dict(), indent=2, ensure_ascii=False))

    def stop(self) -> None:
        self._running = False


# ---------------------------------------------------------------------------
# MQTT Publisher
# ---------------------------------------------------------------------------

class MqttPublisher:
    def __init__(self, host: str, port: int, topic: str):
        self.host = host
        self.port = port
        self.topic = topic
        self._connected = False

    async def connect(self) -> bool:
        try:
            import paho.mqtt.publish as publish
            publish.single(
                f"{self.topic}/status",
                "online",
                hostname=self.host,
                port=self.port,
                retain=True,
            )
            self._connected = True
            log.info("MQTT verbunden mit %s:%d", self.host, self.port)
            return True
        except ImportError:
            log.error("paho-mqtt nicht installiert – bitte 'pip3 install paho-mqtt' ausführen")
            return False
        except Exception as e:
            log.error("MQTT-Verbindungsfehler: %s", e)
            return False

    def publish(self, data: BmsData) -> None:
        if not self._connected:
            return
        try:
            import paho.mqtt.publish as publish

            d = data
            msgs: List[Tuple[str, str]] = []

            def add(suffix: str, value: Any) -> None:
                msgs.append((f"{self.topic}/{suffix}", str(value)))

            # Hauptwerte
            add("voltage", d.total_voltage)
            add("current", d.current)
            add("power", d.power)
            add("soc", d.state_of_charge)
            add("soh", d.state_of_health)
            add("capacity_remaining", d.capacity_remaining)
            add("capacity_total", d.battery_capacity)
            add("cycles", d.charging_cycles)
            add("port_voltage", d.port_voltage)

            # Temperaturen
            add("temp/ambient", d.ambient_temperature)
            add("temp/mosfet", d.mosfet_temperature)
            add("temp/average_cell", d.average_cell_temperature)
            for i, t in enumerate(d.temperatures):
                add(f"temp/cell_{i + 1}", t)

            # Zellen
            for i, c in enumerate(d.cells):
                add(f"cell/{i + 1}/voltage", c.voltage)
                add(f"cell/{i + 1}/balancing", "ON" if c.balancing else "OFF")
                add(f"cell/{i + 1}/disconnected", "ON" if c.disconnected else "OFF")

            # Zell-Statistiken
            add("cell/min_voltage", d.min_cell_voltage)
            add("cell/max_voltage", d.max_cell_voltage)
            add("cell/delta", d.delta_cell_voltage)
            add("cell/average", d.average_cell_voltage)
            add("cell/min_cell_num", d.min_voltage_cell)
            add("cell/max_cell_num", d.max_voltage_cell)

            # Schalter
            add("switch/discharge", "ON" if d.discharge_switch else "OFF")
            add("switch/charge", "ON" if d.charge_switch else "OFF")
            add("switch/current_limit", "ON" if d.current_limit_switch else "OFF")
            add("switch/heating", "ON" if d.heating_switch else "OFF")

            # Systemstatus
            add("system/discharging", "ON" if d.system_discharge else "OFF")
            add("system/charging", "ON" if d.system_charge else "OFF")
            add("system/float_charge", "ON" if d.system_float_charge else "OFF")
            add("system/standby", "ON" if d.system_standby else "OFF")
            add("system/shutdown", "ON" if d.system_shutdown else "OFF")

            # Alarme
            add("alarms/count", len(d.alarms))
            add("alarms/list", ";".join(d.alarms) if d.alarms else "none")
            for i, mask in enumerate(d.alarm_bitmasks):
                add(f"alarms/event_{i + 1}", f"0x{mask:02X}")

            # Gerät-Info
            add("info/model", d.device_model)
            add("info/hardware", d.hardware_version)
            add("info/software", d.software_version)
            add("info/battery_type", d.battery_type)
            add("info/can_protocol", d.can_protocol)
            add("info/rs485_protocol", d.rs485_protocol)
            add("info/timestamp", d.timestamp)

            # Komplett-JSON
            add("json", json.dumps(d.to_dict(), ensure_ascii=False))

            publish.multiple(
                [(topic, payload, 0, False) for topic, payload in msgs],
                hostname=self.host,
                port=self.port,
            )
        except Exception as e:
            log.error("MQTT-Publish fehlgeschlagen: %s", e)


# ---------------------------------------------------------------------------
# Ausführungs-Logik
# ---------------------------------------------------------------------------

async def run_once(
    bms: SeplosBmsBle,
    output_format: str = "full",
    mqtt: Optional[MqttPublisher] = None,
) -> None:
    try:
        if not await bms.connect():
            log.error("Verbindung fehlgeschlagen")
            return
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


async def run_monitor(
    bms: SeplosBmsBle,
    interval: int,
    output_format: str = "compact",
    mqtt: Optional[MqttPublisher] = None,
) -> None:
    def signal_handler(sig: int, frame: Any) -> None:
        log.info("Monitor wird beendet ...")
        bms.stop()

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    reconnect_attempts = 0

    try:
        if not await bms.connect():
            log.error("Initiale Verbindung fehlgeschlagen")
            return

        first_cycle = True

        while bms._running:
            if await bms.read_all_data():
                bms.data.timestamp = time.strftime("%Y-%m-%dT%H:%M:%S")
                reconnect_attempts = 0  # Reset nach erfolgreichem Lesen

                if mqtt:
                    mqtt.publish(bms.data)

                if output_format == "json":
                    bms.print_json()
                elif output_format == "mqtt":
                    if first_cycle:
                        log.info("MQTT: Publiziere an %s:%d/%s", mqtt.host, mqtt.port, mqtt.topic)
                    log.info(
                        "Publiziert: %s  %.2fV  %+.2fA  SOC:%.1f%%",
                        bms.data.timestamp, bms.data.total_voltage,
                        bms.data.current, bms.data.state_of_charge,
                    )
                elif output_format == "compact":
                    bms.print_compact()
                else:
                    bms.print_full()

                first_cycle = False
            else:
                log.warning("Lesen fehlgeschlagen – Reconnect-Versuch %d/%d",
                            reconnect_attempts + 1, MAX_RECONNECT_ATTEMPTS)
                await bms.disconnect()
                await asyncio.sleep(RECONNECT_DELAY)
                reconnect_attempts += 1

                if reconnect_attempts >= MAX_RECONNECT_ATTEMPTS:
                    log.error("Maximale Reconnect-Versuche erreicht – Abbruch")
                    break

                if not await bms.connect():
                    continue

            # Warte-Schleife mit vorzeitigem Abbruch
            for _ in range(interval * 10):
                if not bms._running:
                    break
                await asyncio.sleep(0.1)

    except KeyboardInterrupt:
        pass
    finally:
        await bms.disconnect()


# ---------------------------------------------------------------------------
# Hilfsfunktionen CLI
# ---------------------------------------------------------------------------

def parse_interval(arg: str) -> int:
    """Parst Intervall-Strings wie '5', '20s', '2m'."""
    arg = arg.lower().strip()
    if arg.endswith("s"):
        return int(arg[:-1])
    elif arg.endswith("m"):
        return int(arg[:-1]) * 60
    return int(arg)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Seplos BMS 10E Bluetooth Reader",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Beispiele:
  python3 seplos_bms_ble.py                           # Einmalige Abfrage (vollständig)
  python3 seplos_bms_ble.py monitor                   # Monitor alle 20s (kompakt)
  python3 seplos_bms_ble.py monitor 5                 # Monitor alle 5s
  python3 seplos_bms_ble.py json                      # Einmalig als JSON
  python3 seplos_bms_ble.py mqtt                      # MQTT alle 20s, Topic: seplosbms
  python3 seplos_bms_ble.py mqtt 10                   # MQTT alle 10s
  python3 seplos_bms_ble.py mqtt --topic mybms
  python3 seplos_bms_ble.py mqtt --host 192.168.1.50
        """,
    )

    parser.add_argument(
        "mode",
        nargs="?",
        default="full",
        choices=["full", "compact", "json", "monitor", "mqtt"],
        help="Betriebsmodus (default: full)",
    )
    parser.add_argument(
        "interval",
        nargs="?",
        type=str,
        default=None,
        help="Abfrageintervall (z.B. 5, 10, 20s, 1m)",
    )
    parser.add_argument(
        "--host",
        default=DEFAULT_MQTT_HOST,
        help=f"MQTT Broker Host (default: {DEFAULT_MQTT_HOST})",
    )
    parser.add_argument(
        "--port",
        type=int,
        default=DEFAULT_MQTT_PORT,
        help=f"MQTT Broker Port (default: {DEFAULT_MQTT_PORT})",
    )
    parser.add_argument(
        "--topic",
        default=DEFAULT_MQTT_TOPIC,
        help=f"MQTT Topic Prefix (default: {DEFAULT_MQTT_TOPIC})",
    )
    parser.add_argument(
        "--verbose", "-v",
        action="store_true",
        help="Ausführliche Debug-Ausgabe",
    )

    args = parser.parse_args()

    if args.verbose:
        logging.getLogger().setLevel(logging.DEBUG)

    interval = DEFAULT_INTERVAL
    if args.interval:
        try:
            interval = parse_interval(args.interval)
        except ValueError:
            log.error("Ungültiges Intervall: %s", args.interval)
            sys.exit(1)

    bms = SeplosBmsBle(MAC_ADDRESS)
    mqtt: Optional[MqttPublisher] = None

    if args.mode == "mqtt":
        mqtt = MqttPublisher(args.host, args.port, args.topic)
        if not asyncio.run(mqtt.connect()):
            log.error("MQTT-Verbindung fehlgeschlagen – Abbruch")
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

    else:  # "full"
        asyncio.run(run_once(bms, "full"))


if __name__ == "__main__":
    main()
