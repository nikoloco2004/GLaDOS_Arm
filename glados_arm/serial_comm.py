"""USB serial line protocol to Arduino Uno R4 WiFi."""

from __future__ import annotations

import time

import serial

from . import config


class ArmSerial:
    def __init__(
        self,
        port: str | None = None,
        baud: int = config.SERIAL_BAUD,
        timeout: float = config.SERIAL_TIMEOUT_S,
    ) -> None:
        self.port = port or config.SERIAL_DEFAULT_PORT
        self.baud = baud
        self.timeout = timeout
        self._ser: serial.Serial | None = None

    def connect(self) -> None:
        if self._ser and self._ser.is_open:
            return
        self._ser = serial.Serial(self.port, self.baud, timeout=self.timeout)
        time.sleep(0.1)
        self._ser.reset_input_buffer()

    def close(self) -> None:
        if self._ser and self._ser.is_open:
            self._ser.close()
        self._ser = None

    def __enter__(self) -> "ArmSerial":
        self.connect()
        return self

    def __exit__(self, *exc) -> None:
        self.close()

    def _require(self) -> serial.Serial:
        if not self._ser or not self._ser.is_open:
            raise RuntimeError("serial not connected")
        return self._ser

    def write_line(self, line: str) -> None:
        s = self._require()
        data = (line.rstrip("\r\n") + "\n").encode("ascii", errors="replace")
        s.write(data)
        s.flush()

    def read_line(self) -> str:
        s = self._require()
        raw = s.readline()
        return raw.decode("ascii", errors="replace").strip()

    def transact(self, line: str, read_lines: int = 1) -> list[str]:
        self.write_line(line)
        out: list[str] = []
        for _ in range(read_lines):
            out.append(self.read_line())
        return out

    def ping(self) -> bool:
        lines = self.transact("PING", 1)
        return len(lines) > 0 and lines[0].strip() == "PONG"


def list_ports_windows_hint() -> str:
    return "On Windows, set SERIAL_DEFAULT_PORT or pass --port COM3 (Device Manager)."
