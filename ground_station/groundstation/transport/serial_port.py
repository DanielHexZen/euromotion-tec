"""Thin pyserial wrapper. Same code for the ST-Link VCP and the FT232RL radio (R1)."""

import serial


class PortClosed(Exception):
    """The port went away (cable pulled) or was closed."""


class SerialPort:
    def __init__(self, port: str, baud: int):
        self.name, self.baud = port, baud
        # timeout bounds how long the reader thread takes to notice close().
        self._ser = serial.Serial(port, baud, timeout=0.05, write_timeout=1.0)

    def read(self) -> bytes:
        """Returns whatever arrived (possibly b''). Raises PortClosed when the port is gone."""
        try:
            return self._ser.read(self._ser.in_waiting or 1)
        except (serial.SerialException, OSError, TypeError) as e:
            raise PortClosed(str(e)) from e

    def write(self, data: bytes) -> None:
        try:
            self._ser.write(data)
        except (serial.SerialException, OSError) as e:
            raise PortClosed(str(e)) from e

    def close(self) -> None:
        self._ser.close()
