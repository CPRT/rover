import threading
import serial
from pyubx2 import (
    RTCM3_PROTOCOL,
    UBXReader,
    VALCKSUM,
)


class UbxIoManager:
    """
    Manages serial I/O operations, ensuring thread safety for reading and writing data.

    Attributes:
        lock (threading.Lock): Ensures thread-safe access to the serial port.
        worker (serial.Serial): Serial connection instance.
        ubr (UBXReader): UBXReader instance for parsing RTCM/UBX data.
    """

    def __init__(self, port="/dev/ttyACM0", baud=9600, msg_filter=RTCM3_PROTOCOL):
        """
        Initializes the serial connection and UBXReader.

        Args:
            port (str): Serial port to connect to.
            baud (int): Baud rate for the serial connection.

        Raises:
            RuntimeError: If the serial port cannot be opened.
        """
        self.lock = threading.Lock()
        try:
            self.worker = serial.Serial(port, baud, timeout=1)
        except serial.SerialException as e:
            raise RuntimeError(f"Failed to open serial port {port}: {e}") from e
        self.ubr = UBXReader(
            self.worker,
            protfilter=msg_filter,
            validate=VALCKSUM,
        )

    def read(self) -> tuple:
        """
        Reads data from the serial port using UBXReader.

        Returns:
            tuple: Raw data and parsed data from UBXReader.

        Raises:
            RuntimeError: If reading from the serial port fails.
        """
        with self.lock:
            return self.ubr.read()

    def write(self, data: bytes):
        """
        Writes data to the serial port.

        Args:
            data (bytes): Data to write to the serial port.

        Raises:
            RuntimeError: If writing to the serial port fails.
        """
        with self.lock:
            try:
                self.worker.write(data)
            except Exception as e:
                raise RuntimeError(f"Error writing to serial port: {e}") from e

    def data_available(self) -> bool:
        """
        Checks if there is data available to read from the serial port.

        Returns:
            bool: True if data is available, False otherwise.
        """
        with self.lock:
            return self.worker.in_waiting > 0
