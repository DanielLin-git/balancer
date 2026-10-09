import serial


class UART:
    def __init__(self):
        self.ser = None

    def uart_setup(self):
        self.ser = serial.Serial(
            port="/dev/serial0",
            baudrate=115200,
            timeout=0.01
        )

    def send_servo_command(self, servo_id: int, microsecond: int):
        if self.ser is None or not self.ser.is_open:
            raise RuntimeError("UART is not initialized")

        if not 0 <= servo_id <= 255:
            raise ValueError("servo_id must be in [0, 255]")

        if not 0 <= microsecond <= 65535:
            raise ValueError("microsecond must fit in 16 bits")

        header = b"\xAA\x55"
        cmd = 0x10
        length = 3

        ms_high = (microsecond >> 8) & 0xFF
        ms_low = microsecond & 0xFF

        payload = bytes([servo_id, ms_high, ms_low])
        checksum = cmd ^ length ^ servo_id ^ ms_high ^ ms_low

        packet = (
            header
            + bytes([cmd, length])
            + payload
            + bytes([checksum])
        )

        self.ser.write(packet)

    def close(self):
        if self.ser is not None and self.ser.is_open:
            self.ser.close()