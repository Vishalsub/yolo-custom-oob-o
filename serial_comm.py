import serial
import time

class SerialConnection:
    def __init__(self, port='/dev/ttyUSB0', baudrate=115200, timeout=1):
        try:
            self.ser = serial.Serial(port, baudrate, timeout=timeout)
            time.sleep(2)  # wait for connection
            print(f"[Serial] Connected to {port} at {baudrate} baud.")
        except serial.SerialException as e:
            print(f"[Serial] Failed to connect: {e}")
            self.ser = None

    def send(self, data):
        if self.ser and self.ser.is_open:
            self.ser.write((data + "\n").encode('utf-8'))

    def read_line(self):
        if self.ser and self.ser.is_open:
            return self.ser.readline().decode('utf-8').strip()

    def close(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
