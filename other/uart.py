import time
import board
import busio
import struct
import math

UART_BAUDRATE = 115200

class UARTRecever:
    def __init__(self, tx_pin=board.GP0, rx_pin=board.GP1, baudrate=UART_BAUDRATE):
        self.uart = busio.UART(tx_pin, rx_pin, baudrate=baudrate, timeout=0.1)
        print("UART receiver initialized")

    def send_go_signal(self):
        # Send "go" signal (0x01) to master
        self.uart.write(bytes([0x01]))
        print("Sent 'go' signal (0x01)")

    def wait_for_data(self, expected_bytes=24):
        while True:
            data = self.uart.read(expected_bytes)
            if data and len(data) == expected_bytes:
                x, y, z, ox, oy, _ = struct.unpack('ffffff', data)
                angle = math.atan2(oy, ox)
                print(f"Received data: ({x}, {y}, {z}), ({angle} degrees)")
                return (x, y, z, angle)
            else:
                print("Invalid data received:", len(data) if data else "None")
            time.sleep(0.01)