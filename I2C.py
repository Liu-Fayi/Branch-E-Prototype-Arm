import time
import board
import busio

SLAVE_ADDRESS = 0x42

class I2C:
    def __init__(self, scl_pin=board.GP17, sda_pin=board.GP16, address=SLAVE_ADDRESS):
        self.i2c = busio.I2C(scl=scl_pin, sda=sda_pin)
        self.address = address
        self.i2c.configure(address=address, slave=True)
        print(f"I2C slave initialized at address 0x{address:02x}")

    def send_go_signal(self):
        while not self.i2c.try_lock():
            pass
        try:
            self.i2c.writeto(self.address, bytes([0x01]))
            print("Sent 'go' signal (0x01)")
        except OSError as e:
            print(f"Failed to send 'go' signal: {e}")
        finally:
            self.i2c.unlock()

    def wait_for_data(self, expected_bytes=24):
        while True:
            if self.i2c.requested():
                buffer = self.i2c.read(expected_bytes)
                if buffer and len(buffer) == expected_bytes:
                    return buffer
                else:
                    print("Invalid data received:", len(buffer) if buffer else "None")
            time.sleep(0.01)

