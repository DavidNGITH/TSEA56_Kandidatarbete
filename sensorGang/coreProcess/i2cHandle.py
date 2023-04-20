"""I2C Communication."""
import smbus
import time


class I2C():
    """Class for I2C."""

    def __init__(self):
        """Set up variables."""
        print("i2c init")
        self.bus = smbus.SMBus(1)

        self.motorAdress = 0x4a
        self.sensorAdress = 0x6a

    def send(self, offsetAndMessage):
        """Send I2C message, (offset, message)."""
        try:
            self.bus.write_byte_data(
                self.motorAdress, offsetAndMessage[0], offsetAndMessage[1])
        except Exception as e:
            print(e)

    def get(self):
        """Get I2C data, (distance, speed)."""
        return ([(0, self.bus.read_byte_data(self.sensorAdress, 0)),
                 (1, self.bus.read_byte_data(self.sensorAdress, 1))])

    def close(self):
        """Close I2C connection."""
        print(1)
        self.send((0, 0))
        self.send((1, 50))
        self.send((2, 0))
        print(2)

        time.sleep(0.5)
        self.bus.close()
