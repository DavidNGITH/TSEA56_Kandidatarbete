import smbus


class I2C():
    def __init__(self):
        print("i2c init")
        self.bus = smbus.SMBus(1)
        
        self.motorAdress = 0x4a
        self.sensorAdress = 0x6a
        
        
    def send(self, offsetAndMessage):
        self.bus.write_byte_data(self.motorAdress,offsetAndMessage[0],offsetAndMessage[1])
        
    
    def get(self):
        return ([(0,self.bus.read_byte_data(self.sensorAdress, 0)), (1,self.bus.read_byte_data(self.sensorAdress, 1))])

        
        