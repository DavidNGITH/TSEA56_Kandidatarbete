import smbus
import time
import multiprocessing

bus = smbus.SMBus(1)

motorAdress = 0x4a
sensorAdress = 0x6a

def sendGetI2C(qMotors : multiprocessing.Queue, qData : multiprocessing.Queue):
    timeElapsed = time.time()
    while True:
        if not qMotors.empty():
            addressData = qMotors.get()
            if addressData == None:
                #Set speed 0 and steering 50
                bus.write_byte_data(motorAdress,1,50)
                bus.write_byte_data(motorAdress,0,0)
                return

            bus.write_byte_data(motorAdress,addressData[0],addressData[1])

        if time.time() - timeElapsed > 2:
            qData.put((0,bus.read_byte_data(sensorAdress, 0)))
            qData.put((1,bus.read_byte_data(sensorAdress, 1)))