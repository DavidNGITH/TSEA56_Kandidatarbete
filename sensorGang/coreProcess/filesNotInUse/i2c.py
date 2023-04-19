import smbus
import time
import multiprocessing

bus = smbus.SMBus(1)

motorAdress = 0x4a
sensorAdress = 0x6a

def sendGetI2C(qMotors : multiprocessing.Queue, qData : multiprocessing.Queue, qStatus):
    timeElapsed = time.time()
    print("Start sendgeti2c")

    status = qStatus.value

    while status:
        if not qMotors.empty():
            addressData = qMotors.get()
            if addressData == 100:
                stop()
                return
            
            print("Sending to buss")
            print(addressData[0])
            print(addressData[1])
            bus.write_byte_data(motorAdress,addressData[0],addressData[1])

        if time.time() - timeElapsed > 2:
            try:
                qData.put((0,bus.read_byte_data(sensorAdress, 0)))
                qData.put((1,bus.read_byte_data(sensorAdress, 1)))
            except:
                print("Couldn't read i2c")
            timeElapsed = time.time()

            status = qStatus.value
            time.sleep(0.01)
        
    stop()

def stop():
    print("I2C stopped")
    bus.write_byte_data(motorAdress,1,50)
    bus.write_byte_data(motorAdress,0,0)
