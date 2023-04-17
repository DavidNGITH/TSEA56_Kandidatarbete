from compVision import compVision
import multiprocessing
import time
import paho.mqtt.client as mqtt
import i2cHandle
import numpy as np



MQTT_TOPIC = [("stop",0),("ping",0)]


class Autonomous():
    def __init__(self, mqttClient : mqtt.Client(), timeOut, roiPerc, resolution = (640,480)):
        self.timeOut = timeOut
        
        self.resolution = resolution
        self.laneData = compVision(roiPerc, self.resolution)
        self.status = True
    
        self.mqttClient = mqttClient
        self.mqttClient.on_message = self.onMessage
        self.mqttClient.subscribe(MQTT_TOPIC)

        self.qMessage = multiprocessing.Queue()
        self.qCenterOffset = multiprocessing.Queue()
        self.qMotors = multiprocessing.Queue()
        self.qData = multiprocessing.Queue()

        self.statusCenterOffset = multiprocessing.Value('i',1)
        self.statusHandleMessage = multiprocessing.Value('i',1)


        self.p1 = multiprocessing.Process(target=self.handleMessage, args=(self.qMessage, self.qData, self.statusHandleMessage))
        self.p2 = multiprocessing.Process(target=self.laneData.getCenterOffset, args =(self.qCenterOffset, self.statusCenterOffset))

        self.p1.start()
        self.p2.start()


    def onMessage(self,client, userdata, message):
        try:
            m = int(message.payload.decode("utf-8"))
            t = message.topic
            self.qMessage.put((t,m))
        except:
            print("Couldn't read mqtt message")

    def handleMessage (self, qMQTT, qI2C ,status):
        print("In handleMessage autonomous!")
        I2C_proc = i2cHandle.I2C()
        i2cTimeElapsed = time.time()
        pingTime = time.time()
        while status.value:
            if not qMQTT.empty():
                print("qMessage not empty")
                if qMQTT.get()[0] == "stop":
                    print("Recived stop in autonomous")
                    try:
                        I2C_proc.send((1, 50))
                        I2C_proc.send((0, 0))
                    except:
                        print("Couldn't read i2c")
                        
                    self.stop()
                    return
                elif qMQTT.get()[0] == "ping":
                    print("Recived ping in autonomous")
                    pingTime = time.time()
            
            if time.time() - pingTime > self.timeOut:
                print("Timed out in autonomous")
                self.stop()
                return
            
            if time.time() - i2cTimeElapsed > 2:
                try:
                    data = I2C_proc.get()
                
                    qI2C.put(data[0])
                    qI2C.put(data[1])
                except Exception as e:
                    print("Couldn't read i2c")
                    print(e)
                    
                    
                i2cTimeElapsed = time.time()
                
            time.sleep(0.01)


    def stop(self):
        #Stop all motors
        while not self.qMotors.empty():
            self.qMotors.get()
    

        self.qMotors.put(100)
        
        time.sleep(0.5)
        
        # Stopping center offset
        self.statusCenterOffset.value = 0 

        # Stopping mqqt handle message 
        self.statusHandleMessage.value = 0 

        #mqtt disconnect     
        self.mqttClient.disconnect()
        
        
       

    def mainLoop(self):
        while self.statusHandleMessage.value:
            if not self.qCenterOffset.empty():
                centerOffset = self.qCenterOffset.get()
                #Send data to PD-controller
                
            if not self.qData.empty():
                messageToSend = self.qData.get()
                if messageToSend[0]:
                    speed = int((messageToSend[1]/10) * 8 * np.pi)
                    print("Speed: {} cm/s".format(speed))
                    self.mqttClient.publish("data/speed", speed)
                else:
                    distance = int(1.1 * messageToSend[1])
                    print("Distance: {} cm".format(distance))
                    self.mqttClient.publish("data/distance", distance)
                
            
            time.sleep(0.01)
        
        print("Main loop autonomous stopped")
        