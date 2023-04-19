from compVision import compVision
import multiprocessing
import time
import paho.mqtt.client as mqtt
import i2cHandle
import numpy as np
from PD_reg import PDcontroller



MQTT_TOPIC = [("stop",0),("ping",0),("speed",0),("PD/Kp",0),("PD/kd",0)]


class Autonomous():
    def __init__(self, mqttClient : mqtt.Client(), timeOut, roiPerc, resolution = (640,480)):
        self.timeOut = timeOut

        ##PD-Controller        Kp  Kd
        self.PD = PDcontroller(15, 10)
        
        self.resolution = resolution
        self.laneData = compVision(self.PD, roiPerc, self.resolution)
        self.status = True
        self.object = False
    
        self.mqttClient = mqttClient
        self.mqttClient.on_message = self.onMessage
        self.mqttClient.subscribe(MQTT_TOPIC)

        #Incoming MQTT message
        self.qMessageMQTT = multiprocessing.Queue()
        #Steering data to motors
        self.qSteering = multiprocessing.Queue()
        #Speed data to motors
        #self.qMotors = multiprocessing.Queue()
        #Recived I2C data
        self.qI2CDataRecived = multiprocessing.Queue()
        #Speed data to motors
        self.qSpeed = multiprocessing.Queue()

        self.statusCenterOffset = multiprocessing.Value('i',1)
        self.statusHandleMessage = multiprocessing.Value('i',1)


        self.p1 = multiprocessing.Process(target=self.handleMessage, args=(self.qMessageMQTT, self.qI2CDataRecived, self.qSteering, self.qSpeed, self.statusHandleMessage))
        self.p2 = multiprocessing.Process(target=self.laneData.getCenterOffset, args =(self.qSteering, self.statusCenterOffset, self.qSpeed))

        self.p1.start()
        self.p2.start()


    def onMessage(self,client, userdata, message):
        try:
            m = int(message.payload.decode("utf-8"))
            t = message.topic
            self.qMessageMQTT.put((t,m))
        except:
            print("Couldn't read mqtt message")

    def handleMessage (self, qMessageMQTT, qI2CDataRecived, qSteering, qSpeed ,status):
        print("In handleMessage autonomous!")
        I2C_proc = i2cHandle.I2C()
        time.sleep(4)
        i2cTimeElapsed = time.time()
        pingTime = time.time()
        
        #I2C_proc.send((0, 100))
        print("speed")

        
        while status.value:
            if not qMessageMQTT.empty():
                print("qMessage not empty")
                message = qMessageMQTT.get()

                if message[0] == "stop":
                    print("Recived stop in autonomous")
                    try:
                        I2C_proc.send((1, 50))
                        I2C_proc.send((0, 0))
                        I2C_proc.close()
                    except:
                        print("Couldn't read i2c")
                        
                    self.stop()
                    return
                elif message[0] == "ping":
                    print("Recived ping in autonomous")
                    pingTime = time.time()
                    
                elif message[0] == "speed":
                    print("Recived speed data in autonomous")
                    I2C_proc.send((0, message[1]))

                elif message[0] == "PD/Kp":
                    print("Recived Kp data in autonomous")
                    self.PD.updateKp(message[1])

                elif message[0] == "PD/Kd":
                    print("Recived Kd data in autonomous")
                    self.PD.updateKd(message[1])
                    
                
            
            if not qSteering.empty():
#                print("Recived steering")
                steering = qSteering.get()
                
                I2C_proc.send((1, steering))

            if not qSpeed.empty():
                print("Recived speed")
                speed = qSpeed.get()
                
                I2C_proc.send((0, speed))
            
            
            if time.time() - pingTime > self.timeOut:
                print("Timed out in autonomous")
                self.stop()
                return
            
            if time.time() - i2cTimeElapsed > 2:
                try:
                    data = I2C_proc.get()
                    print(type(data[0]))
                    if (int(data[0]) < 10) & self.object:
                        I2C_proc.send((2, 1))
                        self.object = True
                    elif (int(data[0]) >= 10) & self.object:
                        I2C_proc.send((2, 0))
                        self.object = False
                
                    qI2CDataRecived.put(data[0])
                    qI2CDataRecived.put(data[1])
                except Exception as e:
                    print("Couldn't read i2c")
                    print(e)
                    
                    
                i2cTimeElapsed = time.time()
                
            time.sleep(0.01)


    def stop(self):
        #Stop all motors
        #while not self.qMotors.empty():
        #    self.qMotors.get()
    

        #self.qMotors.put(100)
        
        time.sleep(0.5)
        
        # Stopping center offset
        self.statusCenterOffset.value = 0 

        # Stopping mqqt handle message 
        self.statusHandleMessage.value = 0 

        #mqtt disconnect     
        self.mqttClient.disconnect()
        
        
       

    def mainLoop(self):
        while self.statusHandleMessage.value:
            #if not self.qCenterOffset.empty():
                #centerOffset = self.qCenterOffset.get()
                
                #Send data to PD-controller
                
            if not self.qI2CDataRecived.empty():
                messageToSend = self.qI2CDataRecived.get()
                if messageToSend[0]:
                    speed = int((messageToSend[1]/10) * 8 * np.pi)
                    #print("Speed: {} cm/s".format(speed))
                    self.mqttClient.publish("data/speed", speed)
                else:
                    distance = int(1.1 * messageToSend[1])
                    #print("Distance: {} cm".format(distance))
                    self.mqttClient.publish("data/distance", distance)
                
            
            time.sleep(0.01)
        
        print("Main loop autonomous stopped")
        
