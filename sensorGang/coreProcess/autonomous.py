import compVision
import multiprocessing
import time
import paho.mqtt.client as mqtt
from i2c import sendGetI2C


MQTT_TOPIC = [("stop",0),("ping",0)]


class Autonomous():
    def __init__(self, mqttClient : mqtt.Client(), timeOut, roiPerc, resolution = (640,480)):
        self.timeOut = timeOut
        
        self.resolution = resolution
        self.laneData = compVision.compVision(roiPerc, self.resolution)
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
        self.statusI2C = multiprocessing.Value('i',1)


        self.p1 = multiprocessing.Process(target=self.handleMessage, args=(self.qMessage, self.statusHandleMessage))
        self.p2 = multiprocessing.Process(target=self.laneData.getCenterOffset, args =(self.qCenterOffset, self.statusCenterOffset))
        self.p3 = multiprocessing.Process(target=sendGetI2C, args=(self.qMotors, self.qData, self.statusI2C))

        self.p1.start()
        self.p2.start()
        self.p3.start()


    def onMessage(self,client, userdata, message):
        try:
            m = int(message.payload.decode("utf-8"))
            t = message.topic
            self.qMessage.put((t,m))
        except:
            print("Couldn't read mqtt message")

    def handleMessage (self,q,status):
        print("In handleMessage autonomous!")
        pingTime = time.time()
        while status.value:
            if not q.empty():
                print("qMessage not empty")
                if q.get()[0] == "stop":
                    print("Recived stop in autonomous")
                    print("Handle message autonomous stopped")
                    self.stop()
                    return
                elif q.get()[0] == "ping":
                    print("Recived ping in autonomous")
                    pingTime = time.time()
            
            if time.time() - pingTime > self.timeOut:
                print("Timed out in autonomous")
                self.stop()
                return
            time.sleep(0.01)


    def stop(self):
        #Stop all motors
        while not self.qMotors.empty():
            self.qMotors.get()
    

        self.qMotors.put(100)
        
        time.sleep(0.5)
        
        self.statusI2C.value = 0

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
            time.sleep(0.01)
        
        print("Main loop autonomous stopped")
        