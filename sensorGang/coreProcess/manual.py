import paho.mqtt.client as mqtt
import multiprocessing
import time
from i2c import sendGetI2C
import numpy as np

MQTT_TOPIC = [("stop",0),("ping",0),("steering",0),("speed",0)]

class Manual():
    def __init__(self, mqttClient : mqtt.Client(), timeOut):
        self.timeOut = timeOut
        
        self.mqttClient = mqttClient
        self.mqttClient.on_message = self.onMessage
        self.mqttClient.subscribe(MQTT_TOPIC)

        self.qMessage = multiprocessing.Queue()
        self.qMotors = multiprocessing.Queue()
        self.qData = multiprocessing.Queue()

        self.statusHandleMessage = multiprocessing.Value('i',1)
        self.statusI2C = multiprocessing.Value('i',1)


        self.p1 = multiprocessing.Process(target=self.handleMessage, args=(self.qMessage, self.statusHandleMessage))
        self.p2 = multiprocessing.Process(target=sendGetI2C, args=(self.qMotors, self.qData, self.statusI2C))
        
        self.p1.start()
        self.p2.start()

        self.topicDic = {
            0 : "speedData",
            1 : "distanceData"
        }


    def onMessage(self,client, userdata, message):
        print("I got mail in manual")
        try:
            m = int(message.payload.decode("utf-8"))
            t = message.topic
            self.qMessage.put((t,m))
        except Exception as e:
            print("Couldn't read mqtt message")
            print(e)


    def handleMessage (self,q, status):
        print("In handleMessage manual!")
        pingTime = time.time()
        while status.value:
            if not q.empty():
                print("qMessage not empty")
                message = q.get()
                if message[0] == "stop":
                    print("Recived stop in manual")
                    self.stop()
                    print("Stopping handle message in manual")
                    return
                elif message[0] == "ping":
                    print("Recived ping in manual")
                    pingTime = time.time()
                elif message[0] == "steering":
                    print("Recived steering data in manual")
                    self.qMotors.put((1, message[1]))
                elif message[0] == "speed":
                    print("Recived speed data in manual")
                    self.qMotors.put((0, message[1]))
            time.sleep(0.01)

            if time.time() - pingTime > self.timeOut:
                print("Timed out, stopping in manual")
                self.stop()
                return
        
            

    def stop(self):
        #Stop all motors
        print("In stop manual")

        #Clear motor data
        while not self.qMotors.empty():
            self.qMotors.get()
    

        self.qMotors.put(100)
        
        time.sleep(0.5)
        
        self.statusI2C.value = 0

        self.statusHandleMessage.value = 0

        self.mqttClient.disconnect()


    def mainLoop(self):
        while self.statusHandleMessage.value:
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

        print("Stopping main loop in manual")
