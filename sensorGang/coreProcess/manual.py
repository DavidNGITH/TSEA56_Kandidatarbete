import paho.mqtt.client as mqtt
import multiprocessing
import time
from i2c import sendGetI2C

MQTT_TOPIC = [("stop",0),("ping",0),("steering",0),("speed",0)]

class Manual():
    def __init__(self, mqttClient : mqtt.Client()):
        self.status = True
        
        self.mqttClient = mqttClient
        self.mqttClient.on_message = self.onMessage
        self.mqttClient.subscribe(MQTT_TOPIC)

        self.qMessage = multiprocessing.Queue()
        self.qMotors = multiprocessing.Queue()
        self.qData = multiprocessing.Queue()

        self.p1 = multiprocessing.Process(target=self.handleMessage, args=(self.qMessage,))
        self.p2 = multiprocessing.Process(target=sendGetI2C, args=(self.qMotors,self.qData))

        self.topicDic = {
            0 : "speedData",
            1 : "distanceData"
        }


    def onMessage(self,client, userdata, message):
        print("I got mail")
        try:
            m = int(message.payload.decode("utf-8"))
            t = message.topic
            self.qMessage.put((t,m))
        except Exception as e:
            print("Couldn't read mqtt message")
            print(e)


    def handleMessage (self,q):
        pingTime = time.time()
        while True:
            if not q.empty():
                if q.get()[0] == "stop":
                    self.stop()
                    return
                elif q.get()[0] == "ping":
                    pingTime = time.time()
                elif q.get()[0] == "steering":	
                    self.qMotors = (0, q.get()[1])
                elif q.get()[0] == "speed":
                    self.qMotors = (1, q.get()[1])

            if time.time() - pingTime > 2:
                self.stop()
                return
            

    def stop(self):
        #Stop all motors

        #Clear motor data
        while not self.qMotors.empty:
            self.qMotors.get()

        self.qMotors.put(None)

        self.p2.join()

        self.mqttClient.disconnect()
        self.status = False


    def mainLoop(self):
        while self.status:
            if not self.qData.empty():
                messageToSend = self.qData.get()
                self.mqttClient.publish(self.topicDic[messageToSend[0]], messageToSend[1])
