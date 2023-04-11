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
        
        self.p1.start()
        #self.p2.start()

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


    def handleMessage (self,q):
        print("In handleMessage!")
        pingTime = time.time()
        while True:
            if not q.empty():
                print("qMessage not empty")
                message = q.get()
                if message[0] == "stop":
                    print("Recived stop in manual")
                    self.stop()
                    return
                elif message[0] == "ping":
                    print("Recived ping in manual")
                    pingTime = time.time()
                elif message[0] == "steering":
                    print("Recived steering data in manual")
                    self.qMotors = (0, message[1])
                elif message[0] == "speed":
                    print("Recived speed data in manual")
                    self.qMotors = (1, message[1])
            time.sleep(0.01)

            if time.time() - pingTime > 30:
                print("Timed out, stopping")
                self.stop()
                return
            

    def stop(self):
        #Stop all motors
        print("In stop")

        #Clear motor data
        while not self.qMotors.empty():
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
            time.sleep(0.01)