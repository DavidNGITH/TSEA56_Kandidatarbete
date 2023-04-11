import compVision
import multiprocessing
import time
import paho.mqtt.client as mqtt


MQTT_TOPIC = [("stop",0),("ping",0)]


class Autonomous():
    def __init__(self, mqttClient : mqtt.Client(), resolution = (640,480)):
        self.resolution = resolution
        self.laneData = compVision.compVision(self.resolution)
        self.status = True
    
        self.mqttClient = mqttClient
        self.mqttClient.on_message = self.onMessage
        self.mqttClient.subscribe(MQTT_TOPIC)

        self.qMessage = multiprocessing.Queue()
        self.qCenterOffset = multiprocessing.Queue()
        
        self.statusCenterOffset = multiprocessing.Value('i',1)
        self.statusHandleMessage = multiprocessing.Value('i',1)

        self.p1 = multiprocessing.Process(target=self.handleMessage, args=(self.qMessage,self.statusHandleMessage))
        self.p2 = multiprocessing.Process(target=self.laneData.getCenterOffset, args =(self.qCenterOffset,self.statusCenterOffset))

        self.p1.start()
        self.p2.start()

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
        while True:
            if not q.empty():
                print("qMessage not empty")
                if q.get()[0] == "stop":
                    print("Recived stop in autonomous")
                    self.stop()
                    return
                elif q.get()[0] == "ping":
                    print("Recived ping in autonomous")
                    pingTime = time.time()
            
            if time.time() - pingTime > 3000:
                print("Timed out in autonomous")
                self.stop()
                return
            time.sleep(0.01)

    def stop(self):
        #Stop all motors
        self.laneData.stopProcess()
        self.statusCenterOffset.value = 0        
        self.mqttClient.disconnect()
        #self.status = False
        self.statusHandleMessage.value = 0
        
       

    def mainLoop(self):
        while self.statusHandleMessage.value:
            if not self.qCenterOffset.empty():
                centerOffset = self.qCenterOffset.get()
                #Send data to PD-controller
            time.sleep(0.01)
        