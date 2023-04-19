"""Handles the manual driving."""
import paho.mqtt.client as mqtt
import multiprocessing
import time
import numpy as np
import i2cHandle
from record import record

MQTT_TOPIC = [("stop", 0), ("ping", 0), ("steering", 0),
              ("speed", 0), ("breaking", 0)]
MQTT_TOPIC_UNSUB = ["stop", "ping", "steering", "speed", "breaking"]


class Manual():
    """Class for manual driving."""

    def __init__(self, mqttClient: mqtt.Client(), timeOut, resolution,
                 framerate, recordMode):
        """Set up variables and processes."""
        self.timeOut = timeOut

        # Record mode
        self.recordMode = recordMode
        if recordMode:
            self.recordMode = recordMode
            self.statusVideo = multiprocessing.Value('i', 1)
            self.pRecording = multiprocessing.Process(target=record,
                                                      args=(self.statusVideo,
                                                            resolution,
                                                            framerate))
            self.pRecording.start()

        # Init MQTT
        self.mqttClient = mqttClient
        self.mqttClient.on_message = self.onMessage
        self.mqttClient.subscribe(MQTT_TOPIC)

        # Init multiprocessing
        self.initMultiproccessing()

    def onMessage(self, client, userdata, message):
        """MQTT callback function."""
        print("I got mail in manual")
        try:
            m = int(message.payload.decode("utf-8"))
            t = message.topic
            self.qMessage.put((t, m))
        except Exception as e:
            print("Couldn't read mqtt message")
            print(e)

    def handleMessage(self, qMQTT, qI2C, status):
        """Handle messages from other processes."""
        print("In handleMessage manual!")
        I2C_proc = i2cHandle.I2C()
        pingTime = time.time()
        i2cTimeElapsed = time.time()

        while status.value:
            if not qMQTT.empty():
                print("qMessage not empty")
                message = qMQTT.get()
                if message[0] == "stop":
                    print("Recived stop in manual")
                    try:
                        I2C_proc.close()
                    except Exception:
                        print("Couldn't read i2c")
                    self.stop()
                    return

                elif message[0] == "ping":
                    print("Recived ping in manual")
                    pingTime = time.time()

                elif message[0] == "steering":
                    print("Recived steering data in manual")
                    I2C_proc.send((1, message[1]))

                elif message[0] == "speed":
                    print("Recived speed data in manual")
                    I2C_proc.send((0, message[1]))

                elif message[0] == "breaking":
                    print("Recived breaking data in manual")
                    I2C_proc.send((2, message[1]))

            # Time out checker
            if time.time() - pingTime > self.timeOut:
                print("Timed out, stopping in manual")
                self.stop()
                return

            # Get sensor data
            if time.time() - i2cTimeElapsed > 1:
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
        """Stop processes."""
        print("In stop manual")

        if self.recordMode:
            self.statusVideo.value = 0

        self.statusHandleMessage.value = 0
        self.mqttClient.unsubscribe(MQTT_TOPIC_UNSUB)

    def mainLoop(self):
        """Publish data to MQTT broker."""
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

    def initMultiprocessing(self):
        """Initiate of processes."""
        self.qMessage = multiprocessing.Queue()
        self.qData = multiprocessing.Queue()

        self.statusHandleMessage = multiprocessing.Value('i', 1)

        self.p1 = multiprocessing.Process(target=self.handleMessage, args=(
            self.qMessage, self.qData, self.statusHandleMessage))

        self.p1.start()
