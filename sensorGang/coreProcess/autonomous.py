"""Handles the autonomous driving."""
from compVision import compVision
import multiprocessing
import time
import paho.mqtt.client as mqtt
import i2cHandle
import numpy as np
from PD_reg import PDcontroller


MQTT_TOPIC = [("stop", 0), ("ping", 0), ("speed", 0),
              ("PD/Kp", 0), ("PD/kd", 0)]


class Autonomous():
    """Class for autonomous driving."""

    def __init__(self, mqttClient: mqtt.Client(),
                 timeOut, roiPerc, resolution=(640, 480)):
        """Set up variables and processes."""
        self.timeOut = timeOut

        # PD-Controller        Kp  Kd
        self.PD = PDcontroller(15, 10)

        # Init compVision
        self.resolution = resolution
        self.laneData = compVision(self.PD, roiPerc, self.resolution)
        self.object = False

        # Init MQTT
        self.mqttClient = mqttClient
        self.mqttClient.on_message = self.onMessage
        self.mqttClient.subscribe(MQTT_TOPIC)

        # Init multiprocessing
        self.initMultiproccessing()

    def onMessage(self, client, userdata, message):
        """MQTT callback function."""
        try:
            m = int(message.payload.decode("utf-8"))
            t = message.topic
            self.qMessageMQTT.put((t, m))
        except Exception:
            print("Couldn't read mqtt message")

    def handleMessage(self, qMessageMQTT, qI2CDataRecived,
                      qSteering, qSpeed, status):
        """Handle messages from other processes."""
        print("In handleMessage autonomous!")
        I2C_proc = i2cHandle.I2C()
        time.sleep(4)
        i2cTimeElapsed = 0
        sendI2C = 0
        pingTime = time.time()

        # I2C_proc.send((0, 100))
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
                    except Exception:
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

            if time.time() - i2cTimeElapsed > 0.1:
                try:
                    data = I2C_proc.get()
                    if (int(data[0][1]) < 40) & (self.object is False):
                        print("Obstacle")
                        I2C_proc.send((2, 1))
                        self.object = True
                    elif (int(data[0][1]) >= 40) & (self.object):
                        print("Release")
                        I2C_proc.send((2, 0))
                        self.object = False
                    if time.time() - sendI2C > 1:
                        qI2CDataRecived.put(data[0])
                        qI2CDataRecived.put(data[1])
                        sendI2C = time.time()

                except Exception as e:
                    print("Couldn't read i2c")
                    print(e)

                i2cTimeElapsed = time.time()

            time.sleep(0.01)

    def stop(self):
        """Stop processes."""
        time.sleep(0.5)

        # Stopping center offset
        self.statusCenterOffset.value = 0

        # Stopping mqqt handle message
        self.statusHandleMessage.value = 0

        # mqtt disconnect
        self.mqttClient.disconnect()

    def mainLoop(self):
        """Publish data to MQTT broker."""
        while self.statusHandleMessage.value:
            if not self.qI2CDataRecived.empty():
                messageToSend = self.qI2CDataRecived.get()
                if messageToSend[0]:
                    speed = int((messageToSend[1]/10) * 8 * np.pi)
                    # print("Speed: {} cm/s".format(speed))
                    self.mqttClient.publish("data/speed", speed)
                else:
                    distance = int(1.1 * messageToSend[1])
                    # print("Distance: {} cm".format(distance))
                    self.mqttClient.publish("data/distance", distance)

            time.sleep(0.01)

        print("Main loop autonomous stopped")

    def initMultiproccessing(self):
        """Initiate of processes."""
        self.qMessageMQTT = multiprocessing.Queue()     # Incoming MQTT message
        self.qSteering = multiprocessing.Queue()        # Steering data
        # self.qMotors = multiprocessing.Queue()         # Speed data to motors
        self.qI2CDataRecived = multiprocessing.Queue()  # Recived I2C data
        self.qSpeed = multiprocessing.Queue()           # Speed data to motors

        self.statusCenterOffset = multiprocessing.Value('i', 1)
        self.statusHandleMessage = multiprocessing.Value('i', 1)

        self.p1 = multiprocessing.Process(target=self.handleMessage,
                                          args=(self.qMessageMQTT,
                                                self.qI2CDataRecived,
                                                self.qSteering,
                                                self.qSpeed,
                                                self.statusHandleMessage))

        self.p2 = multiprocessing.Process(target=self.laneData.getCenterOffset,
                                          args=(self.qSteering,
                                                self.statusCenterOffset,
                                                self.qSpeed))

        self.p1.start()  # Starts handleMessage process
        self.p2.start()  # Starts getCenterOffset process
