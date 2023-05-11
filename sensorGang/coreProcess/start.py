"""Start script."""
from autonomous import Autonomous
from semiAutonomous import SemiAutonomous
from manual import Manual
import mqttSetup as mq
from queue import Queue


q = Queue()
MQTT_TOPIC = [("stop", 0), ("mode", 0)]
MQTT_TOPIC_UNSUB = ["stop", "mode"]
FRAME_RATE = 60
RESOLUTION = (640, 480)


# Settings
TIME_OUT_TIME = 500
ROI_PERC = [0, 0.45, 1, 0.45, 1, 1, 0, 1]


def getMode(mqttClient):
    """Handle MQTT message."""
    print("Ready")
    while True:
        if not q.empty():
            mode = q.get()
            if mode[0] == "stop":
                print("Stopping")
                return None
            if mode[1] == 1 or mode[1] == 2 or mode[1] == 3:
                mqttClient.unsubscribe(MQTT_TOPIC_UNSUB)
                return mode[1]
            else:
                print("Unvalid mode")


def on_message(client, userdata, message):
    """MQTT callback function."""
    try:
        print("I got mail")
        m = int(message.payload.decode("utf-8"))
        t = message.topic
        q.put((t, m))
        print("Message: {}, topic: {}".format(m, t))
    except Exception as e:
        print("Couldn't read mqtt message")
        print(e)


mqttClient = mq.initMqtt()
mqttClient.on_message = on_message
mqttClient.subscribe(MQTT_TOPIC)


while True:
    modeSetting = getMode(mqttClient)  # Get info from PC with mode car is in
    if modeSetting is not None:
        if modeSetting == 1:
            # Manual
            print("Manual")
            mode = Manual(mqttClient, TIME_OUT_TIME,
                          RESOLUTION, FRAME_RATE, True)
        elif modeSetting == 2:
            # Semi autonoumous
            print("SemiAutonomous")
            mode = SemiAutonomous(
                mqttClient, TIME_OUT_TIME, ROI_PERC, RESOLUTION)

        else:
            # Autonomous
            print("Autonomous")
            mode = Autonomous(mqttClient, TIME_OUT_TIME, ROI_PERC, RESOLUTION)

        try:
            mode.mainLoop()
            mode.stop()
            mqttClient.on_message = on_message
            mqttClient.subscribe(MQTT_TOPIC)

        except Exception as e:
            print("Exception in start, stopping")
            print(e)

    else:
        break

mqttClient.disconnect()
