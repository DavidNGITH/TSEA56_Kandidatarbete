from autonomous import Autonomous
from semiAutonomous import SemiAutonomous
from manual import Manual
import mqttSetup as mq
from queue import Queue

q = Queue()
MQTT_TOPIC = [("stop",0),("mode",0)]
MQTT_TOPIC_UNSUB = ["stop", "mode"]



def getMode(mqttClient):
    while True:
        if not q.empty():
            mode = q.get()
            print(mode)
            if mode[0] == "stop":
                print("Stopping")
                return None
            if mode[1] == 1 or mode[1] == 2 or mode[1] ==3:
                mqttClient.unsubscribe(MQTT_TOPIC_UNSUB)
                return mode[1]
            else:
                print("Unvalid mode")


def on_message(client, userdata, message):
    try:
        print("I got mail")
        m = int(message.payload.decode("utf-8"))
        t = message.topic
        q.put((t,m))
    except Exception as e:
        print("Couldn't read mqtt message")
        print(e)



mqttClient = mq.initMqtt()
mqttClient.on_message = on_message
mqttClient.subscribe(MQTT_TOPIC)

## Get info from PC with mode car is in
modeSetting = getMode(mqttClient)

if not modeSetting == None:
    print(modeSetting)
    if modeSetting == 1:
        #Manual
        print("Manual")
        mode = Manual(mqttClient)
    elif modeSetting == 2:
        #Semi autonoumous
            print("SemiAutonomous")
            mode = SemiAutonomous(mqttClient)

    else:
        #Autonomous
        print("Autonomous")
        mode = Autonomous(mqttClient)

    try:
        mode.mainLoop()

    except Exception as e:
        print(e)
        print("Exception, stopping")
        mode.stop()

mqttClient.disconnect()