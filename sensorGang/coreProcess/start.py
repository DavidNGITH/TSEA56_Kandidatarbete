from autonomous import Autonomous
from semiAutonomous import SemiAutonomous
from manual import Manual
import mqttSetup as mq
from queue import Queue

q = Queue
MQTT_TOPIC = [("stop",0),("mode",0)]



def getMode(client, userdata, message):
    while True:
        if not q.empty():
            mode = mqttClient.get_message().get()
            if mode[0] == "stop":
                print("Stopping")
                return None
            match mode[1]:
                case 1 | 2 | 3:
                    mqttClient.unsubscribe(MQTT_TOPIC)
                    return mode
                case _:
                    print("Unvalid mode")


def on_message(client, userdata, message):
    try:
        m = int(message.payload.decode("utf-8"))
        t = message.topic
        q.put((t,m))
    except:
        print("Couldn't read mqtt message")



mqttClient = mq.initMqtt
mqttClient.on_message = on_message
mqttClient.subscribe(MQTT_TOPIC)

## Get info from PC with mode car is in
modeSetting = getMode(mqttClient)

if not modeSetting == None:
    if modeSetting == 1:
        #Manual
        mode = Manual(mqttClient)
    elif modeSetting == 2:
        #Semi autonoumous
            mode = SemiAutonomous(mqttClient)

    else:
        #Autonomous
        mode = Autonomous(mqttClient)

    try:
        mode.mainLoop()

    except Exception as e:
        print(e)
        mode.stop()

mqttClient.disconnect()