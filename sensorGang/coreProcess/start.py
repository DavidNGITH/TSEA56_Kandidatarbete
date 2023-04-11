from autonomous import Autonomous
from semiAutonomous import SemiAutonomous
from manual import Manual
import mqttSetup as mq
from queue import Queue

q = Queue


def getMode(client, userdata, message):
    mode = 0
    topic = "mode"
    mqttClient.subscribe(topic)

    while True:
        if not q.empty():
            mode = mqttClient.get_message().get()[1]
            match mode:
                case 1 | 2 | 3:
                    mqttClient.unsubscribe(topic)
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

## Get info from PC with mode car is in
modeSetting = getMode(mqttClient)

match modeSetting:
    case 1:
        #Manual
        mode = Manual(mqttClient)
    case 2:
        #Semi autonoumous
        mode = SemiAutonomous(mqttClient)
    case 3:
        #Autonomous
        mode = Autonomous(mqttClient)

try:
    mode.mainLoop()

except Exception as e:
    print(e)
    mode.stop()
    mqttClient.disconnect()