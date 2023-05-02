"""Set up MQTT client."""
import paho.mqtt.client as mqtt
import socket
from queue import Queue


def initMqtt():
    """Set up MQTT client."""
    hostname = socket.gethostname()
    ip_adress = socket.gethostbyname(hostname)

    mqttClient = mqtt.Client()
    mqttClient.username_pw_set("tsea56G09", "mindset")
    mqttClient.connect(ip_adress, 1883)

    mqttClient.loop_start()

    return mqttClient
