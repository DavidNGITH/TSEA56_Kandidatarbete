import paho.mqtt.client as mqtt
import socket
from queue import Queue

def initMqtt():
    hostname = socket.gethostname()
    ip_adress = socket.gethostbyname(hostname)

    mqttClient = mqtt.Client()
    mqttClient.connect(ip_adress, 1833)
    
    mqttClient.loop_start

    return mqttClient

    
