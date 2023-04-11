import paho.mqtt.client as mqtt
import socket


def mqtt_init():
    hostname = socket.gethostname()

    ip_adress = socket.gethostbyname(hostname)

    print(hostname)
    print(ip_adress)

    topic = "Test"
    broker_ip = ip_adress
    broker_port = 1883

    mqttclient = mqtt.Client()

    mqttclient.connect(broker_ip, broker_port)
    
    mqttclient.publish(topic, "Hello World ")
    
    def on_message(client, userdata, message):
        print("Recieved:", str(message.payload.decode("utf-8")))
        
    mqttclient.on_message = on_message
    
    mqttclient.subscribe(topic)
    
    mqttclient.loop_forever()
    
    #mqttclient.disconnect()
    
mqtt_init()
