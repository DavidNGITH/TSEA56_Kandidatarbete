import time 
import paho.mqtt.client as mqtt
import keyboard


mqtt_client = mqtt.Client()
mqtt_topic = "manualdrive"

def mqtt_init():

    broker_ip = "10.241.218.244"   #kolla rätt IP på RPi:n
    broker_port = 1883 #standardport för MQTT



    mqtt_client.connect(broker_ip, broker_port)

    mqtt_client.subscribe(mqtt_topic)

    mqtt_client.loop_start()

    #message = "Hej på dig"

    #mqtt_client.publish(mqtt_topic, "Hej på dig")

def main():

    mqtt_init()

    while True:
        try: 
            if keyboard.is_pressed("up"):
                print("up")
                mqtt_client.publish(mqtt_topic, "1")
                time.sleep(0.01)
            elif keyboard.is_pressed("down"):
                print("down")
                mqtt_client.publish(mqtt_topic, "4")
                time.sleep(0.01)
            elif keyboard.is_pressed("left"):
                print("left")
                mqtt_client.publish(mqtt_topic, "2")
                time.sleep(0.01)
            elif keyboard.is_pressed("right"):
                print("right")
                mqtt_client.publish(mqtt_topic, "3")
                time.sleep(0.01)
            elif keyboard.is_pressed("z"):
                print("straight ahead, clear target")
                mqtt_client.publish(mqtt_topic, "5")
                time.sleep(0.01)
            elif keyboard.is_pressed("e"):
                time.sleep(0.01)
                mqtt_client.loop_stop()
                break
            else:
                #mqtt_client.publish(mqtt_topic, "1")
                #print("f")
                pass
            
        except:
            pass            


main()
