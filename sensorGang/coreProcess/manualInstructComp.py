import keyboard
import time
from pynput import keyboard
#import os
import paho.mqtt.client as mqtt

currentSpeed = 0
steerServo = 50

mqtt_client = mqtt.Client()


def mqtt_init():

    broker_ip = "10.241.218.244" 
    broker_port = 1883 



    mqtt_client.connect(broker_ip, broker_port)

    mqtt_client.publish("speed", "50")

    #mqtt_client.loop_start()



def main(key):
    #mqtt_init()
    global steerServo
    global currentSpeed
    if key == keyboard.Key.left:
        if steerServo > 0:
            steerServo -= 1
            mqtt_client.publish("steering",str(steerServo))
            print(steerServo)
    if key == keyboard.Key.right:
        if steerServo < 100:
            steerServo += 1
            mqtt_client.publish("steering",str(steerServo))
            print(steerServo)
    if key == keyboard.Key.up:
        if currentSpeed < 255:
            currentSpeed += 1
            mqtt_client.publish("speed",str(currentSpeed))
            print(currentSpeed)
    if key == keyboard.Key.down:
        if currentSpeed > 0:
            currentSpeed -= 1
            mqtt_client.publish("speed",str(currentSpeed))
            print(currentSpeed)



def on_release(key):
    if key == keyboard.Key.esc:
        # Stop listener
        return False

# Collect events until released
with keyboard.Listener(
        on_press= main, on_release= on_release) as listener:
    listener.join()