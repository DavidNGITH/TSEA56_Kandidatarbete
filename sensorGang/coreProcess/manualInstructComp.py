import keyboard
import time
from pynput import keyboard
import paho.mqtt.client as mqtt
import time 

currentSpeed = 0
steerServo = 50

state = True

mqtt_client = mqtt.Client()


def mqtt_init():

    broker_ip = "10.241.218.244" 
    broker_port = 1883 



    mqtt_client.connect(broker_ip, broker_port)

    mqtt_client.loop_start()

mqtt_init()



def main(key):
    #mqtt_init()
    global steerServo
    global currentSpeed
    if key == keyboard.Key.left:
        steerServo -= 10
        if steerServo < 0:
            steerServo = 0
        mqtt_client.publish("steering",str(steerServo))
        print(steerServo)
    elif key == keyboard.Key.right:
        steerServo += 10
        if steerServo >= 100:
            steerServo = 100
        mqtt_client.publish("steering",str(steerServo))
        print(steerServo)
    elif key == keyboard.Key.up:
        if currentSpeed < 240:
            currentSpeed += 10
            mqtt_client.publish("speed",str(currentSpeed))
            print(currentSpeed)
            time.sleep(0.0025)
    elif key == keyboard.Key.down:
        if currentSpeed > 0:
            currentSpeed -= 10
            mqtt_client.publish("speed",str(currentSpeed))
            print(currentSpeed)
            time.sleep(0.0025)
    elif key == keyboard.Key.space:
        currentSpeed = 0
        mqtt_client.publish("speed",str(currentSpeed))
        mqtt_client.publish("steering",str(steerServo))
        time.sleep(0.0025)




def on_release(key):
    if key == keyboard.Key.esc:
        # Stop listener
        global state 
        state = False
        currentSpeed = 0 
        mqtt_client.publish("speed",str(currentSpeed))
        return False

# Collect events until released
"""with keyboard.Listener(on_press= main, on_release= on_release) as listener:
    print("hej")
    listener.join()"""

l = keyboard.Listener(on_press= main, on_release= on_release)
l.start()
t1 = 0
while state:
    if time.time() - t1 > 2:
        mqtt_client.publish("ping", 1)
        t1 = time.time()
        time.sleep(1)
l.join()


print("Hej")