import keyboard
import time
from pynput import keyboard
import paho.mqtt.client as mqtt
import time


state = True

mqtt_client = mqtt.Client()

speed = 0

steering = 50


def mqtt_init():

    broker_ip = "10.241.239.117"
    broker_port = 1883
    mqtt_client.username_pw_set("tsea56G09", "mindset")
    mqtt_client.connect(broker_ip, broker_port)

    mqtt_client.loop_start()


mqtt_init()


def main(key):
    global speed
    global steering
    """Main function."""
    if key == keyboard.KeyCode.from_char('1'):
        mqtt_client.publish("command/turning", "1")

    elif key == keyboard.KeyCode.from_char('2'):
        mqtt_client.publish("command/turning", "2")

    elif key == keyboard.KeyCode.from_char('3'):
        mqtt_client.publish("command/turning", "3")

    elif key == keyboard.KeyCode.from_char('4'):
        mqtt_client.publish("command/turning", "4")

    elif key == keyboard.KeyCode.from_char("g"):
        mqtt_client.publish("mode", "2")

    elif key == keyboard.KeyCode.from_char("h"):
        mqtt_client.publish("mode", "3")

    elif key == keyboard.KeyCode.from_char("f"):
        speed = 0
        steering = 50
        mqtt_client.publish("stop", "2")

    elif key == keyboard.KeyCode.from_char("m"):
        mqtt_client.publish("mode", "1")

    elif key == keyboard.KeyCode.from_char("w"):
        if speed < 251:
            speed = speed + 5
        else:
            speed = 255
        mqtt_client.publish("speed", "{}".format(speed))

    elif key == keyboard.KeyCode.from_char("s"):
        if speed > 4:
            speed = speed - 5
        else:
            speed = 0
        mqtt_client.publish("speed", "{}".format(speed))

    elif key == keyboard.KeyCode.from_char("a"):
        if steering > 4:
            steering = steering - 5
        else:
            steering = 0
        mqtt_client.publish("steering", "{}".format(steering))

    elif key == keyboard.KeyCode.from_char("d"):
        if steering < 116:
            steering = steering + 5
        else:
            steering = 120
        mqtt_client.publish("steering", "{}".format(steering))


def on_release(key):
    if key == keyboard.Key.esc:
        # Stop listener
        global state
        state = False
        return False


# Collect events until released

l = keyboard.Listener(on_press=main, on_release=on_release)
l.start()
t1 = 0
while state:
    if time.time() - t1 > 2:
       # mqtt_client.publish("ping", 1)
        t1 = time.time()
        time.sleep(1)

l.join()


print("Hej")
