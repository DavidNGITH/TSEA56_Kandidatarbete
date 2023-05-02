import keyboard
import time
from pynput import keyboard
import paho.mqtt.client as mqtt
import time


state = True

mqtt_client = mqtt.Client()


def mqtt_init():

    broker_ip = "10.241.226.165"
    broker_port = 1883
    mqtt_client.username_pw_set("tsea56G09", "mindset")
    mqtt_client.connect(broker_ip, broker_port)

    mqtt_client.loop_start()


mqtt_init()


def main(key):
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

    elif key == keyboard.KeyCode.from_char("s"):
        mqtt_client.publish("stop", "2")


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
        mqtt_client.publish("ping", 1)
        t1 = time.time()
        time.sleep(1)

l.join()


print("Hej")
