"""Record from camera module."""
from picamera import PiCamera
from datetime import datetime
import time


def record(statusValue, resolution, framerate):
    """Start recording."""
    date = datetime.now()

    camera = PiCamera()
    camera.resolution = resolution
    camera.framerate = framerate
    camera.shutter_speed = 10000

    print("Starting recording")
    # camera.start_preview()
    camera.start_recording("recordings/recording_{}_{}_{}.h264".format(
        date.day, date.hour, date.minute))

    while (statusValue.value):
        time.sleep(0.1)

    camera.stop_recording()
    # camera.stop_preview()
    camera.close()
