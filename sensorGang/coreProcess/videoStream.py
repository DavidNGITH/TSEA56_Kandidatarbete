"""Start video stream from camera."""
from picamera.array import PiRGBArray
from picamera import PiCamera
from threading import Thread
import cv2
import time


class VideoStream:
    """Class for video stream."""

    def __init__(self, resolution, framerate=60):
        """Init camera."""
        self.camera = PiCamera()
        self.camera.resolution = resolution
        self.camera.framerate = framerate
        self.cameraRaw = PiRGBArray(self.camera, resolution)
        self.stream = self.camera.capture_continuous(
            self.cameraRaw, 'bgr', use_video_port=True)
        self.img = None
        self.stopped = False

    def start(self):
        """Start camera stream."""
        t = Thread(target=self.update)
        t.start()
        time.sleep(5)
        return

    def update(self):
        """Update frame."""
        for frame in self.stream:
            self.img = frame.array
            self.cameraRaw.truncate(0)

            if self.stopped:
                print("update stopped")
                self.stream.close()
                self.cameraRaw.close()
                self.camera.close()
                return
            time.sleep(0.01)

    def read(self):
        """Get latest frame."""
        return self.img

    def stop(self):
        """Stop stream."""
        self.stopped = True
