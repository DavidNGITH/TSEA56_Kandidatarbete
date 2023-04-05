from picamera.array import PiRGBArray
from picamera import PiCamera
from threading import Thread
import cv2

class VideoStream:
    def __init__(self, resolution=(640,480), framerate=20):
        self.camera = PiCamera()
        self.camera.resolution = resolution
        self.camera.framerate = framerate
        self.cameraRaw = PiRGBArray(self.camera, resolution)
        self.stream = self.camera.capture_continuous(self.cameraRaw, 'bgr', use_video_port = True)
        self.img = None
        self.stopped = False

    def start(self):
        t = Thread(target=self.update)
        t.start()
        return

    def update(self):
        for frame in self.stream:
            self.img = frame.array
            self.cameraRaw.truncate(0)

            if self.stopped:
                self.stream.close()
                self.cameraRaw.close()
                self.camera.close()
                return

    def read(self):
        return self.img

    def stop(self):
        self.stopped = True



    