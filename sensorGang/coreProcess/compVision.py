from picamera import PiCamera
from picamera.array import PiRGBArray
from time import sleep
import cv2


class compVision:
    def __init__(self, resolution, frameRate):
        self.resoultion = resolution

        self.camera = PiCamera()
        self.camera.framerate = frameRate
        self.cameraRaw = PiRGBArray(self.camera, self.resoultion)

