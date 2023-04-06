from picamera.array import PiRGBArray
import cv2
import numpy
from picamera import PiCamera
import time 

camera = PiCamera()
camera.resolution = (640,480)
camera.framerate = 15
cameraRaw = PiRGBArray(camera,(640,480))



for frame in camera.capture_continuous(cameraRaw, 'bgr', use_video_port = True):
    image = frame.array
    roi = cv2.selectROI(image)
    print(roi)
    cameraRaw.truncate(0)

    break

camera.close()