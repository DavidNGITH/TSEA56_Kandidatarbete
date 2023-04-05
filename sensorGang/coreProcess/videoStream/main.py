from videoStream import VideoStream
import cv2
import picamera.array import PiRGBArray
from picamera import PiCamera
import time

threadStream = VideoStream()
threadStream.start()
time.sleep(2)

def frameFromStream():
    t1 = time.time()
    image = thread_stream.read()
    t2 = time.time()

    print((t2-t1)*1000)

    time.sleep(1)




if __name__ == "__main__":
    frameFromStream()