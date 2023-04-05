from videoStream import VideoStream
import cv2
from picamera.array import PiRGBArray
from picamera import PiCamera
import time

threadStream = VideoStream()
threadStream.start()
time.sleep(2)

def frameFromStream():
    for i in range(20):
        t1 = time.time()
        image = threadStream.read()
        t2 = time.time()

        print((t2-t1)*1000)

        cv2.imshow("Image", image)
        time.sleep(1)

    cv2.destroyAllWindows()
    threadStream.stop()



if __name__ == "__main__":
    try:
        frameFromStream()
    except:
        threadStream.stop()