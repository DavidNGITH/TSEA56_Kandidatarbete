import cv2



class VideoStreamFile:
    def __init__(self):
        self.vid_capture = cv2.VideoCapture('victor.mp4')


    def start(self):
        return

    
    def read(self):
        ret, frame = self.vid_capture.read()
        return frame



    def stop(self):
        self.vid_capture.release()