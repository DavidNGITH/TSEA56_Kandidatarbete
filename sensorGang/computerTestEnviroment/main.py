from CTEcompVision import compVision
from CTEvideoStreamFile import VideoStreamFile
import cv2

RESOLUTION = (640, 480)
ROI_PERC = [0, 0.45, .99, 0.45, .99, 1, 0, 1]


def main():
    # Create a video stream object
    videoStream = VideoStreamFile()
    # Create a computer vision object
    computerVision = compVision(ROI_PERC, RESOLUTION)
    # Loop through the video stream
    computerVision.getCenterOffset()


main()
