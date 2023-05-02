"""Display data on imgage."""
import cv2
import time
import datetime
import numpy as np


def saveImageData(self):
    """Save self.img with data to file."""
    turnInstruction = None

    try:
        if ((self.lineCenter - self.center) > 0):
            turnInstruction = "TurnRight"

        else:
            turnInstruction = "TurnLeft"
    except Exception:
        turnInstruction = "NoLines"

    self.img = cv2.putText(
        img=self.img,
        text=turnInstruction,
        org=(int(0.10*self.width), int(0.9 * self.height)),
        fontFace=cv2.FONT_HERSHEY_DUPLEX,
        fontScale=0.5,
        color=(125, 246, 55),
        thickness=1
    )

    cv2.imwrite("savedImages/{}_{}.jpeg".format(turnInstruction,
                str(datetime.now())), self.img)


def addLines(self):
    """Add detected lines to image."""
    lineImage = np.zeros_like(self.orgImg)
    if self.laneLines is not None:
        for line in self.laneLines:
            for x1, y1, x2, y2 in line:
                cv2.line(lineImage, (x1, y1), (x2, y2), (0, 0, 255), 2)

    if (self.lineCenter is not None):
        pass
        # cv2.line(lineImage, (int(self.lineCenter), 0),
        #         (int(self.lineCenter), int(self.height)), (0, 255, 0), 2)

    cv2.line(lineImage, (int(self.center), 0),
             (int(self.center), int(self.height)), (255, 0, 0), 2)
    self.img = cv2.addWeighted(self.orgImg, 0.8, lineImage, 1, 1)


def displayROI(self):
    """Display ROI lines on image."""
    self.img = cv2.line(
        self.img, self.roiDim[0], self.roiDim[1], (0, 140, 255), 2)
    self.img = cv2.line(
        self.img, self.roiDim[1], self.roiDim[2], (0, 140, 255), 2)
    self.img = cv2.line(
        self.img, self.roiDim[2], self.roiDim[3], (0, 140, 255), 2)
    self.img = cv2.line(
        self.img, self.roiDim[3], self.roiDim[0], (0, 140, 255), 2)

    # print("ROI coordinates: {}, {}, {}, {}".format(
    #    self.roiDim[0], self.roiDim[1], self.roiDim[2], self.roiDim[3]))


def drawLine(self, coordinates, color, width):
    """Draw line on image."""
    self.img = cv2.line(
        self.img, coordinates[0], coordinates[1], color, width)
