"""Computer vision."""
import time
import numpy as np
import cv2
from videoStream import VideoStream
from sharedFunctions import (lineInterceptFunction,
                             regionOfInterestFunction,
                             getDataFromLinesFunction,
                             getOffsetLeftTurnFunction,
                             getOffsetRightTurnFunction,
                             setupFunction)

from videoStreamFile import VideoStreamFile


class compVision:
    """Class for computer vision."""

    def __init__(self, PD, roiPerc, resolution):
        """Set up variables and processes."""
        self.setup()
        # Variables
        self.resolution = resolution
        self.width = resolution[0]
        self.height = resolution[1]
        self.center = int(self.width/2)

        # ROIDIM: Upperleft, UpperRight, LowerRight, LowerLeft
        roiP1X = int(roiPerc[0] * self.width)
        roiP1Y = int(roiPerc[1] * self.height)
        roiP2X = int(roiPerc[2] * self.width)
        roiP2Y = int(roiPerc[3] * self.height)
        roiP3X = int(roiPerc[4] * self.width)
        roiP3Y = int(roiPerc[5] * self.height)
        roiP4X = int(roiPerc[6] * self.width)
        roiP4Y = int(roiPerc[7] * self.height)
        self.roiDim = [(roiP1X, roiP1Y), (roiP2X, roiP2Y),
                       (roiP3X, roiP3Y), (roiP4X, roiP4Y)]

        # Stops/Nodes
        self.stopLine = False           # If stopline is found
        self.nodeLine = False           # If nodeline is found
        self.stopLineActivated = True   # Whether it should look for stopLines
        self.nodeTimeOut = 0            # Time out for finding nodes
        self.stopAtNode = False         # Whether it should stop at node

        # PD-Controller
        self.PD = PD

    lineIntercept = lineInterceptFunction
    regionOfInterest = regionOfInterestFunction
    getDataFromLines = getDataFromLinesFunction
    getOffsetLeftTurn = getOffsetLeftTurnFunction
    getOffsetRightTurn = getOffsetRightTurnFunction
    setup = setupFunction

    def makeStopLine(self, line, minX, maxX):
        """Create endpoints for stop line."""
        width = maxX-minX

        # If stopline
        if width > self.widthStopLine and width < 300:
            # If stop required
            if self.stopLineActivated:
                self.stopLine = True

        # Checks if node
        elif width < self.widthNodeLine:
            # Left node
            if minX < 200:
                print("Node to the left")
            # Right node
            elif maxX > 400:
                if (self.stopAtNode and
                        (time.time() - self.intersectionTime > 3 and
                         time.time() - self.nodeTimeOut > 2)):
                    self.nodeLine = True
                else:
                    self.nodeLine = False
                print("Node to the right")

    def waitForCommand(self, qCommand):
        """Get steering command."""
        while qCommand.empty():
            time.sleep(0.01)

        getCommand = qCommand.get()

        if not getCommand == 10:
            self.getOffset = self.casesDict[getCommand]

    def getCenterOffset(self, qSteering, statusValue, qSpeed, qBreak,
                        qCommand, qCommandNode, qPD, qOffsetData):
        """Calculate the center offset in frame."""
        threadStream = VideoStream(self.resolution)  # Creates Video stream
        threadStream.start()  # Starts Video stream

        status = statusValue.value

        qSpeed.put(self.normalSpeed)  # Start car

        while status:
            # Handles PD messages from computer
            if qPD.empty() is False:
                message = qPD.get()
                if message[0] == 0:
                    self.PD.updateKp(message[1])
                else:
                    self.PD.updateKp(message[1])

            self.img = threadStream.read()  # Retrive image

            self.img = cv2.cvtColor(self.img, cv2.COLOR_BGR2GRAY)

            self.img = cv2.threshold(self.img, 50, 255, cv2.THRESH_BINARY)[1]

            # Apply canny
            self.img = cv2.Canny(self.img, self.lowerThreshold,
                                 self.upperThreshold, self.appetureSize)

            self.regionOfInterest()  # Apply ROI

            # Histogram calc from canny image
            histogram = np.sum(self.img[400:480, 5:635], axis=0)

            # Left histogram
            self.leftHistogram = np.argmax(histogram[:int(self.center-60)]) + 5
            if self.leftHistogram == 5:
                self.leftHistogram = None

            # Right histogram
            self.rightHistogram = (np.argmax(histogram[int(self.center+60):]) +
                                   self.center + 65)
            if self.rightHistogram == self.center + 65:
                self.rightHistogram = None

            # Midpoint from histogram
            if (self.rightHistogram is not None and
                    self.leftHistogram is not None):
                self.midpointHistogram = int((self.rightHistogram -
                                              self.leftHistogram)
                                             / 2 + self.leftHistogram)
            else:
                self.midpointHistogram = None

            # Apply Hough transfrom
            lineSegments = cv2.HoughLinesP(self.img, self.rho, self.angle,
                                           self.minThreshold,
                                           cv2.HOUGH_PROBABILISTIC,
                                           minLineLength=self.minLineLength,
                                           maxLineGap=self.minLineLength)

            self.lineIntercept(lineSegments)  # Calc line equations

            self.lastSpeed = self.currentSpeed  # Set last speed to current

            self.getOffset()  # Get offset

            # Send offset data to computer
            qOffsetData.put(str(self.newOffset))

            # Checks if speed is updated
            # if self.currentSpeed is not self.lastSpeed:
            # If turning speed
            if self.currentSpeed is not self.normalSpeed:
                self.slowDownTimer = time.time()  # Reset timer
                # If speed hasn't been sent already
                if self.speedToSend is not self.turningSpeed:
                    self.speedToSend = self.turningSpeed
                    qSpeed.put(self.speedToSend)

            # Switches to normal speed
            if (time.time() - self.slowDownTimer > 0.5 and
                    self.speedToSend is not self.normalSpeed):
                self.speedToSend = self.normalSpeed
                qSpeed.put(self.speedToSend)

            # If stopline
            if self.stopLine:
                print("Stopline")
                qSteering.put(52)  # Send steering data to car
                qBreak.put(1)  # Apply break
                self.waitForCommand(qCommand)  # Get command
                qBreak.put(0)  # Release break
                self.stopLine = False
                self.intersectionTime = time.time()
                self.normalSteering = False
                self.stopLineActivated = False

            # If node line
            if (self.nodeLine and (time.time()-self.nodeTimeOut > 2)):
                print("Stopping at Node")
                qSteering.put(52)  # Send steering data to car
                qBreak.put(1)  # Apply break
                while qCommand.empty() and qCommandNode.empty():
                    time.sleep(0.01)
                self.stopAtNode = False
                self.nodeLine = False
                self.nodeTimeOut = time.time()
                qBreak.put(0)

            # PD controller
            else:
                steering_raw = self.PD.get_control(self.newOffset)
                steering = int((steering_raw)*0.2 + 52)

                if steering < 0:
                    steering = 0

                elif steering > 120:
                    steering = 120

                qSteering.put(steering)  # Send steering data to car

            # If in intersection and > 3 s
            if (time.time()-self.intersectionTime > 3 and
                    not self.normalSteering):
                print("normal")
                self.getOffset = self.getDataFromLines
                self.normalSteering = True

            # Sets stop required
            if (time.time() - self.intersectionTime > 4.5):
                self.stopLineActivated = True

            if not qCommandNode.empty():
                while not qCommandNode.empty():
                    qCommandNode.get()

                print("Stopping at next node")
                self.stopAtNode = True

            status = statusValue.value  # Check status value

            t2 = time.time()

        threadStream.stop()  # Stop stream
