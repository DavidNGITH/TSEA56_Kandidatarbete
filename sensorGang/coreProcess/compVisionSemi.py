"""Computer vision."""
import time
import numpy as np
import cv2
from PD_reg import PDcontroller
from videoStream import VideoStream
from datetime import datetime
import matplotlib.pyplot as plt

from videoStreamFile import VideoStreamFile


class compVision:
    """Class for computer vision."""

    def __init__(self, PD, roiPerc, resolution):
        """Set up variables and processes."""
        # Variables
        self.resolution = resolution
        self.width = resolution[0]
        self.height = resolution[1]
        self.center = int(self.width/2)

        self.img = None
        self.laneLines = []
        self.lineCenter = None
        self.newOffset = 0
        self.lastOffset = 0

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

        # Canny settings
        self.lowerThreshold = 135
        self.upperThreshold = 195
        self.appetureSize = 3

        # Hough settings
        self.rho = 1
        self.angle = np.pi / (180*1.8)
        self.minThreshold = 3
        self.minLineLength = 6
        self.maxLineGap = 3

        # Undistort
        self.my = np.load('distortionfiles/mapy.npy')
        self.mx = np.load('distortionfiles/mapx.npy')
        self.roiFile = np.load('distortionfiles/roiFile.npy')

        self.roiFile[1] = int(self.roiFile[1] * self.width / 640)
        self.roiFile[3] = int(self.roiFile[3] * self.width / 640)
        self.roiFile[0] = int(self.roiFile[0] * self.height / 480)
        self.roiFile[2] = int(self.roiFile[2] * self.height / 480)

        # Find lines
        self.laneLines = []
        self.xPointRight = None
        self.xPointLeft = None
        self.slopeLeft = None
        self.slopeRight = None

        # Stop lines coordinates
        self.widthStopLine = 220
        self.widthNodeLine = 170

        # Stop
        self.stopLine = False
        self.stopLineDistance = 0  # Distance to stop line
        self.lastStopLineDistance = 0
        self.stopRequired = True

        # PD-Controller
        self.PD = PD

        # Cases
        self.steerLeft = False
        self.steerRight = False
        self.slowDownTimer = 0
        self.slowDown = False
        self.lastSlowDown = None
        self.normalSteering = True

        # Speed
        self.normalSpeed = 110
        self.turningSpeed = 85
        self.currentSpeed = 110
        self.lastSpeed = 110
        self.speedToSend = None
        self.intersectionTime = 0
        self.intersectionTimer = 3

        # Get offset
        self.getOffset = self.getDataFromLines

        self.casesDict = {
            0: self.getDataFromLines,
            1: self.getOffsetStraightLeft,
            2: self.getOffsetStraightRight,
            3: self.getOffsetLeftTurn,
            4: self.getOffsetRightTurn
        }

    def displayImage(self):
        """Display self.image on screen."""
        cv2.imshow("Bild", self.img)
        cv2.waitKey()
        cv2.destroyAllWindows()

    # Undistortioon
    def undistortImage(self):
        """Undistort self.img."""
        self.img = cv2.remap(self.img, self.mx, self.my, cv2.INTER_LINEAR)
        self.img = self.img[self.roiFile[1]:self.roiFile[1] +
                            self.roiFile[3], self.roiFile[0]:self.roiFile[0] +
                            self.roiFile[2]]

    # Region of interest
    def regionOfInterest(self):
        """ROI on self.img."""
        mask = np.zeros_like(self.img)

        # (Left Top) (Right
        #
        # Top) (Right Bottom) (Left Bottom)
        polygon = np.array([self.roiDim], np.int32)
        cv2.fillPoly(mask, polygon, 255)
        # applying mask on original image

        self.img = cv2.bitwise_and(self.img, mask)

    def makePoints(self, line, which):
        """Create endpoints to detected lines."""
        slope, intercept = line
        y1 = self.height  # bottom of the frame
        y2 = int(y1 * 0)  # make points from middle of the frame down

        # bound the coordinates within the frame
        x1 = max(-self.width, min(2 * self.width,
                 int((y1 - intercept) / slope)))
        x2 = max(-self.width, min(2 * self.width,
                 int((y2 - intercept) / slope)))

        # Find x value for y = 100
        # 1 = Right
        if which:
            self.xPointRight = int((100-intercept)/slope)
        # 0 = Left
        else:
            self.xPointLeft = int((100-intercept)/slope)

        return [[x1, y1, x2, y2]]

    def makeStopLine(self, line, minX, maxX):
        """Create endpoints for stop line."""
        slope, intercept = line

        y1 = int(slope * minX + intercept)
        y2 = int(slope * maxX + intercept)

        width = maxX-minX

        print("Width: {}".format(width))

        # If stopline
        if width > self.widthStopLine and width < 300:
            # If stop required
            if self.stopRequired:
                self.stopLine = True

        # Checks if node
        elif width < self.widthNodeLine:
            # Left node
            if minX < 200:
                self.stopLine = False
                print("Node to the left")
            # Right node
            elif maxX > 400:
                self.stopLine = False
                print("Node to the right")

        else:
            # print("No stopline")
            self.stopLine = False

    def lineIntercept(self, lineSegments):
        """Handle detected lines from Hough transform."""
        self.laneLines = []
        self.xPointLeft = None
        self.xPointRight = None
        self.slopeLeft = 0
        self.slopeRight = 0

        if lineSegments is None:
            # print('No line_segment segments detected')
            return self.laneLines

        leftFit = []
        rightFit = []
        stopFit = []

        minX = 1000
        maxX = 0

        boundary = 1/3
        # left lane line segment should be on left 1/3 of the screen
        leftRegionBoundary = self.width * boundary
        # right lane line segment should be on left 1/3 of the screen
        rightRegionBoundary = self.width * (1 - boundary)

        for lineSegment in lineSegments:
            for x1, y1, x2, y2 in lineSegment:
                if x1 != x2 and y2 < 477 and y1 < 477:
                    fit = np.polyfit((x1, x2), (y1, y2), 1)
                    slope = fit[0]
                    intercept = fit[1]
                    if slope < -1:
                        if x1 < leftRegionBoundary and x2 < leftRegionBoundary:
                            leftFit.append((slope, intercept))
                    elif slope > 1:
                        if (x1 > rightRegionBoundary and
                                x2 > rightRegionBoundary):
                            rightFit.append((slope, intercept))

                    elif (slope < 0.1 and slope > -0.1 and
                          x1 > 0.15 * self.width and x2 < 0.85 * self.width):

                        stopFit.append((slope, intercept))
                        if x1 < minX:
                            minX = x1
                        if x2 > maxX:
                            maxX = x2

        if len(leftFit) > 2:
            leftFitAverage = np.average(leftFit, axis=0)
            self.slopeLeft = leftFitAverage[0]
            self.laneLines.append(self.makePoints(leftFitAverage, 0))

        if len(rightFit) > 2:
            rightFitAverage = np.average(rightFit, axis=0)
            self.slopeRight = rightFitAverage[0]
            self.laneLines.append(self.makePoints(rightFitAverage, 1))

        if len(stopFit) > 0:
            print("Length: {}".format(len(stopFit)))
        if len(stopFit) > 5:
            stopFitAverage = np.average(stopFit, axis=0)
            self.makeStopLine(stopFitAverage, minX, maxX)

        try:
            self.lineCenter = ((rightFitAverage[1]-leftFitAverage[1]) /
                               (leftFitAverage[0]-rightFitAverage[0]))

        except Exception:
            self.lineCenter = None
            # print("Not enough lines captured")
            return

    def waitForCommand(self, qCommand):
        """Get steering command."""
        while qCommand.empty():
            time.sleep(0.01)

        getCommand = qCommand.get()

        if not getCommand == 10:
            self.getOffset = self.casesDict[getCommand]

    def getCenterOffset(self, qSteering, statusValue, qSpeed, qBreak,
                        qCommand, qPD):
        """Calculate the center offset in frame."""
        threadStream = VideoStream(self.resolution)  # Creates Video stream
        threadStream.start()  # Starts Video stream

        status = statusValue.value

        # qSpeed.put(self.normalSpeed)  # Start car

        while status:

            # Handles PD messages from computer
            if qPD.empty() is False:
                message = qPD.get()
                if message[0] == 0:
                    self.PD.updateKp(message[1])
                else:
                    self.PD.updateKp(message[1])

            self.img = threadStream.read()  # Retrive image

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

            # Checks if speed is updated
            # if self.currentSpeed is not self.lastSpeed:
            # If turning speed
            if self.currentSpeed is not self.normalSpeed:
                self.slowDownTimer = time.time()  # Reset timer
                # If speed hasn't been sent already
                if self.speedToSend is not self.turningSpeed:
                    self.speedToSend = self.turningSpeed
                    # qSpeed.put(self.speedToSend)

            # Switches to normal speed
            if (time.time() - self.slowDownTimer > 0.5 and
                    self.speedToSend is not self.normalSpeed):
                self.speedToSend = self.normalSpeed
                # qSpeed.put(self.speedToSend)

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
                self.stopRequired = False

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
                self.stopRequired = True

            status = statusValue.value  # Check status value

        threadStream.stop()  # Stop stream

    def getDataFromLines(self):
        """Get offset from lines."""
        self.currentSpeed = self.turningSpeed

        self.lastOffset = self.newOffset

        # Histogrammet har hittat båda linjerna
        if self.leftHistogram is not None and self.rightHistogram is not None:
            # Båda linjernas lutning har hittats
            if self.slopeLeft and self.slopeRight:
                casePrint = "Case 1"
                # self.newOffset = (0.6 * self.midpointHistogram +
                #                  0.5 * self.lineCenter)
                # self.currentSpeed = self.normalSpeed

                self.newOffset = self.midpointHistogram

            # Endast vänstra linjens lutning har hittats
            elif self.slopeLeft:
                casePrint = "Case 2"
                self.newOffset = (self.midpointHistogram -
                                  1/self.slopeLeft * 420)

            # Endast högra linjens lutning har hittats
            elif self.slopeRight:
                casePrint = "Case 3"
                self.newOffset = (self.midpointHistogram -
                                  1/self.slopeRight * 370)

            # Inga lutningar har hittats
            else:
                casePrint = "Case 4"
                self.newOffset = self.midpointHistogram

        # Histogrammet har endast hittat vänstra linjen
        elif self.leftHistogram is not None:
            # Vänstra linjens lutning har hittats
            if self.slopeLeft:
                casePrint = "Case 5"
                midpointHistogram = (
                    self.width - self.leftHistogram)/2 + self.center
                self.newOffset = midpointHistogram

                pass
            # Ingen lutning har hittats
            else:
                casePrint = "Case 6"
                midpointHistogram = (
                    self.width - self.leftHistogram)/2 + self.center
                self.newOffset = midpointHistogram

                pass

            pass

        # Histogrammet har endast hittat högra linjen
        elif self.rightHistogram is not None:
            # Vänstra linjens lutning har hittats
            if self.slopeRight:
                casePrint = "Case 7"
                midpointHistogram = self.center - (self.rightHistogram)/2
                self.newOffset = midpointHistogram

                pass
            # Ingen lutning har hittats
            else:
                casePrint = "Case 8"
                midpointHistogram = self.center - (self.rightHistogram)/2
                self.newOffset = midpointHistogram

                pass

        else:
            print("Case 9")
            self.newOffset = self.lastOffset

            return

        print(casePrint)

        self.newOffset -= (self.center + 40)

        self.newOffset = int(self.newOffset)

        # print(self.newOffset)

        print("Left histo: {}".format(self.leftHistogram))
        print("Right histo: {}".format(self.rightHistogram))
        print("Middle histo: {}".format(self.midpointHistogram))
        print("Crossing : {}".format(self.lineCenter))

    def getOffsetStraightLeft(self):
        """Get offset on straight, left line avalible."""
        self.currentSpeed = self.normalSpeed
        self.intersectionTimer = 2
        if self.leftHistogram is not None:
            self.newOffset = (self.leftHistogram - 130)*2
        else:
            self.newOffset = - 50

    def getOffsetStraightRight(self):
        """Get offset on straight, right line avalible."""
        self.currentSpeed = self.normalSpeed
        self.intersectionTimer = 2
        if self.rightHistogram is not None:
            self.newOffset = (self.rightHistogram - 530)*2
        else:
            self.newOffset = 50

    def getOffsetLeftTurn(self):
        """Get offset on left turn."""
        self.currentSpeed = self.turningSpeed + 15
        self.intersectionTimer = 1.75
        if self.leftHistogram is not None:
            self.newOffset = (self.leftHistogram - 130)*3.5
        else:
            self.newOffset = - 150

    def getOffsetRightTurn(self):
        """Get offset on right turn."""
        self.currentSpeed = self.turningSpeed + 15
        self.intersectionTimer = 1.75
        if self.rightHistogram is not None:
            self.newOffset = (self.rightHistogram - 530)*3
        else:
            self.newOffset = 150
