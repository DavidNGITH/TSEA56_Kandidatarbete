"""Computer vision."""
import time
import numpy as np
import cv2
from datetime import datetime
import matplotlib.pyplot as plt

from CTEvideoStreamFile import VideoStreamFile


class compVision:
    """Class for computer vision."""

    def __init__(self, roiPerc, resolution):
        """Set up variables and processes."""
        # Variables
        self.resolution = resolution
        self.width = resolution[0]
        self.height = resolution[1]
        self.center = int(self.width/2)

        self.img = None
        self.laneLines = []
        self.lineCenter = None
        self.lastOffset = None
        self.newOffset = None

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
        self.lowerThreshold = 100
        self.upperThreshold = 150
        self.appetureSize = 3

        # Hough settings
        self.rho = 1
        self.angle = np.pi / (180*2)
        self.minThreshold = 0
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
        self.stopLine = False
        self.xPointRight = None
        self.xPointLeft = None
        self.slopeLeft = None
        self.slopeRight = None
        self.stopLineTimer = 0
        self.stopLineCoordinates = 0

        # Stop
        self.stop = False
        self.stopLineDistance = 0  # Distance to stop line
        self.lastStopLineDistance = 0
        self.stopRequired = True

        # Stop lines coordinates
        self.widthStopLine = 250
        self.widthNodeLine = 220
        # self.heightMin = 200
        self.heightMax = 390

        self.gaussianKernelSize = 5

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

        x1 = int(minX)
        x2 = int(maxX)

        y1 = int(slope * minX + intercept)
        y2 = int(slope * maxX + intercept)

        width = maxX-minX
        height = (y1+y2)/2

        # print("{}, {}, {}, {}".format(minX, maxX, y1, y2))
        # print("Width: {}".format(maxX-minX))
        # print("Height: {}".format(height))

        if width > self.widthStopLine:
            # print("Width for stopline OK")
            pass

        elif width < self.widthNodeLine:
            # print("Width for node OK")
            pass

        if height < self.heightMax:
            # print("Height OK")
            pass
        else:
            # print("Height not OK")
            pass

        if height < self.heightMax:
            if width > self.widthStopLine:
                print("Yes, stopline")
                self.stopLineDistance = abs(self.height-y1)
                print("Stopline distance: {}".format(self.stopLineDistance))

                if self.stopLineDistance >= self.lastStopLineDistance:
                    if self.stopRequired:
                        print("Stopping")
                        self.stop = True
                        self.stopRequired = False
                    else:
                        print("Making stop required")
                        self.stopRequired = True
                else:
                    print("Already stopped")
                self.lastStopLineDistance = self.stopLineDistance

                self.stopLine = True

            elif width < self.widthNodeLine:
                if minX < 200:
                    # print("Node to the left")
                    self.stopLine = True

                elif maxX > 400:
                    # print("Node to the right")
                    self.stopLine = True

        else:
            print("No stopline")
            self.stopLine = False
            self.stop = False

        self.stopLineCoordinates = [(x1, y1), (x2, y2)]

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

        boundary = 2/3
        # left lane line segment should be on left 1/3 of the screen
        leftRegionBoundary = self.width * (1 - boundary)
        # right lane line segment should be on left 1/3 of the screen
        rightRegionBoundary = self.width * boundary

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

                    elif (slope < 0.10 and slope > -0.10 and
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

        if len(stopFit) > 5:
            # stopFitAverage = np.average(stopFit, axis=0)
            stopFitAverage = np.median(stopFit, axis=0)
            self.makeStopLine(stopFitAverage, minX, maxX)

        else:
            self.stopLine = False

        try:
            self.lineCenter = ((rightFitAverage[1]-leftFitAverage[1]) /
                               (leftFitAverage[0]-rightFitAverage[0]))

        except Exception:
            self.lineCenter = None
            # print("Not enough lines captured")
            return

    def getCenterOffset(self):
        """Calculate the center offset in frame."""
        # threadStream = VideoStreamFile()  # Creates Video stream
        threadStream = VideoStreamFile()
        threadStream.start()  # Starts Video stream

        while True:

            # t1 = time.time()

            self.img = threadStream.read()  # Retrive image

            # self.undistortImage() # Undistort image

            self.orgImg = self.img  # Save original image

            # Apply gaussian blur

            self.img = cv2.cvtColor(self.img, cv2.COLOR_BGR2GRAY)

            # self.img = cv2.GaussianBlur(self.img, (self.gaussianKernelSize,
            #                                       self.gaussianKernelSize),
            #                            5)

            ret, self.img = cv2.threshold(self.img, 50, 255, cv2.THRESH_BINARY)

            self.displayImage()  # Display image

            # Apply canny
            self.img = cv2.Canny(self.img, self.lowerThreshold,
                                 self.upperThreshold, self.appetureSize)

            self.regionOfInterest()  # Apply ROI

            # self.displayImage()
            # Histogram calc from canny image
            histogram = np.sum(self.img[400:480, 5:635], axis=0)
            self.leftHistogram = np.argmax(histogram[:int(self.center-60)]) + 5
            self.rightHistogram = (np.argmax(histogram[int(self.center+60):]) +
                                   self.center + 65)
            self.midpointHistogram = int((self.rightHistogram -
                                          self.leftHistogram)
                                         / 2 + self.leftHistogram)

            y1 = [(self.leftHistogram, 0), (self.leftHistogram, self.height)]

            y2 = [(self.rightHistogram, 0),
                  (self.rightHistogram, self.height)]

            y3 = [(self.midpointHistogram, 0),
                  (self.midpointHistogram, self.height)]

            # plt.plot(histogram)
            # plt.vlines(self.leftHistogram, ymin=0,
            # ymax=self.height, colors='red')
            # plt.vlines(self.rightHistogram, ymin=0,
            # ymax=self.height, colors='red')
            # plt.show()

            # Apply Hough transfrom
            lineSegments = cv2.HoughLinesP(self.img, self.rho, self.angle,
                                           self.minThreshold,
                                           cv2.HOUGH_PROBABILISTIC,
                                           minLineLength=self.minLineLength,
                                           maxLineGap=self.minLineLength)

            self.lineIntercept(lineSegments)  # Calc line equations

            # t2 = time.time()

            # print("Time elapsed: {} in ms".format((t2-t1)*1000))

            # Add lines to original image
            self.addLines()

            """# CALC X VALUE FOR Y = 10
            if (self.xPointRight and self.xPointLeft):
                self.midpointFromPoints = int(
                    (self.xPointRight - self.xPointLeft)/2 + self.xPointLeft)
                # y4 = [(self.midpointFromPoints, 0),
                #      (self.midpointFromPoints, self.height)]
                # self.drawLine(y4, (0,242,255), 2)
                # print("y1:{} y2:{} y3:{} y4:{}".format(y1,y2,y3,y4))"""

            self.drawLine(y1, (0, 242, 255), 2)  # Left line
            self.drawLine(y2, (0, 242, 255), 2)  # Right line
            # self.drawLine(y3, (128, 0, 128), 2)  # Midpoint line

            self.getDataFromLines()  # Get offset
            y6 = [(self.newOffset, 0), (self.newOffset, self.height)]
            self.drawLine(y6, (200, 144, 255), 2)  # Calculated offset

            # y5 = [(self.newOffset + self.center, 0),
            #      (self.newOffset + self.center, self.height)]

            # self.drawLine(y5, (128,0,128), 2) # Calculated offset

            # steering = int((self.newOffset + self.center)*3/8 - 60)

            # self.displayROI() # Display ROI

            # Display stop line
            if self.stopLine:
                self.drawLine(self.stopLineCoordinates, (0, 0, 255), 5)

            # self.saveImageData() # Save image

            self.displayImage()  # Display image

            # t2 = time.time()

            # print("Time elapsed: {} in ms".format((t2-t1)*1000))

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

    def getDataFromLines(self):
        """Get offset from lines."""
        # HISTOGRAM
        # self.leftHistogram
        # self.rightHistogram
        # self.midpointHistogram

        # POINTS
        # self.xPointLeft
        # self.xPointRight
        # self.midpointFromPoints

        # INTERCEPT
        # self.lineCenter
        # self.slopeRight
        # self.slopeLeft

        # Histogrammet har hittat båda linjerna
        if self.leftHistogram > 0 and self.leftHistogram < 640:
            # Båda linjernas lutning har hittats
            if self.slopeLeft and self.slopeRight:
                print("Case 1")

                # Här kan vi använda alla variabler

                self.newOffset = (0.7 * self.midpointHistogram +
                                  0.3 * self.lineCenter)

            # Endast vänstra linjens lutning har hittats
            elif self.slopeLeft:
                print("Case 2")

                # Här kan vi använda
                # self.leftHistogram
                # self.rightHistogram
                # self.midpointHistogram

                # self.slopeLeft
                # self.xPointLeft

                self.newOffset = (self.midpointHistogram -
                                  1/self.slopeLeft * 350)

            # Endast högra linjens lutning har hittats
            elif self.slopeRight:
                print("Case 3")

                # Här kan vi använda
                # self.leftHistogram
                # self.rightHistogram
                # self.midpointHistogram

                # self.slopeRight
                # self.xPointRight

                # print(self.slopeRight)

                self.newOffset = (self.midpointHistogram -
                                  1/self.slopeRight * 350)

            # Inga lutningar har hittats
            else:
                print("Case 4")

                # Här kan vi använda:
                # self.leftHistogram
                # self.rightHistogram
                # self.midpointHistogram

                self.newOffset = self.midpointHistogram

        # Histogrammet har endast hittat vänstra linjen
        elif self.leftHistogram > 0:
            # Vänstra linjens lutning har hittats
            if self.slopeLeft:
                print("Case 5")

                # Här kan vi använda:
                # self.leftHistogram
                # self.xPointLeft
                # self.slopeLeft

                # Tar reda på hur många pixlar mellan linjer

                midpointHistogram = (
                    self.width - self.leftHistogram)/2 + self.center
                self.newOffset = midpointHistogram

                pass
            # Ingen lutning har hittats
            else:
                print("Case 6")

                # Här kan vi använda:
                # self.leftHistogram

                midpointHistogram = (
                    self.width - self.leftHistogram)/2 + self.center
                self.newOffset = midpointHistogram

                pass

            pass

        # Histogrammet har endast hittat högra linjen
        elif self.rightHistogram < 640:
            # Högra linjens lutning har hittats
            if self.slopeRight:
                print("Case 7")

                # Här kan vi anväda:
                # self.rightHistogram
                # self.xPointRight
                # self.slopeRight

                midpointHistogram = self.center - (self.rightHistogram)/2
                self.newOffset = midpointHistogram

                pass
            # Ingen lutning har hittats
            else:
                print("Case 8")

                # Här kan vi använda:
                # self.rightHistogram
                midpointHistogram = self.center - (self.rightHistogram)/2
                self.newOffset = midpointHistogram

                pass

        else:
            print("Nothing detected")
            self.newOffset = self.lastOffset

            return

        # print(self.newOffset)
        # print(self.lastOffset)

        # self.newOffset -= self.center

        # if self.lastOffset:
        #    if abs((self.newOffset-self.lastOffset)/self.lastOffset) > 0.10:
        #        print("Change in offset to large")
        #        self.newOffset = self.lastOffset
        #        return

        self.newOffset = int(self.newOffset)

        self.lastOffset = self.newOffset

    def turnRightJunction(self):
       # turn right at junction

        return

    def turnLeftJunction(self):
        # turn left at junction

        return

    def straightJunction(self):
        # go straight at junction

        return
