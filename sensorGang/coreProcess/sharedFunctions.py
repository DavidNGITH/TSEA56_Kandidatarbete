"""Functions in semi and auto."""
import numpy as np
import cv2


def lineInterceptFunction(self, lineSegments):
    """Handle detected lines from Hough transform."""
    leftFitAverage = None
    rightFitAverage = None
    leftFit = []
    rightFit = []
    stopFit = []
    self.slopeLeft = 0
    self.slopeRight = 0

    if lineSegments is None:
        # print('No line_segment segments detected')
        return

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

                elif (slope < 0.4 and slope > -0.4 and
                      x1 > 0.15 * self.width and x2 < 0.85 * self.width):

                    stopFit.append((slope, intercept))
                    if x1 < minX:
                        minX = x1
                    if x2 > maxX:
                        maxX = x2

    if len(leftFit) > 2:
        leftFitAverage = np.average(leftFit, axis=0)
        self.slopeLeft = leftFitAverage[0]

    if len(rightFit) > 2:
        rightFitAverage = np.average(rightFit, axis=0)
        self.slopeRight = rightFitAverage[0]

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


def regionOfInterestFunction(self):
    """ROI on self.img."""
    mask = np.zeros_like(self.img)

    # (Left Top) (Right Top) (Right Bottom) (Left Bottom)
    polygon = np.array([self.roiDim], np.int32)
    cv2.fillPoly(mask, polygon, 255)
    # applying mask on original image
    self.img = cv2.bitwise_and(self.img, mask)


def getDataFromLinesFunction(self):
    """Get offset from lines."""
    self.currentSpeed = self.turningSpeed

    self.lastOffset = self.newOffset

    # Histogrammet har hittat båda linjerna
    if self.leftHistogram is not None and self.rightHistogram is not None:
        # Båda linjernas lutning har hittats
        if self.slopeLeft and self.slopeRight:
            casePrint = "Case 1"
            self.newOffset = (0.6 * self.midpointHistogram +
                              0.5 * self.lineCenter)
            self.currentSpeed = self.normalSpeed

        # Endast vänstra linjens lutning har hittats
        elif self.slopeLeft:
            casePrint = "Case 2"
            self.newOffset = (self.midpointHistogram -
                              1/self.slopeLeft * 420)

        # Endast högra linjens lutning har hittats
        elif self.slopeRight:
            casePrint = "Case 3"
            self.newOffset = (self.midpointHistogram -
                              1/self.slopeRight * 380)

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

        # Ingen lutning har hittats
        else:
            casePrint = "Case 6"
            midpointHistogram = (
                self.width - self.leftHistogram)/2 + self.center
            self.newOffset = midpointHistogram

    # Histogrammet har endast hittat högra linjen
    elif self.rightHistogram is not None:
        # Vänstra linjens lutning har hittats
        if self.slopeRight:
            casePrint = "Case 7"
            midpointHistogram = self.center - (self.rightHistogram)/2
            self.newOffset = midpointHistogram

        # Ingen lutning har hittats
        else:
            casePrint = "Case 8"
            midpointHistogram = self.center - (self.rightHistogram)/2
            self.newOffset = midpointHistogram

    else:
        # print("Case 9")
        self.newOffset = self.lastOffset

        return

    # print(casePrint)

    self.newOffset -= (self.center+40)

    self.newOffset = int(self.newOffset)


def getOffsetLeftTurnFunction(self):
    """Get offset on left turn."""
    self.currentSpeed = self.turningSpeed + 15
    if self.leftHistogram is not None:
        self.newOffset = (self.leftHistogram - 115)
        if self.newOffset >= 0:
            return
        else:
            self.newOffset *= 3.5
    else:
        self.newOffset = - 150


def getOffsetRightTurnFunction(self):
    """Get offset on right turn."""
    self.currentSpeed = self.turningSpeed + 15
    if self.rightHistogram is not None:
        self.newOffset = (self.rightHistogram - 530)
        if self.newOffset <= 0:
            return
        else:
            self.newOffset *= 3.5
    else:
        self.newOffset = 150


def setupFunction(self):
    """Settings and variables"""
    self.img = None
    self.lineCenter = None
    self.newOffset = 0
    self.lastOffset = 0

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

    # Histogram
    self.leftHistogram = 0
    self.rightHistogram = 0
    self.midpointHistogram = 0

    # Find lines
    self.slopeLeft = None
    self.slopeRight = None

    # Stop lines coordinates
    self.widthStopLine = 220
    self.widthNodeLine = 170

    # Cases
    self.slowDownTimer = 0
    self.normalSteering = True

    # Speed
    self.normalSpeed = 100
    self.turningSpeed = 85
    self.currentSpeed = self.normalSpeed
    self.lastSpeed = self.normalSpeed
    self.speedToSend = None
    self.intersectionTime = 0

    # Get offset
    self.getOffset = self.getDataFromLines

    self.casesDict = {
        0: self.getDataFromLines,
        1: self.getOffsetLeftTurn,
        2: self.getOffsetRightTurn
    }
