from picamera import PiCamera
from picamera.array import PiRGBArray
from time import sleep
import numpy as np
import cv2


class compVision:
    def __init__(self, resolution):
        self.resoultion = resolution
        self.width = resolution(0)
        self.height = resolution(1)
        self.center = self.width/2
        self.camera = PiCamera()
        self.cameraRaw = PiRGBArray(self.camera, self.resoultion)
        self.img = np.empty((self.height, self.width, 3), dtype=np.uint8)

        self.roiDim = [(0,0), (self.width,0), (self.width,self.height), (0,self.height)]

        self.lowerThreshold = 200
        self.upperThreshold = 300
        self.appetureSize = 3

        self.rho = 1
        self.angle = np.pi / 180
        self.minThreshold = 0
        self.minLineLength = 8
        self.maxLineGap = 4

        self.laneLines = []
    def displayImage(self):
        cv2.imshow("Bild", self.img)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    def takePicture(self):
        self.camera.capture(self.img, 'rgb')
    
    def regionOfInterest(self):
        mask = np.zeros_like(self.img)

        #Vänster topp, Höger topp, Höger botten, Vänster botten
        polygon = np.array([self.roiDim], np.int32)
        cv2.fillPoly(mask, polygon, 255)
        # applying mask on original image
    
        self.image = cv2.bitwise_and(self.img, mask) 
    
    def makePoints(self, line):

        slope, intercept = line
        y1 = self.height  # bottom of the frame
        y2 = int(y1 * 0)  # make points from middle of the frame down

        # bound the coordinates within the frame
        x1 = max(-self.width, min(2 * self.width, int((y1 - intercept) / slope)))
        x2 = max(-self.width, min(2 * self.width, int((y2 - intercept) / slope)))

        return [[x1, y1, x2, y2]]

    
    def LineIntercept(self, lineSegments):

        self.laneLines = []
        if lineSegments is None:
            print('No line_segment segments detected')
            return laneLines

        leftFit = []
        rightFit = []

        boundary = 1/3
        leftRegionBoundary = self.width * (1 - boundary)  # left lane line segment should be on left 2/3 of the screen
        rightRegionBoundary = self.width * boundary # right lane line segment should be on left 2/3 of the screen

        for lineSegment in lineSegments:
            for x1, y1, x2, y2 in lineSegment:
                if x1 != x2:
                    fit = np.polyfit((x1, x2), (y1, y2), 1)
                    slope = fit[0]
                    intercept = fit[1]
                    if slope < -1.5:
                        if x1 < leftRegionBoundary and x2 < leftRegionBoundary:
                            leftFit.append((slope, intercept))
                    elif slope > 1.5:
                        if x1 > rightRegionBoundary and x2 > rightRegionBoundary:
                            rightFit.append((slope, intercept))


        leftFitAverage = np.average(leftFit, axis=0)
        if len(leftFit) > 0:
            self.laneLines.append(self.makePoints(leftFitAverage))

        rightFitAverage = np.average(rightFit, axis=0)
        if len(rightFit) > 0:
            self.laneLines.append(self.makePoints(rightFitAverage))

        lineCenter = (rightFitAverage[1]-leftFitAverage[1])/(leftFitAverage[0]-rightFitAverage[0])

        return(lineCenter)
    

    def getCenterOffset(self):
        self.takePicture()

        self.img = cv2.Canny(self.img, self.lowerThreshold, self.upperThreshold, self.appetureSize)

        self.regionOfInterest()

        lineSegments = cv2.HoughLinesP(self.img, self.rho, self.angle, self.minThreshold, np.array([]),
                                 minLineLength=self.minLineLength, maxLineGap=self.minLineLength)

        lineCenter = self.lineIntercept(lineSegments)

        return (lineCenter - self.center)


