from picamera import PiCamera
from picamera.array import PiRGBArray
from time import sleep
import numpy as np
import cv2
from videoStream import VideoStream


class compVision:
    def __init__(self, resolution):
        print(cv2.useOptimized())
        self.resolution = resolution
        self.width = resolution[0]
        self.height = resolution[1]
        self.center = self.width/2

        self.img = None
        #ROIDIM: Upperleft, UpperRight, LowerRight, LowerLeft 
        self.roiDim = [(0,62), (640,62), (0,355), (640,355)]

        self.lowerThreshold = 200
        self.upperThreshold = 300
        self.appetureSize = 3

        self.rho = 1
        self.angle = np.pi / 180
        self.minThreshold = 0
        self.minLineLength = 8
        self.maxLineGap = 4

        self.laneLines = []
        self.lineCenter = None
        self.threadStream = VideoStream()
        self.threadStream.start()

    def displayImage(self):
        cv2.imshow("Bild", self.img)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    def regionOfInterest(self):
        
        #roi = cv2.selectROI(self.img)
        
        #print(roi)

        mask = np.zeros_like(self.img)

        #Vänster topp, Höger topp, Höger botten, Vänster botten
        polygon = np.array([self.roiDim], np.int32)
        cv2.fillPoly(mask, polygon, 255)
        # applying mask on original image
    
        self.img = cv2.bitwise_and(self.img, mask) 
    
    def makePoints(self, line):

        slope, intercept = line
        y1 = self.height  # bottom of the frame
        y2 = int(y1 * 0)  # make points from middle of the frame down

        # bound the coordinates within the frame
        x1 = max(-self.width, min(2 * self.width, int((y1 - intercept) / slope)))
        x2 = max(-self.width, min(2 * self.width, int((y2 - intercept) / slope)))

        return [[x1, y1, x2, y2]]

    
    def lineIntercept(self, lineSegments):

        self.laneLines = []
        if lineSegments is None:
            print('No line_segment segments detected')
            return self.laneLines

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
        try:
            self.lineCenter = (rightFitAverage[1]-leftFitAverage[1])/(leftFitAverage[0]-rightFitAverage[0])

        except:
            self.lineCenter = None
            print("Not enough lines captured")
            return

    

    def getCenterOffset(self):
        self.img = self.threadStream.read()
        self.orgImg = self.img

        self.img = cv2.Canny(self.img, self.lowerThreshold, self.upperThreshold, self.appetureSize)

        self.regionOfInterest()

        lineSegments = cv2.HoughLinesP(self.img, self.rho, self.angle, self.minThreshold, cv2.HOUGH_PROBABILISTIC,
                                 minLineLength=self.minLineLength, maxLineGap=self.minLineLength)

        self.lineIntercept(lineSegments)
        
        #print(self.lineCenter - self.center)

        #return (lineCenter - self.center)
        try:
            if((self.lineCenter - self.center) > 0):
                print("Turn Right")
            else:
                print("Turn Left")
        except:
            print("No lines detected")
           

    def addLines(self):
        lineImage = np.zeros_like(self.orgImg)
        if self.laneLines is not None:
            for line in self.laneLines:
                for x1, y1, x2, y2 in line:
                    cv2.line(lineImage, (x1, y1), (x2, y2), (0,0,255), 2)

        if (self.lineCenter != None):
            cv2.line(lineImage, (int(self.lineCenter), 0), (int(self.lineCenter), int(self.height)), (0,255,0), 2)

        cv2.line(lineImage, (int(self.center), 0), (int(self.center), int(self.height)), (255,0,0), 2)
        self.img = cv2.addWeighted(self.orgImg, 0.8, lineImage, 1, 1)

    def stopProcess(self):
        self.threadStream.stop()


