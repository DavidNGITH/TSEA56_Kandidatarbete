import time
import numpy as np
import cv2
from videoStream import VideoStream
import multiprocessing
from datetime import datetime
import matplotlib.pyplot as plt

from videoStreamFile import VideoStreamFile


class compVision:
    def __init__(self, roiPerc, resolution):
        #This is the main class of this file. It contains all the functions needed to process an image.
        #The class is initialized with the ROI percentage and the resolution of the image.
        #The ROI percentage is a list of 8 values. The first 4 values are the percentage of the image width and height for the upper left corner of the ROI.
        #The last 4 values are the percentage of the image width and height for the lower right corner of the ROI.
        #The resolution is a tuple of the width and height of the image.
        #The class contains the following functions:
        #   - displayImage: Displays the image stored in self.img
        #   - undistortImage: Undistorts the image stored in self.img
        #   - regionOfInterest: Applies a ROI to the image stored in self.img
        #   - canny: Applies the canny edge detection algorithm to the image stored in self.img
        #   - houghLines: Applies the hough lines algorithm to the image stored in self.img
        #   - findLines: Finds the lane lines in the image stored in self.img
        #   - findStopLine: Finds the stop line in the image stored in self.img
        #   - findLaneCenter: Finds the center of the lane in the image stored in self.img
        #Variables
        self.resolution = resolution
        self.width = resolution[0]
        self.height = resolution[1]
        self.center = int(self.width/2)

        self.img = None
        self.laneLines = []
        self.lineCenter = None

        #ROIDIM: Upperleft, UpperRight, LowerRight, LowerLeft
        roiP1X = int(roiPerc[0] * self.width)
        roiP1Y = int(roiPerc[1]  * self.height)
        roiP2X = int(roiPerc[2]  * self.width)
        roiP2Y = int(roiPerc[3]  * self.height)
        roiP3X = int(roiPerc[4]  * self.width)
        roiP3Y = int(roiPerc[5]  * self.height)
        roiP4X = int(roiPerc[6]  * self.width)
        roiP4Y = int(roiPerc[7]  * self.height)
        self.roiDim = [(roiP1X,roiP1Y), (roiP2X,roiP2Y), (roiP3X,roiP3Y), (roiP4X,roiP4Y)]

        


        #Canny settings
        self.lowerThreshold = 200
        self.upperThreshold = 300
        self.appetureSize = 3

        #Rho settings
        self.rho = 1
        self.angle = np.pi / 180
        self.minThreshold = 0
        self.minLineLength = 8
        self.maxLineGap = 4


        #Undistort
        self.my = np.load('mapy.npy')
        self.mx = np.load('mapx.npy')
        self.roiFile = np.load('roiFile.npy')

        self.roiFile[1] = int(self.roiFile[1] * self.width / 640)
        self.roiFile[3] = int(self.roiFile[3] * self.width / 640)
        self.roiFile[0] = int(self.roiFile[0] * self.height / 480)
        self.roiFile[2] = int(self.roiFile[2] * self.height / 480)
        
        #Find lines
        self.laneLines = []
        self.stopLine = None
        self.xPointRight = None
        self.xPointLeft = None





    #Displays image stored in self.img
    def displayImage(self):
        cv2.imshow("Bild", self.img)
        cv2.waitKey()
        cv2.destroyAllWindows()

    def undistortImage(self):
        self.img = cv2.remap(self.img, self.mx, self.my, cv2.INTER_LINEAR)
        self.img = self.img[self.roiFile[1]:self.roiFile[1]+ self.roiFile[3],self.roiFile[0]:self.roiFile[0]+ self.roiFile[2]]

    def regionOfInterest(self):

        mask = np.zeros_like(self.img)

        #(Left Top) (Right 
        # 
        # Top) (Right Bottom) (Left Bottom)
        polygon = np.array([self.roiDim], np.int32)
        cv2.fillPoly(mask, polygon, 255)
        # applying mask on original image
    
        self.img = cv2.bitwise_and(self.img, mask) 
    
    def makePoints(self, line, which):

        slope, intercept = line
        y1 = self.height  # bottom of the frame
        y2 = int(y1 * 0)  # make points from middle of the frame down

        # bound the coordinates within the frame
        x1 = max(-self.width, min(2 * self.width, int((y1 - intercept) / slope)))
        x2 = max(-self.width, min(2 * self.width, int((y2 - intercept) / slope)))
        
        # 1 = Right
        if which:
            self.xPointRight = int((100-intercept)/slope)
        # 0 = Left
        else:
            self.xPointLeft  = int((100-intercept)/slope)

        return [[x1, y1, x2, y2]]
    
    def makeStopLine(self, line, minX, maxX):
    
        slope, intercept = line
        
        x1 = int(minX)
        x2 = int(maxX)
        
        y1 = int(slope * minX + intercept)
        y2 = int(slope * maxX + intercept)
        
        
        self.stopLine = [(x1,y1), (x2,y2)]

    
    def lineIntercept(self, lineSegments):

        self.laneLines = []
        if lineSegments is None:
            #print('No line_segment segments detected')
            return self.laneLines

        leftFit = []
        rightFit = []
        stopFit = []
        
        minX = 1000
        maxX = 0

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
                            
                    elif slope < 0.1 and slope > -0.1 and x1 > 0.15 * self.width and x2 < 0.85 * self.width:
                        stopFit.append((slope, intercept))
                        if x1 < minX:
                            minX = x1
                        if x2 > maxX:
                            maxX = x2

        
        
        
        leftFitAverage = np.average(leftFit, axis=0)
        if len(leftFit) > 0:
            self.laneLines.append(self.makePoints(leftFitAverage, 0))
            #print("Left slope: k {}, m {}".format(leftFitAverage[0], leftFitAverage[1]))


        rightFitAverage = np.average(rightFit, axis=0)
        if len(rightFit) > 0:
            self.laneLines.append(self.makePoints(rightFitAverage, 1))
            #print("Right slope: k {}, m {}".format(rightFitAverage[0], rightFitAverage[1]))
        
        if len(stopFit) > 10:
            stopFitAverage = np.average(stopFit, axis=0)
            self.makeStopLine(stopFitAverage, minX, maxX)
            #print("Stop line: k {}, m {}".format(stopFitAverage[0], stopFitAverage[1]))

            
        else:
            self.stopLine = None
    
        try:
            self.lineCenter = (rightFitAverage[1]-leftFitAverage[1])/(leftFitAverage[0]-rightFitAverage[0])

        except:
            self.lineCenter = None
            #print("Not enough lines captured")
            return
    def birdsView(self):

        src = np.float32([[0, 200], [self.width, 200], [0, self.height], [self.width , self.height]]) # Four source coordinates Top left, top right, bottom left, bottom right
        dst = np.float32([[0, 0], [self.width, 0], [0, self.height], [self.width, self.height]])
        M = cv2.getPerspectiveTransform(src, dst) # The transformation matrix

        self.img = self.img[450:(450+self.height), 0:self.width] # Apply np slicing for ROI crop
        self.img = cv2.warpPerspective(self.img, M, (self.width, self.height)) # Image warping
        self.img = cv2.cvtColor(self.img, cv2.COLOR_BGR2RGB) # Show results

    def getCenterOffset(self, q : multiprocessing.Queue, statusValue : multiprocessing.Value):
        #Starting Video stream
        #threadStream = VideoStream(self.resolution)
        threadStream = VideoStreamFile()
        threadStream.start()
        
        status = statusValue.value
        
        while status:
            
            t1 = time.time()
            self.img = threadStream.read()
            
            self.undistortImage()
            self.orgImg = self.img
            self.birdsView()
            self.displayImage()
            self.img = cv2.Canny(self.img, self.lowerThreshold, self.upperThreshold, self.appetureSize)
            
            #self.displayImage()

            self.regionOfInterest()
            
            histogram = np.sum(self.img, axis =0)
            
            leftXBase = np.argmax(histogram[:self.center])
            
            rightXBase = np.argmax(histogram[self.center:]) + self.center
            
            midpointHistogram = int((rightXBase - leftXBase) / 2 + leftXBase )
            
            
            #plt.plot(histogram)
            #plt.vlines(leftXBase, ymin=0, ymax=self.height, colors = 'red')
            #plt.vlines(rightXBase, ymin=0, ymax=self.height, colors = 'red')

            #plt.show()

            lineSegments = cv2.HoughLinesP(self.img, self.rho, self.angle, self.minThreshold, cv2.HOUGH_PROBABILISTIC,
                                    minLineLength=self.minLineLength, maxLineGap=self.minLineLength)
        
            self.lineIntercept(lineSegments)
            
            t2 = time.time()
        
            #print("Time elapsed: {} in ms".format((t2-t1)*1000))
            self.addLines()

            if(self.xPointRight and self.xPointLeft):
                midpointFromLines = int((self.xPointRight - self.xPointLeft)/2 + self.xPointLeft)
                #y4 = [(midpointFromLines, 0), (midpointFromLines, self.height)]
                #self.drawLine(y4, (0,242,255), 2)
                y1 = [(leftXBase, 0), (leftXBase, self.height)]
                y2 = [(rightXBase, 0), (rightXBase, self.height)]
                y3 = [(midpointHistogram, 0), (midpointHistogram, self.height)]

                self.drawLine(y1, (0,242,255), 2)
                self.drawLine(y2, (0,242,255), 2)
                self.drawLine(y3, (128,0,128), 2)
            
            
            
            
            
                        

            self.displayROI()
            if self.stopLine:
                self.drawLine(self.stopLine, (0,0,255), 5)
                
            


            #self.saveImageData()
            self.displayImage()
            #print(self.lineCenter - self.center)
                        
            try:
                if((self.lineCenter - self.center) > 0):
                    #print("Turn Right")
                    pass

                else:
                    #print("Turn Left")
                    pass
                    
                q.put(self.lineCenter - self.center)
            except:
                #print("No lines detected")
                pass
            
            status = statusValue.value

                
        print("Stopped again compVision")
        #print(self.status)
        threadStream.stop()
           

    def saveImageData(self):
        turnInstruction = None

        try:
            if((self.lineCenter - self.center) > 0):
                turnInstruction = "TurnRight"

            else:
                turnInstruction = "TurnLeft"
        except:
            turnInstruction = "NoLines"

        self.img = cv2.putText(
            img = self.img,
            text = turnInstruction,
            org = (int(0.10*self.width), int(0.9 * self.height)),
            fontFace = cv2.FONT_HERSHEY_DUPLEX,
            fontScale = 0.5,
            color = (125, 246, 55),
            thickness = 1
        )

        cv2.imwrite("savedImages/{}_{}.jpeg".format(turnInstruction,str(datetime.now())), self.img)
                    

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

    def displayROI(self):
        self.img = cv2.line(self.img, self.roiDim[0], self.roiDim[1], (0,140,255), 2)
        self.img = cv2.line(self.img, self.roiDim[1], self.roiDim[2], (0,140,255), 2)
        self.img = cv2.line(self.img, self.roiDim[2], self.roiDim[3], (0,140,255), 2)
        self.img = cv2.line(self.img, self.roiDim[3], self.roiDim[0], (0,140,255), 2)
        print("ROI coordinates: {}, {}, {}, {}".format(self.roiDim[0], self.roiDim[1], self.roiDim[2], self.roiDim[3]))


    def drawLine(self, coordinates, color, width):
            self.img = cv2.line(self.img, coordinates[0], coordinates[1], color, width)