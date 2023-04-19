import time
import math
import numpy as np
import cv2
from videoStream import VideoStream
import multiprocessing
from datetime import datetime
import matplotlib.pyplot as plt
from PD_reg import PDcontroller

from videoStreamFile import VideoStreamFile


class compVision:
    def __init__(self, PD, roiPerc, resolution):

        #Variables
        self.resolution = resolution
        self.width = resolution[0]
        self.height = resolution[1]
        self.center = int(self.width/2)

        self.img = None
        self.laneLines = []
        self.lineCenter = None
        self.lastOffset = None
        self.newOffset = None

        #ROIDIM: Upperleft, UpperRight, LowerRight, LowerLeft
        roiP1X = int(roiPerc[0]  * self.width)
        roiP1Y = int(roiPerc[1]  * self.height)
        roiP2X = int(roiPerc[2]  * self.width)
        roiP2Y = int(roiPerc[3]  * self.height)
        roiP3X = int(roiPerc[4]  * self.width)
        roiP3Y = int(roiPerc[5]  * self.height)
        roiP4X = int(roiPerc[6]  * self.width)
        roiP4Y = int(roiPerc[7]  * self.height)
        self.roiDim = [(roiP1X,roiP1Y), (roiP2X,roiP2Y), (roiP3X,roiP3Y), (roiP4X,roiP4Y)]

        


        #Canny settings
        self.lowerThreshold = 100
        self.upperThreshold = 200
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
        self.stopLine = False
        self.xPointRight = None
        self.xPointLeft = None
        self.slopeLeft = None
        self.slopeRight = None
        self.stopAtLine = True
        self.stopLineTimer = 0
        
        ##PD-Controller
        self.PD = PD



    #Displays image stored in self.img
    def displayImage(self):
        cv2.imshow("Bild", self.img)
        cv2.waitKey()
        cv2.destroyAllWindows()

    # Undistortioon
    def undistortImage(self):
        self.img = cv2.remap(self.img, self.mx, self.my, cv2.INTER_LINEAR)
        self.img = self.img[self.roiFile[1]:self.roiFile[1]+ self.roiFile[3],self.roiFile[0]:self.roiFile[0]+ self.roiFile[2]]

    # Region of interest
    def regionOfInterest(self):

        mask = np.zeros_like(self.img)

        #(Left Top) (Right 
        # 
        # Top) (Right Bottom) (Left Bottom)
        polygon = np.array([self.roiDim], np.int32)
        cv2.fillPoly(mask, polygon, 255)
        # applying mask on original image
    
        self.img = cv2.bitwise_and(self.img, mask) 
    
    # 
    def makePoints(self, line, which):

        slope, intercept = line
        y1 = self.height  # bottom of the frame
        y2 = int(y1 * 0)  # make points from middle of the frame down

        # bound the coordinates within the frame
        x1 = max(-self.width, min(2 * self.width, int((y1 - intercept) / slope)))
        x2 = max(-self.width, min(2 * self.width, int((y2 - intercept) / slope)))
        

        #Find x value for y = 100
        # 1 = Right
        if which:
            self.xPointRight = int((100-intercept)/slope)
        # 0 = Left
        else:
            self.xPointLeft  = int((100-intercept)/slope)

        return [[x1, y1, x2, y2]]
    
    def makeStopLine(self, line, minX, maxX):
    
        slope, intercept = line

        y1 = int(slope * minX + intercept)
        y2 = int(slope * maxX + intercept)


        if (maxX-minX) < 100 and (y1+y2)/2 < 400:
            self.stopLine = False
        
        #Stopplinje detekterad
        else:
            if self.stopAtLine:
                self.stopLine = True
                self.stopAtLine = False
                self.stopLineTimer = time.time()

            elif time.time() - self.stopLineTimer > 5:
                self.stopAtLine = True

        return
        
        x1 = int(minX)
        x2 = int(maxX)
        
        y1 = int(slope * minX + intercept)
        y2 = int(slope * maxX + intercept)
        
        
        self.stopLine = [(x1,y1), (x2,y2)]

    
    def lineIntercept(self, lineSegments):
        self.laneLines = []
        self.xPointLeft = None
        self.xPointRight = None
        self.slopeLeft = 0
        self.slopeRight = 0

        if lineSegments is None:
            #print('No line_segment segments detected')
            return self.laneLines

        leftFit = []
        rightFit = []
        stopFit = []
        
        minX = 1000
        maxX = 0
        
        boundary = 2/3
        leftRegionBoundary = self.width * (1 - boundary)  # left lane line segment should be on left 1/3 of the screen
        rightRegionBoundary = self.width * boundary # right lane line segment should be on left 1/3 of the screen

        for lineSegment in lineSegments:
            for x1, y1, x2, y2 in lineSegment:
                if x1 != x2:
                    fit = np.polyfit((x1, x2), (y1, y2), 1)
                    slope = fit[0]
                    intercept = fit[1]
                    if slope < -1:
                        if x1 < leftRegionBoundary and x2 < leftRegionBoundary:
                            leftFit.append((slope, intercept))
                    elif slope > 1:
                        if x1 > rightRegionBoundary and x2 > rightRegionBoundary:
                            rightFit.append((slope, intercept))
                            
                    elif slope < 0.1 and slope > -0.1 and x1 > 0.15 * self.width and x2 < 0.85 * self.width:
                        stopFit.append((slope, intercept))
                        if x1 < minX:
                            minX = x1
                        if x2 > maxX:
                            maxX = x2

        
        
        
        
        if len(leftFit) > 2:
            leftFitAverage = np.average(leftFit, axis=0)
            self.slopeLeft = leftFitAverage[0]
            self.laneLines.append(self.makePoints(leftFitAverage, 0))
            #print("Left slope: k {}, m {}".format(leftFitAverage[0], leftFitAverage[1]))


        
        if len(rightFit) > 2:
            rightFitAverage = np.average(rightFit, axis=0)
            self.slopeRight = rightFitAverage[0]
            self.laneLines.append(self.makePoints(rightFitAverage, 1))
            #print("Right slope: k {}, m {}".format(rightFitAverage[0], rightFitAverage[1]))
        
        if len(stopFit) > 6:
            stopFitAverage = np.average(stopFit, axis=0)
            self.makeStopLine(stopFitAverage, minX, maxX)
            #print("Stop line: k {}, m {}".format(stopFitAverage[0], stopFitAverage[1]))

            
        else:
            self.stopLine = False
    
        try:
            self.lineCenter = (rightFitAverage[1]-leftFitAverage[1])/(leftFitAverage[0]-rightFitAverage[0])

        except:
            self.lineCenter = None
            #print("Not enough lines captured")
            return

    

    def getCenterOffset(self, qSteering : multiprocessing.Queue, statusValue : multiprocessing.Value, qSpeed: multiprocessing.Queue):
        #Starting Video stream
        threadStream = VideoStream(self.resolution)
        #threadStream = VideoStreamFile()
        threadStream.start()
        
        status = statusValue.value
        
        while status:
            
            #t1 = time.time()
            
            # RETRIVE
            self.img = threadStream.read()
            

            
            # UNDISTORT
            #self.undistortImage()

            # SAVE ORIGINIAL IMAGE
            #self.orgImg = self.img

            # APPLY CANNY
            self.img = cv2.Canny(self.img, self.lowerThreshold, self.upperThreshold, self.appetureSize)
            
            #self.displayImage()

            # APPLY ROI
            self.regionOfInterest()
            
            # HISTOGRAM CALC FROM CANNY
            histogram = np.sum(self.img[400:480,5:635], axis =0)
            self.leftHistogram = np.argmax(histogram[:int(self.center)]) + 5
            self.rightHistogram = np.argmax(histogram[int(self.center):]) + self.center + 5
            self.midpointHistogram = int((self.rightHistogram - self.leftHistogram) / 2 + self.leftHistogram )

            #y1 = [(self.leftHistogram, 0), (self.leftHistogram, self.height)]
            #y2 = [(self.rightHistogram, 0), (self.rightHistogram, self.height)]
            #y3 = [(self.midpointHistogram, 0), (self.midpointHistogram, self.height)]
            
            """plt.plot(histogram)
            #plt.vlines(self.leftHistogram, ymin=0, ymax=self.height, colors = 'red')
            #plt.vlines(self.rightHistogram, ymin=0, ymax=self.height, colors = 'red')
            #plt.show()"""
          
            # APPLY HOUGH
            lineSegments = cv2.HoughLinesP(self.img, self.rho, self.angle, self.minThreshold, cv2.HOUGH_PROBABILISTIC,
                                    minLineLength=self.minLineLength, maxLineGap=self.minLineLength)
        
            # CALC EQUATIONS FOR LINES
            self.lineIntercept(lineSegments)
            
            #t2 = time.time()
        
            #print("Time elapsed: {} in ms".format((t2-t1)*1000))
            
            

            # ADD LINES TO ORIGINAL IMG
            #self.addLines()
            

            # CALC X VALUE FOR Y = 10
            if(self.xPointRight and self.xPointLeft):
                self.midpointFromPoints = int((self.xPointRight - self.xPointLeft)/2 + self.xPointLeft)
                #y4 = [(self.midpointFromPoints, 0), (self.midpointFromPoints, self.height)]
                #self.drawLine(y4, (0,242,255), 2)
                #print("y1:{} y2:{} y3:{} y4:{}".format(y1,y2,y3,y4))


            #self.drawLine(y1, (0,242,255), 2) # Left line
            #self.drawLine(y2, (0,242,255), 2) # Right line
            #self.drawLine(y3, (128,0,128), 2) # Midpoint line
                
            self.getDataFromLines()

            #y5 = [(self.newOffset + self.center, 0), (self.newOffset + self.center, self.height)]

            #self.drawLine(y5, (128,0,128), 2) # Calculated offset
            
            #steering = int((self.newOffset + self.center)*3/8 - 60)

            if not self.stopLine:
                steering = self.PD.get_control(self.newOffset)
                steering = int((self.newOffset)*0.2 + 60)
                
                #print(self.newOffset)
                #print(steering)
                
                if steering < 0:
                    steering = 0
                    
                elif steering > 120:
                    steering = 120
                
                #print("Steering: {}".format(steering))
                                        
                qSteering.put(steering)

            else:
                qSpeed.put(0)
                self.stopLine = False
                print("Stop line detected")
            
                   
            # DISPLAY ROI
            #self.displayROI()

            # DISPLAY STOP LINE
            #if self.stopLine:
               #self.drawLine(self.stopLine, (0,0,255), 5)
                
            

            # SAVE IMAGE
            #self.saveImageData()

            # DISPLAY IMAGE
            #self.displayImage()

            
            
            status = statusValue.value
            
            #t2 = time.time()
        
            #print("Time elapsed: {} in ms".format((t2-t1)*1000))
            


        # STOP STREAM
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
            pass
            #cv2.line(lineImage, (int(self.lineCenter), 0), (int(self.lineCenter), int(self.height)), (0,255,0), 2)

        cv2.line(lineImage, (int(self.center), 0), (int(self.center), int(self.height)), (255,0,0), 2)
        self.img = cv2.addWeighted(self.orgImg, 0.8, lineImage, 1, 1)

    def displayROI(self):
        self.img = cv2.line(self.img, self.roiDim[0], self.roiDim[1], (0,140,255), 2)
        self.img = cv2.line(self.img, self.roiDim[1], self.roiDim[2], (0,140,255), 2)
        self.img = cv2.line(self.img, self.roiDim[2], self.roiDim[3], (0,140,255), 2)
        self.img = cv2.line(self.img, self.roiDim[3], self.roiDim[0], (0,140,255), 2)
        
        #print("ROI coordinates: {}, {}, {}, {}".format(self.roiDim[0], self.roiDim[1], self.roiDim[2], self.roiDim[3]))


    def drawLine(self, coordinates, color, width):
            self.img = cv2.line(self.img, coordinates[0], coordinates[1], color, width)

    def getDataFromLines(self):

        # HISTOGRAM
        #self.leftHistogram
        #self.rightHistogram
        #self.midpointHistogram

        # POINTS
        #self.xPointLeft
        #self.xPointRight
        #self.midpointFromPoints

        # INTERCEPT
        #self.lineCenter
        #self.slopeRight
        #self.slopeLeft


        # Histogrammet har hittat båda linjerna
        if self.leftHistogram > 0 and self.leftHistogram < 640:
            # Båda linjernas lutning har hittats
            if self.slopeLeft and self.slopeRight:
                #print("Case 1")

                #Här kan vi använda alla variabler

                self.newOffset = 0.7 * self.midpointHistogram  + 0.3 * self.lineCenter

                pass
            #Endast vänstra linjens lutning har hittats
            elif self.slopeLeft:
                #print("Case 2")

                #Här kan vi använda
                #self.leftHistogram
                #self.rightHistogram
                #self.midpointHistogram

                #self.slopeLeft
                #self.xPointLeft

                self.newOffset = self.midpointHistogram - 1/self.slopeLeft * 350

                pass
            #Endast högra linjens lutning har hittats
            elif self.slopeRight:
                #print("Case 3")

                #Här kan vi använda
                #self.leftHistogram
                #self.rightHistogram
                #self.midpointHistogram

                #self.slopeRight
                #self.xPointRight
                
                #print(self.slopeRight)
                
                
                self.newOffset = self.midpointHistogram - 1/self.slopeRight * 350
                
                pass
            #Inga lutningar har hittats
            else:
                #print("Case 4")

                #Här kan vi använda:
                #self.leftHistogram
                #self.rightHistogram
                #self.midpointHistogram

                self.newOffset = self.midpointHistogram

                pass

            pass
        
        # Histogrammet har endast hittat vänstra linjen
        elif self.leftHistogram > 0:
            # Vänstra linjens lutning har hittats
            if self.slopeLeft:
                #print("Case 5")

                #Här kan vi använda:
                #self.leftHistogram
                #self.xPointLeft
                #self.slopeLeft

                #Tar reda på hur många pixlar mellan linjer

                midpointHistogram = (self.width - self.leftHistogram)/2 + self.center
                self.newOffset = midpointHistogram 

                pass
            # Ingen lutning har hittats
            else: 
                #print("Case 6")

                #Här kan vi använda:
                #self.leftHistogram

                midpointHistogram = (self.width - self.leftHistogram)/2 + self.center
                self.newOffset = midpointHistogram 

                pass
            
            pass

        # Histogrammet har endast hittat högra linjen
        elif self.rightHistogram < 640:
            # Vänstra linjens lutning har hittats
            if self.slopeRight:
                #print("Case 7")

                #Här kan vi anväda:
                #self.rightHistogram
                #self.xPointRight
                #self.slopeRight

                midpointHistogram = (self.rightHistogram)/2 + self.center
                self.newOffset = midpointHistogram

                pass
            # Ingen lutning har hittats
            else:
                #print("Case 8")
                
                #Här kan vi använda:
                #self.rightHistogram
                midpointHistogram = (self.rightHistogram)/2 + self.center
                self.newOffset = midpointHistogram

                pass

        else:
            print("Nothing detected")
            self.newOffset = self.lastOffset

            return
        
        #print(self.newOffset)
        #print(self.lastOffset)
        
        self.newOffset -= self.center

        
        #if self.lastOffset:
        #    if abs((self.newOffset-self.lastOffset)/self.lastOffset) > 0.10:
        #        print("Change in offset to large")
        #        self.newOffset = self.lastOffset
        #        return
            

        
        self.newOffset = int(self.newOffset)

        self.lastOffset = self.newOffset

