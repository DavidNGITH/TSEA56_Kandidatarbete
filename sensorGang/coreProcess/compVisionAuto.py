"""Computer vision."""
import time
import numpy as np
import cv2
from PD_reg import PDcontroller
from videoStream import VideoStream
from sharedFunctions import (lineInterceptFunction,
                             regionOfInterestFunction,
                             getDataFromLinesFunction,
                             getOffsetLeftTurnFunction,
                             getOffsetRightTurnFunction,
                             setupFunction,
                             imageProcessingFunction)

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

        # PD-Controller
        self.PD = PD

        # Stops/Nodes
        self.stopLine = False           # If stopline is found
        self.stopLineActivated = True   # Whether it should look for stopLines
        self.stopAtNode = False         # Whether it should stop at node
        self.getCommand = 0             # Contains next steering command
        self.instructions = []          # Contains current assignment
        self.nodeTimeOut = 0            # Time out for finding nodes
        self.nodeTimeOutStopLine = 0    # Time out for finding stops/nodes
        self.nodeStopTimer = 0          # Wait timer at stop
        self.waitAtNode = 2             # Wait time at stop
        self.stopped = False            # If car has stopped at node
        self.nodeCounter = 0

        self.sendDataTimer = 0

        self.lineSegments = None

    lineIntercept = lineInterceptFunction
    regionOfInterest = regionOfInterestFunction
    getDataFromLines = getDataFromLinesFunction
    getOffsetLeftTurn = getOffsetLeftTurnFunction
    getOffsetRightTurn = getOffsetRightTurnFunction
    setup = setupFunction
    imageProcessing = imageProcessingFunction

    def makeStopLine(self, line, minX, maxX):
        """Create endpoints for stop line."""
        width = maxX-minX

        # print("Width: {}".format(width))

        # If stopline
        if width > self.widthStopLine and width < 300:
            # If stop required
            if self.stopLineActivated:
                self.nodeTimeOutStopLine = time.time()
                self.stopLine = True

        # Checks if node
        elif width < self.widthNodeLine and not self.stopped:
            # Left node
            if minX < 200:
                print("Node to the left")
            # Right node
            elif maxX > 400:
                if ((time.time() - self.nodeTimeOut > 2) and
                        (time.time() - self.nodeTimeOutStopLine > 3)):
                    self.nodeTimeOut = time.time()
                    self.nodeCounter += 1
                    if self.instructions:
                        self.getCommand = self.instructions.pop(0)
                    else:
                        self.stopAtNode = True

                    print("Node to the right")

        else:
            # print("No stopline")
            self.stopLine = False

    def getCenterOffset(self, qSteering, statusValue, qSpeed, qBreak,
                        qCommand, nodeCounter, assignmentCounter, qPD,
                        qOffsetData, statusAutonomous):
        """Calculate the center offset in frame."""
        threadStream = VideoStream(self.resolution)  # Creates Video stream
        threadStream.start()  # Starts Video stream

        status = statusValue.value

        qSpeed.put(self.normalSpeed)  # Start car

        self.instructions = qCommand.get()  # First assignment

        self.getCommand = self.instructions.pop(0)  # First turning instuction

        self.nodeTimeOut = time.time()

        while status:
            # Handles PD messages from computer
            if qPD.empty() is False:
                message = qPD.get()
                if message[0] == 0:
                    self.PD.updateKp(message[1])
                else:
                    self.PD.updateKp(message[1])   

            self.img = threadStream.read()  # Retrive image

            self.imageProcessing()

            self.lineIntercept(self.lineSegments)  # Calc line equations

            nodeCounter.value = self.nodeCounter

            self.lastSpeed = self.currentSpeed  # Set last speed to current

            self.getOffset()  # Get offset

            # Send offset data to computer
            if (time.time()-self.sendDataTimer > 1):
                self.sendDataTimer = time.time()
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
                self.getOffset = self.casesDict[self.getCommand]
                self.stopLine = False
                self.intersectionTime = time.time()
                self.normalSteering = False
                self.stopLineActivated = False

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

            status = statusValue.value  # Check status value

            if self.stopAtNode:
                self.stopped = True
                qBreak.put(1)

                if qCommand.empty() is False:
                    print("Get next command")
                    self.nodeCounter = 0
                    assignmentCounter.value += 1
                    self.instructions = qCommand.get()  # Get next assignment
                    self.getCommand = self.instructions.pop(0)
                    self.nodeStopTimer = time.time()
                    self.stopAtNode = False

                else:
                    print("No more commands")
                    qSpeed.put(0)
                    time.sleep(1)
                    statusAutonomous.value = 0
                    status = 0

            if (self.stopped and
                    (time.time() - self.nodeStopTimer > self.waitAtNode)):
                print("Start driving again")
                self.nodeTimeOut = time.time()
                qBreak.put(0)
                self.stopAtNode = False
                self.stopped = False

        print("CompVision stopped")

        threadStream.stop()  # Stop stream
