import compVision

class SemiAutonomous():
    def __init__(self, mqttClient, timeOut, roiPerc, resolution = (640,480)):
        self.timeOut = timeOut
        
        self.resolution = resolution
        self.laneData = compVision(self.resolution)

    def stop(self):
        #Stop all motors
        self.laneData.stopProcess()

    def mainLoop(self):
        self.laneData.getCenterOffset()