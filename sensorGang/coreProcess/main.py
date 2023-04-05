#import multiprocessing as mp
import compVision

#Initializing functions

savedImageTest = compVision.compVision((640,480))

#savedImageTest.takePicture()

#savedImageTest.getCenterOffset()

#savedImageTest.regionOfInterest()



def takePicture():
    
    savedImageTest.getCenterOffset()

    savedImageTest.addLines()

    savedImageTest.displayImage()


#Main-loop
while(True):

    if key == ord("q"):
        break
    if key == ord("c"):
        takePicture()
        
    #Retrieve inputs from Computer

    #Collect data from camera and calculate center off-set

    #Retrieve data from AVR

    #Send data to AVR


