#import multiprocessing as mp
import compVision
import time

#Initializing functions

savedImageTest = compVision.compVision((640,480))

#savedImageTest.takePicture()

#savedImageTest.getCenterOffset()

#savedImageTest.regionOfInterest()



def takePicture():
    start = time.time()

    savedImageTest.getCenterOffset()

    finished = time.time()
    print("Tidsåtgång i ms {}".format((finished-start)*1000))

    savedImageTest.addLines()

    savedImageTest.displayImage()


#Main-loop
i=0

#try:
while(i < 10):
    takePicture()
    i += 1
#except:
#savedImageTest.stopProcess()   

savedImageTest.stopProcess()    
    #Retrieve inputs from Computer

    #Collect data from camera and calculate center off-set

    #Retrieve data from AVR

    #Send data to AVR


