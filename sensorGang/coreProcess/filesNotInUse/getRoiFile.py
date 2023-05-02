#make a copy of getRoi.py but for an image jpeg file

import cv2
import numpy
import time
import sys

image = cv2.imread("Stillbild.PNG")
roi = cv2.selectROI(image)
print(roi)
roi_cropped = image[int(roi[1]):int(roi[1]+roi[3]), int(roi[0]):int(roi[0]+roi[2])]
cv2.imshow("Roi", roi_cropped)
cv2.waitKey(0)

#make a copy of getRoi.py but for an image file