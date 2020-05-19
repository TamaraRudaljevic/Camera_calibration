from cv2 import cv2
import numpy as numpy


img = "/home/tamarar/Desktop/Camera_calibration/calibration/image.jpg"
image = cv2.imread(img)
gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

dst = cv2.cornerHarris(gray, 2, 3, 0.04)
dst = cv2.dilate(dst,None)
image[dst > 0.01 * dst.max()]=[0, 0, 255] 
cv2.imwrite("/home/tamarar/Desktop/Camera_calibration/calibration/corners.png", image)
#cv2.imshow('DST', dst)
#cv2.waitKey(0)
