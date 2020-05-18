import numpy as np
from cv2 import cv2

def gaussian(image):
    out = cv2.GaussianBlur(image, (3, 3), 0)
    Out = 'Out'
    cv2.imshow(Out, out)
    cv2.waitKey(0)

def median(image):
    out = cv2.medianBlur(image, 3)
    Out = 'Out'
    cv2.imshow(Out, out)
    cv2.waitKey(0)

def bilateral(image):
    out = cv2.bilateralFilter(image, 30, 30 * 2, 30 / 2)
    Out = 'Out'
    cv2.imshow(Out, out)
    cv2.waitKey(0)

        
if __name__ == "__main__":
    image = cv2.imread("D:/camera-calibration/image.jpg")
    gaussian(image)
    median(image)
    bilateral(image)

