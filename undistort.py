from cv2 import cv2 
import numpy as np 

def undistort(distCoef, distImage, undistImage, intrinsics):
   