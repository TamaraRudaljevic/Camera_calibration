import pickle
import numpy as np
from cv2 import cv2

# Read in the saved objpoints and imgpoints
dist_pickle = pickle.load( open( "D:/camera-calibration/calibrateWithExistingFunction/wide_dist_pickle.p", "rb" ) )
objpoints = dist_pickle["objpoints"]
imgpoints = dist_pickle["imgpoints"]

# Read in an image
img = cv2.imread('D:/camera-calibration/calibrateWithExistingFunction/test_image.png')


def cal_undistort(img, objpoints, imgpoints):
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, img.shape[1:], None, None)
    undist = cv2.undistort(img, mtx, dist, None, mtx)
    return undist

undistorted = cal_undistort(img, objpoints, imgpoints)

cv2.imshow("DISTORTED", img)
cv2.imshow('UNDISTORTED',undistorted)
cv2.waitKey()