import numpy as np
from cv2 import cv2

fname = '/home/tamarar/Desktop/Camera_calibration/calibrateWithExistingFunction/test_image.png'
img = cv2.imread(fname)

gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

objpoints = [] # 3D points; z = 0
imgpoints = [] # 2D points

objp = np.zeros((6*8, 3), np.float32)
objp[:, :2] = np.mgrid[0:8, 0:6].T.reshape(-1, 2)

# Find the chessboard corners
ret, corners = cv2.findChessboardCorners(gray, (8, 6), None)

if ret == True:
    imgpoints.append(corners)
    objpoints.append(objp)

    # Draw the corners
    img = cv2.drawChessboardCorners(img, (8,6), corners, ret)
    cv2.imshow('IMAGE', img)
    cv2.waitKey()


# Calibrate camera
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

#dst = cv2.undistort(img, mtx, dist, None, mtx)

cv2.imwrite(dst, '/home/tamarar/Desktop/Camera_calibration/calibrateWithExistingFunction/test.png')