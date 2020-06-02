import numpy as np 
from cv2 import cv2

K = np.asarray([[464.96636373,   0.,         338.173685  ],
 [  0.,         465.87851905, 219.62293951],
 [  0.,           0.,           1.        ]])


distCoef =  np.asarray([-0.01047118, -0.01900932, -0.00080068,  0.00036829,  0.07084063])

image = cv2.imread('/home/tamarar/Desktop/novo/Camera_calibration/calibration/newCalibrationImages/Pic_1.jpg')
print(image[2:])

# def undistort(distorted, undistorted, K, distCoef, knew, size):
#     map1 = []
#     map2 = []

#     undistortMap(K, distcoef, R = [], knew, size, cv2.CV_16SC2, map1, map2)

# def undistortMap(K, distCoef, R, P, size, m1type, map1, map2):
#     k1 = distCoef[0]
#     k2 = distCoef[1]
#     p1 = distCoef[2]
#     p2 = distCoef[3]
#     k3 = distCoef[4]

#     v0 = K[1].item(2)
#     u0 = K[0].item(2)
#     fx = K[0].item(0)
#     fy = K[1].item(1)


