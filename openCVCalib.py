import numpy as np
from cv2 import cv2
import NewCalibration as calib

# vector of distortion cof from openCV function calibrateCamera()
def calibrationWithOpenCV():
    objp = np.zeros((9*6,3), np.float32)
    objp[:,:2] = np.mgrid[0:9,0:6].T.reshape(-1,2)
    objpoints = [] # 3d point in real world space
    imgpoints = [] # 2d points in image plane.


    found = 0
    for img in range(1,14+1):  # Here, 10 can be changed to whatever number you like to choose
        readpath = '/home/tamarar/Desktop/Novo/Camera_calibration/images_tan_distortion/Pic_'
        image=cv2.imread(readpath + str(img) + '.jpg')
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(gray, (9,6), None)
        # If found, add object points, image points (after refining them)
        if ret == True:
            objpoints.append(objp)   
            imgpoints.append(corners)
            found += 1
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
   
    print("mtx = ", mtx)
    print("\n")
    print("*****************")
    print("dist = ", dist)
    print("\n")
    print("*****************")
    return dist



##################################################

def calibrate():
    chessboard_correspondences = calib.getChessboardCorners(images=None)
    chessboard_correspondences_normalized = calib.normalize_points(chessboard_correspondences)


    H = []
    for correspondence in chessboard_correspondences_normalized:
        H.append(calib.compute_view_based_homography(correspondence, reproj=0))

    H_r = []
    for i in range(len(H)):
        h_opt = calib.refine_homographies(H[i], chessboard_correspondences_normalized[i], skip=False)
        H_r.append(h_opt)

    k = calib.get_intrinsic_parameters(H_r)
    dist = calibrationWithOpenCV()
    return k, dist

k, dist = calibrate()
# print(dist)


# for i in range(1, 14):
#     image = cv2.imread('/home/tamarar/Desktop/novo/Camera_calibration/calibration/newCalibrationImages/Pic_' + str(i)+ '.jpg')
#     dst = cv2.undistort(image, k, calibrationWithOpenCV(), None, k)
#     cv2.imwrite('/home/tamarar/Desktop/novo/Camera_calibration/calibration/newCalibrationImages/Test_' + str(i) + '.jpg', dst)