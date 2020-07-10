import numpy as np
from cv2 import cv2
#import NewCalibration as calib
import glob


# criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
# # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((9*6,3), np.float32)
objp[:,:2] = np.mgrid[0:9,0:6].T.reshape(-1,2)
# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.

# #images = glob.glob(r'images/*.jpg')

# # path = 'results'
# # pathlib.Path(path).mkdir(parents=True, exist_ok=True) 

found = 0
for img in range(1,5+1):  # Here, 10 can be changed to whatever number you like to choose
    #readpath = '/home/tamarar/Desktop/novo/Camera_calibration/calibration/newCalibrationImages/Pic_'
    #readpath = '/home/tamarar/Desktop/novo/Camera_calibration/calibration/newCalibrationImages/Pic_'
    readpath = '/home/tamarar/Desktop/Novo/Camera_calibration/CameraCalibration/imagesCalib/3/'
    image=cv2.imread(readpath + str(img) + '.png')
    #print(images[im_i])
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    # Find the chess board corners
    ret, corners = cv2.findChessboardCorners(gray, (9,6), None)
    #print(corners)
    # If found, add object points, image points (after refining them)
    if ret == True:
        objpoints.append(objp)   # Certainly, every loop objp is the same, in 3D.
        #corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
        imgpoints.append(corners)
        # Draw and display the corners
        #img = cv2.drawChessboardCorners(image, (9,6), corners, ret)
        found += 1
        # if you want to save images with detected corners 
        # uncomment following 2 lines and lines 5, 18 and 19
        # image_name = path + '/calibresult' + str(found) + '.png'
        # cv2.imwrite(image_name, img)
# for img in range(1, 5):
#     readpath = '/home/tamarar/Desktop/novo/Camera_calibration/calibration/images_calibration/Rad_'
#     image=cv2.imread(readpath + str(img) + '.png')
#     #print(images[im_i])
#     gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
#     # Find the chess board corners
#     ret, corners = cv2.findChessboardCorners(gray, (8,6), None)
#     #print(corners)
#     # If found, add object points, image points (after refining them)
#     if ret == True:
#         objpoints.append(objp)   # Certainly, every loop objp is the same, in 3D.
#         #corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
#         imgpoints.append(corners)
#         # Draw and display the corners
#         #img = cv2.drawChessboardCorners(image, (9,6), corners, ret)
#         found += 1
# print(found)
    

#print("Number of images used for calibration: ", found)

# DATA_DIR = '/home/tamarar/Desktop/novo/Camera_calibration/calibration/images_calibration/Pic_'
# DATA_DIR_RAD = '/home/tamarar/Desktop/novo/Camera_calibration/calibration/images_calibration/Rad_'
# PATTERN_SIZE = (9, 6)
# PATTERN_SIZE_RAD = (8, 6)
# SQUARE_SIZE = 1.0 

#########################################################################
# Loading images for calibration
# def get_camera_images():
#     images = [each for each in glob.glob(DATA_DIR + "*.jpg")]
#     images = sorted(images)
#     for each in images:
#         yield (each, cv2.imread(each, 0))

# def get_camera_images_rad():
#     images = [each for each in glob.glob(DATA_DIR_RAD + "*.png")]
#     images = sorted(images)
#     for each in images:
#         yield (each, cv2.imread(each, 0))

# #########################################################################
# # Finding chessboard corners
# def getChessboardCorners(images = None):
#     objp = np.zeros((PATTERN_SIZE[1]*PATTERN_SIZE[0], 3), dtype=np.float64)
#     objp[:, :2] = np.indices(PATTERN_SIZE).T.reshape(-1, 2)
#     objp *= SQUARE_SIZE

#     chessboard_corners = []
#     image_points = []
#     object_points = []
#     correspondences = []
#     ctr=0
#     for (path, each) in get_camera_images(): 
#         ret, corners = cv2.findChessboardCorners(each, patternSize=PATTERN_SIZE)
#         if ret:
#             corners = corners.reshape(-1, 2)
#             if corners.shape[0] == objp.shape[0] :
#                 image_points.append(corners)
#                 object_points.append(objp[:,:-1]) 
#                 correspondences.append([corners.astype(np.int), objp[:, :-1].astype(np.int)])
#         else:
#             print ("Error in detection points", ctr)
#         ctr+=1


#     for (path, each) in get_camera_images_rad(): 
#         ret, corners = cv2.findChessboardCorners(each, patternSize=PATTERN_SIZE_RAD)
#         if ret:
#             corners = corners.reshape(-1, 2)
#             if corners.shape[0] == objp.shape[0] :
#                 image_points.append(corners)
#                 object_points.append(objp[:,:-1]) 
#                 correspondences.append([corners.astype(np.int), objp[:, :-1].astype(np.int)])
#         else:
#             print ("Error in detection points", ctr)

#         ctr+=1
#     #print(ctr)

#     return correspondences, image_points, object_points
    
# image = cv2.imread('/home/tamarar/Desktop/novo/Camera_calibration/calibration/images_calibration/Pic_14.jpg')
# gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

# correspondences, imgpoints, objpoints = getChessboardCorners()
# calibration
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
print("mtx = ", mtx)
print("\n")
print("*****************")
print("dist = ", dist)
print("\n")
# print("*****************")
#############################
# chessboard_correspondences = calib.getChessboardCorners(images=None)
      

# chessboard_correspondences_normalized = calib.normalize_points(chessboard_correspondences)

# #print("M = ", len(chessboard_correspondences_normalized), " view images")
# #print("N = ", len(chessboard_correspondences_normalized[0][0]),  " points per image")

# H = []
# for correspondence in chessboard_correspondences_normalized:
#     H.append(calib.compute_view_based_homography(correspondence, reproj=0))

# H_r = []
# for i in range(len(H)):
#     h_opt = calib.refine_homographies(H[i], chessboard_correspondences_normalized[i], skip=False)
#     H_r.append(h_opt)

# k = calib.get_intrinsic_parameters(H_r)

#####################################


# transform the matrix and distortion coefficients to writable lists
# data = {'camera_matrix': np.asarray(mtx).tolist(),
#         'dist_coeff': np.asarray(dist).tolist()}

# # and save it to a file
# with open("calibration_matrix.yaml", "w") as f:
#     yaml.dump(data, f)

# imgpoints = []
# objpoints = []
# readpath = '/home/tamarar/Desktop/novo/Camera_calibration/calibration/image_radial_distortion/Pic_4.png'
# image=cv2.imread(readpath)
# #print(images[im_i])
# gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
# # Find the chess board corners
# ret, corners = cv2.findChessboardCorners(gray, (8,6), None)
# #print(corners)
# # If found, add object points, image points (after refining them)
# if ret == True:
#     objpoints.append(objp)   # Certainly, every loop objp is the same, in 3D.
#     #corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
#     imgpoints.append(corners)
#     # Draw and display the corners
#     #img = cv2.drawChessboardCorners(image, (8,6), corners, ret)

# ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
# print(dist)

############################################

# image1 = cv2.imread('/home/tamarar/Desktop/novo/Camera_calibration/calibration/image_radial_distortion/Pic_1.png')
# image2 = cv2.imread('/home/tamarar/Desktop/novo/Camera_calibration/calibration/image_radial_distortion/Pic_2.png')
# image3 = cv2.imread('/home/tamarar/Desktop/novo/Camera_calibration/calibration/image_radial_distortion/Pic_3.png')
# image4 = cv2.imread('/home/tamarar/Desktop/novo/Camera_calibration/calibration/image_radial_distortion/Pic_4.png')

# dst1 = cv2.undistort(image1, k, dist, None, mtx)
# cv2.imwrite('/home/tamarar/Desktop/novo/Camera_calibration/calibration/images_calibration/test1.jpg', dst1)

# dst2 = cv2.undistort(image2, k, dist, None, mtx)
# cv2.imwrite('/home/tamarar/Desktop/novo/Camera_calibration/calibration/images_calibration/test2.jpg', dst2)

# dst3 = cv2.undistort(image3, k, dist, None, mtx)
# cv2.imwrite('/home/tamarar/Desktop/novo/Camera_calibration/calibration/images_calibration/test3.jpg', dst3)

# dst4 = cv2.undistort(image4, k, dist, None, mtx)
# cv2.imwrite('/home/tamarar/Desktop/novo/Camera_calibration/calibration/images_calibration/test4.jpg', dst4)

# for i in range(1, 14):
#     image = cv2.imread('/home/tamarar/Desktop/novo/Camera_calibration/calibration/newCalibrationImages/Pic_' + str(i)+ '.jpg')
#     dst = cv2.undistort(image, mtx, dist, None, mtx)
#     cv2.imwrite('/home/tamarar/Desktop/novo/Camera_calibration/calibration/newCalibrationImages/Test_' + str(i) + '.jpg', dst)

image = cv2.imread('/home/tamarar/Desktop/novo/Camera_calibration/calibration/rad/Rad_1.png')
dst = cv2.undistort(image, mtx, dist, None, mtx)
cv2.imwrite('/home/tamarar/Desktop/novo/Camera_calibration/calibration/rad/test.png', dst)