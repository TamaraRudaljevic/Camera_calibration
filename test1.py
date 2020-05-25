import numpy as np
from cv2 import cv2


criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((9*6,3), np.float32)
objp[:,:2] = np.mgrid[0:9,0:6].T.reshape(-1,2)
# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.

#images = glob.glob(r'images/*.jpg')

# path = 'results'
# pathlib.Path(path).mkdir(parents=True, exist_ok=True) 

found = 0
for img in range(1,14+1):  # Here, 10 can be changed to whatever number you like to choose
    readpath = '/home/tamarar/Desktop/novo/Camera_calibration/calibration/images_calibration/Pic_'
    image=cv2.imread(readpath + str(img) + '.jpg')
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
        img = cv2.drawChessboardCorners(image, (9,6), corners, ret)
        found += 1
        # if you want to save images with detected corners 
        # uncomment following 2 lines and lines 5, 18 and 19
        # image_name = path + '/calibresult' + str(found) + '.png'
        # cv2.imwrite(image_name, img)

print("Number of images used for calibration: ", found)

# When everything done, release the capture
# cap.release()

# calibration
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
print("mtx = ", mtx)
print("\n")
print("*****************")
print("dist = ", dist)
print("\n")
print("*****************")


# transform the matrix and distortion coefficients to writable lists
# data = {'camera_matrix': np.asarray(mtx).tolist(),
#         'dist_coeff': np.asarray(dist).tolist()}

# # and save it to a file
# with open("calibration_matrix.yaml", "w") as f:
#     yaml.dump(data, f)
