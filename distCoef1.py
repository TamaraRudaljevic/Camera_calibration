import numpy as np
from cv2 import cv2
import NewCalibration as calib


def estimate_lens_distortion(intrinsics, extrinsics, model, sensor):

    uc = intrinsics[0, 2]
    vc = intrinsics[1, 2]

    D = []
    d = []

    l = 0
    #print(sensor)

    for i in range(0, len(extrinsics)):
        for j in range(0, len(model)):

            homog_model_coords = np.array([model[j][0], model[j][1], 0, 1])
            homog_coords = np.dot(extrinsics[i], homog_model_coords)

            #print("homog_coords = ", homog_coords)
            #print("homog_coords[-1] = ", homog_coords[-1])
            coords = homog_coords / homog_coords[-1]
            [x, y, hom] = coords

            r = np.sqrt(x*x + y*y)

            P = np.dot(intrinsics, homog_coords)
            #print("P = ", P)
            #print("P[2] = ", P[2])
            P = P / P[2]

            [u, v, trash] = P

            du = u - uc
            dv = v - vc

            D.append(
                np.array([
                    du * r**2, du * r**4
                ])
            )

            D.append(
                np.array([
                    dv * r**2, dv * r**4
                ])
            )

            up = sensor[i][j][0]
            vp = sensor[i][j][1]

            d.append(up - u)
            d.append(vp - v)

    k = np.linalg.lstsq(
        np.array(D),
        np.array(d)
    )

    return k

######################################################

chessboard_correspondences = calib.getChessboardCorners(images=None)
      

chessboard_correspondences_normalized = calib.normalize_points(chessboard_correspondences)

#print("M = ", len(chessboard_correspondences_normalized), " view images")
#print("N = ", len(chessboard_correspondences_normalized[0][0]),  " points per image")

H = []
for correspondence in chessboard_correspondences_normalized:
    H.append(calib.compute_view_based_homography(correspondence, reproj=0))

H_r = []
for i in range(len(H)):
    h_opt = calib.refine_homographies(H[i], chessboard_correspondences_normalized[i], skip=False)
    H_r.append(h_opt)

k = calib.get_intrinsic_parameters(H_r)
extrinsics = calib.get_extrinsics_parameters(k, H_r)

PATTERN_SIZE = (14, 2)
SQUARE_SIZE = 1.0 

objp = np.zeros((PATTERN_SIZE[1]*PATTERN_SIZE[0], 3), dtype=np.float64)
objp[:, :2] = np.indices(PATTERN_SIZE).T.reshape(-1, 2)
objp *= SQUARE_SIZE
#print(objp)
#print(chessboard_correspondences)
imageReal = []
for i in range(1, 14+1):
    image = cv2.imread('/home/tamarar/Desktop/novo/Camera_calibration/calibration/images_calibration/Pic_' + str(i) + '.jpg')
    imageReal.append(image)

image = cv2.imread('/home/tamarar/Desktop/novo/Camera_calibration/calibration/images_calibration/Pic_1.jpg')

estimate_lens_distortion(k, extrinsics, image, imageReal)