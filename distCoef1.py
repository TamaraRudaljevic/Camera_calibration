import numpy as np
import NewCalibration as calib


def estimate_lens_distortion(intrinsics, extrinsics, model, sensor):

    uc = intrinsics[0, 2]
    vc = intrinsics[1, 2]

    D = []
    d = []

    l = 0

    for i in range(0, len(extrinsics)):
        for j in range(0, len(model)):

            homog_model_coords = np.array([model[j][0], model[j][1], 0, 1])
            homog_coords = np.dot(extrinsics[i], homog_model_coords)

            coords = homog_coords / homog_coords[-1]
            print(coords)
            [x, y, hom] = coords

            r = np.sqrt(x*x + y*y)

            P = np.dot(intrinsics, homog_coords)
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

###################################################

chessboard_correspondences = calib.getChessboardCorners(images=None)
chessboard_correspondences_normalized = calib.normalize_points(chessboard_correspondences)


PATTERN_SIZE = (9, 6)
SQUARE_SIZE = 1.0 

model = np.zeros((PATTERN_SIZE[1]*PATTERN_SIZE[0], 3), dtype=np.float64)
model[:, :2] = np.indices(PATTERN_SIZE).T.reshape(-1, 2)
model *= SQUARE_SIZE

H = []
for correspondence in chessboard_correspondences_normalized:
    H.append(calib.compute_view_based_homography(correspondence, reproj=0))

H_r = []
for i in range(len(H)):
    h_opt = calib.refine_homographies(H[i], chessboard_correspondences_normalized[i], skip=False)
    H_r.append(h_opt)

k = calib.get_intrinsic_parameters(H_r)
extrinsics = calib.get_extrinsics_parameters(k, H_r)

estimate_lens_distortion(k, extrinsics, model, chessboard_correspondences_normalized)