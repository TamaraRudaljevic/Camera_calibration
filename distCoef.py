import numpy as np
from cv2 import cv2 
import NewCalibration as calib


# PATTERN_SIZE = (14, 2)
SQUARE_SIZE = 1.0 

# model = np.zeros((PATTERN_SIZE[1]*PATTERN_SIZE[0], 3), dtype=np.float64)
# model[:, :2] = np.indices(PATTERN_SIZE).T.reshape(-1, 2)
model = np.zeros((54, 2), dtype=np.float64)
#model[:, :2] = np.indices((14, 2)).T.reshape(-1, 2)
model *= SQUARE_SIZE

def to_homogeneous_3d(A):
    """Convert a stack of inhomogeneous vectors (without a Z component)
       to a homogeneous full-form representation.
    """
    if A.ndim != 2 or A.shape[-1] != 2:
        raise ValueError('Stacked vectors must be 2D inhomogeneous')

    N = A.shape[0]
    A_3d = np.hstack((A, np.zeros((N,1))))
    A_3d_hom = to_homogeneous(A_3d)

    return A_3d_hom

def to_homogeneous(A):
    """Convert a stack of inhomogeneous vectors to a homogeneous 
       representation.
    """
    A = np.atleast_2d(A)

    N = A.shape[0]
    A_hom = np.hstack((A, np.ones((N,1))))

    return A_hom

def to_inhomogeneous(A):
    """Convert a stack of homogeneous vectors to an inhomogeneous
       representation.
    """
    A = np.atleast_2d(A)

    N = A.shape[0]
    A /= A[:,-1][:, np.newaxis]
    A_inhom = A[:,:-1]

    return A_inhom

def calculate_lens_distortion(model, all_data, K, extrinsic_matrices):
    """Calculate least squares estimate of distortion coefficients.
    Args:
       model: Nx2 planar points in the world frame
       all_data: M-length list of Nx2 sensor frame correspondences
       K: 3x3 intrinsics matrix
       exrinsic_matrices: M-length list of 3x4 extrinsic matrices
    Returns:
       Radial distortion coefficients [k0, k1]
    """
    M = len(all_data)
    N = model.shape[0]
    print("N* = ", N )
    print("M* = ", M)
    

    model = to_homogeneous_3d(model)

    u_c, v_c = K[0,2], K[1,2]

    # Form radius vector
    r = np.zeros(2 * M * N)
    for e, E in enumerate(extrinsic_matrices):
        normalized_projection = np.dot(model, E.T)
        normalized_projection = to_inhomogeneous(normalized_projection)

        x_normalized_proj, y_normalized_proj = normalized_projection[:, 0], normalized_projection[:, 1]
        r_i = np.sqrt(x_normalized_proj**2 + y_normalized_proj**2)
        r[e*N:(e+1)*N] = r_i
    r[M*N:] = r[:M*N]

    # Form observation vector
    obs = np.zeros(2 * M * N)
    u_data, v_data = np.zeros(M * N), np.zeros(M * N)
    for d, data in enumerate(all_data):
        u_i, v_i = data[:, 0], data[:, 1]
        u_data[d*N:(d+1)*N] = u_i
        v_data[d*N:(d+1)*N] = v_i
    obs[:M*N] = u_data
    obs[M*N:] = v_data

    # Form prediction vector
    pred = np.zeros(2 * M * N)
    pred_centered = np.zeros(2 * M * N)
    u_pred, v_pred = np.zeros(M * N), np.zeros(M * N)
    for e, E in enumerate(extrinsic_matrices):
        P = np.dot(K, E)
        projection = np.dot(model, P.T)
        projection = to_inhomogeneous(projection)
        u_pred_i = projection[:, 0]
        v_pred_i = projection[:, 1]

        u_pred[e*N:(e+1)*N] = u_pred_i
        v_pred[e*N:(e+1)*N] = v_pred_i
    pred[:M*N] = u_pred
    pred[M*N:] = v_pred
    pred_centered[:M*N] = u_pred - u_c
    pred_centered[M*N:] = v_pred - v_c

    # Form distortion coefficient constraint matrix
    D = np.zeros((2 * M * N, 2))
    D[:, 0] = pred_centered * r**2
    D[:, 1] = pred_centered * r**4

    # Form values (difference between sensor observations and predictions)
    b = obs - pred

    # Use pseudoinverse technique to compute least squares solution for distortion coefficients
    D_inv = np.linalg.pinv(D)
    k = np.dot(D_inv, b)

    return k
    #print(k)


##########################################################

chessboard_correspondences = calib.getChessboardCorners(images=None)
      

chessboard_correspondences_normalized = calib.normalize_points(chessboard_correspondences)

print("M = ", len(chessboard_correspondences_normalized), " view images")
print("N = ", len(chessboard_correspondences_normalized[0][0]),  " points per image")

H = []
for correspondence in chessboard_correspondences_normalized:
    H.append(calib.compute_view_based_homography(correspondence, reproj=0))

H_r = []
for i in range(len(H)):
    h_opt = calib.refine_homographies(H[i], chessboard_correspondences_normalized[i], skip=False)
    H_r.append(h_opt)

K = calib.get_intrinsic_parameters(H_r)
extrinsics = calib.get_extrinsics_parameters(K, H_r)
#print(chessboard_correspondences_normalized)
#print(model)
imageReal =[]
for i in range(1, 14+1):
    image = cv2.imread('/home/tamarar/Desktop/novo/Camera_calibration/calibration/images_calibration/Pic_' + str(i) + '.jpg')
    imageReal.append(image)
image1 = cv2.imread('/home/tamarar/Desktop/novo/Camera_calibration/calibration/images_calibration/Pic_1.jpg')
#print(chessboard_correspondences_normalized)

#image = image1.T.reshape(-1, 2)
image = image1[:,:, ]
print(image)


calculate_lens_distortion(image, imageReal, K, extrinsics)