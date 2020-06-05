from __future__ import print_function, division
import os
import glob
import sys, argparse
import pprint
import numpy as np
from cv2 import cv2
from scipy import optimize as opt

DATA_DIR = '/home/tamarar/Desktop/novo/Camera_calibration/calibration/newCalibrationImages/Pic_'
#DATA_DIR = '/home/tamarar/Desktop/novo/Camera_calibration/calibration/rad/Rad_'
PATTERN_SIZE = (9, 6)
SQUARE_SIZE = 1.0 

#########################################################################
# Loading images for calibration
def get_camera_images():
    images = [each for each in glob.glob(DATA_DIR + "*.jpg")]
    images = sorted(images)
    for each in images:
        yield (each, cv2.imread(each, 0))

#########################################################################
# Finding chessboard corners
def getChessboardCorners(images = None):
    objp = np.zeros((PATTERN_SIZE[1]*PATTERN_SIZE[0], 3), dtype=np.float64)
    objp[:, :2] = np.indices(PATTERN_SIZE).T.reshape(-1, 2)
    objp *= SQUARE_SIZE

    chessboard_corners = []
    image_points = []
    object_points = []
    correspondences = []
    ctr=0
    for (path, each) in get_camera_images(): 
        ret, corners = cv2.findChessboardCorners(each, patternSize=PATTERN_SIZE)
        print(corners)
        if ret:
            corners = corners.reshape(-1, 2)
            if corners.shape[0] == objp.shape[0] :
                image_points.append(corners)
                object_points.append(objp[:,:-1]) 
                print(object_points)
                correspondences.append([corners.astype(np.int), objp[:, :-1].astype(np.int)])
        else:
            print ("Error in detection points", ctr)

        ctr+=1
    #print(correspondences)
    return correspondences

#########################################################################
# Normalization
def normalize_points(chessboard_correspondences):
    views = len(chessboard_correspondences)

    def get_normalization_matrix(pts):
        pts = pts.astype(np.float64)
        x_mean, y_mean = np.mean(pts, axis=0)
        var_x, var_y = np.var(pts, axis=0)

        s_x , s_y = np.sqrt(2/var_x), np.sqrt(2/var_y)

        n = np.array([[s_x, 0, -s_x*x_mean], [0, s_y, -s_y*y_mean], [0, 0, 1]])
    
        n_inv = np.array([ [1./s_x ,  0 , x_mean], [0, 1./s_y, y_mean] , [0, 0, 1] ])
        return n.astype(np.float64), n_inv.astype(np.float64)


    ret_correspondences = [] 
    for i in range(views):
        imp, objp = chessboard_correspondences[i]
        N_x, N_x_inv = get_normalization_matrix(objp)
        N_u, N_u_inv = get_normalization_matrix(imp)
        print(N_u)
        hom_imp = np.array([ [[each[0]], [each[1]], [1.0]] for each in imp])
        hom_objp = np.array([ [[each[0]], [each[1]], [1.0]] for each in objp])


        normalized_hom_imp = hom_imp
        normalized_hom_objp = hom_objp

        for i in range(normalized_hom_objp.shape[0]):
            n_o = np.matmul(N_x,normalized_hom_objp[i])
            normalized_hom_objp[i] = n_o/n_o[-1]
            
            n_u = np.matmul(N_u,normalized_hom_imp[i])
            normalized_hom_imp[i] = n_u/n_u[-1]

        normalized_objp = normalized_hom_objp.reshape(normalized_hom_objp.shape[0], normalized_hom_objp.shape[1])
        normalized_imp = normalized_hom_imp.reshape(normalized_hom_imp.shape[0], normalized_hom_imp.shape[1])

        normalized_objp = normalized_objp[:,:-1]        
        normalized_imp = normalized_imp[:,:-1]

        ret_correspondences.append((imp, objp, normalized_imp, normalized_objp, N_u, N_x, N_u_inv, N_x_inv))

    return ret_correspondences

#########################################################################
def compute_view_based_homography(correspondence, reproj = False):
    #print(correspondence)
    image_points = correspondence[0]
    #print(image_points)
    object_points = correspondence[1]
    normalized_image_points = correspondence[2]
    normalized_object_points = correspondence[3]
    N_u = correspondence[4]
    N_x = correspondence[5]
    N_u_inv = correspondence[6]
    N_x_inv = correspondence[7]



    N = len(image_points)
    #print("Number of points in current view : ", N)

    M = np.zeros((2*N, 9), dtype=np.float64)
    #print("Shape of Matrix M : ", M.shape)

    #print("N_model\n", N_x)
    #print("N_observed\n", N_u)

    for i in range(N):
        X, Y = normalized_object_points[i] #A
        u, v = normalized_image_points[i] #B

        row_1 = np.array([ -X, -Y, -1, 0, 0, 0, X*u, Y*u, u])
        row_2 = np.array([ 0, 0, 0, -X, -Y, -1, X*v, Y*v, v])
        M[2*i] = row_1
        M[(2*i) + 1] = row_2


    u, s, vh = np.linalg.svd(M)


    h_norm = vh[np.argmin(s)]
    h_norm = h_norm.reshape(3, 3)

    #print(N_u_inv)
    #print(N_x)
 
    h = np.matmul(np.matmul(N_u_inv,h_norm), N_x)
    
    h = h[:,:]/h[2, 2]

    if reproj:
        reproj_error = 0
        for i in range(len(image_points)):
            t1 = np.array([[object_points[i][0]], [object_points[i][1]], [1.0]])
            t = np.matmul(h, t1).reshape(1, 3)
            t = t/t[0][-1]
            formatstring = "Imp {0} | ObjP {1} | Tx {2}".format(image_points[i], object_points[i], t)
            #print(formatstring)
            reproj_error += np.sum(np.abs(image_points[i] - t[0][:-1]))
        reproj_error = np.sqrt(reproj_error/N)/100.0
        print("Reprojection error : ", reproj_error)

    return h



def minimizer_func(initial_guess, X, Y, h, N):
    # X : normalized object points flattened
    # Y : normalized image points flattened
    # h : homography flattened
    # N : number of points
    # 
    x_j = X.reshape(N, 2)
    # Y = Y.reshape(N, 2)
    # h = h.reshape(3, 3)
    projected = [0 for i in range(2*N)]
    for j in range(N):
        x, y = x_j[j]
        w = h[6]*x + h[7]*y + h[8]
        projected[2*j] = (h[0] * x + h[1] * y + h[2]) / w
        projected[2*j + 1] = (h[3] * x + h[4] * y + h[5]) / w

    # return projected
    return (np.abs(projected - Y))**2
        

def jac_function(initial_guess, X, Y, h, N):
    x_j = X.reshape(N, 2)
    jacobian = np.zeros( (2*N, 9) , np.float64)
    for j in range(N):
        x, y = x_j[j]
        sx = np.float64(h[0]*x + h[1]*y + h[2])
        sy = np.float64(h[3]*x + h[4]*y + h[5])
        w = np.float64(h[6]*x + h[7]*y + h[8])
        jacobian[2*j] = np.array([x/w, y/w, 1/w, 0, 0, 0, -sx*x/w**2, -sx*y/w**2, -sx/w**2])
        jacobian[2*j + 1] = np.array([0, 0, 0, x/w, y/w, 1/w, -sy*x/w**2, -sy*y/w**2, -sy/w**2])

    return jacobian


def refine_homographies(H, correspondences, skip=False):
    if skip:
        return H

    image_points = correspondence[0]
    object_points = correspondence[1]
    normalized_image_points = correspondence[2]
    normalized_object_points = correspondence[3]
    N_u = correspondence[4]
    N_x = correspondence[5]
    N_u_inv = correspondence[6]
    N_x_inv = correspondence[7]

    N = normalized_object_points.shape[0]
    X = object_points.flatten()
    Y = image_points.flatten()
    h = H.flatten()
    h_prime = opt.least_squares(fun=minimizer_func, x0=h, jac=jac_function, method="lm" , args=[X, Y, h, N], verbose=0)

    
    if h_prime.success:
        H =  h_prime.x.reshape(3, 3)
    H = H/H[2, 2]
    return H


#########################################################################
def get_intrinsic_parameters(H_r):
    M = len(H_r)
    V = np.zeros((2*M, 6), np.float64)


    # matrix G
    # V = [G1
    #      G2
    #      ...
    #      Gn]
    # Vb = 0
    def v_pq(p, q, H):
        v = np.array([
                H[0, p]*H[0, q],
                H[0, p]*H[1, q] + H[1, p]*H[0, q],
                H[1, p]*H[1, q],
                H[2, p]*H[0, q] + H[0, p]*H[2, q],
                H[2, p]*H[1, q] + H[1, p]*H[2, q],
                H[2, p]*H[2, q]
            ])
        #print("v = ", v)
        return v

    for i in range(M):
        H = H_r[i]
        V[2*i] = v_pq(p=0, q=1, H=H)
        V[2*i + 1] = np.subtract(v_pq(p=0, q=0, H=H), v_pq(p=1, q=1, H=H))

    # solve V.b = 0
    u, s, vh = np.linalg.svd(V)
    b = vh[np.argmin(s)]

    # according to zhangs method
    # vc = (b[1]*b[3] - b[0]*b[4])/(b[0]*b[2] - b[1]**2)
    # l = b[5] - (b[3]**2 + vc*(b[1]*b[2] - b[0]*b[4]))/b[0]
    # alpha = np.sqrt((l/(b[0])))
    # beta = np.sqrt(((l*b[0])/(b[0]*b[2] - b[1]**2)))
    # gamma = -1*((b[1])*(alpha**2) *(beta/l))
    # uc = (gamma*vc/beta) - (b[3]*(alpha**2)/l)
    w = b[0] * b[2] * b[5] - b[1]**2 * b[5] - b[0] * b[4]**2 + 2 * b[1] * b[3] * b[4] - b[2] * b[3]**2
    d = b[0] * b[2] - b[1]**2

    if (d < 0):
        d = 0.01
    #d = -d

    #
    alpha = np.sqrt(w / (d * b[0]))
    beta = np.sqrt(w / d**2 * b[0])
    gamma = np.sqrt(w / (d**2 * b[0])) * b[1]
    uc = (b[1] * b[4] - b[2] * b[3]) / d
    vc = (b[1] * b[3] - b[0] * b[4]) / d

    # print([vc,
    #         l,
    #         alpha,
    #         beta,
    #         gamma,
    #     uc])

    # k = np.array([
    #         [alpha, gamma, uc],
    #         [0, beta, vc],
    #         [0, 0, 1.0],
    #     ])
    return np.array([
            [alpha, gamma, uc],
            [0,     beta,  vc],
            [0,     0,      1]
        ])
    # print("*******************************")
    # print("        **INTRINSIC** ")
    # print("k = ", k)
    # print("\n")
    #return k


#########################################################################
def extrinsicsCalculation(intrinsic, H_r):
    homography = H_r.reshape(3, 3)
    intrinsicsInv = np.linalg.inv(intrinsic)

    h1 = homography[:, 0]
    h2 = homography[:, 1]
    h3 = homography[:, 2]

    lam_r1 = 1 / np.linalg.norm(np.dot(intrinsicsInv, h1))
    lam_r2 = 1 / np.linalg.norm(np.dot(intrinsicsInv, h2))
    lam_r3 = (lam_r1 + lam_r2) / 2
    r1 = lam_r1 * np.dot(intrinsicsInv, h1)
    r2 = lam_r2 * np.dot(intrinsicsInv, h2)
    r3 = np.cross(r1, r2, axis=0)
    t = np.array(lam_r3 * np.dot(intrinsicsInv, h3)).transpose()

    # r = np.append(r1, r2, axis=1)
    # r = np.append(r, r3, axis=1)
    # rt = np.append(r, np.transpose(t), axis=1)

    rt = np.array(
            [r1.transpose(), r2.transpose(), r3.transpose(), t.transpose()]
         ).transpose()
    r = np.array(
            [r1.transpose(), r2.transpose(), r3.transpose()]
        ).transpose()

    return rt, r



def get_extrinsics_parameters(intrinsics, homographies):

    extrinsics = []
    rotation = []
    rotation = []
    for i in range(0, len(homographies)):
        rt, r = extrinsicsCalculation(intrinsics, homographies[i])
        #rotation.append(r)
        extrinsics.append(rt)
        rotation.append(r)
        #extrinsics.append(extrinsicsCalculation(intrinsics, homographies[i]))

    
    # print("*******************************")
    # print("        **EXTRINSIC** ")
    # print("extrinsics = ", extrinsics)
    # print("\n")
    # print("*******************************")
    return extrinsics, rotation

#########################################################################




#########################################################################

chessboard_correspondences = getChessboardCorners(images=None)


      

chessboard_correspondences_normalized = normalize_points(chessboard_correspondences)
#print(chessboard_correspondences_normalized)
#print(chessboard_correspondences_normalized)
#print(chessboard_correspondences_normalized)

#print("M = ", len(chessboard_correspondences_normalized), " view images")
#print("N = ", len(chessboard_correspondences_normalized[0][0]),  " points per image")

H = []
for correspondence in chessboard_correspondences_normalized:
    H.append(compute_view_based_homography(correspondence, reproj=0))

H_r = []
for i in range(len(H)):
    h_opt = refine_homographies(H[i], chessboard_correspondences_normalized[i], skip=False)
    H_r.append(h_opt)

k = get_intrinsic_parameters(H_r)
print(k)
extrinsics, rotation = get_extrinsics_parameters(k, H_r)
print(extrinsics[0])

# print(rotation)
# print(extrinsics)