import numpy as np 
from cv2 import cv2

K = np.asarray([[464.96636373,   0.,         338.173685  ],
 [  0.,         465.87851905, 219.62293951],
 [  0.,           0.,           1.        ]])


distCoef =  np.asarray([-0.01047118, -0.01900932, -0.00080068,  0.00036829,  0.07084063])

image = cv2.imread('/home/tamarar/Desktop/novo/Camera_calibration/calibration/newCalibrationImages/Pic_1.jpg')
gray = 
def undistortion(x_dist, y_dist, distCoef):
#     k1 = distCoef[0]
#     k2 = distCoef[1]
#     p1 = distCoef[2]
#     p2 = distCoef[3]
#     k3 = distCoef[4]

#     r = np.sqrt(x_dist**2 + y_dist**2);

#     # x = p2*(3*x_dist**2+y_dist**2) + x_dist*(k2*(x_dist**2+y_dist**2)**2 + k1*(x_dist**2+y_dist**2)+1) + 2*p1*x_dist*y_dist
#     # y = p1*(3*x_dist**2+3*y_dist**2) + y_dist*(k2*(x_dist**2+y_dist**2)**2 + k1*(x_dist**2+y_dist**2)+1) + 2*p2*x_dist*y_dist
#     # x = x_dist + (2*x_dist*y_dist + p2*(r**2 + 2*x_dist**2))
#     # y = y_dist + (p1*(r**2+ 2*y_dist**2)+2*p2*x_dist*y_dist)
#     x = x_dist + x_dist*(k1*r**2 + k2*r**4 + k3*r**6) + (p1*(r**2 + 2*(x_dist**2)) + 2*p2*x_dist*y_dist)
#     y = y_dist + y_dist*(k1*r**2 + k2*r**4 + k3*r**6) + (p2*(r**2 + 2*(y_dist**2)) + 2*p2*x_dist*y_dist)
#     # x = x_dist + (2*p1*x_dist*y_dist + p2*(r**2 + 2*x_dist*x_dist))
#     # y = y_dist + (p1*(r**2 + 2*y_dist**2) + 2*p2*x_dist*y_dist)
#     #r = np.sqrt(x_p**2 + y_p**2);
#     # theta = np.arctan(r);
#     # theta_d = theta * (1 + k[0] * theta**2 + k[1] * theta**4 + k[2] * theta**6 + k[3] * theta**8);
#     # x_dist = (theta_d / r) * x_dist;
#     # y_dist = (theta_d / r) * y_dist;
     

#     return x, y

def undistort(x_dist, y_dist, distCoef, K):
    k1 = distCoef[0]
    k2 = distCoef[1]
    p1 = distCoef[2]
    p2 = distCoef[3]
    k3 = distCoef[4]

    v0 = K[1].item(2)
    u0 = K[0].item(2)
    fx = K[0].item(0)
    fy = K[1].item(1)
    
    r = np.sqrt(x_dist**2 + y_dist**2)
    theta = np.arctan(r)

    theta2 = theta**2
    theta4 = theta2**2
    theta6 = theta4**2

    theta_d = theta * (1 + k1*theta2 + k1*theta4 + k3*theta6)

    if r == 0:
        scale = 1
    else:
        scale = theta_d/r 

    u = fx*x_dist*scale + u0
    v = fy*y_dist*scale + v0
   
    #print("v0 = ", v0)
    # du = x_dist - u0
    # dv = y_dist - u0

    #r = np.sqrt(du**2 + dv**2)

    # delta_u = du*(k1*r**2 + k2*r**4 + k3*r**6) + p1*(3*dv**2 + dv**2) + 2*p2*du*dv
    # delta_v = dv*(k1*r**2 + k2*r**4 + k3*r**6) + 2*p1*du*dv + p2*(du**2 + dv**2)

    # x = x_dist - delta_u
    # y = y_dist - delta_v
    # x = x_dist + du*(k1*r**2 + k2*r**4 + k3*r**6) + (p1*(r**2 + 2*(du**2))) + 2*p2*du*dv
    # y = y_dist + dv*(k1*r**2 + k2*r**4 + k3*r**6) + (2*p1*du*dv + p2*(r**2 + 2*(dv**2)))

    # x = u0 + du / (1 + k1*r**2 + k2*r**4 + k3*r**4)
    # y = v0 + dv / (1 + k1*r**2 + k2*r**4 + k3*r**6)

    
    #print(x, y)
    return u, v

# def undistort(image_dist, image, K, distCoef, knew, new_size):
#     map1 = []
#     map2 = []
#     undistortMap(K, distCoef, knew, size, cv2.CV_16SC2, map1, map2)

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



undistort_image(min_y = -2, max_y = 2, step_y = 0.01, min_x = -2, max_x = 2, step_x = 0.01, image_name = image_name, 
                    K = K, k = k)