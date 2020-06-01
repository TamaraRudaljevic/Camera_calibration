import numpy as np 
from cv2 import cv2
#import NewCalibration as calib 
#import openCVCalib as calibration


#K, dist = calibration.calibrate()
#print(dist)
k = np.asarray([-0.01047118, -0.01900932, -0.00080068,  0.00036829,  0.07084063]);   

#k = np.asarray([-0.20221402,  0.0479132,  -0.01854401,  0.00590073, -0.00675606]); 


# K = np.asarray([[ 1.05792775e+03, -1.60363681e+01,  7.29404410e+02],
#     [ 0.00000000e+00,  1.03057080e+03,  4.02180795e+02],
#     [ 0.00000000e+00,  0.00000000e+00,  1.00000000e+00]]); 

K = np.asarray([[464.96636373,   0.,         338.173685  ],
 [  0.,         465.87851905, 219.62293951],
 [  0.,           0.,           1.,        ]]); 

# K = np.asarray( [[533.04057692,   0.,         624.46292591],
#  [  0.,         528.37259521, 475.58891714],
#  [  0.,           0.,          1.        ]]); 

#K = [1.05792775e+03, -1.60363681e+01,  7.29404410e+02, 0.00000000e+00,  1.03057080e+03,  4.02180795e+02, 0.00000000e+00,  0.00000000e+00,  1.00000000e+00]
#print(K)

#image_name = '/home/tamarar/Desktop/novo/Camera_calibration/calibration/newCalibrationImages/Pic_2.jpg'  
image_name = '/home/tamarar/Desktop/novo/Camera_calibration/calibration/newCalibrationImages/Pic_1.jpg' 

def undistort_image(min_y = -2, max_y = 2, step_y = 0.01, min_x = -2, max_x = 2, step_x = 0.01, image_name = image_name, 
                    K = K, k = k):
                    # image not used in the calibration:       
    Im = cv2.imread(image_name);

    # we *start* from normalized coordinates and then determine the distorted ones - in the pixel frame of the distorted images
    x = np.arange(min_x, max_x, step_x);
    y = np.arange(min_y, max_y, step_y);
    N = len(x);
    Undistorted = np.zeros([N, N, 3], dtype=np.uint8);

    v = np.zeros([3,1]);

    # show the progress per 10 percent of the image:
    n_steps = 10;
    pct_per_step = 100 / n_steps;
    progress_step = int(N/n_steps);

    # for learning the inverse mapping:
    Samples = [];
    Targets = [];

    for i in range(N):

        # if(np.mod(i, progress_step) == 0):
        #     print('{} percent'.format(int(i/progress_step) * pct_per_step));
        
        for j in range(N):
            
            # normalized coordinates
            x_p = x[i];
            y_p = y[j];
            
            # distorted normalized coordinates:
            #undistortion = False; # undistortion means that it can be undistorted by means of an invertible function
        
            #x_dist, y_dist = fisheye_distortion(x_p, y_p, k);
            #x_dist, y_dist = undistortion(x_p, y_p, k)
            x_dist, y_dist = undistort(x_p, y_p, k, K)
            #x_dist, y_dist = Dhane_undistortion(x_p, y_p)
            #print(x_p, y_p)
                    
            # use the camera matrix to retrieve the image coordinate in pixels in the distorted image:
            v[0] = x_dist;
            v[1] = y_dist;
            v[2] = 1;
            hom_coord_dist = np.dot(K, v);
            
            # for now, just round the coordinate:
            x_rounded = int(np.round(hom_coord_dist[0]));
            y_rounded = int(np.round(hom_coord_dist[1]));
            if(x_rounded >= 0 and x_rounded < Im.shape[1] and y_rounded >= 0 and y_rounded < Im.shape[0]):
                Undistorted[j,i,:] = np.mean(Im[y_rounded, x_rounded,:]);
                # if(not undistortion):
                #     # print('Uninvertible.');
                #     Undistorted[j,i,0:2] = 0;
    
            # we add the normalized distorted coordinates to samples, as the step from image coordinates to such coords is simple:
            Samples.append(np.asarray([x_dist, y_dist]));
            Targets.append(np.asarray([x_p, y_p]))

    
    cv2.imwrite('/home/tamarar/Desktop/novo/Camera_calibration/calibration/rad/undistorted.jpg', Undistorted);
    #cv2.imshow('Undistorted', Undistorted);

def undistortion(x_dist, y_dist, distCoef):
    k1 = distCoef[0]
    k2 = distCoef[1]
    p1 = distCoef[2]
    p2 = distCoef[3]
    k3 = distCoef[4]

    r = np.sqrt(x_dist**2 + y_dist**2);

    # x = p2*(3*x_dist**2+y_dist**2) + x_dist*(k2*(x_dist**2+y_dist**2)**2 + k1*(x_dist**2+y_dist**2)+1) + 2*p1*x_dist*y_dist
    # y = p1*(3*x_dist**2+3*y_dist**2) + y_dist*(k2*(x_dist**2+y_dist**2)**2 + k1*(x_dist**2+y_dist**2)+1) + 2*p2*x_dist*y_dist
    # x = x_dist + (2*x_dist*y_dist + p2*(r**2 + 2*x_dist**2))
    # y = y_dist + (p1*(r**2+ 2*y_dist**2)+2*p2*x_dist*y_dist)
    x = x_dist + x_dist*(k1*r**2 + k2*r**4 + k3*r**6) + (p1*(r**2 + 2*(x_dist**2)) + 2*p2*x_dist*y_dist)
    y = y_dist + y_dist*(k1*r**2 + k2*r**4 + k3*r**6) + (p2*(r**2 + 2*(y_dist**2)) + 2*p2*x_dist*y_dist)
    # x = x_dist + (2*p1*x_dist*y_dist + p2*(r**2 + 2*x_dist*x_dist))
    # y = y_dist + (p1*(r**2 + 2*y_dist**2) + 2*p2*x_dist*y_dist)
    #r = np.sqrt(x_p**2 + y_p**2);
    # theta = np.arctan(r);
    # theta_d = theta * (1 + k[0] * theta**2 + k[1] * theta**4 + k[2] * theta**6 + k[3] * theta**8);
    # x_dist = (theta_d / r) * x_dist;
    # y_dist = (theta_d / r) * y_dist;
     

    return x, y

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
    ifx = 1./fx 
    ify = 1./fy

    x0 = (x_dist - u0)*ifx
    y0 = (y_dist - v0)*ify

    r = x_dist**2 + y_dist**2
    icdist = 1
    deltaX = 2*p1*x_dist*y_dist + p2 *(r + 2*(x_dist**2))
    deltaY = p1* (r + 2*(y_dist**2)) + 2*p2*x_dist*y_dist
    x = (x0 - deltaX)*icdist
    y = (y0 - deltaY)*icdist
   
   
   
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
    return x, y




undistort_image(min_y = -2, max_y = 2, step_y = 0.01, min_x = -2, max_x = 2, step_x = 0.01, image_name = image_name, 
                    K = K, k = k)