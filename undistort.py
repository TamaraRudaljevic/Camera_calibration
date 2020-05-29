from cv2 import cv2 
import numpy as np 
import NewCalibration as calib 
import openCVCalib as calibration


K, dist = calibration.calibrate()
print(dist)
k = np.asarray([-0.20735375,  0.05202131, -0.01828434,  0.00591405, -0.0077002 ]);   


# K = np.asarray([[ 1.05792775e+03, -1.60363681e+01,  7.29404410e+02],
#     [ 0.00000000e+00,  1.03057080e+03,  4.02180795e+02],
#     [ 0.00000000e+00,  0.00000000e+00,  1.00000000e+00]]); 

K = np.asarray([[536.35697826,   0.,         625.23371273],
 [  0.,         531.82565659, 474.5924729 ],
 [  0.,           0.,           1.        ]]); 

#K = [1.05792775e+03, -1.60363681e+01,  7.29404410e+02, 0.00000000e+00,  1.03057080e+03,  4.02180795e+02, 0.00000000e+00,  0.00000000e+00,  1.00000000e+00]
print(K)

image_name = '/home/tamarar/Desktop/novo/Camera_calibration/calibration/rad/Pic_1.png'   


##############################################

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
            x_dist, y_dist = Dhane_undistortion(x_p, y_p, k = 1.25, f = 1.0)
                    
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

    
    cv2.imwrite('/home/tamarar/Desktop/novo/Camera_calibration/calibration/rad/undistorted.png', Undistorted);
    #cv2.imshow('Undistorted', Undistorted);

##########################################
def fisheye_distortion(x_p, y_p, k): 
    r = np.sqrt(x_p**2 + y_p**2);
    theta = np.arctan(r);
    theta_d = theta * (1 + k[0] * theta**2 + k[1] * theta**4 + k[2] * theta**6 + k[3] * theta**8);
    x_dist = (theta_d / r) * x_p;
    y_dist = (theta_d / r) * y_p;
    
    return x_dist, y_dist;

def Dhane_undistortion(x_d, y_d, k = 1.25, f = 1.0):
    r = np.sqrt(x_d**2 + y_d**2);
    inner_part = np.sin( np.arctan( r / f) ) * k;
    if inner_part > 0.99:
        return None, None;
    R = f * np.tan( np.arcsin( inner_part ));
    enlargement_factor = R/r;
    x_p = enlargement_factor * x_d;
    y_p = enlargement_factor * y_d;
    
    return x_p, y_p;


##########################################

undistort_image();





    