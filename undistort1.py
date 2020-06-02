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


    for i in range(N):
        
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


def undistortMapfisheye(K, distCoef, R, P, size): 
    k1 = distCoef[0]
    k2 = distCoef[1]
    p1 = distCoef[2]
    p2 = distCoef[3]
    k3 = distCoef[4]

    v0 = K[1].item(2)
    u0 = K[0].item(2)
    fx = K[0].item(0)
    fy = K[1].item(1)

    iR = (P*R)

    print(iR[0].item(0))

    height, width = size[:2] # size = img.shape

    for i in range(1, height+1):
        _x = i*iR[0].item(1) + iR[0].item(2)
        _y = i*iR[1].item(1) + iR[1].item(2)
        _w = i*iR[2].item(1) + iR[2].item(2)

        for j in range(1, width+1):
            x = _x/_w
            y = _y/_w 

            r = np.sqrt(x**2 + y**2)
            theta = np.arctan(r)

            theta2 = theta**2
            theta4 = theta2**2
            theta6 = theta4**2
            theta_d = theta * (1 + k1*theta2 + k2*theta4 + k3*theta6)

            if r == 0:
                scale = 1
            else:
                scale = theta_d/r
            
            u = fx*x*scale + u0
            v = fy*y*scale + v0

            _x += iR[0].item(0)
            _y += iR[1].item(0)
            _w += iR[2].item(0)



def undistortImagefisheye(distorted, undistorted, K, distCoef, knew):
    size = distorted.shape
    R = np.eye(3)
    #print(R)
    map1 = np.zeros((distorted.shape[0], distorted.shape[1]))
    map2 = np.zeros((distorted.shape[0], distorted.shape[1]))
    undistortMapfisheye(K, distCoef, R , knew, size)
    
    cv2.remap(distorted, undistorted, map1, map2, cv2.INTER_LINEAR, cv2.BORDER_CONSTANT)

def undistort():

image = cv2.imread('/home/tamarar/Desktop/novo/Camera_calibration/calibration/newCalibrationImages/Pic_1.jpg')
undistorted = []
undistortImagefisheye(image, undistorted, K, k, K)


