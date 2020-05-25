from cv2 import cv2
import numpy as np
import calibration as calib 

def radialDistortion():
    objPoints = []
    for x in range(10):
        for y in range(8):
            objPoints.append([x,y,1])
    imgPoints = calib.findCorners()
    k = calib.intrinsic(imgPoints, objPoints)
    print("*******************************")
    print("        **INTRINSIC** ")
    print("k = ", k)
    print("\n")
    rt = calib.extrinsic(k, imgPoints[0], objPoints)
    print("*******************************")
    print("        **EXTRINSIC** ")
    print("rt = ", rt)
    print("\n")
    print("*******************************")

    img = cv2.imread('/home/tamarar/Desktop/Camera_calibration/calibration/proba_image/Pic_1.png')
    image = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    u0 = k[0, 2]
    v0 = k[1, 2]

    D = []
    d = []

    
radialDistortion()