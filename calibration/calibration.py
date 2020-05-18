from cv2 import cv2
import numpy as np



def Homography(p, world):
    a = []
    point = np.asarray(p)

    for p in range(len(point)):
        p1 = world[p]
        p2 = np.append(point[p], 1)

        a1 = [-p2.item(2)*p1.item(0), -p2.item(2)*p1.item(1), -p2.item(2)*p1.item(2), 0, 0, 0, p2.item(0)*p1.item(0), p2.item(0)*p1.item(1), p2.item(0)*p1.item(2)]
        a2 = [0, 0, 0, -p2.item(2) * p1.item(0), -p2.item(2) * p1.item(1), -p2.item(2) * p1.item(2), p2.item(1) * p1.item(0), p2.item(1) * p1.item(1), p2.item(1) * p1.item(2)]

        a.append(a1)
        a.append(a2)

    A = np.matrix(a)

    u, s, v = np.linalg.svd(A)

    h = v[8]
    H = np.reshape(h, (3,3))

    H = (H/H.item(8))
    return H

def Intrinsic(point, world):
    for p in point:
        h = Homography(p, world)
        
