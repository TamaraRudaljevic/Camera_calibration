from cv2 import cv2
import numpy as np

def val(i, j, H):

    return np.array([
        H[0, i] * H[0, j],
        H[0, i] * H[1, j] + H[1, i] * H[0, j],
        H[1, i] * H[1, j],
        H[2, i] * H[0, j] + H[0, i] * H[2, j],
        H[2, i] * H[1, j] + H[1, i] * H[2, j],
        H[2, i] * H[2, j]
    ])

def corner(img):
    image = cv2.imread(img)
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(gray, 390, 410)
    lines = cv2.HoughLines(edges, 1, np.pi/180, 55)

    h = []
    v = []

    for rho, theta in lines[0]:
        if(abs(np.tan(theta)) > 1):
            v.append([rho, theta])
        else:
            h.append([rho, theta])
    
    for line in lines:
        rho, theta = line[0]
        a = np.cos(theta)
        b = np.sin(theta)
        x0 = a*rho
        y0 = b*rho
        x1 = int(x0 + 1000*(-b))
        y1 = int(y0 + 1000*(a))
        x2 = int(x0 - 1000*(-b))
        y2 = int(y0 - 1000*(a))


# creating marix A
# svd(A) => H
def Homography(p, world):
    a = []
    point = np.asarray(p)

    # calculating matrix A from points and world points
    for p in range(len(point)):
        p1 = world[p]
        p2 = np.append(point[p], 1)

        a1 = [-p2.item(2)*p1.item(0), -p2.item(2)*p1.item(1), -p2.item(2)*p1.item(2), 0, 0, 0, p2.item(0)*p1.item(0), p2.item(0)*p1.item(1), p2.item(0)*p1.item(2)]
        a2 = [0, 0, 0, -p2.item(2) * p1.item(0), -p2.item(2) * p1.item(1), -p2.item(2) * p1.item(2), p2.item(1) * p1.item(0), p2.item(1) * p1.item(1), p2.item(1) * p1.item(2)]

        a.append(a1)
        a.append(a2)

    A = np.matrix(a)

    u, s, v = np.linalg.svd(A)

    # last column is h
    h = v[8]
    H = np.reshape(h, (3,3))

    # normalization
    H = (H/H.item(8))
    return H

def Intrinsic(point, world):
    list_v = []
    for p in point:
        h = Homography(p, world)
        v1 = val(0, 1, h)
        v2 = val(0, 0, h) - val(1, 1, h)
        list_v.append(v1)
        list_v.append(v2)
        
    V = np.matrix(list_v)
    u, s, v = np.linalg.svd(V)
    b = v[5]

    v0 = (b.item(1)*b.item(3) - b.item(0)*b.item(4))/ (b.item(0) * b.item(2) - b.item(1)**2)
    lam = b.item(5) - (b.item(3)**2 + v0*(b.item(1)*b.item(3) - b.item(0)*b.item(4))) / b.item(0)
    alpha = np.sqrt(lam/b.item(0))
    beta = np.sqrt(lam*b.item(0) / (b.item(0)*b.item(2) - b.item(1)**2))
    gama = -b.item(1)*alpha**2*beta/lam
    u0 = gama*v0/beta - b.item(3)*alpha**2/gama

    k = np.array([[alpha, gama, u0], [0, beta, v0], [0, 0, 1]])
    return k

def Extrinsic(intrinsic, cornerPoint, worldPoint):
    h = Homography(cornerPoint, worldPoint)
    h = np.linalg.inv(h)
    intrinsicsInv = np.linalg.inv(intrinsic)

    h1 = h[:, 0]
    h2 = h[:, 1]
    h3 = h[:, 2]

    lam_r1 = 1 / np.linalg.norm(np.dot(intrinsicsInv, h1))
    lam_r2 = 1 / np.linalg.norm(np.dot(intrinsicsInv, h2))
    lam_r3 = (lam_r1 + lam_r2) / 2
    r1 = lam_r1 * np.dot(intrinsicsInv, h1)
    r2 = lam_r2 * np.dot(intrinsicsInv, h2)
    r3 = np.cross(r1, r2, axis=0)
    t = np.array(lam_r3 * np.dot(intrinsicsInv, h3)).transpose()

    r = np.append(r1, r2, axis=1)
    r = np.append(r, r3, axis=1)
    rt = np.append(r, np.transpose(t), axis=1)
    return rt
    

image = "/home/tamarar/Desktop/Camera_calibration/calibration/calibration_test.png"