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

def corner(path, num):
    imagelist=[]
    c=0
    corner_list=[]
    for img in range(1,num+1):
        image=cv2.imread(path+str(img)+'.jpg')
        image1=cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
        edge=cv2.Canny(image1,390,410)
        line = cv2.HoughLines(edge,1,np.pi/180,55)
        
        hl=[]
        hv=[]
        for rho,theta in line[0]:
            
            if (abs(np.tan(theta))>1):
                hl.append([rho,theta])
            else:
                hv.append([rho,theta])
        
      
    
       
                 

# creating marix A
# svd(A) => H
def Homography(p, world):
    a = []
    point = np.asarray(p)

    # calculating matrix A from points and world points
    for p in range(len(point)):
        if len(world[p]) == 3:
            p1 = world[p]
            p2 = np.append(point[p], 1)
        else:
            p1 = np.append(a, 1)
            p2 = np.append(point[p], 1)
        

        a1 = [-p2[2]*p1[0], -p2[2]*p1[1], -p2[2]*p1[2], 0, 0, 0, p2[0]*p1[0], p2[0]*p1[1], p2[0]*p1[2]]
        a2 = [0, 0, 0, -p2[2] * p1[0], -p2[2] * p1[1], -p2[2] * p1[2], p2[1] * p1[0], p2[1] * p1[1], p2[1] * p1[2]]

        a.append(a1)
        a.append(a2)

    A = np.matrix(a)
    #print(A)

    u, s, v = np.linalg.svd(A)

    # last column is h
    h = v[8]
    #print(h)
    H = np.reshape(h, (3,3))
    #print(H)

    # normalization
    H = (H/H.item(8))
    return H

def Intrinsic(point, world):
    #print(point)
    list_v = []
    for p in point:
        h = Homography(p, world)
        #print(h)
        v1 = val(0, 1, h)
        v2 = val(0, 0, h) - val(1, 1, h)
        list_v.append(v1)
        list_v.append(v2)
        #print(list_v)
        
    V = np.matrix(list_v)
    #print(V)
    u, s, v = np.linalg.svd(V)
    #print(v)
    b = v[5]
    #print(b)

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
    


image = '/home/tamarar/Desktop/Camera_calibration/calibration/calibration_test.jpg'
dst = "/home/tamarar/Desktop/Camera_calibration/calibration/dst.png"
#print(corners)
