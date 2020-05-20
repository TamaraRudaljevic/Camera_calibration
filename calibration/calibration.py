from cv2 import cv2
import numpy as np

# matrix G
# V = [G1
#      G2
#      ...
#      Gn]
# Vb = 0
def val(i, j, H):

    return np.array([
        H[0, i] * H[0, j], H[0, i] * H[1, j] + H[1, i] * H[0, j], H[1, i] * H[1, j], H[2, i] * H[0, j] + H[0, i] * H[2, j], H[2, i] * H[1, j] + H[1, i] * H[2, j], H[2, i] * H[2, j]
    ])

# num => number of input images
# finds corners on chessboard, 8x6
def findCorners(num = 3):

    for img in range(1,num+1):
        corners = []
        readpath = '/home/tamarar/Desktop/Camera_calibration/calibration/proba_image/Pic_'
        image=cv2.imread(readpath + str(img) + '.png')
        gray=cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        ret, corner = cv2.findChessboardCorners(gray, (8, 6), None)
        corners.append(corner)
    return corners
          

# find corners on any chessboards
# num => number of input images
def corners(num = 40):
    for img in range(1, num+1):
        readpath = '/home/tamarar/Desktop/Camera_calibration/calibration/images/Pic_'
        image = cv2. imread(readpath + str(image) + '.jpg')
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        edge = cv2.Canny(gray, 390, 410)
        lines = cv2.HoughLines(edge, 1, np.pi / 180, 55)

        v = []
        h = []
        for rho, theta in lines[0]:
            if(abs(np.tan(theta)) > 0):
                v.append([rho, theta])
            else:
                h.append([rho, theta])
        lines = h + v

        for rho, theta in lines:
            a = np.cos(theta)
            b = np.sin(theta)  
            x0 = a * rho
            y0 = b * rho   
            x1 = int(x0 + 1000 * (-b))
            y1 = int(y0 + 1000 * (a))
            x2 = int(x0 - 1000 * (-b))
            y2 = int(y0 - 1000 * (a))



# creating marix A
# svd(A) => H
def homography(imgPoint, objPoints):
    a = []
    point = np.asarray(imgPoint)

    # calculating matrix A from img_point and obj_points
    for p in range(len(point)):
        if len(objPoints[p]) == 3:
            p1 = objPoints[p]
            p2 = np.append(point[p], 1)
        else:
            p1 = np.append(a, 1)
            p2 = np.append(point[p], 1)
        

        a1 = [-p2[2]*p1[0], -p2[2]*p1[1], -p2[2]*p1[2], 0, 0, 0, p2[0]*p1[0], p2[0]*p1[1], p2[0]*p1[2]]
        a2 = [0, 0, 0, -p2[2] * p1[0], -p2[2] * p1[1], -p2[2] * p1[2], p2[1] * p1[0], p2[1] * p1[1], p2[1] * p1[2]]

        a.append(a1)
        a.append(a2)

    A = np.matrix(a)

    u, s, v = np.linalg.svd(A)

    h = v[8]
    H = np.reshape(h, (3,3))

    # normalization
    H = (H/H.item(8))
    return H


# Intrinsic parameters => matrix k => focal lenght, principal points and skew parameter
# alpha , beta => focal length
# u0, v0 => principal points
# gama => skew
def intrinsic(imgPoints, objPoints):
    #print(point)
    list_v = []
    for p in imgPoints:
        h = homography(p, objPoints)
       
        v1 = val(0, 1, h)
        v2 = val(0, 0, h) - val(1, 1, h)
        list_v.append(v1)
        list_v.append(v2)
        
    V = np.matrix(list_v)
    
    u, s, v = np.linalg.svd(V)
   
    b = v[5]
   
    v0 = (b.item(1)*b.item(3) - b.item(0)*b.item(4))/ (b.item(0) * b.item(2) - b.item(1)**2)
    lam = b.item(5) - (b.item(3)**2 + v0*(b.item(1)*b.item(3) - b.item(0)*b.item(4))) / b.item(0)
    alpha = np.sqrt(lam/(-b.item(0)))
    beta = np.sqrt(lam*b.item(0) / (b.item(0)*b.item(2) - b.item(1)**2))
    gama = -b.item(1)*alpha**2*beta/lam
    u0 = gama*v0/beta - b.item(3)*alpha**2/lam

    k = np.array([[alpha, gama, u0], [0, beta, v0], [0, 0, 1]])
    return k


# Extrinsic => camera rotation and translation matrices
# Orientation between the camera and object 
def extrinsic(intrinsic, imgPoint, objPoints):
    h = homography(imgPoint, objPoints)
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
    

def calibrate():
    objPoints = []
    for x in range(10):
        for y in range(8):
            objPoints.append([x,y,1])

    imgPoints = findCorners()
    #print(corners)
    k = intrinsic(imgPoints, objPoints)
    print("*******************************")
    print("        **INTRINSIC** ")
    print("k = ", k)
    print("\n")
    rt = extrinsic(k, imgPoints[0], objPoints)
    print("*******************************")
    print("        **EXTRINSIC** ")
    print("rt = ", rt)
    print("\n")
    print("*******************************")


calibrate()