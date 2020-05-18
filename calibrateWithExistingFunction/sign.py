import numpy as np
from cv2 import cv2
import matplotlib.pyplot as plt
import matplotlib.image as mpimg

def warp(image):
    img_size = (image.shape[1], image.shape[0])

    src = np.float32([[63, 168], [88, 169], [70, 113], [35, 110]])

    dst = np.float32([[63, 168], [88, 169], [70, 113], [35, 110]])

    M = cv2.getPerspectiveTransform(src, dst)

    warped = cv2.warpPerspective(image, M, img_size, flags=cv2.INTER_LINEAR)

    return warped

name = 'D:/camera-calibration/calibrateWithExistingFunction/siggn.jpg'
img = cv2.imread(name)

plt.imshow(img)
plt.waitforbuttonpress()

plt.plot(63,168,'.')
plt.plot(88,169,'.')
plt.plot(70,113,'.')
plt.plot(35,110,'.')


warped = warp(img)
 
f, (ax1, ax2) = plt.subplots(1, 2, figsize=(20,10))
ax1.set_title('source image')
ax1.imshow(img)
ax2.set_title('warped image')
ax2.imshow(warped)
plt.waitforbuttonpress()