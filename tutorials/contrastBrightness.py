import numpy as np
from cv2 import cv2

if __name__ == "__main__":
    image = cv2.imread("D:/camera-calibration/image.jpg")

    out = np.zeros(image.shape, image.dtype)
    alpha = 2.2 # Contrast control
    beta = 50    # Brightness control
   
    ### red, green, blue ###
    for y in range(image.shape[0]): 
        for x in range(image.shape[1]):
            for c in range(image.shape[2]):
                out[y,x,c] = np.clip(alpha*image[y,x,c] + beta, 0, 255)

    Image = 'Image' 
    Out = 'Out'
    cv2.imshow(Image, image)
    cv2.imshow(Out, out)
    cv2.waitKey()