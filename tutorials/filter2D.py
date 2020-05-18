import numpy as np
from cv2 import cv2


def filter2D(image):
    kernel = np.array([[0, -1, 0],
                       [-1, 5, -1],
                       [0, -1, 0]], np.float32)  # kernel should be floating point type

    Filter2D = 'Filter2D' 
    Image = 'Image'

    ### Applying mask to filter the image ###
    out = cv2.filter2D(image, -1, kernel)
    cv2.imshow(Filter2D, out)
    cv2.imshow(Image, image)
    cv2.waitKey(0)

    cv2.imwrite('D:/camera-calibration/Filter2D.jpg', out) # saving out image


### Geting pixel value ###
def pixelIntensityValue(image, x, y):
    _blue = image[x,y,0]
    _green = image[y,x,1]
    _red = image[y,x,2]
    print("Blue: ", _blue)
    print("Green: ", _green)
    print("Red: ", _red)


### Sobel operator ###
def Sobel(image):
    out = cv2.Sobel(image,cv2.CV_64F,0,1,ksize=5)

    Sobel = 'Sobel' 
    Image = 'Image'
    cv2.imshow(Sobel, out)
    cv2.imshow(Image, image)
    cv2.waitKey(0)
    cv2.imwrite('D:/camera-calibration/Sobel2D.jpg', out) # saving out image


if __name__ == "__main__":
    image = cv2.imread("D:/camera-calibration/image.jpg")

    filter2D(image)
    pixelIntensityValue(image, 100, 100)
    Sobel(image)
