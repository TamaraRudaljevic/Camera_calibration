import numpy as np
from cv2 import cv2

if __name__ == "__main__":
    ### Loading image ###
    image = cv2.imread("D:/camera-calibration/image.jpg")

    Image = 'Image' #window where original image will appear
    Out = 'Out' #window where out image will appear

    ### Type and dimensions of image ###
    print("This image is: ", type(image), "with dimensions: ", image.shape)

    ### cvtColor is changing the color space ###
    out = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    ### Displaying the image ###
    cv2.imshow(Out, out) 
    cv2.imshow(Image, image)

    cv2.waitKey(0)
    cv2.destroyAllWindows()
    