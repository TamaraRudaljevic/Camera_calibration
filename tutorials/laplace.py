import sys
import cv2 as cv2

if __name__ == "__main__":
    image = cv2.imread("D:/camera-calibration/image.jpg", cv2.IMREAD_COLOR) # Load an image
   
    # Remove noise by blurring with a Gaussian filter
    image = cv2.GaussianBlur(image, (3, 3), 0)
   
    # Convert the image to grayscale
    imageGray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
   
    # Apply Laplace function
    dst = cv2.Laplacian(imageGray, ddepth = cv2.CV_16S, ksize = 3)
   
    # converting back to uint8
    out = cv2.convertScaleAbs(dst)
   
    Out = 'Out'
    # [display]
    cv2.imshow(Out, out)
    cv2.waitKey(0)
    # [display]