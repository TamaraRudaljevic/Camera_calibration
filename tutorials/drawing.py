import numpy as np
from cv2 import cv2

W = 400

def line(img, start, end):
    thickness = 10
    lineType = 8
    cv2.line(img,
             start,
             end,
             (0, 0, 0),
             thickness,
             lineType)
Image = "Image"
Out = "Out"

size = W, W, 3
image =cv2.imread("D:/camera-calibration/image.jpg", cv2.IMREAD_GRAYSCALE)

line(image, (int(100), int(150)), (int(150), int(160)))

cv2.imshow(Image, image)


cv2.waitKey(0)
cv2.destroyAllWindows()