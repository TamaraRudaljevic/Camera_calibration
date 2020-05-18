import cv2 as cv2

src = cv2.imread("D:/camera-calibration/image.jpg")
if src is None:
    print('Could not open or find the image:', args.input)
    exit(0)
src = cv2.cvtColor(src, cv2.COLOR_BGR2GRAY)
dst = cv2.equalizeHist(src)
cv2.imshow('Source image', src)
cv2.imshow('Equalized Image', dst)
cv2.waitKey()