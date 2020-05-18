from cv2 import cv2

def Canny(low):
    low_threshold = low
    imageGray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    imageBlur = cv2.blur(imageGray, (3,3))
    detected_edges = cv2.Canny(imageBlur, low_threshold, low_threshold*ratio, kernel_size)
    mask = detected_edges != 0
    out = image * (mask[:,:,None].astype(image.dtype))

    canny = 'Canny'
    cv2.imshow(canny, out)
    cv2.waitKey()


if __name__ == "__main__":
    max_lowThreshold = 100
    ratio = 3
    kernel_size = 3
    image = cv2.imread("D:/camera-calibration/image.jpg")

    Canny(2)
