import cv2 as cv2


if __name__ == "__main__":
    alpha = 0.5

    image1 = cv2.imread("D:/camera-calibration/image.jpg")
    image2 = cv2.imread("D:/camera-calibration/image1.jpg")

    if image1 is None:
        print("Error loading src1")
        exit(-1)
    elif image2 is None:
        print("Error loading src2")
        exit(-1)

    beta = (1.0 - alpha)
    dst = cv2.addWeighted(image1, alpha, image2, beta, 0.0)

    cv2.imshow('Blended', dst)
    cv2.waitKey(0)

    cv.destroyAllWindows()