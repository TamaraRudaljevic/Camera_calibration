import numpy as np
from cv2 import cv2

def main():
   
    # Load the image
    src = cv2.imread("D:/camera-calibration/image.jpg")
    # Check if image is loaded fine
       
    while 1:
        rows, cols, _channels = map(int, src.shape)
        
        cv2.imshow('Pyramids Demo', src)
        
        k = cv2.waitKey(0)
    
       
        src = cv2.pyrUp(src, dstsize=(2 * cols, 2 * rows))
        cv2.imshow('In', src)
        cv2.waitKey(0)
        print ('** Zoom In: Image x 2')
        

        src = cv2.pyrDown(src, dstsize=(cols // 2, rows // 2))
        cv2.imshow('Out', src)
        cv2.waitKey(0)
        print ('** Zoom Out: Image / 2')
            
    cv2.destroyAllWindows()
   
if __name__ == "__main__":
    main()