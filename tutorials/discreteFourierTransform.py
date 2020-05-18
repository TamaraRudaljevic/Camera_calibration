import numpy as np
from cv2 import cv2

def main():

    image = cv2.imread("D:/camera-calibration/image.jpg", cv2.IMREAD_GRAYSCALE)
    
    rows, cols = image.shape
    m = cv2.getOptimalDFTSize( rows )
    n = cv2.getOptimalDFTSize( cols )
    padded = cv2.copyMakeBorder(image, 0, m - rows, 0, n - cols, cv2.BORDER_CONSTANT, value=[0, 0, 0])
    
    planes = [np.float32(padded), np.zeros(padded.shape, np.float32)]
    complexI = cv2.merge(planes)         # Add to the expanded another plane with zeros
    
    cv2.dft(complexI, complexI)         # this way the result may fit in the source matrix
    
    cv2.split(complexI, planes)                   # planes[0] = Re(DFT(I), planes[1] = Im(DFT(I))
    cv2.magnitude(planes[0], planes[1], planes[0])# planes[0] = magnitude
    magI = planes[0]
    
    matOfOnes = np.ones(magI.shape, dtype=magI.dtype)
    cv2.add(matOfOnes, magI, magI) #  switch to logarithmic scale
    cv2.log(magI, magI)
    
    magI_rows, magI_cols = magI.shape
    # crop the spectrum, if it has an odd number of rows or columns
    magI = magI[0:(magI_rows & -2), 0:(magI_cols & -2)]
    cx = int(magI_rows/2)
    cy = int(magI_cols/2)
    q0 = magI[0:cx, 0:cy]         # Top-Left - Create a ROI per quadrant
    q1 = magI[cx:cx+cx, 0:cy]     # Top-Right
    q2 = magI[0:cx, cy:cy+cy]     # Bottom-Left
    q3 = magI[cx:cx+cx, cy:cy+cy] # Bottom-Right
    tmp = np.copy(q0)               # swap quadrants (Top-Left with Bottom-Right)
    magI[0:cx, 0:cy] = q3
    magI[cx:cx + cx, cy:cy + cy] = tmp
    tmp = np.copy(q1)               # swap quadrant (Top-Right with Bottom-Left)
    magI[cx:cx + cx, 0:cy] = q2
    magI[0:cx, cy:cy + cy] = tmp
    
    cv2.normalize(magI, magI, 0, 1, cv2.NORM_MINMAX) # Transform the matrix with float values into a
    
    cv2.imshow("Input Image"       , image   )    # Show the result
    cv2.imshow("spectrum magnitude", magI)
    cv2.waitKey()
if __name__ == "__main__":
    main()