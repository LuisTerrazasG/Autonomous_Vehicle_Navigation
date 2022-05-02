import cv2 as cv
import numpy as np


def nothing(x):
    pass

# Trackbar
cv.namedWindow("frame")
cv.createTrackbar("H", "frame", 0, 179, nothing)
cv.createTrackbar("S", "frame", 255, 255, nothing)
cv.createTrackbar("V", "frame", 255, 255, nothing)

img_hsv = np.zeros((250, 500, 3), np.uint8)

while True:
    h = cv.getTrackbarPos("H", "frame")
    s = cv.getTrackbarPos("S", "frame")
    v = cv.getTrackbarPos("V", "frame")

    img_hsv[:] = (h, s, v)
    img_bgr = cv.cvtColor(img_hsv, cv.COLOR_HSV2BGR)

    cv.imshow("frame", img_bgr)
    key = cv.waitKey(1)
    
    #If you press ESC key, the program quits
    if key == 27:
        break

cv.destroyAllWindows()

