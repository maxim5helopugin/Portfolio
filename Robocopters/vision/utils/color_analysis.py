import cv2
import numpy as np
from image_utils import *


def nothing(x):
    pass


def color_picker(event, x, y, flags, param):
    global hsv
    if event == cv2.EVENT_LBUTTONDOWN:
        color = hsv[y, x]
        print(color)

hsv = None


def color_analysis(img):
    global hsv
    tw = 'Color Analysis'   # trackbar window
    cv2.namedWindow(tw)

    iw = 'Colors'   # image window
    cv2.namedWindow(iw)
    cv2.setMouseCallback(iw, color_picker)

    max_val = 255
    init = [2, 12,
            0, max_val,
            0, max_val]

    cv2.createTrackbar('low H', tw, init[0], 180, nothing)
    cv2.createTrackbar('high H', tw, init[1], 180, nothing)
    cv2.createTrackbar('low S', tw, init[2], max_val, nothing)
    cv2.createTrackbar('high S', tw, init[3], max_val, nothing)
    cv2.createTrackbar('low V', tw, init[4], max_val, nothing)
    cv2.createTrackbar('high V', tw, init[5], max_val, nothing)

    while True:
        lowH = cv2.getTrackbarPos('low H', tw)
        highH = cv2.getTrackbarPos('high H', tw)
        lowS = cv2.getTrackbarPos('low S', tw)
        highS = cv2.getTrackbarPos('high S', tw)
        lowV = cv2.getTrackbarPos('low V', tw)
        highV = cv2.getTrackbarPos('high V', tw)

        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # Color range thresholding
        low_hsv = np.array([lowH, lowS, lowV])
        high_hsv = np.array([highH, highS, highV])
        hsv_mask = cv2.inRange(hsv, low_hsv, high_hsv)

        # Noise reduction
        hsv_mask = cv2.erode(hsv_mask, (3, 3), iterations=3)

        masked = cv2.bitwise_and(img, img, mask=hsv_mask)

        cv2.imshow(iw, masked)
        key = cv2.waitKey(10) & 0xFF
        if key == 27:
            break


if __name__ == "__main__":
    img_file = "C:/Users/seanr/PycharmProjects/QUAD/vision/saved/ORB Detection testing1847.jpg"
    img = cv2.imread(img_file)
    color_analysis(img)
