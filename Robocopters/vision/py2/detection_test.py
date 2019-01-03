import cv2

orb_extractor = cv2.ORB_create(scaleFactor=1.3, nfeatures=30, scoreType=cv2.ORB_FAST_SCORE)

vid = cv2.VideoCapture(-1)
ret, img = vid.read()
if ret is False:
    print('Failed to open video')

while True:
    ret, img = vid.read()
    if ret is False:
        break
    img = cv2.pyrDown(img)

    kp, des = orb_extractor.detectAndCompute(img, mask=None)
    img = cv2.drawKeypoints(img, kp)

    cv2.imshow('Python2/OpenCV Test', img)
    key = cv2.waitKey(10) & 0xff
    if key == 27:   # escape key
        break

vid.release()
cv2.destroyAllWindows()
