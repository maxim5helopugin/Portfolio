import cv2

face_cascade = cv2.CascadeClassifier('../data/haarcascade_frontalface_default.xml')
smile_cascade = cv2.CascadeClassifier('../data/haarcascade_smile.xml')


vid = cv2.VideoCapture(0)
ret, img = vid.read()
if ret is False:
    print('Failed to open video')

while True:
    ret, img = vid.read()
    if ret is False:
        break

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    faces = face_cascade.detectMultiScale(gray)
    for x, y, w, h in faces:
        cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 1)
        roi = gray[y:y+h, x:x+w]
        smiles = smile_cascade.detectMultiScale(roi)
        for x1, y1, w1, h1 in smiles:
            cv2.rectangle(roi, (x1, y1), (x1 + w1, y1 + h1), (255, 0, 0), 1)

    cv2.imshow('OpenCV Test', img)
    key = cv2.waitKey(10) & 0xff
    if key == 27:   # escape key
        break

vid.release()
cv2.destroyAllWindows()
