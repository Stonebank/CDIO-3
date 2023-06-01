import cv2
import cvzone
from cvzone import ColorFinder

cap = cv2.VideoCapture(1, cv2.CAP_DSHOW)

cf = ColorFinder(True)

hsvVals = "red"

while True:

    success, img = cap.read()

    imgColor, mask = cf.update(img, hsvVals)

    cv2.imshow("Image", imgColor)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break