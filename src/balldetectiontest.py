import cv2
import detection

cap = cv2.VideoCapture(1, cv2.CAP_DSHOW)

while True:

    success, img = cap.read()

    balls = detection.detectBalls(img)

    cv2.imshow("Image", img)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break