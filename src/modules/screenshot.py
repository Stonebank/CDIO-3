import cv2

cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)

ret, frame = cap.read()

cv2.imwrite("screenshot.png", frame)

cap.release()
