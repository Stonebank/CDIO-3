import cv2 as cv

cam = cv.VideoCapture(0)

print("Entering while loop")
while(True):
    print("1")
    ##if not cam.isOpened():
    ##    print("Cannot open camera")
    ##    exit()
    # Capture the video frame
    # by frame
    ret, frame = cam.read()
    print("2")
    # Display the resulting frame
    cv.imshow('frame', frame)

    # the 'q' button is set as the
    # quitting button you may use any
    # desired button of your choiceq
    print("3")
    if cv.waitKey(1) & 0xFF == ord('q'):
        break

# After the loop release the cap object
cam.release()
# Destroy all the windows
cv.destroyAllWindows()