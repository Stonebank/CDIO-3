import cv2
import numpy as np

# nam


class FrameTransformer:
    manMode = False
    selectCount = 0
    # Field dimensions
    width = 750
    height = 500
    corners = [[0, 0], [750, 0], [0, 500], [750, 500]]
    goal1 = None

    def __init__(self):
        pass

    def transform(self, frame, frameCount):
        if (not self.manMode and frameCount % 20 == 0):
            self.corners = self.getCorners(frame)

        if (self.corners != None):
            # Coordinates for the corners
            oldCoordinates = np.float32([self.corners[0], self.corners[1],
                                         self.corners[2], self.corners[3]])

            # New coordinates for the corners
            newCoordinates = np.float32([[0, 0], [self.width, 0],
                                         [0, self.height], [self.width, self.height]])

            # Transform image
            matrix = cv2.getPerspectiveTransform(
                oldCoordinates, newCoordinates)
            transformed = cv2.warpPerspective(
                frame, matrix, (self.width, self.height))
            # Create circles to indicate detected corners
            frame = cv2.circle(frame, tuple(
                self.corners[0]), 20, (255, 0, 0), 2)
            frame = cv2.circle(frame, tuple(
                self.corners[1]), 20, (255, 0, 0), 2)
            frame = cv2.circle(frame, tuple(
                self.corners[2]), 20, (255, 0, 0), 2)
            frame = cv2.circle(frame, tuple(
                self.corners[3]), 20, (255, 0, 0), 2)
            return transformed

    def get_point(self, event, x, y, flags, param):

        if event == cv2.EVENT_LBUTTONUP:

            self.selectCount += 1
            if (self.selectCount == 1):
                self.corners[0] = [x, y]
                print("Upper left(", x, ",", y, ")")
            elif (self.selectCount == 2):
                self.corners[1] = [x, y]
                print("Upper right(", x, ",", y, ")")
            elif (self.selectCount == 3):
                self.corners[2] = [x, y]
                print("Lower left(", x, ",", y, ")")
            elif (self.selectCount == 4):
                self.corners[3] = [x, y]
                print("Lower right(", x, ",", y, ")")
                self.selectCount = 0

    def get_goal(self, event, x, y, flags, param):

        if event == cv2.EVENT_LBUTTONUP:

            self.goal1 = (x,y)
            print(x,y)

    def getCorners(self, frame):
        upper_left = [99999, 99999]
        upper_right = [0, 99999]
        lower_left = [99999, 0]
        lower_right = [0, 0]

        # Define lower/upper color field for red
        lower_red = np.array([0, 0, 200], dtype="uint8")
        upper_red = np.array([100, 100, 255], dtype="uint8")

        # Frame dimensions
        height = frame.shape[0]
        width = frame.shape[1]

        redMask = cv2.inRange(frame, lower_red, upper_red)

        # Filter out everything that is not red
        wall = frame.copy()
        wall[np.where(redMask == 0)] = 0
        # wall = cv2.GaussianBlur(wall,(41,41) ,5)
        # Convert to grey
        wall = cv2.cvtColor(wall, cv2.COLOR_BGR2GRAY)

        # Find corners
        # dest = cv2.cornerHarris(np.float32(wall), 20, 3, 0.2496)
        dest = cv2.cornerHarris(np.float32(wall), 20, 3, 0.14)
        dest = cv2.dilate(dest, None)

        # frame[dest > 0.01 * dest.max()]=[0, 0, 255]

        # Iterate through the detected corners, and use the appropiate ones
        # TODO should be optimized in the future
        thresh = 0.1*dest.max()
        # if lower_right[0] == 0 :
        for j in range(0, dest.shape[0]):
            for i in range(0, dest.shape[1]):
                if (dest[j, i] > thresh):
                    if i < upper_left[0] and i < width/2 and j < upper_left[1] and j < height/2:
                        upper_left[0] = i
                        upper_left[1] = j
                    if i > upper_right[0] and i > width/2 and j < upper_right[1] and j < height/2:
                        upper_right[0] = i
                        upper_right[1] = j
                    if i < lower_left[0] and i < width/2 and j > lower_left[1] and j > height/2:
                        lower_left[0] = i
                        lower_left[1] = j
                    if i > lower_right[0] and i > width/2 and j > lower_right[1] and j > height/2:
                        lower_right[0] = i
                        lower_right[1] = j
        return [upper_left, upper_right, lower_left, lower_right]

