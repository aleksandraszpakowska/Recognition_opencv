import cv2
import numpy as np

size_from_pygame_chart = (542, 373)
cap = cv2.VideoCapture(1)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, size_from_pygame_chart[0])
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, size_from_pygame_chart[1])
center_point = [(0, 0)]

def color_detection(frame):
    global center_point #define a global value
    kernel = np.ones((2, 2), np.uint8)
    lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
    a_channel = lab[:, :, 1]

    ret, thresh = cv2.threshold(a_channel, 105, 255, cv2.THRESH_BINARY_INV)  # to binary
    thresh = cv2.morphologyEx(thresh, cv2.MORPH_GRADIENT, kernel)  # to get outer boundery only

    thresh = cv2.dilate(thresh, kernel, iterations=5)  # to strength week pixels
    contours, hierarchy = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

    if len(contours) > 0:
        # find the biggest countour (c) by the area
        c = max(contours, key=cv2.contourArea)
        x, y, w, h = cv2.boundingRect(c)
        x1 = x + w
        y1 = y + h
        # draw the biggest contour (c) in green
        cv2.rectangle(frame, (x, y), (x1, y1), (255, 0, 0), 2)
        center_point = [int((x + x1) / 2), int((y + y1) / 2)] #still updating the global value
        # coordinates of the rectangle
        print("Top left corner: ", x, y)
        print("Top right corner: ", x1, y)
        print("Bottom left corner: ", x, y1)
        print("Bottom right corner: ", x1, y1)
        print("Center point: ", center_point)

while True:
    _, frame = cap.read()
    center_point = [(0,0)] #init global value
    color_detection(frame) # the global value exist
    print(center_point)
    cv2.imshow("Frame", frame)

    if cv2.waitKey(1) & 0xFF == ord('d'):
        break

cap.release()
cv2.destroyAllWindows()
