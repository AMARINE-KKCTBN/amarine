import numpy as np
import imutils
import cv2
import json

with open('red_hsv.json', 'r') as openfile:
 
    # Reading from json file
    red_hsv = json.load(openfile)

with open('circle_params.json', 'r') as openfile:
 
    # Reading from json file
    circle_params = json.load(openfile)

redLower = (red_hsv['H_Lower'], red_hsv['S_Lower'], red_hsv['V_Lower'])
redUpper = (red_hsv['H_Higher'], red_hsv['S_Higher'], red_hsv['V_Higher'])

camera = cv2.VideoCapture(0)

width = camera.get(cv2.CAP_PROP_FRAME_WIDTH)
height = camera.get(cv2.CAP_PROP_FRAME_HEIGHT)

while camera.isOpened():
    # grab the current frame
    (grabbed, frame) = camera.read()

    frame = imutils.resize(frame)

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    mask = cv2.inRange(hsv, redLower, redUpper)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)

    edges = cv2.Canny(mask, 50, 150)

    circles = cv2.HoughCircles(edges, cv2.HOUGH_GRADIENT, circle_params['param3'], circle_params['param4'],
                               param1=circle_params['param1'], param2=circle_params['param2'],
                               minRadius=circle_params['min_radius'], maxRadius=circle_params['max_radius'])

    if circles is not None:
        circles = np.uint16(np.around(circles))
        # print(circles , width, height)
        for i in circles[0, :]:
            # draw the outer circle
            cv2.circle(frame, (i[0], i[1]), i[2], (0, 255, 0), 2)
            # draw the center of the circle
            cv2.circle(frame, (i[0], i[1]), 2, (0, 0, 255), 3)
            circle_x = round(i[0] / width * 2 - 1, 3)
            circle_y = round(i[1] / height * 2 - 1, 3)
            circle_z = i[2]

        print('x : ', circle_x, ' y : ', circle_y, ' radius : ',circle_z)

    cv2.imshow("Frame", frame)
    cv2.imshow("Mask", mask)
    cv2.imshow("Edge", edges)
    key = cv2.waitKey(1) & 0xFF

    # if the 'q' key is pressed, stop the loop
    if key == ord("q"):
        break

# cleanup the camera and close any open windows
camera.release()
cv2.destroyAllWindows()
