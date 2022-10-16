# Python program to illustrate the concept
# of threading
# importing the threading module
import threading
import cv2
import RPi.GPIO as GPIO
import numpy as np
import json
import time

with open("red_hsv.json", "r") as openfile:

    # Reading from json file
    red_hsv = json.load(openfile)

with open("circle_params.json", "r") as openfile:

    # Reading from json file
    circle_params = json.load(openfile)

redLower = (red_hsv["H_Lower"], red_hsv["S_Lower"], red_hsv["V_Lower"])
redUpper = (red_hsv["H_Higher"], red_hsv["S_Higher"], red_hsv["V_Higher"])

camera = cv2.VideoCapture(0)

width = camera.get(cv2.CAP_PROP_FRAME_WIDTH)
height = camera.get(cv2.CAP_PROP_FRAME_HEIGHT)

enable_ = True

# SERVO INIT

GPIO.setmode(GPIO.BCM)

servo_pin_bcm = 24
servo_pin_board = 18

GPIO.setup(servo_pin_bcm, GPIO.OUT)
servo_pwm = GPIO.PWM(servo_pin_bcm, 50)  # 50 Hz
servo_pwm.start(0)

# PUSH BUTTON INIT

# Stop Button
stop_button_bmc = 25
GPIO.setup(
    stop_button_bmc, GPIO.IN, pull_up_down=GPIO.PUD_DOWN
)  # Set pin 25 to be an input pin and set initial value to be pulled low (off)


# STEPPER MOTOR INIT

out1 = 16
out2 = 19
out3 = 20
out4 = 26

# careful lowering this, at some point you run into the mechanical limitation of how quick your motor can move
step_sleep = 0.002

step_count = 2400  # 200 = 1 step/180

# setting up
# GPIO.setmode( GPIO.BCM )
GPIO.setup(out1, GPIO.OUT)
GPIO.setup(out2, GPIO.OUT)
GPIO.setup(out3, GPIO.OUT)
GPIO.setup(out4, GPIO.OUT)

# initializing
GPIO.output(out1, GPIO.LOW)
GPIO.output(out2, GPIO.LOW)
GPIO.output(out3, GPIO.LOW)
GPIO.output(out4, GPIO.LOW)


def cleanup():
    GPIO.output(servo_pin_bcm, GPIO.LOW)
    GPIO.output(out1, GPIO.LOW)
    GPIO.output(out2, GPIO.LOW)
    GPIO.output(out3, GPIO.LOW)
    GPIO.output(out4, GPIO.LOW)
    GPIO.cleanup()


def stepper_test():
    global enable_, step_count, step_sleep
    # while enable_:
    # #pull
    # for i in range(0, step_count, 1):
    # 	setStepperMotor(i, step_sleep)
    # #push
    # for i in range(step_count, 0, -1):
    # 	setStepperMotor(i, step_sleep)
    # forward(step_count, step_sleep)
    # backward(step_count, step_sleep)
    # cleanup()
    # exit( 1 )


def forward(step_count, step_sleep):
    i = 0
    for i in range(step_count):
        print("step: " + str(i) + "/" + str(step_count))
        if i % 4 == 0:
            GPIO.output(out4, GPIO.HIGH)
            GPIO.output(out3, GPIO.LOW)
            GPIO.output(out2, GPIO.LOW)
            GPIO.output(out1, GPIO.LOW)
        elif i % 4 == 1:
            GPIO.output(out4, GPIO.LOW)
            GPIO.output(out3, GPIO.LOW)
            GPIO.output(out2, GPIO.HIGH)
            GPIO.output(out1, GPIO.LOW)
        elif i % 4 == 2:
            GPIO.output(out4, GPIO.LOW)
            GPIO.output(out3, GPIO.HIGH)
            GPIO.output(out2, GPIO.LOW)
            GPIO.output(out1, GPIO.LOW)
        elif i % 4 == 3:
            GPIO.output(out4, GPIO.LOW)
            GPIO.output(out3, GPIO.LOW)
            GPIO.output(out2, GPIO.LOW)
            GPIO.output(out1, GPIO.HIGH)
        time.sleep(step_sleep)


def backward(step_count, step_sleep):
    for i in range(step_count, 0, -1):
        print("step: " + str(i) + "/" + str(step_count))
        if i % 4 == 0:
            GPIO.output(out4, GPIO.HIGH)
            GPIO.output(out3, GPIO.LOW)
            GPIO.output(out2, GPIO.LOW)
            GPIO.output(out1, GPIO.LOW)
        elif i % 4 == 1:
            GPIO.output(out4, GPIO.LOW)
            GPIO.output(out3, GPIO.LOW)
            GPIO.output(out2, GPIO.HIGH)
            GPIO.output(out1, GPIO.LOW)
        elif i % 4 == 2:
            GPIO.output(out4, GPIO.LOW)
            GPIO.output(out3, GPIO.HIGH)
            GPIO.output(out2, GPIO.LOW)
            GPIO.output(out1, GPIO.LOW)
        elif i % 4 == 3:
            GPIO.output(out4, GPIO.LOW)
            GPIO.output(out3, GPIO.LOW)
            GPIO.output(out2, GPIO.LOW)
            GPIO.output(out1, GPIO.HIGH)
        time.sleep(step_sleep)


def setStepperMotor(step_count, step_sleep):
    if step_count % 100 == 0:
        print("step: " + str(step_count))
    if step_count % 4 == 0:
        GPIO.output(out4, GPIO.HIGH)
        GPIO.output(out3, GPIO.LOW)
        GPIO.output(out2, GPIO.LOW)
        GPIO.output(out1, GPIO.LOW)
    elif step_count % 4 == 1:
        GPIO.output(out4, GPIO.LOW)
        GPIO.output(out3, GPIO.LOW)
        GPIO.output(out2, GPIO.HIGH)
        GPIO.output(out1, GPIO.LOW)
    elif step_count % 4 == 2:
        GPIO.output(out4, GPIO.LOW)
        GPIO.output(out3, GPIO.HIGH)
        GPIO.output(out2, GPIO.LOW)
        GPIO.output(out1, GPIO.LOW)
    elif step_count % 4 == 3:
        GPIO.output(out4, GPIO.LOW)
        GPIO.output(out3, GPIO.LOW)
        GPIO.output(out2, GPIO.LOW)
        GPIO.output(out1, GPIO.HIGH)
    time.sleep(step_sleep)


# circle_x = 0


def hsv_detector():
    global enable_, circle_x, circle_y, circle_z
    while enable_:
        (grabbed, frame) = camera.read()

        # frame = imutils.resize(frame)

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        mask = cv2.inRange(hsv, redLower, redUpper)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        edges = cv2.Canny(mask, 50, 150)

        circles = cv2.HoughCircles(
            edges,
            cv2.HOUGH_GRADIENT,
            circle_params["param3"],
            circle_params["param4"],
            param1=circle_params["param1"],
            param2=circle_params["param2"],
            minRadius=circle_params["min_radius"],
            maxRadius=circle_params["max_radius"],
        )

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

            print("x : ", circle_x, " y : ", circle_y, " radius : ", circle_z)

        cv2.imshow("Frame", frame)
        cv2.imshow("Mask", mask)
        cv2.imshow("Edge", edges)
        key = cv2.waitKey(1) & 0xFF

        # if the 'q' key is pressed, stop the loop
        if key == ord("q"):
            enable_ = False
            return

        time.sleep(0.1)


def SetServoAngle(angle):

    duty = angle / 30 + 4
    print("duty:", duty)
    # duty = angle
    GPIO.output(servo_pin_bcm, True)
    servo_pwm.ChangeDutyCycle(duty)
    time.sleep(0.5)


def servo_control_test():
    global enable_, circle_x
    avg_limit = 5
    avg_count = 0
    avg_coord = 0
    avg_sum = 0
    angle_offset = 0
    while enable_:
        print("SERVO TEST: " + str(circle_x))
        if avg_count < avg_limit:
            avg_sum += circle_x
            avg_count += 1
            time.sleep(0.2)
        else:
            avg_coord = avg_sum / avg_limit
            avg_count = 0
            avg_sum = 0
            desired_angle = (avg_coord + 1) * 90
            desired_angle += angle_offset
            print("Rudder Angle: " + str(desired_angle))
            SetServoAngle(desired_angle)
        # for i in range(0, 180, 10):
        # 	print(i)
        # 	SetServoAngle(i)
        # for i in range(180, 0, -20):
        # 	print(i)
        # 	SetServoAngle(i)


def print_status():
    global enable_
    while enable_:
        if GPIO.input(stop_button_bmc) == GPIO.HIGH:
            print("Button was pushed!")
            enable_ = False
        else:
            print("Running")
        time.sleep(1)


if __name__ == "__main__":
    # creating thread
    t1 = threading.Thread(target=stepper_test, args=())
    t2 = threading.Thread(target=print_status, args=())
    t3 = threading.Thread(target=hsv_detector, args=())
    t4 = threading.Thread(target=servo_control_test, args=())

    # starting thread 1
    t1.start()
    # starting thread 2
    t2.start()
    # starting thread 3
    t3.start()
    # starting thread 4
    time.sleep(2)
    t4.start()

    # wait until thread 1 is completely executed
    t1.join()
    # wait until thread 2 is completely executed
    t2.join()
    # wait until thread 3 is completely executed
    t3.join()
    # wait until thread 4 is completely executed
    t4.join()

    cleanup()

    camera.release()
    cv2.destroyAllWindows()

    # both threads completely executed
    print("Done!")
