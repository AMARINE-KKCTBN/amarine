# import threading
# import multiprocessing
from time import sleep
# import RPi.GPIO as GPIO
# from adafruit_servokit import ServoKit
# from controller import controller as cntrl
import os
import sys

sys.path.append(os.getcwd())
from vision import vision_lib

def objectDetection():
    coord_x = 0
    offset_x = 0.5
    vision = vision_lib.hsv_detector(camera_height=1280, camera_width=720, masking_enabled=False)
    while True:
        vision.detect_circle_object()
        # vision.show_image()
        if coord_x < 0:
            coord_x = -1
        else:
            coord_x -= offset_x
        # print("coord: " + str(round(coord_x, 3)))

if __name__ == "__main__":
    print("SUCESS")
    objectDetection()
    # vision.detect_circle_object()

    