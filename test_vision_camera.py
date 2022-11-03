# import threading
# import multiprocessing
import multiprocessing
import threading
from time import sleep
# import RPi.GPIO as GPIO
# from adafruit_servokit import ServoKit
# from controller import controller as cntrl
import os
import sys
import serial

# sys.path.append(os.getcwd())
from vision import vision_lib

def objectDetection(vision):
    offset_x = 0.5
    while True:
        vision.detect_circle_object()
        coord_x, coord_y, coord_z = vision.get_circle_coord()
        if coord_x < 0:
            coord_x = -1
        else:
            coord_x -= offset_x
        print("coord: " + str(vision.get_circle_coord()))
        

def runMainThruster(serial):
    while True:
        if serial.in_waiting > 0:
            data = serial.readline().decode('utf-8')
            print("DATA RECEIVE: ", data)
            if data == "0\r\n":
                print("RUNNING THRUSTER")
                # cnt.mainThruster(main_thruster_speed)
            else:
                print("STOP THRUSTER")
            sleep(0.125)
        else: 
            # serial.open()
            continue
        

if __name__ == "__main__":
    vision = vision_lib.hsv_detector(camera_height=240, camera_width=320, masking_enabled=False)
    ser = serial.Serial(port='/dev/ttyUSB0', baudrate=9600, timeout=5)

    t1 = threading.Thread(target=runMainThruster, args=(ser, ))
    p1 = multiprocessing.Process(target=objectDetection, args=(vision, ))
    
    t1.start()
    p1.start()

    t1.join()
    p1.join()

    # objectDetection()
    # vision.detect_circle_object()

    