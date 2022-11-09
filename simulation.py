from threading import Thread
from multiprocessing import Process, Value, Queue
from time import sleep
import RPi.GPIO as GPIO
from adafruit_servokit import ServoKit
from controller import controller as cntrl
from vision import vision_lib
import serial 
import sys
from ctypes import c_bool, c_float, c_int

# FUNCTION LIST
# runFourThruster
# runMainThruster
# runVision
# runServo
# runSerCommunication
# Protocol
# exitProcess

def Protocol(data, isRunning, isRelease, isRunningThruster): 
    global last_value, release_status, false_status, true_status
    right = 0
    left = 0

    if data == '2':
        right = 0
        false_status = 0
        true_status = 0

    else:
        if not false_status == 1:
            right = 1
            if data == '0':
                left = 0
                true_status = 1
                isRunning.value = 1
            else:
                if true_status == 0:
                    false_status = 1
                else:
                    left = 1
        else:
            isRelease.value = 1
            print("Please Reset The Right Switch...")

    if right == 0:
        isRunning.value = 0
        isRelease.value = 0
        isRunningThruster.value = 0
        release_status = 0

    else:
        if left == 1 and last_value == 0 or left == 1 and last_value == 1:
            isRelease.value = 1
            isRunningThruster.value = 1
        elif left == 0 and last_value == 1:
            isRelease.value = 1
            release_status = 1
            isRunningThruster.value = 0
        else:
            if release_status == 1:
                false_status = 1
                isRelease.value = 1
                isRunningThruster.value = 0
            # else:
            #     isRelease.value = 0
            #     isRunningThruster.value = 0

    last_value = left

    # left = 0
    # right = 0
    # if right == 0 and left == 0 -> standby program
    # if right == 1 and left == 0 and last_left == 0 -> running cv and with 4 thruster
    # if right == 1 and left == 1 and last_left == 0 -> running cv, servo, with 4 thruster, and runMainThruster
    # if right == 1 and left == 0 and last_left == 1 -> run missile and stop all component except 4 thruster and cv

def exitProcess(pName):
    print("Exiting {} process...".format(pName))
    sys.exit(0)

def runFourThruster(cnt, isRunning):
    last_value = 0
    max_speed = 15
    min_speed = 1
    try:
        while True:
            if isRunning.value == 1:
                if last_value != isRunning.value:
                    for speed in range (min_speed, max_speed+1):
                        cnt.staticThruster(speed)
                        sleep(0.1)
                else:
                    cnt.staticThruster(max_speed)
                    print("RUNNING STATIC 4 THRUSTER")
            else:
                if last_value != isRunning.value:
                    for speed in range(max_speed, -1, -1):
                        cnt.staticThruster(speed)
                        sleep(0.1)
                else:
                    cnt.staticThruster(0)
                    # print("Stopping 4 Thruster...")
            last_value = isRunning.value
            sleep(0.1)
    except KeyboardInterrupt:
        exitProcess('runFourThruster')

def runMainThruster(cnt, isRunning, isRunningThruster):
    last_value = 0
    max_speed = 12
    min_speed = 1
    try:
        while True:
            if isRunning.value == 1 and isRunningThruster.value == 1:
                if last_value != isRunningThruster.value :
                    for speed in range (min_speed, max_speed+1):
                        cnt.mainThruster(speed)
                        sleep(0.1)
                else:
                    cnt.mainThruster(max_speed)
                print("RUNNING MAIN THRUSTER")
            else:
                if last_value != isRunningThruster.value:
                    for speed in range(max_speed, -1, -1):
                        cnt.mainThruster(speed)
                        sleep(0.1)
                else:
                    cnt.mainThruster(0)
                # print("Stopping Main Thruster...")
            last_value = isRunningThruster.value
            sleep(0.1)
    except KeyboardInterrupt:
        exitProcess('runMainThruster')
    
def runMissile(port, isRelease):
    try:
        ser = serial.Serial(port, 9600, timeout=None)
        ser.reset_input_buffer()
    except Exception as exception:
        print("Cannot open serial port at {}".format(port))
        exitProcess('runMissile')
    try:
        while True:
            print("IS RELEASE VALUE,", isRelease.value)
            if isRelease.value == 1:
                ser.write('1\n'.encode('utf-8'))
                print("================================================================RELEASE TORPEDO")
            else: 
                print("================================================================LOCK TORPEDO")
            sleep(1)
    except KeyboardInterrupt: 
        print("Closing Serial Port... (/dev/ttyUSB0) at sending Data")
        ser.close()
        exitProcess('runMissile')

def runSerialCommunication(port, isRunning, isRelease, isRunningThruster):
    try:
        ser = serial.Serial(port, 9600, timeout=None)
        ser.reset_input_buffer()
    except Exception as exception:
        print("Cannot open serial port at {}".format(port))
        exitProcess('runSerialCommunication')
    try:
        while True:
                if ser.in_waiting > 0:
                    data = ser.readline().decode('utf-8').rstrip()
                    print("DATA RECEIVE: ", data)
                    Protocol(data, isRunning, isRelease, isRunningThruster)
                else: 
                    print("Initiation of serial communication or empty data...")
                sleep(0.1)
    except KeyboardInterrupt:
        print("Closing Serial Port... (/dev/ttyUSB0) at Receive Data")
        ser.close()
        exitProcess('runSerialCommunication')

def runServo(cnt, cx, isRunning, isRunningThruster):
    coord_x = 0
    try:
        while True:
            if isRunning.value == 1:
                coord_x = cx.value
                if coord_x == -1:
                    cnt.left()
                else:
                    cnt.dynamicServo(coord_x)
                # if coord_x >= -0.200 and coord_x <= 0.200:
                #     cnt.forward()
                #     print("FORWARD")
                # elif coord_x > 0.200:
                #     cnt.right()
                #     print("RIGHT")
                # else:
                #     cnt.left()            
                #     print("LEFT")
            else:
                cnt.forward()
                # print("Stopping Servo...")
            sleep(0.1)
    except KeyboardInterrupt:
        exitProcess("runServo")
 
def runVision(vision, cx, isRunning):
    offset_x = 0.75
    # vision.enable_vertical_limiter(-0.1, 0.4)
    # vision.enable_horizontal_limiter(-1, -0.2)
    vision.enable_vertical_limiter(0, 0.4)
    # vision.enable_horizontal_limiter(-1, -0.2)
    vision.enable_horizontal_limiter(0.2, 1)
    vision.enable_radius_limiter(0.1, 0.5)
    vision.enable_contours_mode()
    vision.visualize()
    vision.stabilize()
    vision.enable_averaging()
    try:
        while True:
            if isRunning.value == 1:
                vision.main_process()
                coord_x, coord_y, coord_z = vision.get_circle_coord()
                if coord_x < 0:
                    coord_x = -1
                else:
                    coord_x -= offset_x
                print("coord(x,y,z): " + str(vision.get_circle_coord()))
                cx.value = coord_x
                if not vision.show_image():
                    break 
            else:
                pass
                # print("Stopping Vision...")
            sleep(0.1)
    except KeyboardInterrupt:
        exitProcess("runVision")

if __name__ == "__main__":

    pca = ServoKit(channels=16)

    # MULTIPROCESS SHARED VARIABEL/VALUE
    isRunning = Value('i', 0)
    isRelease = Value('i', 0)
    isRunningThruster = Value('i', 0)
    coord_x = Value('f', 0.0)

    last_value = 0
    release_status = 0
    false_status = 0
    true_status = 0
    serial_port = '/dev/ttyUSB0'

    # I2C PIN CONFIGURATION
    leftFrontProp = pca.continuous_servo[0]
    leftBackProp = pca.continuous_servo[1]
    rightFrontProp = pca.continuous_servo[2]
    rightBackProp = pca.continuous_servo[3]
    mainProp = pca.continuous_servo[5]
    mainFin_servo = pca.servo[6]
    secondaryFin_servo = pca.servo[7]

    # LIBRARY OBJECT INSTANCE
    controller = cntrl.Controller(
        thrusterPinConfig=[leftFrontProp, leftBackProp, rightFrontProp, rightBackProp],
        mainThrusterPinConfig=mainProp,
        allServoPinConfig=[mainFin_servo, secondaryFin_servo],
    )
    try:
        vision = vision_lib.hsv_detector(
            # image_source = "Recorded video7.mp4",
            camera_height=240, camera_width=320, masking_enabled=False)
        controller.initMainThruster()

        runFourThruster_process = Process(target=runFourThruster ,args=(controller, isRunning, ))
        runMainThruster_process = Process(target=runMainThruster ,args=(controller, isRunning, isRunningThruster, ))
        runVision_process = Process(target=runVision, args=(vision, coord_x, isRunning, ))
        runServo_process = Process(target=runServo, args=(controller, coord_x, isRunning, isRunningThruster, ))
        runSerialCommunication_thread = Process(target=runSerialCommunication, args=(serial_port, isRunning, isRelease, isRunningThruster, ))
        runMissile_process = Process(target=runMissile, args=(serial_port, isRelease, ))
        
        runMainThruster_process.start()
        runFourThruster_process.start()
        runVision_process.start()
        runServo_process.start()
        runSerialCommunication_thread.start()
        runMissile_process.start()

        runMainThruster_process.join()
        runFourThruster_process.join()
        runVision_process.join()
        runServo_process.join()
        runSerialCommunication_thread.join()
        runMissile_process.join()

    except Exception as ex:
        print("CANNOT OPEN CAMERA OR SERIAL PORT!")
        print(ex)
        sys.exit(0)

    
    
