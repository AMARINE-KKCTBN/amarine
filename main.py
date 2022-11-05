from threading import Thread
from multiprocessing import Process, Value, Queue
from time import sleep
import RPi.GPIO as GPIO
from adafruit_servokit import ServoKit
from controller import controller as cntrl
from vision import vision_lib
import serial
from ctypes import c_bool, c_float, c_int

# runFourThruster
# runMainThruster
# runVision
# runServo
# runSerCommunication

def runFourThruster(cnt, isRunning):
    last_value = 0
    max_speed = 15
    min_speed = 1
    while True:
        if isRunning.value == 1:
            if last_value != isRunning.value:
                for speed in range (min_speed, max_speed+1):
                    cnt.staticThruster(speed)
                    sleep(0.1)
                print("DYNAMIC 1====================================")
            else:
                cnt.staticThruster(max_speed)
                print("RUNNING STATIC 4 THRUSTER")
        else:
            if last_value != isRunning.value:
                for speed in range(max_speed, -1, -1):
                    cnt.staticThruster(speed)
                    sleep(0.1)
                print("DYNAMIC 0===================================")
            else:
                print("STATIC 0")
                cnt.staticThruster(0)
                print("Stopping 4 Thruster...")
        last_value = isRunning.value
        sleep(0.1)

def runMainThruster(cnt, isRunning, isRunningThruster):
    last_value = 0
    max_speed = 10
    min_speed = 1
    while True:
        if isRunning.value == 1 and isRunningThruster.value == 1:
            if last_value != isRunning.value:
                for speed in range (min_speed, max_speed+1):
                    cnt.mainThruster(speed)
                    sleep(0.1)
            else:
                cnt.mainThruster(max_speed)
            print("RUNNING THRUSTER")
        else:
            if last_value != isRunning.value:
                for speed in range(max_speed, -1, -1):
                    cnt.mainThruster(speed)
                    sleep(0.1)
            else:
                cnt.mainThruster(0)
            print("Stopping Main Thruster...")
        last_value = isRunning.value
        sleep(0.1)

def runMissile(serial, isRelease, lock):
    while True:
        print("IS RELEASEEEEE========", isRelease.value)
        if isRelease.value == 1:
            lock.acquire()
            serial.write('1'.encode('utf-8'))
            lock.release()
            print("RELEASE TORPEDO")
        else: 
            ser.write('0'.encode('utf-8'))
            ser.flush()
            print("LOCK TORPEDO")
        ser.reset_output_buffer()
        sleep(0.1)

def Protocol(data, isRunning, isRelease, isRunningThruster):
    global last_value, release_status
    right = 0
    left = 0

    print("LAST_VALUE================================", last_value)

    if data == '2\r\n':
        right = 0
    else:
        right = 1
        if data == '0\r\n':
            left = 0
        else:
            left = 1
        print("DATA RECEIVE===============================", data, left)

    if right == 0:
        isRunning.value = 0
        isRelease.value = 0
        isRunningThruster.value = 0
        release_status = 0
    else:
        isRunning.value = 1
        if left == 0 and last_value == 0 and release_status == 0:
            isRelease.value = 0
            isRunningThruster.value = 0
        elif left == 1 and last_value == 0 or left == 1 and last_value == 1:
            isRelease.value = 0
            isRunningThruster.value = 1
        # elif left == 0 and last_value == 1:
        else:
            isRelease.value = 1
            release_status = 1
            isRunningThruster.value = 0

    last_value = left
    # left = 0
    # right = 0
    # if right == 0 and left == 0 -> standby program
    # if right == 1 and left == 0 and last_left == 0 -> running cv and servo or with 4 thruster
    # if right == 1 and left == 1 and last_left == 0 -> running cv and servo or with 4 thruster and runMainThruster
    # if right == 1 and left == 0 and last_left == 1 -> run missile and stop all component


def runSerialCommunication(isRunning, isRelease, isRunningThruster):
    ser = ser.Serial(port='/dev/ttyUSB0', baudrate=9600, timeout=None)
    ser.reset_input_buffer()
    ser.reset_output_buffer()
    while True:
        try:
            if ser.in_waiting > 0:
                data = ser.readline().decode('utf-8')
                print("DATA RECEIVE: ", data)
                Protocol(data, isRunning, isRelease, isRunningThruster)
            else: 
                print("Initiation of serial communication...")

        except Exception as exception:
            print("There's problem with serial communication!", exception)
        sleep(0.1)

def runServo(cnt, cx, isRunning):
    coord_x = 0
    while True:
        if isRunning.value == 1:
            coord_x = cx.value
            if coord_x >= -0.200 and coord_x <= 0.200:
                cnt.forward()
                print("FORWARD")
            elif coord_x > 0.200:
                cnt.right()
                print("RIGHT")
            else:
                cnt.left()            
                print("LEFT")
        else:
            cnt.forward()
            print("Stopping Servo...")
        sleep(0.5)

def runVision(cx, isRunning):
    offset_x = 0.5
    vision = vision_lib.hsv_detector(camera_height=240, camera_width=320, masking_enabled=False)
    vision.visualize()
    vision.record()
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
            # if not vision.show_image():
            #     break 
        else:
            print("Stopping Vision...")
        sleep(0.1)

if __name__ == "__main__":

    pca = ServoKit(channels=16)

    # MULTIPROCESS SHARED VARIABEL/VALUE
    isRunning = Value('i', 0)
    isRelease = Value('i', 0)
    isRunningThruster = Value('i', 0)
    coord_x = Value('f', 0.0)
    last_value = 0
    release_status = 0
    lock = Lock()

    thruster_run = 17
    main_thruster_speed = 10

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
        controller.initMainThruster()

        runFourThruster_process = Process(target=runFourThruster ,args=(controller, isRunning))
        runMainThruster_process = Process(target=runMainThruster ,args=(controller, isRunning, isRunningThruster))
        runVision_process = Process(target=runVision, args=(coord_x, isRunning))
        runServo_process = Process(target=runServo, args=(controller, coord_x, isRunning))
        runSerialCommunication_thread = Process(target=runSerialCommunication, args=(ser, isRunning, isRelease, isRunningThruster))
        runMissile_process = Process(target=runMissile, args=(ser, isRelease, lock))
        
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
        raise SystemExit

    
    
