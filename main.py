import threading
import multiprocessing
from time import sleep
import RPi.GPIO as GPIO
from adafruit_servokit import ServoKit
from controller import controller as cntrl
from vision import vision_lib
import serial

def thrusterSpeedButton():
    global thruster_speed, thruster_speed_button
    while True:
        if GPIO.input(thruster_speed_button) == GPIO.LOW:
            if thruster_speed >= 30:
                thruster_speed = 0
            else:
                last_thruster = thruster_speed
                for i in range (1,11):
                    thruster_speed = last_thruster + i
                    sleep(0.1)
            sleep(1.5)

def runThruster(cnt):
    global thruster_speed
    while True:
        cnt.staticThruster(thruster_speed)

def runMainThruster(cnt, serial):
    global thruster_run, main_thruster_speed
    while True:
        if serial.in_waiting > 0:
            data = serial.readline().decode('utf-8')
            print("DATA: ", data)
            if data == "0\r\n":
                print("RUNNING THRUSTER")
                cnt.mainThruster(main_thruster_speed)
            else:
                print("STOP THRUSTER")
                cnt.mainThruster(0)
            sleep(0.1)

def runMissile():
    global relay_release, missile_button, missile_status
    while True:
        if GPIO.input(missile_button) == GPIO.LOW:
            # false
            if not missile_status: # true
                print("RUN MISSILE")
                GPIO.output(relay_release, GPIO.LOW)
            else:
                GPIO.output(relay_release, GPIO.HIGH)
            missile_status = not missile_status # true

def isRunning():
    global led
    while True:
        GPIO.output(led, GPIO.HIGH)
        sleep(1)
        print("NYALA")
        GPIO.output(led, GPIO.LOW)
        sleep(1)
        print("MATI")

def servoRunning(cnt, val):
    coord_x = 0
    while True:
        coord_x = val.value
        if coord_x >= -0.200 and coord_x <= 0.200:
            cnt.forward()
            # cnt.mainThruster()
            print("FORWARD")
        elif coord_x > 0.200:
            cnt.right()
            # cnt.mainThruster()
            print("RIGHT")
        else:
            cnt.left()            
            print("LEFT")
        sleep(0.1)

def objectDetection(vision, val):
    offset_x = 0.5
    while True:
        vision.detect_circle_object()
        coord_x, coord_y, coord_z = vision.get_circle_coord()
        if coord_x < 0:
            coord_x = -1
        else:
            coord_x -= offset_x
        print("coord: " + str(vision.get_circle_coord()))
        val.value = coord_x
        sleep(0.1)
        # if not vision.show_image():
        #     break 

def shutdownProgram():
    global enabled_, shutdown_program_button
    while True:
        if GPIO.input(shutdown_program_button) == GPIO.LOW:
            enabled_ = not enabled_

if __name__ == "__main__":

    enabled_ = True
    pca = ServoKit(channels=16)

    # INIT VARIABLE
    coord_x = multiprocessing.Value('f', 0.0)
    thruster_speed = 10
    missile_status = False
    thruster_run = 17
    main_thruster_speed = 10
    
    # RASPI PIN CONFIGURATION
    thruster_speed_button = 16
    shutdown_program_button = 20
    missile_button = 24
    relay_release = 25
    led = 5

    # ser = serial.Serial(
    # port='/dev/ttyS0', # Change this according to connection methods, e.g. /dev/ttyUSB0
    # baudrate = 115200,
    # parity=serial.PARITY_NONE,
    # stopbits=serial.STOPBITS_ONE,
    # bytesize=serial.EIGHTBITS,
    # timeout=1
    # )

    # PIN INITIATION
    GPIO.setup(thruster_run, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
    GPIO.setup(thruster_speed_button, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(missile_button, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(relay_release, GPIO.OUT)
    GPIO.setup(led, GPIO.OUT)

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
    vision = vision_lib.hsv_detector(camera_height=240, camera_width=320, masking_enabled=False)
    serial = serial.Serial(port='/dev/ttyUSB0', baudrate=9600, timeout=1)
    serial.reset_input_buffer()

    controller.initMainThruster()

    # MULTIPROCESS=========
    # MAIN PROCESS
    runThruster_thread = threading.Thread(target=runThruster, args=(controller,))
    runMainThruster_thread = multiprocessing.Process(target=runMainThruster, args=(controller, serial))
    buttonThruster_thread = threading.Thread(target=thrusterSpeedButton, args=( ))
    runMissile_thread = threading.Thread(target=runMissile, args=())
    mainFin_servo_thread = threading.Thread(target=servoRunning, args=(controller, coord_x))

    # VISION PROCESS
    vision_process = multiprocessing.Process(target=objectDetection, args=(vision, coord_x))

    # MULTIPROCESS=========

    # RUN THREAD & PROCESS
    vision_process.start()
    runThruster_thread.start()
    runMainThruster_thread.start()
    buttonThruster_thread.start()
    runMissile_thread.start()
    mainFin_servo_thread.start()
    
    vision_process.join()
    runMainThruster_thread.join()
    runThruster_thread.join()
    buttonThruster_thread.join()
    runMissile_thread.join()
    mainFin_servo_thread.join()

    
