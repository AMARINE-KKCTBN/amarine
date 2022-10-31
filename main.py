import threading
import multiprocessing
from time import sleep
import RPi.GPIO as GPIO
from adafruit_servokit import ServoKit
from controller import controller as cntrl
from vision import vision_lib

def thrusterSpeedButton():
    global thruster_speed, thruster_speed_button
    while enabled_:
        if GPIO.input(thruster_speed_button) == GPIO.LOW:
            if thruster_speed >= 50:
                thruster_speed = 0
            else:
                last_thruster = thruster_speed
                for i in range (1,11):
                    thruster_speed = last_thruster + i
                    sleep(0.1)
            sleep(1.5)

def runThruster(cnt):
    global ballast_status, thruster_speed
    while enabled_:
        cnt.staticThruster(thruster_speed)

def runMissile():
    global relay_release, missile_button, missile_status
    while enabled_:
        if GPIO.input(missile_button) == GPIO.LOW:
            # false
            if not missile_status: # true
                # print("RUN MISSILE")
                GPIO.output(relay_release, GPIO.LOW)
            else:
                GPIO.output(relay_release, GPIO.HIGH)
            missile_status = not missile_status # true

def isRunning():
    global led
    while enabled_:
        GPIO.output(led, GPIO.HIGH)
        sleep(1)
        print("NYALA")
        GPIO.output(led, GPIO.LOW)
        sleep(1)
        print("MATI")

def servoRunning(cnt):
    global coord_x
    while enabled_:
        if coord_x > 0:
            cnt.left()
            print("LEFT")
        elif coord_x == 0:
            cnt.forward()
            print("FORWARD")
        else:
            print("RIGHT")
            cnt.right()
        sleep(0.1)

def objectDetection(vision):
    global coord_x
    offset_x = 0.5
    # vision = vision_lib.hsv_detector(camera_height=240, camera_width=320, masking_enabled=False)
    while enabled_:
        vision.detect_circle_object()
        coord_x, coord_y, coord_z = vision.get_circle_coord()
        if coord_x < 0:
            coord_x = -1
        else:
            coord_x -= offset_x
        print("coord: " + str(vision.get_circle_coord()))
        # if not vision.show_image():
        #     break 

def runMainThruster(cnt):
    global main_thruster_speed
    while enabled_:
        print("RUNNING MAIN THRUSTER ON ", main_thruster_speed)
        cnt.mainThruster()

def shutdownProgram():
    global enabled_, shutdown_program_button
    while True:
        if GPIO.input(shutdown_program_button) == GPIO.LOW:
            enabled_ = not enabled_

if __name__ == "__main__":

    enabled_ = True
    coord_x = 0

    thruster_speed = 0
    missile_status = False

    pca = ServoKit(channels=16)

    main_thruster_speed = 10
    thruster_speed_button = 16
    shutdown_program_button = 20
    missile_button = 24
    relay_release = 25
    led = 5

    GPIO.setup(thruster_speed_button, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(missile_button, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(relay_release, GPIO.OUT)
    GPIO.setup(led, GPIO.OUT)

    mainFin = pca.servo[8]
    secondaryFin = pca.servo[9]

    leftFrontProp = pca.continuous_servo[0]
    leftBackProp = pca.continuous_servo[1]
    rightFrontProp = pca.continuous_servo[2]
    rightBackProp = pca.continuous_servo[3]

    mainProp = pca.continuous_servo[4]

    controller = cntrl.Controller(
        thrusterPinConfig=[leftBackProp, leftFrontProp, rightBackProp, rightFrontProp],
        mainThrusterPinConfig=mainProp,
        allServoPinConfig=[mainFin, secondaryFin],
        mainThrusterSpeed=main_thruster_speed
    )

    vision = vision_lib.hsv_detector(camera_height=240, camera_width=320, masking_enabled=False)
    
    vision_thread = threading.Thread(target=objectDetection, args=(vision, ))
    runThruster_thread = threading.Thread(target=runThruster, args=(controller,))
    buttonThruster_thread = threading.Thread(target=thrusterSpeedButton, args=( ))
    runMissile_thread = threading.Thread(target=runMissile, args=())

    # RUN THREAD
    vision_thread.start()
    runThruster_thread.start()
    buttonThruster_thread.start()
    runMissile_thread.start()
    
    vision_thread.join()
    runThruster_thread.join()
    buttonThruster_thread.join()
    runMissile_thread.join()

    
    # t5 = threading.Thread(target=runMainThruster, args=(controller,))

    # thread_vision = threading.Thread(target=objectDetection, args=( ))
    
    # t1 = threading.Thread(target=ballastButton, args=(controller, lock))

    # p1 = multiprocessing.Process(target=ballastButton, args=(controller, lock))
    # p2 = multiprocessing.Process(target=thrusterSpeedButton, args=())
    # p3 = multiprocessing.Process(target=runThruster, args=(controller,))
    # p4 = multiprocessing.Process(target=runMissile, args=())
    # p5 = multiprocessing.Process(target=isRunning, args=())
    # p6 = multiprocessing.Process(target=servoRunning, args=(controller,))
    
    
    # t4 = threading.Thread(target=runMissile, args=())
    # t5 = threading.Thread(target=isRunning, args=())

    # # starting thread 1
    # thread_vision.start()
    # p1.start()
    # p2.start()
    # p3.start()
    # p4.start()
    # p5.start()
    # p6.start()
    # t2.start()
    # t3.start()

    
    # t4.start()
    # t5.start()

    # wait until thread 1 is completely executed
    # thread_vision.join()
    # p1.join()
    # # p2.join()
    # # p3.join()
    # p4.join()
    # p5.join()
    # t2.join()
    # t3.join()
    # # t4.join()
    # t5.join()
