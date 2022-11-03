import threading
import multiprocessing
from time import sleep
import RPi.GPIO as GPIO
from adafruit_servokit import ServoKit
from controller import controller as cntrl
from vision import vision_lib
import serial
from ctypes import c_bool, c_float, c_int
# from concurrent.futures import ThreadPoolExecutor

def thrusterSpeedButton(thruster_speed, isRunning):
    global thruster_speed_button
    while True:
        if isRunning.value:
            if GPIO.input(thruster_speed_button) == GPIO.LOW:
                if thruster_speed.value >= 30:
                    thruster_speed.value = 0
                    sleep(0.99)
                else:
                    last_thruster = thruster_speed.value
                    for i in range (1,11):
                        thruster_speed.value = last_thruster + i
                        sleep(0.1)
                sleep(0.25)
        else:
            thruster_speed.value = 0
            continue

def runThruster(cnt, isRunning, thruster_speed):
    while True:
        if isRunning.value:
            cnt.staticThruster(thruster_speed.value)
        else:
            continue

def runMainThruster(cnt, serial, isRunning):
    global main_thruster_speed
    while True:
        if isRunning.value:
            if serial.in_waiting > 0:
                data = serial.readline().decode('utf-8')
                print("DATA RECEIVE: ", data)
                if data == "0\r\n":
                    print("RUNNING THRUSTER")
                    # cnt.mainThruster(main_thruster_speed)
                    cnt.mainThruster(10)
                else:
                    print("STOP THRUSTER")
                    cnt.mainThruster(0)
                sleep(0.125)
            else: 
                # serial.open()
                continue
        else:
            # serial.close()
            cnt.mainThruster(0)
            continue

def servoRunning(cnt, val, isRunning):
    coord_x = 0
    while True:
        if isRunning.value:
            coord_x = val.value
            if coord_x >= -0.200 and coord_x <= 0.200:
                cnt.forward()
                print("FORWARD")
            elif coord_x > 0.200:
                cnt.right()
                print("RIGHT")
            else:
                cnt.left()            
                print("LEFT")
            sleep(0.1)
        else:
            cnt.forward()
            continue

def objectDetection(vision, val, isRunning):
    offset_x = 0.5
    while True:
        if isRunning.value:
            vision.detect_circle_object()
            coord_x, coord_y, coord_z = vision.get_circle_coord()
            if coord_x < 0:
                coord_x = -1
            else:
                coord_x -= offset_x
            print("coord(x,y,z): " + str(vision.get_circle_coord()))
            val.value = coord_x
            sleep(0.1)
            # if not vision.show_image():
            #     break 
        else:
            continue

def shutdownProgram(isRunning):
    global shutdown_program_button, led
    while True:
        if GPIO.input(shutdown_program_button) == GPIO.LOW:
            isRunning.value = not isRunning.value
        else:
            if isRunning.value:
                GPIO.output(led, GPIO.HIGH)
                sleep(0.1)
                GPIO.output(led, GPIO.LOW)
                sleep(0.1)
            else:
                GPIO.output(led, GPIO.LOW)   

if __name__ == "__main__":

    pca = ServoKit(channels=16)

    # INIT VARIABLE
    isRunning = multiprocessing.Value(c_bool, False)
    coord_x = multiprocessing.Value(c_float, 0.0)
    thruster_speed = multiprocessing.Value(c_int, 0)
    # missile_status = False
    thruster_run = 17
    main_thruster_speed = 10
    
    # RASPI PIN CONFIGURATION
    thruster_speed_button = 16
    shutdown_program_button = 20
    # missile_button = 24
    # relay_release = 25
    led = 5

    # PIN INITIATION
    # GPIO.setup(thruster_run, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
    GPIO.setup(thruster_speed_button, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(shutdown_program_button, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(led, GPIO.OUT)
    GPIO.output(led, GPIO.LOW)
    # GPIO.setup(relay_release, GPIO.OUT)
    # GPIO.setup(led, GPIO.OUT)

    # I2C PIN CONFIGURATION
    leftFrontProp = pca.continuous_servo[0]
    leftBackProp = pca.continuous_servo[1]
    rightFrontProp = pca.continuous_servo[2]
    rightBackProp = pca.continuous_servo[3]
    mainProp = pca.continuous_servo[4]
    mainFin_servo = pca.servo[5]
    secondaryFin_servo = pca.servo[6]

    # LIBRARY OBJECT INSTANCE
    controller = cntrl.Controller(
        thrusterPinConfig=[leftFrontProp, leftBackProp, rightFrontProp, rightBackProp],
        mainThrusterPinConfig=mainProp,
        allServoPinConfig=[mainFin_servo, secondaryFin_servo],
    )
    try:
        vision = vision_lib.hsv_detector(camera_height=240, camera_width=320, masking_enabled=False)
        ser = serial.Serial(port='/dev/ttyUSB0', baudrate=9600, timeout=1)
        ser.reset_input_buffer()
        controller.initMainThruster()

        # MULTIPROCESS=========
        # MAIN PROCESS

        shutdown_program_thread = multiprocessing.Process(target=shutdownProgram, args=(isRunning,))
        runThruster_thread = multiprocessing.Process(target=runThruster, args=(controller, isRunning, thruster_speed))
        buttonThruster_thread = multiprocessing.Process(target=thrusterSpeedButton, args=(thruster_speed, isRunning, ))
        mainFin_servo_thread = multiprocessing.Process(target=servoRunning, args=(controller, coord_x, isRunning))
        runMainThruster_thread = threading.Thread(target=runMainThruster, args=(controller, ser, isRunning))
        # # runMissile_thread = threading.Thread(target=runMissile, args=())
        # def threadPool():
        #     threads = [runThruster_thread, buttonThruster_thread, mainFin_servo_thread]
        #     for thread in threads:
        #         thread.start()
        #     for thread in threads:
        #         thread.join()

        # def runMainThrusterThread():
        #     global ser, controller
        #     # runMainThruster_thread = multiprocessing.Process(target=runMainThruster, args=(controller, serial))
        #     runMainThruster_thread.start()
        #     runMainThruster_thread.join()

        # with multiprocessing.Pool() as multiprocess_executor:
        #     multiprocess_executor.apply(threadPool)
        #     multiprocess_executor.apply(runMainThruster, controller, ser)
            # multiprocess_executor.apply(objectDetection, vision, coord_x)
            # executor.submit(runThruster, controller)
            # executor.submit(runMainThruster, controller, ser)
            # executor.submit(thrusterSpeedButton)
            # executor.submit(servoRunning, controller, coord_x)

        # threads.append()

        # VISION PROCESS
        vision_process = multiprocessing.Process(target=objectDetection, args=(vision, coord_x, isRunning))

        # MULTIPROCESS=========

        # RUN THREAD & PROCESS
        vision_process.start()
        runThruster_thread.start()
        runMainThruster_thread.start()
        buttonThruster_thread.start()
        mainFin_servo_thread.start()
        shutdown_program_thread.start()
        
        vision_process.join()
        runMainThruster_thread.join()
        runThruster_thread.join()
        buttonThruster_thread.join()
        shutdown_program_thread.join()
        mainFin_servo_thread.join()

    except Exception as ex:
        print("CANNOT OPEN CAMERA OR SERIAL PORT!")
        print(ex)

    
    
