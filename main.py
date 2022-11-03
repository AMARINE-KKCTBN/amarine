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
                print("STATIC 1")
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

        print("PASSED VALUE", isRunning.value)
        last_value = isRunning.value
        sleep(0.5)

def runMainThruster(cnt, isRunning):
    last_value = 0
    max_speed = 10
    min_speed = 1
    while True:
        if isRunning.value == 1:
            if last_value != isRunning.value:
                for speed in range (min_speed, max_speed+1):
                    cnt.mainThruster(speed)
                    sleep(0.1)
            else:
                cnt.mainThruster(max_speed)
        else:
            if last_value != isRunning.value:
                for speed in range(max_speed, -1, -1):
                    cnt.mainThruster(speed)
                    sleep(0.1)
            else:
                cnt.mainThruster(0)
                print("Stopping Main Thruster...")
        print("PASSED VALUE", isRunning.value)
        last_value = isRunning.value
        sleep(0.5)

def runSerialCommunication(serial, isRunning):
    while True:
        try:
            if serial.in_waiting > 0:
                data = serial.readline().decode('utf-8')
                print("DATA RECEIVE: ", data)
                if data == "0\r\n":
                    isRunning.value = 1
                    print("RUNNING THRUSTER")
                else:
                    isRunning.value = 0                    
                    print("STOP THRUSTER")
            else: 
                print("Initiation of serial communication...")

        except Exception as exception:
            print("There's problem with serial communication!", exception)
        sleep(0.5)

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

def runVision(vision, cx, isRunning):
    offset_x = 0.5
    while True:
        if isRunning.value == 1:
            vision.detect_circle_object()
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
        sleep(0.5)

if __name__ == "__main__":

    pca = ServoKit(channels=16)

    # MULTIPROCESS SHARED VARIABEL/VALUE
    isRunning = Value('i', 0)
    coord_x = Value('f', 0.0)

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
        vision = vision_lib.hsv_detector(camera_height=240, camera_width=320, masking_enabled=False)
        ser = serial.Serial(port='/dev/ttyUSB0', baudrate=9600, timeout=5)
        ser.reset_input_buffer()

        controller.initMainThruster()

        runFourThruster_process = Process(target=runFourThruster ,args=(controller, isRunning))
        runMainThruster_process = Process(target=runMainThruster ,args=(controller, isRunning))
        runVision_process = Process(target=runVision, args=(controller, coord_x, isRunning))
        runServo_process = Process(target=runServo, args=(controller, coord_x, isRunning))
        runSerialCommunication_thread = Thread(target=runSerialCommunication, args=(serial, isRunning))
        
        runMainThruster_process.start()
        runFourThruster_process.start()
        runVision_process.start()
        runServo_process.start()
        runSerialCommunication_thread.start()

        runMainThruster_process.join()
        runFourThruster_process.join()
        runVision_process.join()
        runServo_process.join()
        runSerialCommunication_thread.join()

        # runMainThruster_process = threading.Thread(target=runMainThruster, args=(controller, ser, isRunning))
        # thrusterRun_process.start()
        # runMainThruster_process.start()
        # thrusterRun_process.join()
        # runMainThruster_process.join()

        # MULTIPROCESS=========
        # MAIN PROCESS

        # shutdown_program_thread = multiprocessing.Process(target=shutdownProgram, args=(isRunning,))
        # buttonThruster_thread = multiprocessing.Process(target=thrusterSpeedButton, args=(thruster_speed, isRunning, ))
        # mainFin_servo_thread = multiprocessing.Process(target=servoRunning, args=(controller, coord_x, isRunning))
        
        # vision_process = multiprocessing.Process(target=objectDetection, args=(vision, coord_x, isRunning))
        
        # runMainThruster_thread = threading.Thread(target=runMainThruster, args=(controller, ser, isRunning))
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

        # MULTIPROCESS=========

        # RUN THREAD & PROCESS
        # vision_process.start()
        # runThruster_thread.start()
        # runMainThruster_thread.start()
        # buttonThruster_thread.start()
        # mainFin_servo_thread.start()
        # shutdown_program_thread.start()
        
        # vision_process.join()
        # runMainThruster_thread.join()
        # runThruster_thread.join()
        # buttonThruster_thread.join()
        # shutdown_program_thread.join()
        # mainFin_servo_thread.join()

    except Exception as ex:
        print("CANNOT OPEN CAMERA OR SERIAL PORT!")
        print(ex)
        ser.close()
        raise SystemExit

    
    
