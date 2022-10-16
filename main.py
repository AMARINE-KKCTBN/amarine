
import threading
from time import sleep
import RPi.GPIO as GPIO
from adafruit_servokit import ServoKit    
from controller import controller as cntrl

def ballastButton(cnt): 
    global ballast_status
    while True:
        if GPIO.input(ballast_button) == GPIO.HIGH:
            if ballast_status == 'empty':
                ballast_status = 'fill'
                cnt.forward_ballast()
            else:
                ballast_status = 'empty'
                cnt.backward_ballast()

def thrusterSpeedButton(): 
    global thruster_speed
    while True:
        if GPIO.input(thruster_speed_button) == GPIO.HIGH:
            print(thruster_speed)
            if thruster_speed >= 100:
                thruster_speed = 0
            else:
                thruster_speed += 10
            sleep(1.125)

def runThruster(cnt):
    global ballast_status, thruster_speed
    while True:
        print(ballast_status, thruster_speed)
        cnt.staticThruster(thruster_speed)

def runMissile(cnt):
    while True:
        # Change relay
        pass

if __name__ == "__main__":

    ballast_status = 'empty'
    thruster_speed = 0
    thruster_speed_button = 23
    ballast_button = 18

    GPIO.setup(thruster_speed_button, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
    GPIO.setup(ballast_button, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

    step_sleep = 0.002
    step_count = 2400 

    pca = ServoKit(channels=16)

    mainFin = pca.servo[0]
    secondaryFin = pca.servo[1]

    leftFrontProp = pca.continuous_servo[5]
    leftBackProp = pca.continuous_servo[6]
    rightFrontProp = pca.continuous_servo[7]
    rightBackProp = pca.continuous_servo[8]

    mainProp = pca.continuous_servo[4]

    controller = cntrl.Controller(
        stepperPinConfig=[11,22,33,44], 
        thrusterPinConfig=[leftBackProp, leftFrontProp, rightBackProp, rightFrontProp], 
        mainThrusterPinConfig=mainProp, 
        allServoPinConfig=[mainFin, secondaryFin]
        )

    controller.setStepperConfiguration(step_sleep, step_count)

    t1 = threading.Thread(target=ballastButton, args=(controller, ))
    t2 = threading.Thread(target=thrusterSpeedButton, args=( ))
    t3 = threading.Thread(target=runThruster, args=(controller, ))

	# starting thread 1
    t1.start()
    t2.start()
    t3.start()

	# # wait until thread 1 is completely executed
    # t1.join()
    # t2.join()
    # t3.join()


