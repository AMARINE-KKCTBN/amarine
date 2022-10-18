
import threading
from time import sleep
import RPi.GPIO as GPIO
from adafruit_servokit import ServoKit    
from controller import controller as cntrl

enabled_ = True

def ballastButton(cnt): 
    global ballast_status
    while enabled_:
        if GPIO.input(ballast_button) == GPIO.HIGH:
            if ballast_status == 'empty':
                ballast_status = 'fill'
                print("Filling...")
                cnt.forward_ballast()
            else:
                ballast_status = 'empty'
                print("Filling...")
                cnt.backward_ballast()

def thrusterSpeedButton(): 
    global thruster_speed
    while enabled_:
        if GPIO.input(thruster_speed_button) == GPIO.HIGH:
            print(thruster_speed)
            if thruster_speed >= 100:
                thruster_speed = 0
            else:
                thruster_speed += 10
            # print("Speed: " + str(thruster_speed))
            sleep(1.125)

def runThruster(cnt):
    global ballast_status, thruster_speed
    while enabled_:
        print("status: ", ballast_status, "speed: ", thruster_speed)
        cnt.staticThruster(thruster_speed)

def runMissile():
    global relay_release, missile_button
    while enabled_:
        if GPIO.input(missile_button) == GPIO.HIGH:
            print("releasing...")
            GPIO.output(relay_release, GPIO.HIGH)

            
if __name__ == "__main__":

    ballast_status = 'empty'
    thruster_speed = 0
    
    pca = ServoKit(channels=16)

    thruster_speed_button = 23
    ballast_button = 18
    missile_button = 24
    relay_release = 25

    GPIO.setup(thruster_speed_button, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
    GPIO.setup(ballast_button, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
    GPIO.setup(missile_button, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
    GPIO.setup(relay_release, GPIO.OUT)

    step_sleep = 0.002
    step_count = 2400 

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
    t4 = threading.Thread(target=runMissile, args=( ))

	# starting thread 1
    t1.start()
    t2.start()
    t3.start()
    t4.start()

    while enabled_:
        for i in range(0,100):
            i+=1
        print("running")

	# # wait until thread 1 is completely executed
    # t1.join()
    # t2.join()
    # t3.join()
    # t4.join()