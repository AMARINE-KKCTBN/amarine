
import threading
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

def runThruster(cnt, percentage):
    global ballast_status
    while True:
        print(ballast_status)
        cnt.staticThruster(percentage)


if __name__ == "__main__":

    ballast_status = 'empty'
    ballast_button = 18
    GPIO.setup(ballast_button, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

    step_sleep = 0.002

    step_count = 2400 

    percentage = 80 

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
    t2 = threading.Thread(target=runThruster, args=(controller, percentage))

	# starting thread 1
    t1.start()
    t2.start()

    # print(t1.is_alive())
    # print(t2.is_alive())


	# # wait until thread 1 is completely executed
    # t1.join()
    # t2.join()


