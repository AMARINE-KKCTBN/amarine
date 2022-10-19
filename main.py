# /etc/init.d/main.py
### BEGIN INIT INFO
# Provides:          sample.py
# Required-Start:    $remote_fs $syslog
# Required-Stop:     $remote_fs $syslog
# Default-Start:     2 3 4 5
# Default-Stop:      0 1 6
# Short-Description: Start daemon at boot time
# Description:       Enable service provided by daemon.
### END INIT INFO

import threading
from time import sleep
import RPi.GPIO as GPIO
from adafruit_servokit import ServoKit
from controller import controller as cntrl



def ballastButton(cnt):
    global ballast_status, ballast_button
    while enabled_:
        if GPIO.input(ballast_button) == GPIO.LOW:
            if ballast_status == "empty":
                ballast_status = "fill"
                cnt.forward_ballast()
            else:
                ballast_status = "empty"
                cnt.backward_ballast()
            cnt.cleanup()



def thrusterSpeedButton():
    global thruster_speed, thruster_speed_button
    while enabled_:
        if GPIO.input(thruster_speed_button) == GPIO.LOW:
            print("ACTIVE")
            if thruster_speed >= 40:
                thruster_speed = 0
            else:
                thruster_speed += 10
            sleep(0.5)


def runThruster(cnt):
    global ballast_status, thruster_speed
    while enabled_:
        cnt.staticThruster(thruster_speed)


def runMissile():
    global relay_release, missile_button
    while enabled_:
        if GPIO.input(missile_button) == GPIO.LOW:
            GPIO.output(relay_release, GPIO.HIGH)


def isRunning():
    global led
    while enabled_:
        GPIO.output(led, GPIO.HIGH)
        sleep(1)
        print("NYALA")
        GPIO.output(led, GPIO.LOW)
        sleep(1)
        print("MATI")


if __name__ == "__main__":

    enabled_ = True

    ballast_status = "empty"
    thruster_speed = 0

    # GPIO.setmode(GPIO.BCM)

    pca = ServoKit(channels=16)

    thruster_speed_button = 23
    ballast_button = 15
    missile_button = 24
    relay_release = 25
    led = 21

    GPIO.setup(thruster_speed_button, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(ballast_button, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(missile_button, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(relay_release, GPIO.OUT)
    GPIO.setup(led, GPIO.OUT)

    step_sleep = 0.00175
    step_count = 1200

    mainFin = pca.servo[10]
    secondaryFin = pca.servo[11]

    leftFrontProp = pca.continuous_servo[0]
    leftBackProp = pca.continuous_servo[1]
    rightFrontProp = pca.continuous_servo[2]
    rightBackProp = pca.continuous_servo[3]

    mainProp = pca.continuous_servo[4]

    controller = cntrl.Controller(
        stepperPinConfig=[6, 13, 19, 26],
        thrusterPinConfig=[leftBackProp, leftFrontProp, rightBackProp, rightFrontProp],
        mainThrusterPinConfig=mainProp,
        allServoPinConfig=[mainFin, secondaryFin],
    )

    controller.setStepperConfiguration(step_sleep, step_count)
    # controller.backward_ballast()
    # controller.cleanup()

    t1 = threading.Thread(target=ballastButton, args=(controller,))
    # t2 = threading.Thread(target=thrusterSpeedButton, args=())
    # t3 = threading.Thread(target=runThruster, args=(controller,))
    # # t4 = threading.Thread(target=runMissile, args=())
    # # t5 = threading.Thread(target=isRunning, args=())

    # # starting thread 1
    t1.start()
    # t2.start()
    # t3.start()
    # t4.start()
    # t5.start()

    # wait until thread 1 is completely executed
    # t1.join()
    # t2.join()
    # t3.join()
    # t4.join()
