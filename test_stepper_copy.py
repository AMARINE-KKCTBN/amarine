#######################################
# Copyright (c) 2021 Maker Portal LLC
# Author: Joshua Hrisko
#######################################
#
# NEMA 17 (17HS4023) Raspberry Pi Tests
# --- rotating the NEMA 17 clockwise
# --- and counterclockwise in a loop
#
#
#######################################
#
import RPi.GPIO as GPIO
from RpiMotorLib import RpiMotorLib
import time

################################
# RPi and Motor Pre-allocations
################################
#
#define GPIO pins
direction= 21 # Direction (DIR) GPIO Pin
step = 20 # Step GPIO Pin
EN_pin = 16 # enable pin (LOW to enable)

# Declare a instance of class pass GPIO pins numbers and the motor type
mymotortest = RpiMotorLib.A4988Nema(direction, step, (26,19,13), "A4988")
GPIO.setup(EN_pin,GPIO.OUT) # set enable pin as output

###########################
# Actual motor control
###########################
#
dir_array = [False,True]
# GPIO.output(EN_pin,GPIO.LOW) # pull enable to low to enable motor
while True:
    mymotortest.motor_go(False, # False=Clockwise, True=Counterclockwise
                         "Full" , # Step type (Full,Half,1/4,1/8,1/16,1/32)
                         200, # number of steps
                         0.01, # step delay [sec]
                         True, # True = print verbose output 
                         .05) # initial delay [sec]

    time.sleep(1)

GPIO.cleanup() # clear GPIO allocations after run