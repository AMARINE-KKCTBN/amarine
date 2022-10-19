#!/usr/bin/python3
import RPi.GPIO as GPIO
import time

# out1 = 16
# out2 = 19
# out3 = 20
# out4 = 26

out1 = 17
out2 = 18
out3 = 27
out4 = 22

# careful lowering this, at some point you run into the mechanical limitation of how quick your motor can move
step_sleep = 0.002

# step_count = 2400  # 200 = 1 step/180
step_count = 2300  # 200 = 1 step/180

# setting up
GPIO.setmode(GPIO.BCM)
GPIO.setup(out1, GPIO.OUT)
GPIO.setup(out2, GPIO.OUT)
GPIO.setup(out3, GPIO.OUT)
GPIO.setup(out4, GPIO.OUT)  

# initializing
GPIO.output(out1, GPIO.LOW)
GPIO.output(out2, GPIO.LOW)
GPIO.output(out3, GPIO.LOW)
GPIO.output(out4, GPIO.LOW)


def cleanup():
    GPIO.output(out1, GPIO.LOW)
    GPIO.output(out2, GPIO.LOW)
    GPIO.output(out3, GPIO.LOW)
    GPIO.output(out4, GPIO.LOW)
    GPIO.cleanup()


def forward(step_):
    i = step_
    for i in range(step_count):
        if i % 4 == 0:
            GPIO.output(out4, GPIO.HIGH)
            GPIO.output(out3, GPIO.LOW)
            GPIO.output(out2, GPIO.LOW)
            GPIO.output(out1, GPIO.LOW)
        elif i % 4 == 1:
            GPIO.output(out4, GPIO.LOW)
            GPIO.output(out3, GPIO.LOW)
            GPIO.output(out2, GPIO.HIGH)
            GPIO.output(out1, GPIO.LOW)
        elif i % 4 == 2:
            GPIO.output(out4, GPIO.LOW)
            GPIO.output(out3, GPIO.HIGH)
            GPIO.output(out2, GPIO.LOW)
            GPIO.output(out1, GPIO.LOW)
        elif i % 4 == 3:
            GPIO.output(out4, GPIO.LOW)
            GPIO.output(out3, GPIO.LOW)
            GPIO.output(out2, GPIO.LOW)
            GPIO.output(out1, GPIO.HIGH)
        time.sleep(step_sleep)


def backward(step_):
    # i = step_
    for i in range(step_count, 0, -1):
        if i % 4 == 0:
            GPIO.output(out4, GPIO.HIGH)
            GPIO.output(out3, GPIO.LOW)
            GPIO.output(out2, GPIO.LOW)
            GPIO.output(out1, GPIO.LOW)
        elif i % 4 == 1:
            GPIO.output(out4, GPIO.LOW)
            GPIO.output(out3, GPIO.LOW)
            GPIO.output(out2, GPIO.HIGH)
            GPIO.output(out1, GPIO.LOW)
        elif i % 4 == 2:
            GPIO.output(out4, GPIO.LOW)
            GPIO.output(out3, GPIO.HIGH)
            GPIO.output(out2, GPIO.LOW)
            GPIO.output(out1, GPIO.LOW)
        elif i % 4 == 3:
            GPIO.output(out4, GPIO.LOW)
            GPIO.output(out3, GPIO.LOW)
            GPIO.output(out2, GPIO.LOW)
            GPIO.output(out1, GPIO.HIGH)
        time.sleep(step_sleep)


# the meat
try:
    forward(step_count)
    backward(step_count)
    # i = 0
    # for i in range(step_count):
    #     if i%4==0:
    #         GPIO.output( out4, GPIO.HIGH )
    #         GPIO.output( out3, GPIO.LOW )
    #         GPIO.output( out2, GPIO.LOW )
    #         GPIO.output( out1, GPIO.LOW )
    #     elif i%4==1:
    #         GPIO.output( out4, GPIO.LOW )
    #         GPIO.output( out3, GPIO.LOW )
    #         GPIO.output( out2, GPIO.HIGH )
    #         GPIO.output( out1, GPIO.LOW )
    #     elif i%4==2:
    #         GPIO.output( out4, GPIO.LOW )
    #         GPIO.output( out3, GPIO.HIGH )
    #         GPIO.output( out2, GPIO.LOW )
    #         GPIO.output( out1, GPIO.LOW )
    #     elif i%4==3:
    #         GPIO.output( out4, GPIO.LOW )
    #         GPIO.output( out3, GPIO.LOW )
    #         GPIO.output( out2, GPIO.LOW )
    #         GPIO.output( out1, GPIO.HIGH )

    # time.sleep( step_sleep )

except KeyboardInterrupt:
    cleanup()
    exit(1)

cleanup()
exit(0)
