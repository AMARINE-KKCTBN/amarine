
import RPi.GPIO as GPIO
import json
from adafruit_servokit import ServoKit    #https://circuitpython.readthedocs.io/projects/servokit/en/latest/

GPIO.setmode(GPIO.BOARD)

pca = ServoKit(channels=16)

# PROPELLER CONSTRAINT
leftFrontProp = pca.continuous_servo[0]
leftBackProp = pca.continuous_servo[1]
rightFrontProp = pca.continuous_servo[2]
rightBackProp = pca.continuous_servo[3]
mainProp = pca.continuous_servo[4]

# SERVO CONSTRAINT
mainFin = pca.servo[5]
launcerGate = pca.servo[6]

# STEPPER CONSTRAINT
out1 = 16
out2 = 19
out3 = 20
out4 = 26

# SETTING UP STEPPER PINOUT
GPIO.setup( out1, GPIO.OUT )
GPIO.setup( out2, GPIO.OUT )
GPIO.setup( out3, GPIO.OUT )
GPIO.setup( out4, GPIO.OUT )

# INIT STEPPER PINOUT
GPIO.output( out1, GPIO.LOW )
GPIO.output( out2, GPIO.LOW )
GPIO.output( out3, GPIO.LOW )
GPIO.output( out4, GPIO.LOW )


if __name__ == "__main__":
    print("SS")
