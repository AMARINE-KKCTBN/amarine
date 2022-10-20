import RPi.GPIO as GPIO
from time import sleep
from adafruit_servokit import (
    ServoKit,
)  

class Controller:
    def __init__(
        self,
        stepperPinConfig,
        thrusterPinConfig,
        mainThrusterPinConfig,
        allServoPinConfig,
    ) -> None:
        self.stepperPinConfig = stepperPinConfig
        self.thrusterPinConfig = thrusterPinConfig
        self.mainThrusterPinConfig = mainThrusterPinConfig
        self.allServoPinConfig = allServoPinConfig
        
    def setStepperConfiguration(self, step_sleep, step_count):
        self.step_count = step_count
        self.step_sleep = step_sleep

        # Set stepper pinout configuration
        GPIO.setup(self.stepperPinConfig[0], GPIO.OUT)
        GPIO.setup(self.stepperPinConfig[1], GPIO.OUT)
        GPIO.setup(self.stepperPinConfig[2], GPIO.OUT)
        GPIO.setup(self.stepperPinConfig[3], GPIO.OUT)

        # Init stepper pinout
        GPIO.setup(self.stepperPinConfig[0], GPIO.LOW)
        GPIO.setup(self.stepperPinConfig[1], GPIO.LOW)
        GPIO.setup(self.stepperPinConfig[2], GPIO.LOW)
        GPIO.setup(self.stepperPinConfig[3], GPIO.LOW)

    def forward_ballast(self):
        i = 0
        for i in range(self.step_count):
            print("FORWARD step: " + str(i) + "/" + str(self.step_count))
            if i % 4 == 0:
                GPIO.output(self.stepperPinConfig[3], GPIO.HIGH)
                GPIO.output(self.stepperPinConfig[2], GPIO.LOW)
                GPIO.output(self.stepperPinConfig[1], GPIO.LOW)
                GPIO.output(self.stepperPinConfig[0], GPIO.LOW)
            elif i % 4 == 1:
                GPIO.output(self.stepperPinConfig[3], GPIO.LOW)
                GPIO.output(self.stepperPinConfig[2], GPIO.LOW)
                GPIO.output(self.stepperPinConfig[1], GPIO.HIGH)
                GPIO.output(self.stepperPinConfig[0], GPIO.LOW)
            elif i % 4 == 2:
                GPIO.output(self.stepperPinConfig[3], GPIO.LOW)
                GPIO.output(self.stepperPinConfig[2], GPIO.HIGH)
                GPIO.output(self.stepperPinConfig[1], GPIO.LOW)
                GPIO.output(self.stepperPinConfig[0], GPIO.LOW)
            elif i % 4 == 3:
                GPIO.output(self.stepperPinConfig[3], GPIO.LOW)
                GPIO.output(self.stepperPinConfig[2], GPIO.LOW)
                GPIO.output(self.stepperPinConfig[1], GPIO.LOW)
                GPIO.output(self.stepperPinConfig[0], GPIO.HIGH)
            sleep(0.002)
        
    def backward_ballast(self):
        for i in range(self.step_count, 0, -1):
            print("BACKWARD step: " + str(i) + "/" + str(self.step_count))
            if i % 4 == 0:
                GPIO.output(self.stepperPinConfig[3], GPIO.HIGH)
                GPIO.output(self.stepperPinConfig[2], GPIO.LOW)
                GPIO.output(self.stepperPinConfig[1], GPIO.LOW)
                GPIO.output(self.stepperPinConfig[0], GPIO.LOW)
            elif i % 4 == 1:
                GPIO.output(self.stepperPinConfig[3], GPIO.LOW)
                GPIO.output(self.stepperPinConfig[2], GPIO.LOW)
                GPIO.output(self.stepperPinConfig[1], GPIO.HIGH)
                GPIO.output(self.stepperPinConfig[0], GPIO.LOW)
            elif i % 4 == 2:
                GPIO.output(self.stepperPinConfig[3], GPIO.LOW)
                GPIO.output(self.stepperPinConfig[2], GPIO.HIGH)
                GPIO.output(self.stepperPinConfig[1], GPIO.LOW)
                GPIO.output(self.stepperPinConfig[0], GPIO.LOW)
            elif i % 4 == 3:
                GPIO.output(self.stepperPinConfig[3], GPIO.LOW)
                GPIO.output(self.stepperPinConfig[2], GPIO.LOW)
                GPIO.output(self.stepperPinConfig[1], GPIO.LOW)
                GPIO.output(self.stepperPinConfig[0], GPIO.HIGH)
            sleep(0.002)
        
    def staticThruster(self, percentage):
        percentage /= 100
        self.thrusterPinConfig[0].throttle = percentage
        sleep(0.002)
        self.thrusterPinConfig[1].throttle = percentage
        sleep(0.002)
        self.thrusterPinConfig[2].throttle = percentage
        sleep(0.002)
        self.thrusterPinConfig[3].throttle = percentage
        sleep(0.002)
        print("RUNNING 4 THRUSTER ON " + str(percentage))
        sleep(0.002)

    def front(self):
        self.allServoPinConfig[0].set_pulse_width_range(500, 2500)
        sleep(0.002)
        self.allServoPinConfig[0].angle = 90
        

    def back(self):
        self.allServoPinConfig[0].set_pulse_width_range(500, 2500)
        sleep(0.002)
        self.allServoPinConfig[0].angle = 90
        
    def left(self):
        self.allServoPinConfig[0].set_pulse_width_range(500, 2500)
        sleep(0.002)
        self.allServoPinConfig[0].angle = 150
        
    def right(self):
        self.allServoPinConfig[0].set_pulse_width_range(500, 2500)
        sleep(0.002)
        self.allServoPinConfig[0].angle = 30
        
    def dynamicMove(self, angle):
        self.allServoPinConfig[0].set_pulse_width_range(500, 2500)
        self.allServoPinConfig[0].angle = angle

    def horizontalFinUp(self):
        self.allServoPinConfig[1].set_pulse_width_range(500, 2500)
        sleep(0.002)
        self.allServoPinConfig[1].angle = 150
        
    def horizontalFinDown(self):
        self.allServoPinConfig[1].set_pulse_width_range(500, 2500)
        sleep(0.002)
        self.allServoPinConfig[1].angle = 30
        
    def cleanup(self):
        GPIO.output(self.stepperPinConfig[3], GPIO.LOW)
        GPIO.output(self.stepperPinConfig[2], GPIO.LOW)
        GPIO.output(self.stepperPinConfig[1], GPIO.LOW)
        GPIO.output(self.stepperPinConfig[0], GPIO.LOW)
        GPIO.cleanup()

