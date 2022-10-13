import RPi.GPIO as GPIO
from time import sleep
from adafruit_servokit import ServoKit    #https://circuitpython.readthedocs.io/projects/servokit/en/latest/


class Controller:
    def __init__(self, stepperPinConfig, thrusterPinConfig, mainThrusterPinConfig, allServoPinConfig) -> None:
        self.stepperPinConfig = stepperPinConfig
        self.thrusterPinConfig = thrusterPinConfig
        self.mainThrusterPinConfig = mainThrusterPinConfig
        self.allServoPinConfig = allServoPinConfig
        pass

    def setStepperConfiguration(self, step_sleep, step_count):
        self.step_count = step_count
        self.step_sleep = step_sleep

        # Set stepper pinout configuration
        GPIO.setup( self.stepperPinConfig[0], GPIO.OUT )
        GPIO.setup( self.stepperPinConfig[1], GPIO.OUT )
        GPIO.setup( self.stepperPinConfig[2], GPIO.OUT )
        GPIO.setup( self.stepperPinConfig[3], GPIO.OUT )

        # Init stepper pinout
        GPIO.setup( self.stepperPinConfig[0], GPIO.LOW )
        GPIO.setup( self.stepperPinConfig[1], GPIO.LOW )
        GPIO.setup( self.stepperPinConfig[2], GPIO.LOW )
        GPIO.setup( self.stepperPinConfig[3], GPIO.LOW )
        
        pass

    def forward_ballast(self):
        i = 0
        for i in range(self.step_count):
            print("step: " + str(i) + "/" + str(self.step_count))
            if i%4==0:
                GPIO.output( self.stepperPinConfig[3], GPIO.HIGH )
                GPIO.output( self.stepperPinConfig[2], GPIO.LOW )
                GPIO.output( self.stepperPinConfig[1], GPIO.LOW )
                GPIO.output( self.stepperPinConfig[0], GPIO.LOW )
            elif i%4==1:
                GPIO.output( self.stepperPinConfig[3], GPIO.LOW )
                GPIO.output( self.stepperPinConfig[2], GPIO.LOW )
                GPIO.output( self.stepperPinConfig[1], GPIO.HIGH )
                GPIO.output( self.stepperPinConfig[0], GPIO.LOW )
            elif i%4==2:
                GPIO.output( self.stepperPinConfig[3], GPIO.LOW )
                GPIO.output( self.stepperPinConfig[2], GPIO.HIGH )
                GPIO.output( self.stepperPinConfig[1], GPIO.LOW )
                GPIO.output( self.stepperPinConfig[0], GPIO.LOW )
            elif i%4==3:
                GPIO.output( self.stepperPinConfig[3], GPIO.LOW )
                GPIO.output( self.stepperPinConfig[2], GPIO.LOW )
                GPIO.output( self.stepperPinConfig[1], GPIO.LOW )
                GPIO.output( self.stepperPinConfig[0], GPIO.HIGH )
            sleep( 0.002 )
        pass

    def backward_ballast(self):
        for i in range(self.step_count, 0, -1):
            print("step: " + str(i) + "/" + str(self.step_count))
            if i%4==0:
                GPIO.output( self.stepperPinConfig[3], GPIO.HIGH )
                GPIO.output( self.stepperPinConfig[2], GPIO.LOW )
                GPIO.output( self.stepperPinConfig[1], GPIO.LOW )
                GPIO.output( self.stepperPinConfig[0], GPIO.LOW )
            elif i%4==1:
                GPIO.output( self.stepperPinConfig[3], GPIO.LOW )
                GPIO.output( self.stepperPinConfig[2], GPIO.LOW )
                GPIO.output( self.stepperPinConfig[1], GPIO.HIGH )
                GPIO.output( self.stepperPinConfig[0], GPIO.LOW )
            elif i%4==2:
                GPIO.output( self.stepperPinConfig[3], GPIO.LOW )
                GPIO.output( self.stepperPinConfig[2], GPIO.HIGH )
                GPIO.output( self.stepperPinConfig[1], GPIO.LOW )
                GPIO.output( self.stepperPinConfig[0], GPIO.LOW )
            elif i%4==3:
                GPIO.output( self.stepperPinConfig[3], GPIO.LOW )
                GPIO.output( self.stepperPinConfig[2], GPIO.LOW )
                GPIO.output( self.stepperPinConfig[1], GPIO.LOW )
                GPIO.output( self.stepperPinConfig[0], GPIO.HIGH )
            sleep( 0.002 )
        pass

    def staticThruster(self, percentage):
        self.thrusterPinConfig[0].throttle = percentage
        self.thrusterPinConfig[1].throttle = percentage
        self.thrusterPinConfig[2].throttle = percentage
        self.thrusterPinConfig[3].throttle = percentage
        pass

    def front(self):
        self.allServoPinConfig[0].set_pulse_width_range(500 , 2500)
        self.allServoPinConfig[0].angle = 90
        pass

    def back(self):
        self.allServoPinConfig[0].set_pulse_width_range(500 , 2500)
        self.allServoPinConfig[0].angle = 90 
        pass

    def left(self):
        self.allServoPinConfig[0].set_pulse_width_range(500 , 2500)
        self.allServoPinConfig[0].angle = 150
        pass

    def right(self):
        self.allServoPinConfig[0].set_pulse_width_range(500 , 2500)
        self.allServoPinConfig[0].angle = 30
        pass

    def horizontalFinUp(self):
        self.allServoPinConfig[1].set_pulse_width_range(500 , 2500)
        self.allServoPinConfig[1].angle = 150
        pass

    def horizontalFinDown(self):
        self.allServoPinConfig[1].set_pulse_width_range(500 , 2500)
        self.allServoPinConfig[1].angle = 30
        pass


if __name__ == "__main__":

    pca = ServoKit(channels=16)

    mainFin = pca.servo[0]
    secondaryFin = pca.servo[1]

    leftFrontProp = pca.continuous_servo[5]
    leftBackProp = pca.continuous_servo[6]
    rightFrontProp = pca.continuous_servo[7]
    rightBackProp = pca.continuous_servo[8]

    mainProp = pca.continuous_servo[4]

    controller = Controller(
        stepperPinConfig=[11,22,33,44], 
        thrusterPinConfig=[leftBackProp, leftFrontProp, rightBackProp, rightFrontProp], 
        mainThrusterPinConfig=mainProp, 
        allServoPinConfig=[mainFin, secondaryFin]
        )