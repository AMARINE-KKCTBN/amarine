import RPi.GPIO as GPIO
from time import sleep
# from adafruit_servokit import ServoKit    #https://circuitpython.readthedocs.io/projects/servokit/en/latest/


class Controller:
    # SET GPIO PINMODE TO ACTUAL PIN NUMBER ON RASPI
    GPIO.setmode(GPIO.BOARD)
    
    # CONSTANT FOR STEPPER
    step_sleep = 0.002
    step_count = 2400 #200 = 1 step/180

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

    def log(self):
        print(self.stepperPinConfig)
        # print(self.pinConfig.stepper[0])
        # print(self.pinConfig.stepper[1])
        # print(self.pinConfig.stepper[2])
        # print(self.pinConfig.stepper[3])
        
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
        
        pass

    def front(self):
        pass

    def back(self):
        pass

    def left(self):
        pass

    def right(self):
        pass

    def horizontalFinUp(self):
        pass

    def horizontalFinDown(self):
        pass


if __name__ == "__main__":
    cnt = Controller(stepperPinConfig=[11,22,33,44])
    cnt.log()