import RPi.GPIO as GPIO
from time import sleep

class Controller:
    def __init__(
        self,
        thrusterPinConfig,
        mainThrusterPinConfig,
        allServoPinConfig,
    ) -> None:
        self.thrusterPinConfig = thrusterPinConfig
        self.mainThrusterPinConfig = mainThrusterPinConfig
        self.allServoPinConfig = allServoPinConfig
        
    def staticThruster(self, percentage):
        if self.thrusterPinConfig[0] or self.thrusterPinConfig[1]:
            percentage+=5
        if self.thrusterPinConfig[1] or self.thrusterPinConfig[3]:
            percentage+=1
        percentage /= 100
        self.thrusterPinConfig[0].throttle = percentage * -1
        sleep(0.002)
        self.thrusterPinConfig[1].throttle = percentage
        sleep(0.002)
        self.thrusterPinConfig[2].throttle = percentage * -1
        sleep(0.002)
        self.thrusterPinConfig[3].throttle = percentage
        sleep(0.002)
    
    def dynamicServo(self, coord_x):
        offset = 90
        deg_range = 60
        coord_range = 0.25
        coord2deg = coord_x / coord_range * deg_range
        output = offset + coord2deg
        self.allServoPinConfig[0].set_pulse_width_range(500, 2500)
        sleep(0.002)
        self.allServoPinConfig[0].angle = output
        sleep(0.002)

    def mainThruster(self, thruster_speed):        
        self.mainThrusterPinConfig.throttle = thruster_speed / 100
        sleep(0.002)

    def initMainThruster(self):        
        self.mainThrusterPinConfig.throttle = 0.0
        sleep(0.002)

    def forward(self):
        self.allServoPinConfig[0].set_pulse_width_range(500, 2500)
        sleep(0.002)
        self.allServoPinConfig[0].angle = 90
        sleep(0.002)

    def left(self):
        self.allServoPinConfig[0].set_pulse_width_range(500, 2500)
        sleep(0.002)
        self.allServoPinConfig[0].angle = 30
        sleep(0.002)

    def right(self):
        self.allServoPinConfig[0].set_pulse_width_range(500, 2500)
        sleep(0.002)
        self.allServoPinConfig[0].angle = 150
        sleep(0.002)

    def stop(self):
        self.mainThrusterPinConfig.throttle = 0.0
        sleep(0.002)

    def cleanup(self):
        GPIO.output(self.stepperPinConfig[3], GPIO.LOW)
        GPIO.output(self.stepperPinConfig[2], GPIO.LOW)
        GPIO.output(self.stepperPinConfig[1], GPIO.LOW)
        GPIO.output(self.stepperPinConfig[0], GPIO.LOW)

