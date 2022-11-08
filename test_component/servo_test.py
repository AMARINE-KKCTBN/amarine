from time import sleep
from adafruit_servokit import ServoKit
import serial

pca = ServoKit(channels=16)
mainFin_servo = pca.servo[6]
secondaryFin_servo = pca.servo[7]
mainFin_servo.set_pulse_width_range(500, 2500)

while True:
    sleep(0.1)
    mainFin_servo.angle = 30
    sleep(0.1)
    mainFin_servo.angle = 90
    sleep(0.1)
    mainFin_servo.angle = 150

ser = serial.Serial(port='/dev/ttUSB0', baudrate=9600, timeout=1)

