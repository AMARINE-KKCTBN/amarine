import RPi.GPIO as GPIO
import time
# from gpiozero import Servo

GPIO.setmode(GPIO.BCM)

GPIO.setup(24, GPIO.OUT)
pwm=GPIO.PWM(24, 50) #50 Hz
pwm.start(0)

# servo = Servo(25)

def SetAngle(angle):
	duty = angle / 30 + 4
	# duty = angle
	GPIO.output(24, True)
	pwm.ChangeDutyCycle(duty)
	time.sleep(0.5)
	# pwm.ChangeDutyCycle(0)

# SetAngle(0)

#GPIOZERO LIBRARY
# try:
# 	while True:
# 		servo.min()
# 		time.sleep(0.5)
# 		servo.mid()
# 		time.sleep(0.5)
# 		servo.max()
# 		time.sleep(0.5)
# except KeyboardInterrupt:
# 	print("Program stopped")

#Test Initial Angles
# SetAngle(90)
# time.sleep(20)

#
while True:
	# for i in range(3, 11, 1):
	# 	print(i)
	# 	SetAngle(i)

	# for i in range(10, 2, -1):
	# 	print(i)
	# 	SetAngle(i)

	for i in range(0, 180, 10):
		print(i)
		SetAngle(i)

	for i in range(180, 0, -20):
		print(i)
		SetAngle(i)

pwm.stop()
GPIO.cleanup()
