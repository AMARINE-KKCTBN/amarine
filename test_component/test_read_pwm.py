import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BCM)
GPIO.setup(17, GPIO.IN)

while True:
    i = GPIO.input(17)       # read status of pin/port and assign to variable i  
    print(i)

# double acc=0.5;
# const double k=0.01;
# for(;;) {
#   bool x=GPIO.read();
#   acc+=k*(x?1:0-acc);
# }