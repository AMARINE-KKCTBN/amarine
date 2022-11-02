import serial

try:
    serial = serial.Serial(port="/dev/ttyUSB0", baudrate=9600, timeout=1)
    while True:
        # print(str(serial.readline().decode("UTF-8")))
        data = serial.readline().decode("UTF-8")
        print(data)
except:
    print("CANNOT CONNECT TO SERIAL PORT")
