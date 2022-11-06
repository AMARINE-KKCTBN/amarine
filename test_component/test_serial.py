import multiprocessing
from time import sleep
import serial
import sys

def serialComSend():
    ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=None)
    status = 0
    try:
        while True:
                if not status:
                    ser.write('1\n'.encode('utf-8'))
                else:
                    ser.write('0\n'.encode('utf-8'))
                status = not status
                ser.flush()
                sleep(1)
    except KeyboardInterrupt:
        print("Hello {}, hope you're well!".format("ABDAN"))
        ser.close()
        sys.exit(0)

def serialComRec():
    ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=None)
    ser.reset_input_buffer()
    try:
        while True:
                if ser.in_waiting > 0:
                    data = ser.readline().decode('utf-8').rstrip()
                    if data == '0':
                        print(data)
                else:
                    print("NO DATA")
                sleep(0.1)
    except KeyboardInterrupt:
        print("EXITING P2")  
        ser.close()  
        sys.exit(0)

if __name__ == '__main__':

    p1 = multiprocessing.Process(target=serialComSend, args=())
    p2 = multiprocessing.Process(target=serialComRec, args=())

    p1.start()
    p2.start()
    p1.join()
    p2.join()


      