import multiprocessing
import threading
from time import sleep
import serial

def serialCom():
    ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=None)
    while True:
        if True:
            a = ser.write('1'.encode('utf-8'))
            ser.flush()
            sleep(0.1)
        else:
            ser.write('0'.encode('utf-8'))
            ser.flush()
            sleep(0.1)
        # ser.reset_input_buffer()
        # ser.reset_output_buffer()

# def serialCom():
#     ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=None)
#     while True:
#         if True:
#             ser.write('1'.encode('utf-8'))
#             ser.flush()
#             sleep(0.1)
#         else:
#             ser.write('0'.encode('utf-8'))
#             ser.flush()
#             sleep(0.1)
#         ser.reset_input_buffer()
#         ser.reset_output_buffer()

if __name__ == '__main__':

    # serialCom(ser)
    p1 = multiprocessing.Process(target=serialCom, args=())
    # p1 = threading.Thread(target=serialCom, args=(ser))

    p1.start()
    p1.join()
        # if ser.in_waiting > 0:
        #     line = ser.readline().decode('utf-8')
        #     # data_left = ser.in_waiting
        #     # line+=str(data_left)
        #     print(line)
            # if line+=data_left.readline().decode('utf-8') == "1\r\n":
            #     print("SATU")
            # else:
            #     print("NOL")
            # else:
            
            
