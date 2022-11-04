from time import sleep
import serial

if __name__ == '__main__':
    ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=None)
    # ser.reset_input_buffer()
    while True:
        ser.write('1'.encode('utf-8'))
        ser.flush()
        sleep(1)
        ser.write('0'.encode('utf-8'))
        ser.flush()
        sleep(1)

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
            
            
