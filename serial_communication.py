'''
*************************
Robotic Pollinator: Mechanics, Planning, Computer Vision and Serial Communication
*************************
Authors: Daniel Medina
Email: dlmedina@uc.cl
Date: December 2020
*************************
'''

import serial
import time
import threading


SERIAL_PORT = '/dev/ttyUSB0' # check in Arduino IDE first
BAUDRATE = 9600
COMMUNICATION_DELAY = 0.5 #0.010  # 0.05

class SerialData:
    def __init__(self):
        self.theta1Goal, self.theta2Goal = -0.1, 0.3
        self.theta3Goal, self.theta4Goal = -1.4, 0.0
        self.servo = 'off' # or on
        #self.theta1, self.theta2, self.theta3, self.theta4 = 0.0, 0.0, 0.0, 0.0
        self.theta1, self.theta2, self.theta3, self.theta4 = -0.1, 0.3, -1.4, 0.0
        self.sw1, self.sw2, self.sw3, self.sw4 = False, False, False, False

se_data = SerialData()
lock = threading.Lock()

def init_serial():
    ser = serial.Serial(SERIAL_PORT, baudrate = BAUDRATE, timeout = 1)
    time.sleep(1)

    return ser

def close_serial(ser):
    ser.close()

def serial_com(ser):
    while True:
        msg_str = f'{se_data.theta1Goal};{se_data.theta2Goal};' + \
            f'{se_data.theta3Goal};{se_data.theta4Goal};{se_data.servo}'
        print(f'out: {msg_str}')       
        msg = msg_str.encode('utf-8')
        ser.write(msg)
        time.sleep(COMMUNICATION_DELAY)

        ser.readline()
        se_input = ser.readline().decode('utf-8').strip().split(';')
        if se_input[0] != '':
            th1, th2, th3, th4 = [float(theta) for theta in se_input[:4]]
            #sw1, sw2, sw3, sw4 = [True if sw=='True' else False for sw in se_input[4:]]
            sw1, sw2, sw3, sw4 = False, False, False, False
            info = f'in: th1:{th1:.2f}, th2:{th2:.2f}, th3:{th3:.2f}, th4:{th4:.2f}' + \
                f', sw1: {sw1}, sw2: {sw2}, sw3: {sw3}, sw4: {sw4}'
            print(info)
            se_data.theta1 = th1
            se_data.theta2 = th2
            se_data.theta3 = th3
            se_data.theta4 = th4
            se_data.sw1, se_data.sw2, se_data.sw3, se_data.sw4 = sw1, sw2, sw3, sw4
        time.sleep(COMMUNICATION_DELAY)



# if __name__ == '__main__':
#     ser = init_serial()
#     #thread_write = threading.Thread(target=write_serial, args=(ser,))
#     thread_read = threading.Thread(target=read_serial, args=(ser,))
#     #thread_write.start()
#     thread_read.start()

#     while True:
#         print(se_data.theta1)
#         time.sleep(0.2)
#     