import serial
import io
import sys
import numpy as np
import struct
from threading import Thread
import threading
import time
temp = []
temp1 = []
data = []

ser = serial.Serial('/dev/ttyUSB1', 115200, timeout=1)
dt = 0.003
currentSample = 0

mag_data = []
value = []
def send_uart(velocity,angle):
    global value,ser
    angle = 45
    velocity = 100

    try:
        # print("a")
        crc = 0
        ba = bytearray(struct.pack("f", velocity))  
        bv = bytearray(struct.pack("f", angle))  
        ser.write(b's')
        # print(len(ba))
        ser.write(ba)
        ser.write(bv)
        crc += sum(ba)
        crc += sum(bv)
        ser.write(bytes([crc % 37]))
        ser.write(b'e') 
        # time.sleep(100)
    except Exception as e:
        print(e)
        
def advance_uart():
    global value,ser
    data = []
    try:
        value = ser.readline()
            # print(value)
        crc = 0
        for i in value[0:41]:
            crc+=ord(i)

            # print(value[40])
        if(((crc)//123 - ord(value[40]))<= 1):
            # print()
            
            for i in range(10):
                b = value[4*i:4*(i+1)]
                data.append(round(float(struct.unpack('f',b)[0]),3))

    except Exception as e:
        print(e)
    return data   
# with serial.Serial('COM5', 115200, timeout=1) as ser:
    # t1 = threading.Thread(target=send_uart)
    # t1.start()
    # advance_uart()


#     i = 0
#     crc_error = 0
#     frame_error = 0
#     true_frame = 0

#     while i < 10000:
#         try:

#             data = []
            
#             i+=1
#             print(value)
#             if(len(value) == 26):
#                 true_frame = true_frame + 1
#                 # print(true_frame)
#             else:
#                 frame_error += 1
            
#                 # b.append()
#             # b1 = value[4:8]
#             # b2 = value[8:12]
            
            
#             else:
#                 print("not OK")
#                 crc_error += 1
#             # print(str(crc) + " vs " + str(temp))
#             # print(len(value))
#             # temp = (value.decode().replace('\n','').replace('\x00','')).split(' ')
            
#             #currentSample += dt
#             #time.append(currentSample)
            
#         except Exception as e:
#             print("")
#     print("frame error rate",frame_error/10000)
#     print("CRC error rate",crc_error/true_frame)
# data = np.array(data)

