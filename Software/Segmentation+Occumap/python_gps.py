import io

import pynmea2
import serial


ser = serial.Serial('COM17', 9600, timeout=5.0)
sio = io.TextIOWrapper(io.BufferedRWPair(ser, ser))

while 1:
    try:
        line = sio.readline()
        msg = pynmea2.parse(line)
        print(msg.latitude,msg.longitude)

    except Exception as e:
        print('Parse error: {}'.format(e))
        continue