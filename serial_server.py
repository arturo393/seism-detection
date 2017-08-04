import socket
import sys
from time import gmtime, strftime
from datetime import datetime
import time
import serial



ser = serial.Serial('/dev/ttyUSB0', 500000);

fname = 'sensor/CA1393BA_4_8_2017_6.csv'
fmode = 'ab'
bufferIn = "";
with open(fname, fmode) as outf:
    try:
        while True:
            # Receive the data one byte at a time
            data = ser.readline();
            print data
            outf.write(data)
    except KeyboardInterrupt:
        print('exiting.')


