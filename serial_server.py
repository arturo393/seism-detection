import socket
import sys
from time import gmtime, strftime
from datetime import datetime
import time
import serial
ser = serial.Serial('/dev/ttyUSB0', 500000);

fname = './paramcheck.csv'
fmode = 'ab'
bufferIn = "";
with open(fname, fmode) as outf:
    try:
        while True:
            # Receive the data one byte at a time
            data = ser.readline();
            bufferIn = "%s%s" % (bufferIn, data)
            utcTime = strftime("%d-%m-%y %H:%M:%S", gmtime()) + "." + str(datetime.now().microsecond) + ","

            if '\n' in data:
            	outf.write(bufferIn[4:])
            	print "received data:", bufferIn
            	bufferIn = ""
    except KeyboardInterrupt:
        print('exiting.')


