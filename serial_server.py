import socket
import sys
from time import gmtime, strftime
from datetime import datetime
import time
import serial
import matplotlib.pyplot as plt
import time, os, fnmatch, shutil


t = time.localtime()
timestamp = time.strftime('%b-%d-%Y_%H%M', t)
#BACKUP_NAME = ("./data/CA1393BA-" + timestamp + ".csv")
#ser = serial.Serial('/dev/ttyUSB1', 500000);

#BACKUP_NAME = ("./data/CA2117SB-" + timestamp + ".csv")
#ser = serial.Serial('/dev/ttyUSB2', 500000);

BACKUP_NAME = ("./data/CC4615BA-" + timestamp +".csv")
ser = serial.Serial('/dev/ttyUSB0', 500000);

<<<<<<< HEAD
fname = './paramcheck.csv'
=======

#BACKUP_NAME = ("./data/ST1679BA-" + timestamp +".csv")
#ser = serial.Serial('/dev/ttyUSB3', 500000);

fname = BACKUP_NAME
>>>>>>> c1622fe70e42c65b35a0fb7d2d95cb2491693a92
fmode = 'ab'
bufferIn = "";
with open(fname, fmode) as outf:
    try:
        while True:
            # Receive the data one byte at a time
            data = ser.readline();
            print (data)
            outf.write(data)
    except KeyboardInterrupt:
        print('exiting.')


