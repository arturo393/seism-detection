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

=======
>>>>>>> 9718f712fb9ec07cc811aa163a91cb5a60610289
#BACKUP_NAME = ("./data/ST1679BA-" + timestamp +".csv")
#ser = serial.Serial('/dev/ttyUSB3', 500000);

fname = BACKUP_NAME
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


