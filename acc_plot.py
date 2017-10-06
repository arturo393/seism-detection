import numpy as np
import scipy as sy
import scipy.fftpack as syfp
import pylab as pyl
import matplotlib.pyplot as plt

SECS_PER_HOUR = 3600
SECS_PER_MINUTE = 60


hour = 0 
minute = 0
secs = 0
unixtime = 1499362200
#usecols=(0,6,7,8,15,16,17,18,19,20,21,22,23)
# Read in data from file here
array = np.loadtxt("./data/CA1111ST-Aug-04-2017_1917.csv", usecols=(0,5,6,7,8,15,16,17,18,19,20,21,22), delimiter=" ", skiprows=1000)
print (array)
Fs = 100;
# Create time data for x axis based on array length
n = array[:,0].size
# Do FFT analysis of array
FFT = np.fft.rfft(array[:,0])
# Getting the related frequencies
freqs = np.fft.rfftfreq(n,d=1./Fs)
# freqs = k/T

#set time windwow

wd = hour*SECS_PER_HOUR + minute*SECS_PER_MINUTE + secs
i = unixtime + wd
j = unixtime - wd


# Create subplot windows and show plot
plt.figure(1)
plt.subplot(411)
plt.plot(array[:,0],array[:,1])
plt.subplot(412)
plt.plot(array[:,0],array[:,2])
plt.subplot(413)
plt.plot(array[:,0],array[:,3])
plt.subplot(414)
plt.plot(array[:,0],array[:,4])

plt.show()
plt.figure(2)
plt.subplot(411)
plt.plot(array[:,0],array[j:1:i,5],array[j:1:i,0],array[j:1:i,9])
plt.subplot(412)
plt.plot(array[j:1:i,0],array[j:1:i,6],array[j:1:i,0],array[:,10])
plt.subplot(413)
plt.plot(array[j:1:i,0],array[j:1:i,7],array[j:1:i,0],array[j:1:i,11])
plt.subplot(414)
plt.plot(array[j:1:i,0],array[j:1:i,8],array[j:1:i,0],array[j:1:i,12])
plt.show()



plt.figure(3)
plt.subplot(3, 2, 1)
plt.plot(array[j:1:i,4])
plt.yscale('linear')
plt.title('Acelerometer Axis value')
plt.subplot(3, 2, 3)
plt.magnitude_spectrum(array[j:1:i,4], Fs=Fs)
plt.subplot(3, 2, 4)
plt.magnitude_spectrum(array[j:1:i,4], Fs=Fs, scale='dB')
plt.show()
