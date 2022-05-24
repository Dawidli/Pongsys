import scipy.signal as sig
import numpy as np
import matplotlib.pyplot as plt



n = np.linspace(0,10,10)
xn = np.sin(50*n)

fs = 100         # Sample rate, Hz
cutoff = 17     # Desired cutoff frequency, Hz
trans_width = 30 # Width of transition from pass band to stop band, Hz
numtaps = 5      # Size of the FIR filter.

hn = sig.remez(numtaps,                                    # Lengde på impulsrespons = filterorden + 1
               [0, cutoff, cutoff + trans_width, 0.5*fs],  # Alle 'kantfrekvensene' etter tur
               [1, 0],                                     # Ønsket filtergain i de ulike båndene
               Hz=fs,                                      # Samplingsfrekvens
               weight=[10, 1],                             # Vi "bryr oss" 10 ganger mer om rippel i passbånd
                                                           # enn demping i stoppbånd
               type='differentiator'
              )

yn = sig.convolve(xn,hn)

plt.close(1); plt.figure(1)
plt.subplot(2,1,1)
plt.stem(xn)
plt.grid()


plt.subplot(2,1,2)
plt.stem(yn)
plt.grid()

plt.show()


print(xn[4:9],yn[4:9])

print(yn[4] + yn[5] + yn[6] + yn[7] + yn[8] + yn[9] )