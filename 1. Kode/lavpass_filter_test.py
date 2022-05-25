import numpy as np
import scipy.signal as sig
import matplotlib.pyplot as plt


xn = np.linspace(0,19,10)
wc = 17
Ts = 100

b = [wc*Ts/2 , wc*Ts]
a = [1, -(2-(wc/2))]

yn = sig.lfilter(b,a,xn)

plt.subplot(2,1,1)
plt.plot(np.abs(yn))
plt.grid()

plt.subplot(2,1,2)
plt.plot(np.angle(yn))
plt.grid()

plt.show()
