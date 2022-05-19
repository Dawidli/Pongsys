import matplotlib
import numpy as np
from scipy import signal as sig
import matplotlib.pyplot as plt

#A = np.array([[0,1],
#     [0,0]])
#B = np.array([[0],
#     [1]])
#C = np.array([[1,0],
#     [0,1]])
#D = np.array([[0]])
#sys = sig.ss2tf(A,B,C,D)
#print(sys)

#2x2
A = [[0,1],
     [0,0]]
#1x2
B = [[0],
     [7]]
#2x2
C = [[1,1]]
#1x1
D = [[0]]


sys = sig.StateSpace(A,B,C,D)
t1,y1 = sig.step(sys)

print(sys)

if name ==
plt.plot(t1,y1)


