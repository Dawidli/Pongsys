import time
import numpy as np


#cycle_time = time.perf_counter()
#print(cycle_time)


time_array = [0]*5

def timerfunksjon():
    for i in range(5):
        ms = time.time()
        time_array[i] = ms
        #np.append(time_array[i], ms)
    return time_array

def velocity(cycle_time, koordinater, v=0):
    for i in range(len(koordinater)):
        if i != 0:
            v = v + ((koordinater[i]-koordinater[i-1]) / (cycle_time))
    return v

timerfunksjon()
cycle_time = time_array[1] - time_array[0]
koordinater = np.linspace(2,3,1000000)
print(cycle_time, 's')
print(velocity(cycle_time, koordinater))

x = np.array([1, 2, 4, 7, 0])
dx = np.diff(x)
print(dx)


