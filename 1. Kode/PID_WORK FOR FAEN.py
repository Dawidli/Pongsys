from simple_pid import PID
import numpy as np

P = 1
I = 0
D = 0.69
x = np.linspace(0, 16, 150)


pid = PID(P,I,D,setpoint=0)

for i in range(len(x)):
    v = x[i]
    # Compute new output from the PID according to the systems current value
    control = pid(v)
    print(v)
    print(control)
    print()