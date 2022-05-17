import numpy as np
import math

#math.radians(float(angle_passed1))

def Preg(xverdi, yverdi):
    Preg_values = [0]*2

    for i in range(2):
        dist = yverdi if i == 0 else xverdi
        cord_ratio = dist/28
        d = 17.5 if i == 0 else 20
        platform_angle = math.radians( (cord_ratio*105) / (7) )
        motor_angle = np.arcsin(( d*np.sin(platform_angle)) /(2*4))
        Preg_values[i] = motor_angle        #indeks 0 er pitch og indeks 1 er roll

    servo_values = [Preg_values[0]-Preg_values[1], Preg_values[0]+Preg_values[1], -Preg_values[0]]
    # have to fix a min/max regulation for servo_values ;)
    return servo_values

x = 27
y = 24

values = Preg(x, y)
print(str(values))
