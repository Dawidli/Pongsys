import math
import numpy as np
import time

setling_time = 6
overshoot = 20
ms = time.time()*1000.0
cycle_time = time.perf_counter()*1000.0 #ms - ms_old
ms_old = ms

koordinater = np.linspace(2,3,10000)

def Preg(xverdi, yverdi):
    global servo_values

    if xverdi == 'i' and yverdi == 'n':
        return
    else:
        Preg_values = [0] * 2

        for i in range(2):
            dist = yverdi if i == 0 else xverdi
            cord_ratio = dist / 28
            d = 17.5 if i == 0 else 20
            platform_angle = math.radians((cord_ratio * 105) / (7))
            motor_angle = np.arcsin((d * np.sin(platform_angle)) / (2 * 4))
            Preg_values[i] = motor_angle  # indeks 0 er pitch og indeks 1 er roll
        servo_values = [Preg_values[0] - Preg_values[1], Preg_values[0] + Preg_values[1], -Preg_values[0]]
    # have to fix a min/max regulation for servo_values ;)

def PDreg(xverdi, yverdi, xold, yold, K, cycle_time):
    global servo_values

    if xverdi == 'i' and yverdi == 'n':
        return
    else:
        Preg_values = [0] * 2

        for i in range(2):
            dist = yverdi if i == 0 else xverdi
            dist_old = yold if i == 0 else xold
            platform_angle = -K[0]*dist - K[1] * (dist - dist_old) / (cycle_time) # Platform vinkel
            d = 17.5 if i == 0 else 20
            motor_angle = np.arcsin((d * np.sin( np.radians(platform_angle) )) / (2 * 4))
            Preg_values[i] = motor_angle  # indeks 0 er pitch og indeks 1 er roll
        servo_values = [Preg_values[0] - Preg_values[1], Preg_values[0] + Preg_values[1], -Preg_values[0]]


def K_values(setling_time, overshoot):
    demping = (-np.log( (overshoot) / (100) )) / ( np.sqrt(np.pi**2 + np.log( (overshoot) / (100) )**2 ) )
    omega = (4) / ( demping * setling_time )
    K = [omega**2, 2*demping*omega]
    return K


K = K_values(setling_time, overshoot)
print(round(K[1]), round(K[0]))

#PDreg(x, y, xo, yo, K_values(setling_time, overshoot), cycle_time)
