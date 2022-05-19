import math
import numpy as np
import time

time_array = [time.time()]*2

def timerfunksjon():
    time_array[1] = time_array[0]
    time_array[0] = time.time()
    cycle_time = (time_array[0] - time_array[1])/1000
    return cycle_time

platform_angle = 0 # intial
delta_t = 1 / 100
current_platform_angle = [0, 0] # initial verdi
velocity = [0, 0] # initial verdi
K = [46, 8]


def PDreg(x_error, y_error, K):
    global velocity

    if x_error == 'i' and y_error == 'n':
        return
    else:
        Regulator_values = [0] * 2

        for i in range(2):
            distance_error = y_error if i == 0 else x_error
            d = 17.5 if i == 0 else 20

            # integrator: calculating ball-velocity
            velocity[i] += (current_platform_angle[i] * delta_t) / 7

            # regulator
            platform_angle = math.radians(-K[0]*distance_error - K[1]*velocity[i])
            current_platform_angle[i] = platform_angle

            # converts platform angle to servo angles and sends away
            motor_angle = np.arcsin((d * np.sin(platform_angle)) / (2 * 4))
            Regulator_values[i] = -motor_angle  # indeks 0 er pitch og indeks 1 er roll
        servo_values = [Regulator_values[0] - Regulator_values[1], Regulator_values[0] + Regulator_values[1], -Regulator_values[0]]
        return current_platform_angle, velocity

for i in range(10):
    print(PDreg(10,10,K))
#for I in range(100):
#    platform_angle, velocity = PDreg(10, 5, K)
#    print('Fart i y retning:',round(velocity[0],3), 'Fart i x retning:', round(velocity[1],3), 'Platform vinkel y:', round(platform_angle[0],3), 'Platform vinkel x:', round(platform_angle[1],3))
#
#    wait = 0.1 - timerfunksjon()
#    print(round(wait,6))
#    time.sleep(wait)
