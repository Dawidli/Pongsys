
import math
import numpy as np

import time

time_array = [time.time()]*2

platform_angle = 0 # intial
delta_t = 1 / 100
velocity = [0] # initial verdi
pos_x = [0, 0]
pos_y = [0, 0]
K = [0.86, 0.69]

wc = 20.0 #hz
Ts = 1.0/30.0

b0 = wc/(wc*Ts + 1.0)
b1 = - b0
a0 = 1.0
a1 = -(1.0/(wc*Ts + 1.0))

x_array = [0.0]*10 # tom y[]
y_array = [0.0]*10
fps = 30
sample_time = 1/fps # fps
counter = 0
grense = len(x_array)




def PDreg(x_error, y_error, K):
    global velocity, servo_values, pos_y, pos_x

    if x_error == 'n' and y_error == 'i':
        return
    else:
        Regulator_values = [0] * 2

        for i in range(2):
            distance_error = y_error if i == 0 else x_error
            d = 17.5 if i == 0 else 20

            # Derivator
            # velocity[i] = ((y_array[counter-1]-y_array[counter-2]) * (0.625/ sample_time)) if i == 0 else ((x_array[counter]-x_array[counter-1]) *( 0.625/ sample_time))
            velocity[i] = b0 * y_array[counter] + b1 * y_array[counter - 1] if i == 0 else a0 * x_array[counter] + a1 * \
                                                                                           x_array[counter - 1]

            #print('velocity x', velocity[0], 'velocity y', velocity[1])
            # print('pos x', pos_x[0], 'pos y', pos_y[0])

            # regulator
            platform_angle = math.radians(-K[0] * distance_error - K[1] * velocity)

            # if distance_error == y_error:
            #    pos_y[1] = pos_y[0]
            #    pos_y[0] = y_error
            # else:
            #    pos_x[1] = pos_x[0]
            #    pos_x[0] = x_error

            # converts platform angle to servo angles and sends away
            motor_angle = np.arcsin((d * np.sin(platform_angle)) / (2 * 4))
            Regulator_values[i] = -motor_angle  # indeks 0 er pitch og indeks 1 er roll
        servo_values = [Regulator_values[0] - Regulator_values[1], Regulator_values[0] + Regulator_values[1],
                        -Regulator_values[0]]


PDreg(5.0, 5.0, K)
print(servo_values)
