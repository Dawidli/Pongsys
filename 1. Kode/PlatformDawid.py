import multiprocessing as mp
from multiprocessing import Queue
import cvzone
from cvzone.ColorModule import ColorFinder
import cv2
import serial
import math
from tkinter import *
import numpy as np
import imutils
import time

# --------- Initial values needed for servo and camera communication

camera_port = 0
cap = cv2.VideoCapture(camera_port)
cap.set(3, 1280)
cap.set(4, 720)

get, img = cap.read()
h, w, _ = img.shape

port_id = '/dev/cu.usbmodem143101'
# initialise serial interface
arduino = serial.Serial(port=port_id, baudrate=250000, timeout=0.1)

# define servo angles and set a value
servo1_angle = 0
servo2_angle = 0
servo3_angle = 0
all_angle = 0

# Set a limit to upto which you want to rotate the servos (You can do it according to your needs)
servo1_angle_limit_positive = 90
servo1_angle_limit_negative = -90

servo2_angle_limit_positive = 90
servo2_angle_limit_negative = -90

servo3_angle_limit_positive = 90
servo3_angle_limit_negative = -90

'''
These are our own defined values that we use in functions
    |   |   |   |   |   |   |   |   |   |   |   |   |   |   |   | 
    V   V   V   V   V   V   V   V   V   V   V   V   V   V   V   V
'''
# ______________________________________________________________________________
time_array = [time.time()] * 2  # Save time values
x = [0.0] * 4
y = [0.0] * 4
speed = [0.0] * 2
counter = 0
grense = len(x)
delta_t = 1 / 30
distance_error = [0.0, 0.0]
K = [0.3, 0.004]
move_time = 90
move_counter = 0
pos_counter = 0
refpos = [-0,-0]
p0 = [10,10]
p1 = [10,-10]
p2 = [-10,-10]
p3 = [-10,10]
path = [p0,p1,p2,p3]

do_you_want_path = False

# ______________________________________________________________________________




def ball_track(key1, queue):
    if key1:
        print('Ball tracking is initiated')

    myColorFinder = ColorFinder(
        False)  # if you want to find the color and calibrate the program we use this *(Debugging)
    # hsvVals = {'hmin': 30, 'smin': 30, 'vmin': 65, 'hmax': 60, 'smax': 130, 'vmax': 255}
    hsvVals = {'hmin': 34, 'smin': 0, 'vmin': 0, 'hmax': 100, 'smax': 255, 'vmax': 255}
    center_point = [600, 360, 2280]  # center point of the plate, calibrated

    while True:
        get, img = cap.read()
        image = imutils.rotate(img, 7)
        imgColor, mask = myColorFinder.update(image, hsvVals)
        imgContour, countours = cvzone.findContours(image, mask)

        if countours:
            data = (countours[0]['center'][0] - center_point[0]) / 10, \
                   (h - countours[0]['center'][1] - center_point[1]) / 10, \
                   int(countours[0]['area'] - center_point[2]) / 100

            queue.put(data)
            # print("The got coordinates for the ball are :", data)
        else:
            data = 'nil'  # returns nil if we can´t find the ball
            queue.put(data)

        imgStack = cvzone.stackImages([imgContour], 1, 1)
        # imgStack = cvzone.stackImages([img,imgColor, mask, imgContour],2,0.5) #use for calibration and correction
        cv2.imshow("Image", imgStack)
        cv2.waitKey(1)


def servo_control(key2, queue):
    if key2:
        print('Servo controls are initiated')

    def all_angle_assign(angle_passed1, angle_passed2, angle_passed3):
        global servo1_angle, servo2_angle, servo3_angle
        servo1_angle = float(angle_passed1)
        servo2_angle = float(angle_passed2)
        servo3_angle = float(angle_passed3)
        write_servo()

    root = Tk()
    root.resizable(0, 0)

    def find_speed(avg):
        global counter
        speed = (avg[counter] - avg[counter - 1]) / (time_array[0] - time_array[1])
        return speed

    def PDreg(x_error, y_error):
        global servo_values, distance_error, K, speed, counter, platform_angle, refpos, move_counter, pos_counter
        if(do_you_want_path):
            if move_time <= move_counter:
                refpos[0] = path[pos_counter][0]
                refpos[1] = path[pos_counter][1]
                pos_counter +=1
                pos_counter = pos_counter % len(path)
                move_counter = 0
            else:
                move_counter += 1


        Regulator_values = [0] * 2
        distance_error[1] = y_error[counter] - refpos[0]
        distance_error[0] = x_error[counter] - refpos[1]
        for i in range(2):
            d = 20.2 if i == 0 else 17.5

            # regulator u = -kx-kv
            platform_angle = math.radians(-K[0] * distance_error[i]) - (K[1] * speed[i])

            # converts platform angle to servo angles
            motor_angle = np.arcsin((d * np.sin(platform_angle)) / (2 * 4))
            Regulator_values[i] = motor_angle  # indeks 0 er pitch og indeks 1 er roll

        servo_values = [-Regulator_values[0] + (0.549 * Regulator_values[1]),
                        Regulator_values[0] + (0.549 * Regulator_values[1]),
                        -Regulator_values[1]]

    def writeCoord():
        global counter
        corrd_info = queue.get()

        if corrd_info == 'nil':
            print('Give me my ball back! >:c')
        else:

# -----------------------THE THING UNDER IS YOUR CODE´S BRAIN SILLY ---------------------
            time_array[1] = time_array[0]
            time_array[0] = time.time()
            counter = counter % grense

            x[counter] = corrd_info[0]
            y[counter] = corrd_info[1]
            speed[0] = find_speed(x)
            speed[1] = find_speed(y)
            PDreg(x, y)

            print('Angles sent to arduino:', np.degrees(servo1_angle), np.degrees(servo2_angle), np.degrees(servo3_angle))
            print('Current position', round(x[counter],2), round(y[counter],2))
            print('Current speed' , round(speed[0], 2), round(speed[1], 2))
            print('Reference position is:', refpos)
            print('Move counter',move_counter,'pos counter',pos_counter)
            print()

            all_angle_assign(servo_values[0], servo_values[1], servo_values[2])
            counter += 1
# -----------------------THE THING OVER IS YOUR CODE´S BRAIN SILLY ---------------------


    def write_arduino(data):

        arduino.write(bytes(data, 'utf-8'))

    def write_servo():
        ang1 = math.degrees(servo1_angle)
        ang2 = math.degrees(servo2_angle)
        ang3 = math.degrees(servo3_angle)
        minValue = 30
        maxValue = -30
        if ang1 > minValue:
            ang1 = minValue
        elif ang1 < maxValue:
            ang1 = maxValue

        if ang2 > minValue:
            ang2 = minValue
        elif ang2 < maxValue:
            ang2 = maxValue

        if ang3 > minValue:
            ang3 = minValue
        elif ang3 < maxValue:
            ang3 = maxValue

        angles: tuple = (round(ang1, 5),
                         round(ang2, 5),
                         round(ang3, 5))
        write_arduino(str(angles))

    while key2:
        writeCoord()

    root.mainloop()


if __name__ == '__main__':
    queue = Queue()
    key1 = 1
    key2 = 2
    p1 = mp.Process(target=ball_track, args=(key1, queue))
    p2 = mp.Process(target=servo_control, args=(key2, queue))
    p1.start()
    p2.start()
    p1.join()
    p2.join()
