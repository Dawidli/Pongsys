import multiprocessing as mp
from multiprocessing import Queue
import cvzone
from cvzone.ColorModule import ColorFinder
import cv2
import serial
import math
import numpy as np
from tkinter import *
import imutils
import time
from scipy.signal import butter, lfilter, freqz

#define servo angles and set a value
servo1_angle = 0
servo2_angle = 0
servo3_angle = -6.3
all_angle = 0

time_array = [time.time()]*2

platform_angle = 0 # intial
delta_t = 1 / 100
velocity = [0] # initial verdi
pos_x = [0, 0]
pos_y = [0, 0]
K = [0.1, 0] #very cool stabiliet

x = [0.0]*4 # tom y[]
y = [0.0]*4 # tom y[]
x_avg = [0.0]*4
y_avg = [0.0]*4
speed = [0.0]*2
counter = 0
grense = len(x)
distance_error = [0.0, 0.0]
fps = 30
sample_time = 1/fps # fps

# Setting standard filter requirements.
order = 6
fs = 17.0
cutoff = 1.78


def ball_track(key1, queue):
    camera_port = 0
    cap = cv2.VideoCapture(camera_port, cv2.CAP_DSHOW)

    cap.set(3, 960)
    cap.set(4, 480)
    cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)  # turn the autofocus off


    get, img = cap.read()
    h, w, _ = img.shape

    if key1:
        print('Ball tracking is initiated')

    myColorFinder = ColorFinder(False)  # if you want to find the color and calibrate the program we use this *(Debugging)
    hsvVals = {'hmin': 30, 'smin': 30, 'vmin': 65, 'hmax': 60, 'smax': 130, 'vmax': 255}


    center_point = [452, 275, 1090] # center point of the plate, calibrated



    while True:
        get, img = cap.read()
        rotated = imutils.rotate(img, 7)
        imgColor, mask = myColorFinder.update(rotated, hsvVals)
        imgContour, countours = cvzone.findContours(rotated, mask)

        #fps = img.get(cv2.cv.CV_CAP_PROP_FPS)
        #print(fps)
        if countours:
            data = (countours[0]['center'][0] - center_point[0]) / 10, \
                   (h - countours[0]['center'][1] - center_point[1]) / 10, \
                   int(countours[0]['area'] - center_point[2])/100
            queue.put(data)

        else:
            data = 'nil' # returns nil if we cant find the ball
            queue.put(data)


        imgStack = cvzone.stackImages([imgContour], 1, 1)
        # imgStack = cvzone.stackImages([img,imgColor, mask, imgContour],2,0.5) #use for calibration and correction
        cv2.imshow("Image", imgStack)

        cv2.waitKey(1)


def servo_control(key2, queue):

    port_id = 'COM3'
    # initialise serial interface
    arduino = serial.Serial(port=port_id, baudrate=250000, timeout=0.1)
    if key2:
        print('Servo controls are initiated')

    def butter_lowpass(cutoff, fs, order=5):
        nyq = 0.5 * fs
        normal_cutoff = cutoff / nyq
        b, a = butter(order, normal_cutoff, btype='low', analog=False)
        return b, a

    def butter_lowpass_filter(data, cutoff, fs, order=5):
        b, a = butter_lowpass(cutoff, fs, order=order)
        y = lfilter(b, a, data)
        return y

    def find_average(x_matrise, y_matrise, x_cord, y_cord, x_avg, y_avg):
        global counter
        x_tot = 0.0
        y_tot = 0.0
        counter = counter % grense
        x_matrise[counter] = x_cord
        y_matrise[counter] = y_cord
        for I in range(len(x_matrise)):
            x_tot += x_matrise[I]
            y_tot += y_matrise[I]
        x_avg[counter] = (x_tot / len(x_matrise)) * 1.277
        y_avg[counter] = (y_tot / len(y_matrise)) * 1.277


    def find_speed(avg):
        global counter
        speed = (avg[counter] - avg[counter-1]) / (sample_time)

        return speed

    def all_angle_assign(angle_passed1, angle_passed2, angle_passed3):
        global servo1_angle, servo2_angle, servo3_angle
        servo1_angle = float(angle_passed1)
        servo2_angle = float(angle_passed2)
        servo3_angle = float(angle_passed3)
        write_servo()

    root = Tk()
    root.resizable(0, 0)

    def PDreg(x_error, y_error):
        global servo_values, pos_y, pos_x, distance_error, K, speed, counter, platform_angle

        Regulator_values = [0] * 2
        distance_error[1] = y_error[counter]
        distance_error[0] = x_error[counter]
        for i in range(2):
            d = 20.2 if i == 0 else 17.5

            # regulator
            platform_angle = math.radians(-K[0] * distance_error[i]) - (K[1] * speed[i])

            # converts platform angle to servo angles and sends away
            motor_angle = np.arcsin((d * np.sin(platform_angle)) / (2 * 4))
            Regulator_values[i] = motor_angle  # indeks 0 er pitch og indeks 1 er roll
        servo_values = [-Regulator_values[0] + (0.549 * Regulator_values[1]),
                        Regulator_values[0] + (0.549 * Regulator_values[1]),
                        -Regulator_values[1]]

    def writeCoord():
        global counter
        corrd_info = queue.get()

        if corrd_info == 'nil':
            return
        else:
            find_average(x, y, corrd_info[0], corrd_info[1], x_avg, y_avg)

            # Filtering and plotting
            x_filtered = butter_lowpass_filter(x, cutoff, fs, order)
            y_filtered = butter_lowpass_filter(y, cutoff, fs, order)

            speed[1] = find_speed(y_filtered)
            speed[0] = find_speed(x_filtered)
            PDreg(x_filtered, y_filtered)
            counter += 1
            print('x=', speed[0])
            print('y=', speed[1])
            print()

            all_angle_assign(servo_values[0], servo_values[1], servo_values[2])

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

        angles: tuple = (round(ang1, 5),  # orginalt 1 komma
                         round(ang2, 5),
                         round(ang3, 5))

        write_arduino(str(angles))

    while key2:
        time_array[1] = time_array[0]
        time_array[0] = time.time()

        writeCoord()

        wait = sample_time - (time_array[0] - time_array[1])
        #time.sleep(wait)

    root.mainloop()  # running loop


if __name__ == '__main__':
    queue = Queue()  # The queue is done inorder for the communication between the two processes.
    key1 = 1  # just two dummy arguments passed for the processes
    key2 = 2
    p1 = mp.Process(target=ball_track, args=(key1, queue))  # initiate ball tracking process
    p2 = mp.Process(target=servo_control, args=(key2, queue))  # initiate servo controls
    p1.start()
    p2.start()
    p1.join()
    p2.join()
