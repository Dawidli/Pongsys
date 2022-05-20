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

# -------------------------------------------Both programs(Servo Control and Ball Tracker) in one -------------------------------------------
"""
For running both programs simultaneously we can use multithreading or multiprocessing
"""

#define servo angles and set a value
servo1_angle = 0
servo2_angle = 0
servo3_angle = -6.3
all_angle = 0
# Set a limit to upto which you want to rotate the servos (You can do it according to your needs)
"kamera greier"
width = 960
heigth = 480
circle_test = np.zeros((480,960,3), np.uint8)
circle_test[:,:] = (255,255,255)

center_coordinates = (427, 240)
radius = 500
color = (200, 0, 0)
thickness = 200
image = cv2.circle(circle_test, center_coordinates, radius, color, thickness)

"FOR LAVPASS FILTER / BÅNDBEGRENSET DERIVASJONS FILTER KAN VI GÅ PÅ 12_DTFT OG PRAKSIS FILTER DESIGN"

time_array = [time.time()]*2

platform_angle = 0 # intial
delta_t = 1 / 100
velocity = [0] # initial verdi
pos_x = [0, 0]
pos_y = [0, 0]
K = [1, 0.00345]

wc = 20 #hz
Ts = 1/30

b0 = wc/(wc*Ts + 1)
b1 = - b0
a0 = 1
a1 = -(1/(wc*Ts + 1))

global counter
x = [0.0]*10 # tom y[]
x_avg = [0.0]*10
y = [0.0]*10 # tom y[]
y_avg = [0.0]*10
speed = [0.0]*2
fps = 30
sample_time = 1/fps # fps
counter = 0
grense = len(x)
distance_error = [0.0, 0.0]


def ball_track(key1, queue):
    camera_port = 0
    cap = cv2.VideoCapture(camera_port,cv2.CAP_DSHOW)

    cap.set(3, 960)
    cap.set(4, 480)
    cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)  # turn the autofocus off


    get, img = cap.read()
    h, w, _ = img.shape

    if key1:
        print('Ball tracking is initiated')

    myColorFinder = ColorFinder(False)  # if you want to find the color and calibrate the program we use this *(Debugging)
    hsvVals = {'hmin': 0, 'smin': 0, 'vmin': 240, 'hmax': 180, 'smax': 15, 'vmax': 255}

    center_point = [626, 337, 2210] # center point of the plate, calibrated


    while True:
        get, img = cap.read()
        rotated = imutils.rotate(img, 7)
        #filter = cv2.addWeighted(rotated, 0.5, circle_test, 0.5, 0)
        #filter = np.concatenate((rotated, circle_test), axis = 0)
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

    def find_average(matrise, input, avg):
        global counter

        tot = 0.0
        counter = counter % grense
        matrise[counter] = input
        for I in range(len(matrise)):
            tot += matrise[I]
        avg[counter] = (tot / len(matrise))
        # print(avg)
        counter += 1

    def find_speed(avg, fix):
        speed = (avg[counter - 1 - fix] - avg[counter - 3 - fix]) * 0.625 / (sample_time)
        return speed

    def all_angle_assign(angle_passed1,angle_passed2,angle_passed3):
        global servo1_angle, servo2_angle, servo3_angle
        servo1_angle = float(angle_passed1)
        servo2_angle = float(angle_passed2)
        servo3_angle = float(angle_passed3)
        write_servo()

    root = Tk()
    root.resizable(0, 0)

    def timerfunksjon():
        time_array[1] = time_array[0]
        time_array[0] = time.time()
        cycle_time = (time_array[0] - time_array[1]) / 1000
        return cycle_time

    def Preg(xverdi, yverdi):
        global servo_values

        if xverdi == 'n' and yverdi == 'i':
            return
        else:
            Preg_values = [0] * 2

            for i in range(2):
                dist = yverdi if i == 0 else xverdi
                cord_ratio = dist / 28
                d = 17.5 if i == 0 else 20
                platform_angle = math.radians((cord_ratio * 105) / (7))
                motor_angle = np.arcsin((d * np.sin(platform_angle)) / (2 * 4))
                Preg_values[i] = -motor_angle # indeks 0 er pitch og indeks 1 er roll
            servo_values = [Preg_values[0] - Preg_values[1], Preg_values[0] + Preg_values[1], -Preg_values[0]]
            #print("Servo 1: ",round(servo_values[0],3),"Servo 2: ",round(servo_values[1],3),"Servo 3: ",round(servo_values[2],3))
        # have to fix a min/max regulation for servo_values ;)

    def PDreg(x_error, y_error):
        global servo_values, pos_y, pos_x, distance_error, K, speed, counter, platform_angle


        Regulator_values = [0] * 2
        distance_error[1] = y_error[counter - 1] * 0.625
        distance_error[0] = x_error[counter - 2] * 0.625
        for i in range(2):

            d = 19.5 if i == 0 else 18.08

            # regulator
            platform_angle = math.radians(-K[0] * distance_error[i]) - (K[1] * speed[i-1])
            #print(platform_angle)
            # converts platform angle to servo angles and sends away
            motor_angle = np.arcsin((d * np.sin(platform_angle)) / (2*4))
            Regulator_values[i] = motor_angle  # indeks 0 er pitch og indeks 1 er roll
        servo_values = [-Regulator_values[0] + (0.549 * Regulator_values[1]), Regulator_values[0] + (0.549 * Regulator_values[1]),
                            -Regulator_values[1]]


    def writeCoord():
        """
        Here in this function we get both coordinate and servo control, it is an ideal place to implement the controller
        """
        corrd_info = queue.get()
        #print(corrd_info)
        if corrd_info == 'nil':
            return
        else:
            find_average(x, corrd_info[0], x_avg)
            find_average(y, corrd_info[1], y_avg)
            speed[0] = find_speed(y_avg, 0)
            speed[1] = find_speed(x_avg, 1)
            PDreg(x_avg, y_avg)
            #Preg(corrd_info[0], corrd_info[1])

            #print('servo 1:', servo_values[1],'servo 2:', servo_values[2],'servo 3', servo_values[0])
            all_angle_assign(servo_values[0], servo_values[1], servo_values[2])


            wait = sample_time - timerfunksjon()
            #time.sleep(wait)



    def write_arduino(data):
        #print('The angles send to the arduino : ', data)

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

        angles: tuple = (round(ang1, 5), #orginalt 1 komma
                         round(ang2, 5),
                         round(ang3, 5))

        write_arduino(str(angles))

    while key2:
        writeCoord()

    root.mainloop()  # running loop

if __name__ == '__main__':

    queue = Queue() # The queue is done inorder for the communication between the two processes.
    key1 = 1 # just two dummy arguments passed for the processes
    key2 = 2
    p1 = mp.Process(target= ball_track, args=(key1, queue)) # initiate ball tracking process
    p2 = mp.Process(target=servo_control,args=(key2, queue)) # initiate servo controls
    p1.start()
    p2.start()
    p1.join()
    p2.join()


