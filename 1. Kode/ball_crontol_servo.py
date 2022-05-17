import multiprocessing as mp
from multiprocessing import Queue
import cvzone
from cvzone.ColorModule import ColorFinder
import cv2
import serial
import math
import numpy as np
from tkinter import *

# -------------------------------------------Both programs(Servo Control and Ball Tracker) in one -------------------------------------------
# -------------------------------------------Both programs(Servo Control and Ball Tracker) in one -------------------------------------------
"""
For running both programs simultaneously we can use multithreading or multiprocessing
"""

#define servo angles and set a value
servo1_angle = -6.3
servo2_angle = 0
servo3_angle = 0
all_angle = 0
# Set a limit to upto which you want to rotate the servos (You can do it according to your needs)
servo1_angle_limit_positive = -61
servo1_angle_limit_negative = 25.3

servo2_angle_limit_positive = -63.5
servo2_angle_limit_negative = 31.7

servo3_angle_limit_positive = -54.7
servo3_angle_limit_negative = 35.1


def ball_track(key1, queue):
    camera_port = 0
    cap = cv2.VideoCapture(camera_port,cv2.CAP_DSHOW)
    cap.set(3, 1280)
    cap.set(4, 720)

    get, img = cap.read()
    h, w, _ = img.shape

    if key1:
        print('Ball tracking is initiated')

    myColorFinder = ColorFinder(False)  # if you want to find the color and calibrate the program we use this *(Debugging)
    hsvVals = {'hmin': 0, 'smin': 0, 'vmin': 240, 'hmax': 180, 'smax': 15, 'vmax': 255}

    center_point = [626, 337, 2210] # center point of the plate, calibrated


    while True:
        get, img = cap.read()
        #Filtring unnecessary camera feed
        center_coordinates = (610, 395)
        radius = 800
        color = (200, 0, 0)
        thickness = 940
        image = cv2.circle(img, center_coordinates, radius, color, thickness)
        imgColor, mask = myColorFinder.update(image, hsvVals)
        imgContour, countours = cvzone.findContours(image, mask)

        if countours:
            data = round((countours[0]['center'][0] - center_point[0]) / 10), \
                   round((h - countours[0]['center'][1] - center_point[1]) / 10), \
                   round(int(countours[0]['area'] - center_point[2])/100)
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

    def all_angle_assign(angle_passed1,angle_passed2,angle_passed3):
        global servo1_angle, servo2_angle, servo3_angle
        servo1_angle = float(angle_passed1)
        servo2_angle = float(angle_passed2)
        servo3_angle = float(angle_passed3)
        write_servo()

    root = Tk()
    root.resizable(0, 0)

    # Husk å lage variabler for verdier som 28, 105 osv.
    # 28 = max koordinat
    # 105 er max akselerasjon på ballen
    # 7 er kun en kostant som aldri forandrer seg, la ligge
    # 4 er konstantlengde til l_horn, som er avstanden fra servo skrua til ytremutteren
    # 17.5 er distansen mellom servoene 1 og 2 til 3
    # 20 er distansen mellom hver og enkel servo

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
    def writeCoord():
        """
        Here in this function we get both coordinate and servo control, it is an ideal place to implement the controller
        """
        corrd_info = queue.get()
        if corrd_info == 'nil' :
            test = corrd_info[1]
        else:
            test = -corrd_info[1]
        Preg(test, corrd_info[0])
        all_angle_assign(servo_values[0], servo_values[1], servo_values[2])

    def write_arduino(data):
        #print('The angles send to the arduino : ', data)

        arduino.write(bytes(data, 'utf-8'))

    def write_servo():
        ang1 = servo1_angle
        ang2 = servo2_angle
        ang3 = servo3_angle

        angles: tuple = (round(math.degrees(ang1), 1),
                         round(math.degrees(ang2), 1),
                         round(math.degrees(ang3), 1))

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


