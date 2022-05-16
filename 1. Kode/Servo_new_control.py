import serial
import math
from tkinter import *

SERVO_HORN_LENGTH = 4.0  # lengths of servohornarm, in cm
PLAT_CONNECTOR_RADIUS = 13  # distance from center of platform to magnetmount point, in cm
PLATFORM_SIDE_LENGTH = math.sqrt(
    3) * PLAT_CONNECTOR_RADIUS  # sidelengths of the equilateral triangle made by the three connection points

port = '/dev/cu.usbmodem143101'
arduino = serial.Serial(port=port, baudrate=250000, timeout=0.1)

MAX_TILT = math.radians(10)  # max tilt-angle of platform, converted from deg to rads
MIN_TILT = -MAX_TILT  # min tilt-angle of platform, negative of the maximum tilt-angle
MAX_HEAVE = 2  # maximum heave, in cm

controller: tuple = (0, 0, 0)

pitch1 = 0
roll1 = 0
heave1 = 0

calibration_pitch = -1
calibration_roll = 0
calibration_heave = 0



def pitch_assign(a):
    global pitch1
    pitch1 = math.radians(float(a)+calibration_pitch)
    write_servo()


def roll_assign(b):
    global roll1
    roll1 = math.radians(float(b)+calibration_roll)
    write_servo()


def heave_assign(c):
    global heave1
    heave1 = math.radians(float(c)+calibration_heave)
    write_servo()



root = Tk()
root.resizable(0,0)

w2 = Label(root, justify= LEFT, text="Control")
w2.config(font=("Elephant",30))
w2.grid(row=3, column=0, columnspan=2, padx=100)

S1Lb = Label(root, text="pitch")
S1Lb.config(font=("Elephant", 15))
S1Lb.grid(row=5, column=0, pady=10)

S2Lb = Label(root, text="roll")
S2Lb.config(font=("Elephant", 15))
S2Lb.grid(row=10, column=0, pady=10)

S3Lb = Label(root, text="heave")
S3Lb.config(font=("Elephant", 15))
S3Lb.grid(row=15, column=0, pady=10)

servo1=Scale(root, from_=-14, to=14, orient=HORIZONTAL,resolution=0.1,length = 400, command =pitch_assign)
servo1.grid(row=5, column = 1)

servo2=Scale(root, from_=-16, to=16, orient=HORIZONTAL,resolution=0.1,length = 400, command =roll_assign)
servo2.grid(row=10, column = 1)

servo3=Scale(root, from_=0, to=90, orient=HORIZONTAL,resolution=0.1,length = 400, command =heave_assign)
servo3.grid(row=15, column = 1)



def platAngleToServo(pitch, roll, heave):
    point1 = (((math.sqrt(3) * PLATFORM_SIDE_LENGTH) / 6) * math.sin(pitch) * math.cos(roll)
              + (PLATFORM_SIDE_LENGTH / 2) * math.sin(roll) - heave)
    point2 = (((math.sqrt(3) * PLATFORM_SIDE_LENGTH) / 6) * math.sin(pitch) * math.cos(roll)
              - (PLATFORM_SIDE_LENGTH / 2) * math.sin(roll) - heave)
    point3 = (((-math.sqrt(3) * PLATFORM_SIDE_LENGTH) / 3) * math.sin(pitch) * math.cos(roll) - heave)

    angles: tuple = (round(math.degrees(math.asin(point1 / SERVO_HORN_LENGTH)), 1),
                     round(math.degrees(math.asin(point2 / SERVO_HORN_LENGTH)), 1),
                     round(math.degrees(math.asin(point3 / SERVO_HORN_LENGTH)), 1))
    return angles

def writeArduino(data):
    print(data)
    arduino.write(bytes(data, 'utf-8'))

def write_servo():
    ang1 = pitch1
    ang2 = roll1
    ang3 = heave1
    controller: tuple = (ang1,ang2,ang3)
    servoValues = platAngleToServo(*controller)
    #print(str(servoValues))
    writeArduino(str(servoValues))

root.mainloop()



