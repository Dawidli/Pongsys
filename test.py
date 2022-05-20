import cv2
import numpy as np

width = 1280
heigth = 720
circle_test = np.zeros((heigth,width,3), np.uint8)
circle_test[:,:] = (255,255,255)

center_coordinates = (610, 395)
radius = 800
color = (200, 0, 0)
thickness = 940
image = cv2.circle(circle_test, center_coordinates, radius, color, thickness)

cv2.imshow('test', image)
cv2.waitKey(0)