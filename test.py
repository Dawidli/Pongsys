import numpy as np
import cv2

width = 854
heigth = 480
circle_test = np.zeros((480,854,3), np.uint8)
circle_test[:,:] = (255,255,255)

center_coordinates = (427, 240)
radius = 500
color = (200, 0, 0)
thickness = -1
image = cv2.circle(circle_test, center_coordinates, radius, color, thickness)

cv2.imshow('penis', image)
cv2.waitKey(0)
cv2.destroyWindow()