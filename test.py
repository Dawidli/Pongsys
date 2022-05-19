import numpy as np
import cv2
import matplotlib.pyplot as plt

# reads image 'opencv-logo.png' as grayscale
sirkel = cv2.imread('C:\\Users\\zaime\\Downloads\\sirkel.png', 0)
h,w = sirkel.shape
scale_w = 320/311
scale_h = 125/151
width = int(h * scale_h)
height = int(w * scale_w)
dim = (width, height)
resized = cv2.resize(sirkel, dim)
# show image
cv2.imshow('image', resized)
print(w,width,h,height)
cv2.waitKey(0)
cv2.destroyAllWindows()