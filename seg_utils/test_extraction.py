import numpy as np
import json
import os, sys, getopt
import glob
import utm  
import math
import cv2
from matplotlib import pyplot as plt

test_output1 = '/home/patraval/caffe/models/adapnet/example_outputs/Network output_screenshot_2.png'
test_output = '/home/patraval/caffe/models/adapnet/example_outputs/Label output_screenshot_21.09.2017.png'
######## oBJECT eXTRACTION

img = cv2.imread(test_output1)
cv2.imshow('image',img)
cv2.waitKey(0)

# clone_img = cv2.copy.copy(img)


imgray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

cv2.imshow('image',imgray)
cv2.waitKey(0)

# ret, poles = cv2.threshold(imgray,255,0,0)

# cv2.imshow('image',poles)
# cv2.waitKey(0)
cv2.blur( imgray, (3,3) );

edges = cv2.Canny(imgray,20,200)


cv2.imshow('edges',edges)
cv2.waitKey(0)

# plt.subplot(121),plt.imshow(imgray,cmap = 'gray')
# plt.title('Original Image'), plt.xticks([]), plt.yticks([])
# plt.subplot(122),plt.imshow(edges,cmap = 'gray')
# plt.title('Edge Image'), plt.xticks([]), plt.yticks([])
# plt.show()

_, contours, hierarchy = cv2.findContours(edges,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)


cnts = sorted(cnts, key = cv2.contourArea, reverse = True)[:10]


cv2.drawContours(new, contours, -1, (0,122,0), 1)


cv2.imshow('contours',img)
cv2.waitKey(0)

# cnt = contours[4]



# cv2.drawContours(img, [cnt], 0, (0,255,0), 3)
# 	