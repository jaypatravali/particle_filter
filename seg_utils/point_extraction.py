import numpy as np
import json
import os, sys, getopt
import glob
import utm  
import math
import cv2

""" Sample JSON Structure"""
# {
#     "gpsHeading": 49, 
#     "gpsLatitude": 50.11227216768544, 
#     "gpsLongitude": 8.667227508165439, 
#     "outsideTemperature": 28.5, 
#     "speed": 11.105500000000001, 
#     "yawRate": 0.007523296908567134
# }

# {
#     "extrinsic": {
#         "baseline": 0.222126, 
#         "pitch": 0.05, 
#         "roll": 0.0, 
#         "x": 1.7, 
#         "y": -0.1, 
#         "yaw": 0.007, 
#         "z": 1.18
#     }, 
#     "intrinsic": {
#         "fx": 2268.36, 
#         "fy": 2225.5405988775956, 
#         "u0": 1048.64, 
#         "v0": 519.277
#     }
# }


test_output = '/home/patraval/caffe/models/adapnet/example_outputs/Network output_screenshot_2.png'

test_disparity = '/export/patraval/cityscapes/disparity_trainvaltest/disparity/val/frankfurt/frankfurt_000000_016005_disparity.png'


file  = 'frankfurt_000000_009561_camera.json'
folder  = '/home/patraval/cityscapes/camera_trainvaltest/camera/val/frankfurt/'
vehicle_json  = folder+file

with open(vehicle_json) as json_data:
    vehicle_data = json.load(json_data)


## Camera Data parser



file  = 'frankfurt_000000_009561_camera.json'
folder  = '/home/patraval/cityscapes/camera_trainvaltest/camera/val/frankfurt/'
cam_json  = folder+file

with open(cam_json) as json_data:
    cam_data = json.load(json_data)





gpsHeading  = 49
gpsLatitude  =  50.11227216768544
gpsLongitude =  8.667227508165439

base = 0.222126
cu   = 1048.64
cv   = 519.277
f    = 2268.36
d    = 14949

seg_pix_X = 537
seg_pix_Y = 369

X  = (seg_pix_X - cu)* base/d
Y =  (seg_pix_Y - cv)* base/d;
Z = f*base/d;


gpsHeading  = 49
gpsLatitude  =  50.09363349723325
gpsLongitude =  8.652660598637606

base = 0.222126
cu   = 1048.64
cv   = 519.277
f    = 2268.36
d    = 6876/256

seg_pix_X = 1317
seg_pix_Y = 285

X  = (seg_pix_X - cu)* base/d
Y =  (seg_pix_Y - cv)* base/d;
Z = f*base/d;



#Camera to World Transformation

pitch =0.05 
roll = 0.0
x =  1.7 
y  = 0.1
yaw =  0.007 
z =  1.18


c_y = np.cos(yaw);
s_y = np.sin(yaw);

c_p = np.cos(pitch);
s_p = np.sin(pitch);

c_r = np.cos(roll);
s_r = np.sin(roll); 


# camera to vehicle rotation
R = np.array([[  c_y * c_p,  c_y * s_p * s_r - s_y*c_r,      c_y*s_p*c_r + s_y * s_r, x ], 
[ s_y * c_p,  s_y * s_p * s_r + c_y * c_r,    s_y*s_p*c_r - c_y * s_r, y], 
[- s_p,      c_p * s_r,                      c_p * c_r, z], [0,0,0,1]])



C = np.array([ X,  Y, Z, 1])

VL = np.dot(R,C)

print(VL)


