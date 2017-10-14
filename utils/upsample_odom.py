import numpy as np
import math
filename = "../map/odom_trajectory.dat"

f = open(filename)

odom_xl = []
odom_yl = []

for line in f:
	line_s  = line.split('\n')
	line_spl = line_s[0].split()
	odx, ody = float(line_spl[0]),float(line_spl[1])
	odom_xl.append(odx)
	odom_yl.append(ody)


def euclidean_dist(p1, p2):
	return(np.sqrt( (p1[0] - p2[0])**2 + (p1[1] - p2[1])**2 ))


odom_upsampleX = []
odom_upsampleY = []


for i in range(0,32):
	x = (odom_xl[i]+odom_xl[i+1])/2
	y = (odom_yl[i] + odom_yl[i+1])/2

	odom_upsampleX.append(odom_xl[i])
	odom_upsampleY.append(odom_yl[i])

	odom_upsampleX.append(x)
	odom_upsampleY.append(y)	

for i in range(32,len(odom_xl)):
	odom_upsampleX.append(odom_xl[i])
	odom_upsampleY.append(odom_yl[i])	



filename = '../map/landmarks_local.dat' 

landmarks = dict()
f = open(filename)
for line in f:
	line_s  = line.split('\n')
	line_spl  = line_s[0].split()
	landmarks[int(line_spl[0])] = [float(line_spl[1]), float(line_spl[2])]


f = open("../map/odom_trajectory_upsample.dat",'w') 


for i in range(len(odom_upsampleX)):
	odom_read = "{} {}\n".format(odom_upsampleX[i],odom_upsampleY[i])
	f.write(odom_read)



f = open("../map/sensor_data_sim_upsample.dat",'w') 

trans =  []
trans.append(0.01)
theta = 1.2490457723982544
delta_rot1 = []
delta_rot1.append(theta)

sensor_range =20

for i in range(len(odom_upsampleX)-1):
	trans.append(euclidean_dist((odom_upsampleX[i],odom_upsampleY[i]),(odom_upsampleX[i+1],odom_upsampleY[i+1])))
	del_rot = math.atan2(odom_upsampleY[i+1]- odom_upsampleY[i], odom_upsampleX[i+1]- odom_upsampleX[i]) - theta
	delta_rot1.append(del_rot)
	theta = math.atan2(odom_upsampleY[i+1]- odom_upsampleY[i], odom_upsampleX[i+1]- odom_upsampleX[i])


for i in range(len(odom_upsampleX)):
	odom_read = "ODOMETRY {} {} {}\n".format(delta_rot1[i],trans[i], 0)
	f.write(odom_read)
	for key in landmarks:
		range_val = euclidean_dist((odom_upsampleX[i],odom_upsampleY[i]), (landmarks[key][0],landmarks[key][1]))
		if (range_val<sensor_range):
			sensor_read = "SENSOR {} {} {}\n".format(key,range_val,0)	
			f.write(sensor_read)