import numpy as np
import math
import matplotlib.pyplot as plt
import utm

x= [413058.940761621,  413210.76645718445, 413038.1568685182, 412838.7656277865]
y = [5318228.741414887, 5318363.85196627, 5318629.931794832, 5318476.536999348]

ox = []
oy = []

for i in range(len(x)):	
	ox.append(x[i] - x[0])
	oy.append(y[i] - y[0])


print("yo", y[1]/x[1], oy[1]/ox[1])

angle = math.atan(oy[1]/ox[1])
angle =  -angle
c = np.cos(angle)
s = np.sin(angle)

print(angle)

limitsx = [] 
limitsy = []

# Limits
for i in range(len(x)):
	limitsx.append(ox[i]*c - oy[i]*s)
	limitsy.append(ox[i]*s + oy[i]*c)


limitsx.append(ox[0])
limitsy.append(oy[0])

# Landmarks

filename = '../map/landmarks.txt' 

landmarks = dict()
landmarks_latlon = dict()

f = open(filename)

lmx = []
lmy = []

for line in f:
	line_s  = line.split('\n')
	line_spl  = line_s[0].split()
	lx, ly, _, _ = utm.from_latlon(float(line_spl[1]),float(line_spl[2]))
	lmx.append(lx- x[0])
	lmy.append(ly- y[0])
	landmarks[int(line_spl[0])] = [lx- x[0], ly- y[0]]      
	landmarks_latlon[int(line_spl[0])] = [float(line_spl[1]),float(line_spl[2])]      


for key in landmarks:
	tempx = landmarks[key][0]*c - landmarks[key][1]* s
	tempy = landmarks[key][0]*s + landmarks[key][1]* c
	landmarks[key] = [tempx, tempy]



# Odom

filename = '../map/odom_latlon.txt' 

odom = dict()

f = open(filename)

odomx = []
odomy = []

odom_xl = []
odom_yl = []

for line in f:
	line_s  = line.split('\n')
	line_spl  = line_s[0].split()
	odx, ody, _, _ = utm.from_latlon(float(line_spl[0]),float(line_spl[1]))
	odomx.append(odx- x[0])
	odomy.append(ody- y[0])
	odom_xl.append(float(line_spl[0]))
	odom_yl.append(float(line_spl[1]))

rLmx = []
rLmy = []

rOdomx = []
rOdomy = []

### rotation of Odom and landmarks
for i in range(len(lmx)):
	rLmx.append(lmx[i]*c - lmy[i]*s)
	rLmy.append(lmx[i]*s + lmy[i]*c)

for i in range(len(odomx)):
	rOdomx.append(odomx[i]*c - odomy[i]*s)
	rOdomy.append(odomx[i]*s + odomy[i]*c)

f = open("../map/odom_trajectory.dat",'w') 


for i in range(len(rOdomy)):
	odom_read = "{} {}\n".format(rOdomx[i],rOdomy[i])
	f.write(odom_read)



plt.plot(rLmx,rLmy,  marker='o', markersize=3, color="red")
plt.plot(rOdomx,rOdomy,  marker='x', markersize=3, color="green")

plt.plot(limitsx,limitsy,  marker='.', markersize=3, color="black")


plt.show()
plt.ion()


def euclidean_dist(p1, p2):
	return(np.sqrt( (p1[0] - p2[0])**2 + (p1[1] - p2[1])**2 ))

sensor_range  = 20


#Range Measurement


f = open("../map/sensor_data_sim.dat",'w') 

trans =  []
trans.append(0.01)
theta = 1.2490457723982544
delta_rot1 = []
delta_rot1.append(theta)
heading = []
heading.append(theta)

for i in range(len(rOdomx)-1):
	trans.append(euclidean_dist((rOdomx[i],rOdomy[i]),(rOdomx[i+1],rOdomy[i+1])))
	del_rot = math.atan2(rOdomy[i+1]- rOdomy[i], rOdomx[i+1]- rOdomx[i]) - theta
	delta_rot1.append(del_rot)
	theta = math.atan2(rOdomy[i+1]- rOdomy[i], rOdomx[i+1]- rOdomx[i])
	heading.append(theta)

for i in delta_rot1:
	print(i, len(delta_rot1), (i*360/(2*math.pi)))


track_length = 0
for i in range(len(rOdomx)):
	odom_read = "ODOMETRY {} {} {}\n".format(delta_rot1[i],trans[i], 0)
	track_length = trans[i] + track_length
	f.write(odom_read)
	for key in landmarks:
		range_val = euclidean_dist((rOdomx[i],rOdomy[i]), (landmarks[key][0],landmarks[key][1]))
		if (range_val<sensor_range):
			bearing = math.atan2(landmarks[key][1]- rOdomy[i], landmarks[key][0] - rOdomx[i]) - heading[i]
			sensor_read = "SENSOR {} {} {}\n".format(key,range_val,bearing)

			f.write(sensor_read)

print("track_length",track_length/1000)

f = open("../map/landmarks_sim.dat",'w') 

for key in landmarks:
	landmarks_coords = "{} {} {}\n".format(key, landmarks[key][0], landmarks[key][1])
	f.write(landmarks_coords)
