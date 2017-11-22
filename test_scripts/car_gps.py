import numpy as np
import math
import utm
import gmplot


def plot_car_on_maps(lm_lat, lm_lon, odom_lat, odom_lon):

	gmap = gmplot.GoogleMapPlotter(48.013497, 7.834188, 15)
	# Odom
	gmap.plot(odom_lat, odom_lon, 'green', edge_width=3)
	gmap.draw("../results/gps_odom.html")


def write_landmarks_data():
f = open("../map/landmarks_sim.dat",'w') 

for key in landmarks:
	landmarks_coords = "{} {} {}\n".format(key, landmarks[key][0], landmarks[key][1])
	f.write(landmarks_coords)


def write_sensor_data():
	f = open("../map/sensor_data_sim.dat",'w') 
	trans =  []
	trans.append(0.01)
	theta = 1.2490457723982544
	delta_rot1 = []
	delta_rot1.append(theta)
	for i in range(len(rOdomx)-1):
		trans.append(euclidean_dist((rOdomx[i],rOdomy[i]),(rOdomx[i+1],rOdomy[i+1])))
		del_rot = math.atan2(rOdomy[i+1]- rOdomy[i], rOdomx[i+1]- rOdomx[i]) - theta
		delta_rot1.append(del_rot)
		theta = math.atan2(rOdomy[i+1]- rOdomy[i], rOdomx[i+1]- rOdomx[i])

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
				bearing = math.atan2(landmarks[key][1]- rOdomy[1], landmarks[key][0] - rOdomx[0])
				sensor_read = "SENSOR {} {} {}\n".format(key,range_val,bearing)
				f.write(sensor_read)

def process_car_data(odom_lat, odom_lon, lm_lat, lm_lon):


def parse_car_data(car_odom_file, landmarks_file, sensor_file):
	f = open(car_odom_file)
	odom_lat = []
	odom_lon = []
	for line in f:
	    line_s  = line.split('\n')
	    line_spl  = line_s[0].split()
	    lat, lon = float(line_spl[1]),float(line_spl[2])
	    odom_lat.append(lat)
	    odom_lon.append(lon)


	landmarks_latlon = dict()
	f = open(landmarks_file)
	lm_lat = []
	lm_lon = []
	for line in f:
	    line_s  = line.split('\n')
	    line_spl  = line_s[0].split()
	    lat, lon = float(line_spl[1]),float(line_spl[2])
	    lm_lat.append(lat)
	    lm_lat.append(lon)


	landmarks = dict()

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



def main():
	car_odom_file = '/export/patraval/robo_car_images/pg_cam/gps/fix.txt' 
	landmarks_file = '../map/landmarks.txt'  
	sensor_file =  '/export/patraval/robo_car_images/pg_cam/sensor.txt' 
