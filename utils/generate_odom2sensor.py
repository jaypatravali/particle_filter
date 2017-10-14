#Script to generate sensor readings for a odom measurement 
import numpy as np


# def transformation():
# """	Does transform with respect to origin coorratinates,
# put flag for bearing
# """


# def euclidean_distance(landmark, odom):

# 	return(np.sqrt( (landmark[0] - odom[0])**2 + (landmark[1] - odom[1])**2 ))

f = open("sensor_data_sim.dat",'w') 


for i in range(odom):
	odom_read = "ODOMETRY {} {}".format(UID,x,y)
	f.write(sensor_read,"\n")
	for i in range(landmarks):
		#call euclidean and check for landmark distance
		sensor_read = "SENSOR {} {} {}".format(UID,dist,bearing)	
		f.write(sensor_read,"\n")
