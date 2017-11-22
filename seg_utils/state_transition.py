import numpy as np
import os, sys, getopt
import glob
import utm  
import math	
import cv2
import rospy
import tf 


class State_Transition():
	""" Convert Pixel Points to Real World Coordinates
	"""
	def __init__(self, odom_file, realtime= None):
		"""Initialize params"""
 		self.realtime = realtime
		self.camera_calib()
		self.param_init()
		self.file_parser(odom_file)

	def camera_calib(self):
		self.base = 0.12
		# self.cu = 674.03076171875 * 0.6
		# self.cv = (348.81689453125)* (384/640)
		# self.f = 698.2396240234375 *0.6
		self.cu = 674.03076171875
		self.cv = 348.81689453125
		self.f = 698.2396240234375 
		self.origin_x =  412940.751955
		self.origin_y =  5318560.37949
		print("Camera Calib Loaded")

	def param_init(self):
		self.header = []
		self.pos  = []
		self.orientation = []
		self.filter_radius = 0.9
		self.file = open("../map/sensor_data_car.dat",'w') 
		self.prev_odom = [177.03757970060997,72.711216426459998]
		self.car_orientation = 1.3540745849092297
		self.flag = 0
		if self.realtime is False:
			self.f1 = open("../map/odom_trajectory_car.dat",'w') 
		self.sensor_readings = dict()
		self.odom_readings  = dict()



	def file_parser(self, odom_file):
		odom_list = [line.rstrip('\n').split() for line in open(odom_file)]
		for i in odom_list:
			self.header.append([float(i[0]), float(i[1]), float(i[2])  , float(i[10])])
			self.pos.append((float(i[3]), float(i[4]), float(i[5])))
			self.orientation.append((float(i[6]), float(i[7]), float(i[8]), float(i[9]))) 
		print("Odometry Loaded")

	def euclidean_dist(self, p1, p2):
		return(np.sqrt( (p1[0] - p2[0])**2 + (p1[1] - p2[1])**2 ))

	def radius_filtering(self, Vehicle_coords):
		pop_index = []
		for i in range(len(Vehicle_coords)-1):
			for j in range(i+1,len(Vehicle_coords)):
				dist = self.euclidean_dist(Vehicle_coords[i][0:2], Vehicle_coords[j][0:2] )
				if dist < self.filter_radius: 
					index = np.argmax([Vehicle_coords[i][2], Vehicle_coords[j][2]])
					pop_index.append([i,j][index]) # Choose the lowest Z value
		return pop_index


	def vis_transform(self, landmark_coords, pos):

		transform_coords  = []
		angle = -0.727210063845
		c = np.cos(angle)
		s = np.sin(angle)

		tx  = self.origin_x + pos[0] - 413058.940761621
		ty  = self.origin_y + pos[1] - 5318228.741414887
		rx = tx*c - ty*s
		ry = tx*s + ty*c

		transform_pos = [rx,ry]

		# for i in range(len(landmark_coords)):
		# 	tx  = self.origin_x + landmark_coords[i][0] 
		# 	ty  = self.origin_y + landmark_coords[i][1]
		# 	local_x, local_y = utm.to_latlon(tx,ty, 32, 'U')
		# 	print("GPS for ", "is ", local_x, local_y )


		for i in range(len(landmark_coords)):
			tx  = self.origin_x + landmark_coords[i][0] - 413058.940761621
			ty  = self.origin_y + landmark_coords[i][1] - 5318228.741414887
			rx = tx*c - ty*s
			ry = tx*s + ty*c
			transform_coords.append([rx,ry])

		return transform_pos, transform_coords

	def odom_compute(self, landmark_coords, landmark_bearing, pos):

		pos, landmark_coords = self.vis_transform(landmark_coords, pos)
		# print(pos)

		trans = self.euclidean_dist((pos[0], pos[1]),(self.prev_odom[0],  self.prev_odom[1]))
		relative_lm_heading  = math.atan2(pos[1] - self.prev_odom[1], pos[0] - self.prev_odom[0])
		print("relative_lm_heading", relative_lm_heading)
		# print("relative_lm_heading", relative_lm_heading)

		del_rot = relative_lm_heading - self.car_orientation

		odom_read = "{} {}\n".format(pos[0], pos[1])
		self.f1.write(odom_read)

		# if(self.flag == 0):
		# 	self.flag = 1
		# 	del_rot = self.car_orientation


		# print("del_rot", del_rot)
		odom_read = "ODOMETRY {} {} {}\n".format(del_rot,trans, 0)
		self.file.write(odom_read)
		for val,angle in zip(landmark_coords, landmark_bearing):
			range_val = self.euclidean_dist((pos[0], pos[1]), ( val[0], val[1]))
			bearing = angle
			# print(val, range_val, bearing)

			sensor_read = "SENSOR {} {} {}\n".format(0,range_val, bearing)
			self.file.write(sensor_read)
			# print(val)
		self.car_orientation  = relative_lm_heading
		self.prev_odom = pos



	def odom_compute_realtime(self, landmark_coords, landmark_bearing, pos):
		lm_ids =[]
		ranges=[]
		bearings=[]

		pos, landmark_coords = self.vis_transform(landmark_coords, pos)
		trans = self.euclidean_dist((pos[0], pos[1]),(self.prev_odom[0],  self.prev_odom[1]))
		relative_lm_heading  = math.atan2(pos[1] - self.prev_odom[1], pos[0] - self.prev_odom[0])

		del_rot = relative_lm_heading - self.car_orientation

		for val,angle in zip(landmark_coords, landmark_bearing):
			range_val = self.euclidean_dist((pos[0], pos[1]), ( val[0], val[1]))
			bearing = angle

			lm_ids.append(0)    
			ranges.append(range_val)
			bearings.append(bearing)

		self.odom_reading = [pos[0], pos[1]]      
		self.sensor_readings['odometry'] = {'r1':del_rot,'t':trans,'r2':0}
		self.sensor_readings['sensor'] = {'id':lm_ids,'range':ranges,'bearing':bearings}            

		self.car_orientation  = relative_lm_heading
		self.prev_odom = pos
		print(self.sensor_readings, self.odom_reading)

	def point_transformation(self, points, seq):
		Vehicle_coords = []
		bearing  = [] 
		self.vehicle_coords_base = []
		for i in range(len(points)):

			seg_pix_X, seg_pix_Y, d = points[i]
			# d_res = d* (0.6)
			
			#Pixel Coordinates to Camera Transformation
			X  =  (seg_pix_X - self.cu )* self.base/d
			Y =   (seg_pix_Y - self.cv )* self.base/d
			Z =   self.f*self.base/d

			Landmarks_Camera =  np.array([ X, Y, Z, 1])

			# Pinhole Camera Coordinates to Robot Camera [Static Transform: Camera to zed_front]
			tranform_matrix  = tf.TransformerROS()
			R_t= tranform_matrix.fromTranslationRotation((1.061, -0.157, 1.372), (-0.484, 0.484, -0.516, 0.516))
			Landmark_Vehicle = np.dot(  R_t, Landmarks_Camera)

			# print("Landmarks_Camera: ", Landmarks_Camera)
			# print("Landmark_Vehicle: ", int(d*256),  Landmark_Vehicle)

			# Robot Camera Coordinates to World [odom_combined to base_link]
			R_t2= tranform_matrix.fromTranslationRotation( self.pos[seq], self.orientation[seq])
			Landmark_World = np.dot(  R_t2, Landmark_Vehicle)

			rtx = Landmark_World[0] + self.origin_x
			rty = Landmark_World[1] + self.origin_y

			Car_x = self.origin_x + self.pos[seq][0]
			Car_y = self.origin_y + self.pos[seq][1]

			local_c_x, local_c_y = utm.to_latlon(Car_x,Car_y, 32, 'U')
			# print(" Car GPS for ", int(d*256) , "is ", local_c_x, local_c_y)
			local_x, local_y = utm.to_latlon(rtx,rty, 32, 'U')
			# print("GPS for ", int(d*256) , "is ", local_x, local_y )
			# print("\n")
			# print("bearing1", math.atan2(Landmark_Vehicle[1],Landmark_Vehicle[0]))
			bearing.append( math.atan2(Landmark_Vehicle[1],Landmark_Vehicle[0]))

			# print("range", np.sqrt( (Landmark_Vehicle[1]**2) + (Landmark_Vehicle[0]**2) ))
			Landmark_Vehicle_odom = [Landmark_Vehicle[0]+ self.pos[seq][0], Landmark_Vehicle[1]+ self.pos[seq][1] , Landmark_Vehicle[2] + self.pos[seq][2] ]
			Vehicle_coords.append( Landmark_Vehicle_odom )
			# print(i, d, points[i])
			self.vehicle_coords_base.append(Landmark_Vehicle)

		self.pop_index = self.radius_filtering(Vehicle_coords)
		# print(sorted(self.pop_index))

		filter_coords = []	
		filter_bearing = []
		for i in range(len(Vehicle_coords)):
			if i not in self.pop_index:
				filter_coords.append(Vehicle_coords[i])
				filter_bearing.append(bearing[i])

		if self.realtime is True:
			self.odom_compute_realtime( filter_coords, filter_bearing, self.pos[seq])	
		else:
			self.odom_compute( filter_coords, filter_bearing, self.pos[seq])
		


# angle = math.atan2(Landmark_Vehicle[1],Landmark_Vehicle[0])
# angle =  - angle  + heading
# c = np.cos(angle)
# s = np.sin(angle)

# rx = Landmark_Vehicle[0]*c - Landmark_Vehicle[1]*s
# ry = Landmark_Vehicle[0]*s + Landmark_Vehicle[1]*c

# rtx = rx + Car_x
# rty = ry + Car_y