import numpy as np
import utm
import math	
import rospy
import tf 
import matplotlib.pyplot as plt

class State_Transition():
	""" Convert Pixel Points to Real World Coordinates
	"""
	def __init__(self, odom_file, cam_type, origin_idx, realtime= None):
		"""Initialize params"""
 		self.realtime = realtime
		self.file_parser(odom_file)
		self.camera_calib(cam_type)
		self.init_origin_state(origin_idx)
		self.param_init(cam_type)

	def camera_calib(self,cam_type):
		print("cam_type",cam_type)
		if cam_type is 'zed':
			self.base = 0.12
			self.cu = 674.03076171875
			self.cv = 348.81689453125
			self.f = 698.2396240234375
			self.cam_trans = (1.061, -0.157, 1.372) 
			self.cam_rot = (-0.484, 0.484, -0.516, 0.516)

		elif cam_type is 'pg':
			self.base = 0.50
			self.cu = 674.03076171875
			self.cv = 348.81689453125
			self.f = 698.2396240234375 
			self.cam_trans = (1.061, -0.157, 1.372) 
			self.cam_rot = (-0.484, 0.484, -0.516, 0.516)

		self.origin_x =  412940.751955
		self.origin_y =  5318560.37949
		print("Camera Calib Loaded")


	def param_init(self, cam_type):
		self.filter_radius = 0.9
		if cam_type is 'zed':
			self.prev_odom = [177.03757970060997,72.711216426459998]
			self.car_orientation = 1.3540745849092297
			if self.realtime is False:	
				self.file = open("../map/sensor_data_car_zed.dat",'w') 
				self.f1 = open("../map/odom_trajectory_car_zed.dat",'w') 

		elif cam_type is 'pg':
			self.prev_odom = [177.03757970060997,72.711216426459998]
			self.car_orientation = 1.3496612278621256

			if self.realtime is False:	
				self.file = open("../map/sensor_data_car_pg.dat",'w') 
				self.f1 = open("../map/odom_trajectory_car_pg.dat",'w') 

		self.flag = 0
		self.sensor_readings = dict()
		self.odom_readings  = dict()
		self.f2 = open("../map/landmarks_pred_car.dat",'w') 

	def file_parser(self, odom_file):
		self.header = []
		self.pos  = []
		self.orientation = []
		odom_list = [line.rstrip('\n').split() for line in open(odom_file)]
		for i in odom_list:
			self.header.append([float(i[0]), float(i[1]), float(i[2])  , float(i[10])])
			self.pos.append((float(i[3]), float(i[4]), float(i[5])))
			self.orientation.append((float(i[6]), float(i[7]), float(i[8]), float(i[9]))) 
		print("Odometry Loaded")


	def init_origin_state(self, origin_idx):
		angle = -0.727210063845
		c = np.cos(angle)
		s = np.sin(angle)

		tx  = self.origin_x + self.pos[origin_idx][0] - 413058.940761621
		ty  = self.origin_y + self.pos[origin_idx][1] - 5318228.741414887
		rx = tx*c - ty*s
		ry = tx*s + ty*c
		Point_1 = [rx,ry]

		tx  = self.origin_x + self.pos[origin_idx-1][0] - 413058.940761621
		ty  = self.origin_y + self.pos[origin_idx-1][1] - 5318228.741414887
		rx = tx*c - ty*s
		ry = tx*s + ty*c
		Point_0 = [rx,ry]
		init_heading = math.atan2(Point_1[1] - Point_0[1], Point_1[0] -Point_0[0])
		self.prev_odom = Point_0
		self.car_orientation = init_heading
		print(self.prev_odom, self.car_orientation)

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



	def ground_projections(self, Vehicle_coords, points, seq ):
		pos, tx_coords = self.vis_transform(Vehicle_coords, self.pos[seq])
		plt.clf()
		plt.ion()		
		plt.figure(2)
		plt.axis([pos[0]-20, pos[0]+20, pos[1]-20, pos[1]+20])
		bbox_props = dict(boxstyle="circle,pad=0.05", fc="green", ec="g", lw=1, alpha=0.5)


		for i in range(len(tx_coords)):
			plt.plot(tx_coords[i][0], tx_coords[i][1], marker='o', markersize=1, color="red")
			plt.text(tx_coords[i][0], tx_coords[i][1], str(round(points[i][2],2)))
			first = []
			second = []
			for j in range(2):
				tx = round(tx_coords[i][j],2)
				first.append(tx)
			px = -round(self.vehicle_coords_base[i][1],2)	
			py = round(self.vehicle_coords_base[i][0],2)	
			second = [px, py]
			# print("tx_coords", first, round(points[i][2],2), second)

		# print(self.vehicle_coords_base)
		plt.plot(pos[0], pos[1], marker='x', markersize=6, color="red")
		plt.figure(3)
		for i in range(len(self.vehicle_coords_base)):
			plt.plot( -self.vehicle_coords_base[i][1], self.vehicle_coords_base[i][0], marker='o', markersize=1, color="red")
			plt.text( -self.vehicle_coords_base[i][1] , self.vehicle_coords_base[i][0], str(round(points[i][2],2)), bbox=bbox_props)
		plt.pause(0.001)

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
		for i in range(len(landmark_coords)):
			tx  = self.origin_x + landmark_coords[i][0] - 413058.940761621
			ty  = self.origin_y + landmark_coords[i][1] - 5318228.741414887
			rx = tx*c - ty*s
			ry = tx*s + ty*c
			transform_coords.append([rx,ry])

		return transform_pos, transform_coords

	def odom_compute(self, landmark_coords, landmark_bearing, pos):

		pos, landmark_coords = self.vis_transform(landmark_coords, pos)
		trans = self.euclidean_dist((pos[0], pos[1]),(self.prev_odom[0],  self.prev_odom[1]))
		relative_lm_heading  = math.atan2(pos[1] - self.prev_odom[1], pos[0] - self.prev_odom[0])
		del_rot = relative_lm_heading - self.car_orientation

		odom_read = "{} {}\n".format(pos[0], pos[1])
		self.f1.write(odom_read)

		for i in range(len(landmark_coords)):
			landmarks_pred = "{} {}\n".format(landmark_coords[i][0], landmark_coords[i][1])
			self.f2.write(landmarks_pred)


		odom_read = "ODOMETRY {} {} {}\n".format(del_rot,trans, 0)
		self.file.write(odom_read)
		for val,angle in zip(landmark_coords, landmark_bearing):
			range_val = self.euclidean_dist((pos[0], pos[1]), ( val[0], val[1]))
			bearing = angle
			sensor_read = "SENSOR {} {} {}\n".format(0,range_val, bearing)
			self.file.write(sensor_read)

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
			# print(range_val, bearing)


		self.odom_reading = [pos[0], pos[1]]      
		self.sensor_readings['odometry'] = {'r1':del_rot,'t':trans,'r2':0}
		self.sensor_readings['sensor'] = {'id':lm_ids,'range':ranges,'bearing':bearings}            

		self.car_orientation  = relative_lm_heading
		self.prev_odom = pos


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
			R_t= tranform_matrix.fromTranslationRotation(self.cam_trans, self.cam_rot)
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

		# self.ground_projections(Vehicle_coords , points, seq)

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
		







"""
	How to get the  self.prev_odom self.car_orientation 

	Go to the initial_offset+ file_offset position.

	eg for zed  self.prev_odom = [177.03757970060997,72.711216426459998] self.car_orientation = 1.3540745849092297
	in gps/fix.txt 6070  is 202.034 -159.503

	in gps/fix.txt 6069  is 202.104 -159.628


	add this to origin_x

	Calculations Dump

	>>> ty  = origin_y + 202.034 - 5318228.741414887
	>>> tx  = origin_x + 202.034 - 413058.940761621
	>>> ry = tx*s + ty*c
	>>> rx = tx*c - ty*s
	>>> ry
	342.93084727840596
	>>> rx
	>>> p1y = ry
	>>> tx  = origin_x + 201.936 - 413058.940761621
	>>> rx = tx*c - ty*s
	>>> ry = tx*s + ty*c
	>>> rx
	177.11018516020448
	>>> ry
	73.04551636204539
	>>> atan2(73.04551636204539- 72.851130593136745, 177.11018516020448- 177.0683859
	6562423)
	Traceback (most recent call last):
	  File "<input>", line 1, in <module>
	    atan2(73.04551636204539- 72.851130593136745, 177.11018516020448- 177.0683859
	6562423)
	NameError: name 'atan2' is not defined
	>>> math.atan2(73.04551636204539- 72.851130593136745, 177.11018516020448- 177.06
	838596562423)
	1.3589894140332754
	>>> tx  = origin_x + 202.104 - 413058.940761621
	>>> ty  = origin_y + 202.104 -159.628 - 5318228.741414887
	>>> ty  = origin_y +  -159.628 - 5318228.741414887
	>>> rx = tx*c - ty*s
	>>> ry = tx*s + ty*c
	>>> rx
	177.03757970060997
	>>> ry
	72.711216426459998
	>>> math.atan2(72.851130593136745 - 72.711216426459998, 177.06838596562423 -  17
	7.03757970060997)
	1.3540745849092297
	>>> a = [[1,3], [4,5],[

	for PG

	tx  = origin_x + 202.896 - 413058.940761621
	KeyboardInterrupt
	>>> origin_y =  5318560.37949
	>>> 
	>>> origin_x =  412940.751955
	>>> tx  = origin_x +  - 413058.940761621
	>>> tx  = origin_x +  - 413058.940761621
	>>> 
	>>> tx  = origin_x + 202.104 - 413058.940761621
	>>> ty  = origin_y + -159.628 - 5318228.741414887
	>>> rx = tx*c - ty*s
	Traceback (most recent call last):
	  File "<input>", line 1, in <module>
	    rx = tx*c - ty*s
	NameError: name 'c' is not defined
	>>> angle = -0.727210063845
	>>> c = np.cos(angle)
	Traceback (most recent call last):
	  File "<input>", line 1, in <module>
	    c = np.cos(angle)
	NameError: name 'np' is not defined
	>>> import numpy as np
	>>> c = np.cos(angle)
	>>> c = np.cos(angle)
	>>> c = np.cos(angle)
	KeyboardInterrupt
	>>> s = np.sin(angle)
	>>> rx = tx*c - ty*s
	>>> ry = tx*s + ty*c
	>>> rx
	177.03757970060997
	>>> ry
	72.711216426459998
	>>> tx  = origin_x + 202.058 - 413058.940761621
	>>> ty  = origin_y + -159.545 - 5318228.741414887
	>>> rx = tx*c - ty*s
	>>> ry = tx*s + ty*c
	>>> rx
	177.05839363573168
	>>> ry
	72.803800334573992
	>>> atan2(73.04551636204539- 72.851130593136745, 177.11018516020448- 177.0683859
	... 
	... 
	... 
	... 
	... atan2(72.803800334573992 - 72.711216426459998,177.05839363573168-177.0375797
	0060997)
	  File "<input>", line 6
	    atan2(72.803800334573992 - 72.711216426459998,177.05839363573168-177.0375797
	0060997)
	        ^
	SyntaxError: invalid syntax
	>>> import math
	>>> math.atan2(72.803800334573992 - 72.711216426459998,177.05839363573168-177.03757970060997)
	1.3496612278621256
	>>> 
"""