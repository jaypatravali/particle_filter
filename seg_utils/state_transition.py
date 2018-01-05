import numpy as np
import utm
import math	
import rospy
import tf 
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from MeanShift_py import mean_shift as ms

import random;
from pyclustering.cluster import cluster_visualizer
from pyclustering.cluster.optics import optics, ordering_analyser, ordering_visualizer
from pyclustering.utils import read_sample, timedcall
from process_clusters import process_clusters
import time


class State_Transition():
	""" Convert Pixel Points to Real World Coordinates
	"""
	def __init__(self, odom_file, cam_type, origin_idx, realtime= None):
		"""Initialize params"""
		self.cluster_write = True

		self.realtime = realtime
		self.file_parser(odom_file)
		self.camera_calib(cam_type)
		self.init_origin_state(origin_idx)
		self.param_init(cam_type)
		self.last_stamp  = 1509713949.363687992
		self.fig_created = True
		self.clusters = process_clusters()
		# if realtime:
		# 	plt.ion()
		# 	fig  = plt.figure()
		# 	self.ax = Axes3D(fig)
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
			self.cu = 1010.2511327686972
			self.cv = 360.8366532960085
			self.f = 1011.8324475746241
			self.cam_trans = (1.604, 0.112, 1.014) 
			self.cam_rot = (-0.484, 0.484, -0.516, 0.516)

		self.origin_x =  412940.751955
		self.origin_y =  5318560.37949
		print("Camera Calib Loaded")


	def param_init(self, cam_type):
		self.filter_radius = 0.9
		if cam_type is 'zed':
			if self.realtime is False:	
				self.file = open("../map/sensor_data_car_zed.dat",'w') 
				self.f1 = open("../map/odom_trajectory_car_zed.dat",'w')
				self.velocity_zed = open("../map/velocity_sensor_data_car_zed.dat",'w') 

		elif cam_type is 'pg':
			if self.realtime is False and self.cluster_write is False:	
				self.file = open("../map/sensor_data_car_pg.dat",'w') 
				self.f1 = open("../map/odom_trajectory_car_pg.dat",'w') 
				self.velocity_pg = open("../map/velocity_sensor_data_car_pg.dat",'w') 

			elif self.realtime is True and self.cluster_write is True:	 
				self.file = open("../map/cluster_sensor_data_car_pg.dat",'w') 

		self.flag = 0
		self.sensor_readings = dict()
		self.odom_readings  = dict()
		self.f2 = open("../map/landmarks_pred_car.dat",'w') 

	def file_parser(self, odom_file):
		self.header = []
		self.pos  = []
		self.orientation = []
		self.linear_vel = [] 
		self.angular_vel = [] 
		self.time_stamp = [] 

		odom_list = [line.rstrip('\n').split() for line in open(odom_file)]
		for i in odom_list:
			self.header.append([float(i[0]), float(i[1]), float(i[2])  , float(i[10])])
			self.pos.append((float(i[3]), float(i[4]), float(i[5])))
			self.orientation.append((float(i[6]), float(i[7]), float(i[8]), float(i[9])))
			self.linear_vel.append((float(i[11]), float(i[12]), float(i[13]))) 
			self.angular_vel.append((float(i[14]), float(i[15]), float(i[16]))) 
			self.time_stamp.append( float(i[17]) ) 
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
		print(Point_0, init_heading)
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
		print(landmark_coords)



	def control_compute(self, landmark_coords, landmark_bearing, landmark_ranges, seq):

		v =  np.sqrt( (self.linear_vel[seq][1]**2) + (self.linear_vel[seq][0]**2) + (self.linear_vel[seq][2]**2) )
		w  = self.angular_vel[seq][2]
		del_time = self.time_stamp[seq] - self.last_stamp

		odom_read = "ODOMETRY {} {} {}\n".format(v,w, del_time)
		self.velocity_pg.write(odom_read)
		for val,angle in zip(landmark_ranges, landmark_bearing):
			range_val =  val
			bearing = angle
			sensor_read = "SENSOR {} {} {}\n".format(0,range_val, bearing)
			self.velocity_pg.write(sensor_read)

		self.last_stamp  = self.time_stamp[seq]



	def control_compute_realtime(self, landmark_coords, landmark_bearing, pos):
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
		ranges = []
		print("\n***************************\n")

		for i in range(len(points)):

			seg_pix_X, seg_pix_Y, d = points[i]
			# d_res = d* (0.6)

			#Pixel Coordinates to Camera Transformation
			X  =  (seg_pix_X - self.cu )* self.base/d
			Y =   (seg_pix_Y - self.cv )* self.base/d
			Z =   self.f*self.base/d

			# print("disparities taken", d, d*256, Z)


			Landmarks_Camera =  np.array([ X, Y, Z, 1])
			# print(Landmarks_Camera, d)
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
			ranges.append( np.sqrt( (Landmark_Vehicle[1]**2) + (Landmark_Vehicle[0]**2) ))
			# print("range", np.sqrt( (Landmark_Vehicle[1]**2) + (Landmark_Vehicle[0]**2) ))
			Landmark_Vehicle_odom = [Landmark_Vehicle[0]+ self.pos[seq][0], Landmark_Vehicle[1]+ self.pos[seq][1] , Landmark_Vehicle[2] + self.pos[seq][2] ]
			Vehicle_coords.append( Landmark_Vehicle_odom )
			# print(i, d, points[i])
			self.vehicle_coords_base.append(Landmark_Vehicle)

			print("Landmarks Vehicle", Landmark_Vehicle[:2])
			print(points[i], self.f, self.base, self.cu, self.cv)

		raw_input("Press Enter to Continue")
		print("\n***************************")

		# self.ground_projections(Vehicle_coords , points, seq)

		self.pop_index = self.radius_filtering(Vehicle_coords)
		# print(sorted(self.pop_index))

		filter_coords = []	
		filter_bearing = []
		filter_ranges = []

		for i in range(len(Vehicle_coords)):
			if i not in self.pop_index:
				filter_coords.append(Vehicle_coords[i])
				filter_bearing.append(bearing[i])
				filter_ranges.append(ranges[i])

		if self.realtime is True:
			self.odom_compute_realtime( filter_coords, filter_bearing, self.pos[seq])	
		else:
			self.odom_compute( filter_coords, filter_bearing, self.pos[seq])
			self.control_compute( filter_coords, filter_bearing, filter_ranges, seq)



	def odom_compute3(self, landmark_coords, landmark_bearing, pos):

		pos, landmark_coords = self.vis_transform(landmark_coords, pos)
		trans = self.euclidean_dist((pos[0], pos[1]),(self.prev_odom[0],  self.prev_odom[1]))
		relative_lm_heading  = math.atan2(pos[1] - self.prev_odom[1], pos[0] - self.prev_odom[0])
		del_rot = relative_lm_heading - self.car_orientation

		odom_read = "ODOMETRY {} {} {}\n".format(del_rot,trans, 0)
		self.file.write(odom_read)
		for val,angle in zip(landmark_coords, landmark_bearing):
			range_val = self.euclidean_dist((pos[0], pos[1]), ( val[0], val[1]))
			bearing = angle
			sensor_read = "SENSOR {} {} {}\n".format(0,range_val, bearing)
			self.file.write(sensor_read)

		self.car_orientation  = relative_lm_heading
		self.prev_odom = pos

		

	def log_transformations(self, logged_dict, seq):
		Vehicle_coords = []
		bearing  = [] 
		print("\n***************************\n")
		if seq not in logged_dict:
			self.odom_compute3( Vehicle_coords, bearing, self.pos[seq])	
		else:
			for coordinate in logged_dict[seq]:	
				bearing.append( math.atan2(coordinate[1],coordinate[0]))
				Landmark_Vehicle_odom = [coordinate[0]+ self.pos[seq][0], coordinate[1]+ self.pos[seq][1] ]
				Vehicle_coords.append( Landmark_Vehicle_odom )
			self.odom_compute3( Vehicle_coords, bearing, self.pos[seq])	

	def cluster_transformation(self, points, points_disp, seq):
		Vehicle_coords = []
		bearing  = [] 
		self.vehicle_coords_base = []
		cluster_list = []
		cluster_list2D = []
		global_list_2D = []
		for i in range(len(points_disp)):

			seg_pix_Y, seg_pix_X, d = points_disp[i]

			#Pixel Coordinates to Camera Transformation
			X  =  (seg_pix_X - self.cu )* self.base/d
			Y =   (seg_pix_Y - self.cv )* self.base/d
			Z =   self.f*self.base/d

			Landmarks_Camera =  np.array([ X, Y, Z, 1])
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

			local_x, local_y = utm.to_latlon(rtx,rty, 32, 'U')


			# print(Landmark_Vehicle)
			# print([Landmark_Vehicle[0], Landmark_Vehicle[1]], d)
			cluster_list.append([Landmark_Vehicle[0], Landmark_Vehicle[1], Landmark_Vehicle[2]] )
			cluster_list2D.append([Landmark_Vehicle[0], Landmark_Vehicle[1]])
			global_list_2D.append([local_x, local_y])

     		# print("Landmarks Cam", Landmarks_Camera)
			# print("Landmarks Vehicle", Landmark_Vehicle)

			# print(points_disp[i], self.f, self.base, self.cu, self.cv)
			# print("\n***************************")

			# print(Landmark_Vehicle, " ",  seg_pix_X, seg_pix_Y, d )


		# print("yo",len(cluster_list2D))

		filter_cluster =[]
		for i in range(len(cluster_list)):
			if cluster_list[i][2]< 7 and cluster_list[i][2] > -2 and cluster_list[i][0] < 35 and  cluster_list[i][1] < 35  and cluster_list[i][0] > -35 and cluster_list[i][1] > -35:
				filter_cluster.append(cluster_list2D[i])
		# print(len(cluster_list2D),len(filter_cluster))		
		# self.meanshift_temp(filter_cluster, seq)
		current_frame = self.meanshift_temp(filter_cluster, seq)
		self.odom_compute_realtime2(self.pos[seq])
		# self.control_compute_realtime2(self.pos[seq])

		# print(self.sensor_readings['odometry'])
		self.logged_cluster = self.clusters.cluster_tracker(self.sensor_readings['odometry'], current_frame, self.pos[seq], seq)
		# self.optics_temp(cluster_list2D)
		# self.meanshift_temp(cluster_list, seq)

	def meanshift_temp(self, point_list, seq):
		if not point_list:
			return 

		data = np.array( point_list)
		mean_shifter = ms.MeanShift()
		mean_shift_result = mean_shifter.cluster(data, kernel_bandwidth = 2)

		original_points =  mean_shift_result.original_points
		shifted_points = mean_shift_result.shifted_points
		cluster_assignments = mean_shift_result.cluster_ids
		cluster_ids, indices = np.unique(mean_shift_result.cluster_ids, return_index=True)
		x = original_points[:,0]
		y = original_points[:,1]
		Cluster = cluster_assignments
		centers = shifted_points

		yo = []
		for index in indices:
			yo.append(mean_shift_result.shifted_points[index].tolist())
		return yo

		# if self.fig_created:
		# 	plt.figure()
		# 	self.fig_created =0
		# plt.clf()
		# plt.ion()
		# scatter = plt.scatter(x,y,c=Cluster,s=50)
		# for i,j in centers:
		#     plt.scatter(i,j,s=50,c='red',marker='+')
		# fig = plt.figure()
		# ax = Axes3D(fig)
		# for i,j,k in centers:
		# 	ax.scatter(i, j, k, zdir='z', s=20)
		# if p is not 0:
		# 	avg_x = p/len(point_list)
		# 	avg_y = q/len(point_list)
			# print(avg_x, avg_y)
		# plt.set_xlabel('x')
		# plt.set_ylabel('y')
		# plt.colorbar(scatter)

		# plt.waitforbuttonpress()

		# plt.savefig("/home/patraval/Downloads/dlr-spatial_cognition_data/particle_filter/temp_imgs/_{}.png".format(seq), dpi=300)
		# bearing= [ math.atan2(centers[0][1],centers[0][0])]
		# Landmark_Vehicle_odom = [[centers[0][0]+ self.pos[seq][0],centers[0][1]+ self.pos[seq][1]]]
		# self.odom_compute_realtime( Landmark_Vehicle_odom, bearing, self.pos[seq])	

	def optics_temp(self, point_list):
		# data = np.array( point_list)
		sample = point_list
		start = time.time()
		optics_instance = optics(sample, 0.5, 6, ccore= True);
		optics_instance.process();
		clusters = optics_instance.get_clusters();
		end = time.time()
		print("imte", end-start)

		noise = optics_instance.get_noise();
		visualizer = cluster_visualizer();
		visualizer.append_clusters(clusters, sample);
		visualizer.append_cluster(noise, sample, marker = 'x');
		visualizer.show();
		ordering = optics_instance.get_ordering()
		analyser = ordering_analyser(ordering)
		ordering_visualizer.show_ordering_diagram(analyser, amount_clusters)

	def vis_transform2(self,  pos):
		angle = -0.727210063845
		c = np.cos(angle)
		s = np.sin(angle)
		tx  = self.origin_x + pos[0] - 413058.940761621
		ty  = self.origin_y + pos[1] - 5318228.741414887
		rx = tx*c - ty*s
		ry = tx*s + ty*c

		transform_pos = [rx,ry]
		return transform_pos

	def odom_compute_realtime2(self, pos):
		pos = self.vis_transform2(pos)
		trans = self.euclidean_dist((pos[0], pos[1]),(self.prev_odom[0],  self.prev_odom[1]))
		relative_lm_heading  = math.atan2(pos[1] - self.prev_odom[1], pos[0] - self.prev_odom[0])
		del_rot = relative_lm_heading - self.car_orientation
		self.sensor_readings['odometry'] = {'r1':del_rot,'t':trans,'r2':0}
		self.car_orientation  = relative_lm_heading
		self.prev_odom = pos


	def sensor_compute_async(self, landmark_coords, landmark_bearing, pos):

		pos, landmark_coords = self.vis_transform(landmark_coords, pos)
		trans = self.euclidean_dist((pos[0], pos[1]),(self.prev_odom[0],  self.prev_odom[1]))
		relative_lm_heading  = math.atan2(pos[1] - self.prev_odom[1], pos[0] - self.prev_odom[0])
		del_rot = relative_lm_heading - self.car_orientation

		odom_read = "ODOMETRY {} {} {}\n".format(del_rot,trans, 0)
		self.cluster_file.write(odom_read)
		for val,angle in zip(landmark_coords, landmark_bearing):
			range_val = self.euclidean_dist((pos[0], pos[1]), ( val[0], val[1]))
			bearing = angle
			sensor_read = "SENSOR {} {} {}\n".format(0,range_val, bearing)
			self.cluster_file.write(sensor_read)

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
		print(landmark_coords)



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