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
		self.realtime = realtime
		self.file_parser(odom_file)
		self.camera_calib(cam_type)
		self.init_origin_state(origin_idx)
		self.param_init(cam_type)
		self.last_stamp  = 1509713949.363687992
		self.fig_created = True
		self.clusters = process_clusters()
		self.cluster_write = True

	def camera_calib(self,cam_type):
		print("cam_type",cam_type)
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
		self.cluster_file = open("../map/cluster_sensor_data_car_pg.dat",'w') 
		self.flag = 0
		self.sensor_readings = dict()
		self.odom_readings  = dict()

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


	def point_transformation(self, points, seq):
		Vehicle_coords = []
		bearing  = [] 
		self.vehicle_coords_base = []
		ranges = []
		print("\n***************************\n")

		for i in range(len(points)):

			seg_pix_X, seg_pix_Y, d = points[i]
			#Pixel Coordinates to Camera Transformation
			X  =  (seg_pix_X - self.cu )* self.base/d
			Y =   (seg_pix_Y - self.cv )* self.base/d
			Z =   self.f*self.base/d
			Landmarks_Camera =  np.array([ X, Y, Z, 1])

			# Pinhole Camera Coordinates to Robot Camera [Static Transform: Camera to zed_front]
			tranform_matrix  = tf.TransformerROS()
			R_t= tranform_matrix.fromTranslationRotation(self.cam_trans, self.cam_rot)
			Landmark_Vehicle = np.dot(  R_t, Landmarks_Camera)


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
		self.pop_index = self.radius_filtering(Vehicle_coords)
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

			# Robot Camera Coordinates to World [odom_combined to base_link]
			R_t2= tranform_matrix.fromTranslationRotation( self.pos[seq], self.orientation[seq])
			Landmark_World = np.dot(  R_t2, Landmark_Vehicle)

			rtx = Landmark_World[0] + self.origin_x
			rty = Landmark_World[1] + self.origin_y

			local_x, local_y = utm.to_latlon(rtx,rty, 32, 'U')

			cluster_list.append([Landmark_Vehicle[0], Landmark_Vehicle[1], Landmark_Vehicle[2]] )
			cluster_list2D.append([Landmark_Vehicle[0], Landmark_Vehicle[1]])
			global_list_2D.append([local_x, local_y])

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
		self.clusters.cluster_tracker(self.sensor_readings['odometry'], current_frame, seq)
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


