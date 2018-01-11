import numpy as np
import math
from termcolor import colored, cprint
"""
threshold
reduce the area for consideration
--------
index for theta
write code for processing datastructure
"""


np.random.seed(123)


class process_clusters():

	def __init__(self):
		self.pred_frame = []
		self.tracked_clusters = []
		self.first_frame =1
		self.logged_cluster = {}
	def initialize_tracker_list(self, current_frame, pos, seq):
		self.tracked_clusters = []
		self.vehicle = []

		for i in range(len(current_frame)):
			particle = dict()
		 	particle['x'] = [current_frame[i][0]]
			particle['y'] = [current_frame[i][1]]
			particle['seq'] = [seq]
			particle['theta'] = 3.0853343280194103
			particle['count'] = 1
			self.tracked_clusters.append(particle)	
		particle = dict()
	 	particle['x'] = pos[0]
		particle['y'] = pos[1]
		particle['seq'] = [seq]
		particle['theta'] = 3.0853343280194103
		particle['type'] = 'car_pos'
		self.vehicle.append(particle)	


	def cluster_tracker(self,  odometry, current_frame, pos, seq):
		if self.first_frame:
			self.first_frame =0
			self.initialize_tracker_list(current_frame, pos,  seq)
			return None

		pred_clusters = []
		for p in self.tracked_clusters:
			particle = dict()
		 	particle['x'] = p['x'][-1]
			particle['y'] = p['y'][-1]
			particle['theta'] = p['theta']
			pred_clusters.append(particle)

		pred_clusters = self.sample_odometry_motion_model( odometry , pred_clusters, False, seq)
		self.vehicle = self.sample_odometry_motion_model( odometry,  self.vehicle , False, seq)

		pred_frame = []
		for particle in pred_clusters:
			pred_frame.append([particle['x'], particle['y']])

		print( "pred_frame", pred_frame)
		print("current_frame", current_frame)


		hypothesis = self.nearest_neighbors_search(current_frame, pred_frame)
		print("hypothesis", hypothesis)
		print("\n \n")

		for i,j in enumerate(hypothesis):
			if j>=0:
			 	self.tracked_clusters[j]['x'] = self.tracked_clusters[j]['x'] + [current_frame[i][0]]
				self.tracked_clusters[j]['y'] = self.tracked_clusters[j]['y'] + [current_frame[i][1]]
				self.tracked_clusters[j]['count'] = self.tracked_clusters[j]['count'] +1
				self.tracked_clusters[j]['theta'] = pred_clusters[j]['theta']
				self.tracked_clusters[j]['seq'] = self.tracked_clusters[j]['seq'] + [seq]

			else:
				particle = dict()
			 	particle['x'] = [current_frame[i][0]]
				particle['y']=  [current_frame[i][1]]
				particle['theta'] = self.vehicle[0]['theta']
				particle['count'] = 1
				particle['seq'] = [seq]
				self.tracked_clusters.append(particle)

		for i in self.tracked_clusters:
			print(i)
		print colored("*************+++++++++++*************", 'cyan')
		self.process_tracked_clusters(seq)
		print colored("*************+++++++++++*************", 'cyan')
		return self.logged_cluster


	def process_tracked_clusters(self, seq):
		print("\n \n")
		remove_keys=  []
		print colored("process_tracked_clusters", "yellow")
		for key in self.tracked_clusters:
			print("key", key)
			print("seq", seq)
			diff = seq - key['seq'][-1]
			print(diff, len(key['seq']))
			if diff>=5 and len(key['seq'])<5:
				print("remove")
				remove_keys.append(key)
			elif diff>5 and len(key['seq'])>=5:
				print("log")
				self.log_clusters(key)
				remove_keys.append(key)
			else:
				print("continue")
		for key in remove_keys:
			print('\n')
			cprint("<<---removing key", 'red')
			cprint(key,   "red")
			self.tracked_clusters.remove(key)
		print("\n \n")



	def log_clusters(self, key):
		for i in range(len(key['seq'])):
			key_val = key['seq'][i]
			x_val, y_val = key['x'][i], key['y'][i]
			if key_val not in self.logged_cluster:
				self.logged_cluster[key_val] = [[x_val, y_val]]
			elif key_val in self.logged_cluster:
				self.logged_cluster[key_val].append([x_val, y_val])
		print("\n//____________logged cluster__________\\", "green")
		cprint( self.logged_cluster, 'blue')


	def nearest_neighbors_search(self, current_frame, pred_frame):
		threshold_param  = 1.5 # 1.5m motion
		H = [] 
		if not current_frame:
			return []
		elif not pred_frame:
			return len(current_frame)*[-1]

		for i in range(len(current_frame)):
			p1 = pred_frame[0]
			p2 = current_frame[i]
			d2_min = self.euclidean_dist(p1, p2)
			nearest = 0
			for j in range(1, len(pred_frame)):
				p1 = pred_frame[j]
				p2 = current_frame[i]
				d2_ij = self.euclidean_dist(p1, p2)
				if d2_ij < d2_min:
					nearest = j
					d2_min = d2_ij
			if d2_min <= threshold_param:
				H.append(nearest)
			else:
				H.append(-1)
		return H


	def euclidean_dist(self, p1, p2):
			return(np.sqrt( (p1[0] - p2[0])**2 + (p1[1] - p2[1])**2 ))

	def sample_odometry_motion_model(self, odometry, particles, add_noise, timestep, sim_odometry = dict()):
		delta_rot1 = odometry['r1']
		delta_trans = odometry['t']
		delta_rot2 = odometry['r2']

		if add_noise:
			noise = [0.025, 0.025, 0.025, 0.025]	
			odom_noise =  [0.01,0,0,0.01]
		else:
			noise = [0,0,0,0]
			odom_noise =  [0,0,0,0]

		# standard deviations of motion noise
		sigma_delta_rot1 = noise[0] * abs(delta_rot1) + noise[1] * delta_trans
		sigma_delta_trans = noise[2] * delta_trans + noise[3] * (abs(delta_rot1) + abs(delta_rot2))
		sigma_delta_rot2 = noise[0] * abs(delta_rot2) + noise[1] * delta_trans
		new_particles = []
		
		for particle in particles:
			new_particle = dict()
			#sample noisy motions
			noisy_delta_rot1 = delta_rot1 + np.random.normal(0, sigma_delta_rot1)
			noisy_delta_trans = delta_trans + np.random.normal(0, sigma_delta_trans)
			noisy_delta_rot2 = delta_rot2 #+ np.random.normal(0, sigma_delta_rot2)

			#calculate new particle pose
			new_particle['x'] = particle['x'] + noisy_delta_trans * np.cos(particle['theta'] + noisy_delta_rot1)
			new_particle['y'] = particle['y'] + noisy_delta_trans * np.sin(particle['theta'] + noisy_delta_rot1)
			new_particle['theta'] = particle['theta'] + noisy_delta_rot1 + noisy_delta_rot2
			new_particles.append(new_particle)

		return new_particles