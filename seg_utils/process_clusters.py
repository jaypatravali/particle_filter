import numpy as np
import math
np.random.seed(123)


class process_clusters():

	def __init__(self):
		self.pred_frame = []
		self.tracked_clusters = []
		self.first_frame =1

	def cluster_tracker(self,  odometry, current_frame, seq):
		if self.first_frame:
			self.tracked_clusters = []
			self.first_frame =0
			for i in range(len(current_frame)):
				particle = dict()
			 	particle['x'] = [current_frame[i][0]]
				particle['y'] = [current_frame[i][1]]
				particle['seq'] = [seq]
				particle['theta'] = 3.0853343280194103
				particle['count'] = 1
				self.tracked_clusters.append(particle)	
			return None
		pred_clusters = []

		self.log_tracked_clusters(seq)

		for p in self.tracked_clusters:
			particle = dict()
		 	particle['x'] = p['x'][-1]
			particle['y'] = p['y'][-1]
			particle['theta'] = p['theta']
			pred_clusters.append(particle)

		pred_clusters = self.sample_odometry_motion_model( odometry , pred_clusters, False, seq)
		pred_frame = []
		for particle in pred_clusters:
			pred_frame.append([particle['x'], particle['y']])

		print( "pred_frame", pred_frame)
		print("current_frame", current_frame)
		# print("tracked_clusters")
		# for i in self.tracked_clusters:
		# 	print(i)
		hypothesis = self.nearest_neighbors_search(current_frame, pred_frame)
		print("****")

		print("hypothesis", hypothesis)
		for i,j in enumerate(hypothesis):
			if j>0:
				print("behen ke")

			 	self.tracked_clusters[j]['x'] = self.tracked_clusters[j]['x'] + [current_frame[i][0]]
				self.tracked_clusters[j]['y'] = self.tracked_clusters[j]['y'] + [current_frame[i][1]]
				self.tracked_clusters[j]['count'] = self.tracked_clusters[j]['count'] +1
				self.tracked_clusters[j]['theta'] = pred_clusters[i]['theta']
				self.tracked_clusters[j]['seq'] = self.tracked_clusters[j]['seq'] + [seq]

			else:
				print("lode")
				particle = dict()
			 	particle['x'] = [current_frame[i][0]]
				particle['y']=  [current_frame[i][1]]
				particle['theta'] = pred_clusters[i]['theta']
				particle['count'] = 1
				particle['seq'] = [seq]
				self.tracked_clusters.append(particle)

		print("*****************")
		print(self.tracked_clusters)
		for i in self.tracked_clusters:
			print(i)
		print("*************+++++++++++*************")



	def log_tracked_clusters(self, seq):
		print("\n \n")

		print("log_tracked_clusters")
		for key in self.tracked_clusters:
			print("key", key)
			print("seq", seq)
			diff = seq - key['seq'][-1]
			print(diff, len(key['seq']))
			if diff>5 and len(key['seq'])<5:
				print("remove")
				self.tracked_clusters.remove(key)
			elif diff>5 and len(key['seq'])>5:
				print("log")
			else:
				print("continue")
		print("\n \n")




	def nearest_neighbors_search(self, current_frame, pred_frame):
		threshold_param  = 1.5 # 1.5m motion
		H = [] 
		if not current_frame:
			return []
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
		sigma_delta_trans = noise[2] * delta_trans + \
		noise[3] * (abs(delta_rot1) + abs(delta_rot2))
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