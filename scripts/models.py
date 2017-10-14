import numpy as np
import math

def observation_model(sensor_data, particle):
	"""For visible landmarks get Sensor Readings
	and add Gaussian Noise.
	"""
	observations = dict()
	R_covariance = []
	z = []
	ids = sensor_data['id']
	ranges = sensor_data['range']
	bearing = sensor_data['bearing']

	obs_ids = []

	px = particle['x']
	py = particle['y']
	ptheta = particle['theta']
	
	for i in range(len(ranges)):
		x = px + ranges[i]* np.cos(bearing[i])
		y =	py + ranges[i]* np.sin(bearing[i])
		z.append([x,y])
		R_covariance.append(np.eye(2,2))
		obs_ids.append(i)

		#Noise considerations
		# z.append(ranges[i])		
		# sigma_range = ranges[i] * aplha_R
		# R_i = np.diag([pow(sigma_range,2), pow(aplha_theta,2)])
		# Ji  =  np.array([[np.cos(bearing[i]), - ranges[i]* np.sin(bearing[i])],\
		# 				 [np.sin(bearing[i]), - ranges[i]* np.cos(bearing[i])],\
		# 			 	 [      0.0         ,		1                        ]])
		# Noise_matrix = estimate_gaussian_value(R_i)
		# noisy_range = ranges[i] + Noise[1]
		# noisy_bearing = bearing[i] + Noise[2]
		# x = px + noise_range * np.cos(bearing[i])
		# y = py + noisy_range * np.sin(bearing[i])
		# z.append([x,y])
		# Ri = Ji* Ri * 
		# R_covariance = block_diag(R_covariance, Ri)

	observations = {'M':len(z), 'z':z, "R_covariance": R_covariance, "ID":obs_ids, "GT":ids }

	return observations


def prediction_model(sensor_data, particle, particle_prev, P_noise_cov, odometry, landmarks):
	"""
	Predict Sensor Measuremets
	TODO: Add Motion Noise
	"""
	predictions  = dict()
	H_P_H = []
	H = []
	ids = sensor_data['id']
	ranges = sensor_data['range']
	Q = np.array([[0.2, 0.0, 0.0],\
				[0.0, 0.2, 0.0],\
				[0.0, 0.0, 0.2]])

	delta_rot1 = odometry['r1']
	delta_trans = odometry['t']
	delta_rot2 = odometry['r2']
	P = []
	# currently noise free motion 
	# Here P is the list of Pose Uncertainity for each particles.

	H =  np.array([[1.0, 0.0, -delta_trans * np.sin(particle_prev['theta'] + delta_rot1)],\
						[0.0, 1.0, delta_trans * np.cos(particle_prev['theta'] + delta_rot1)],\
						[0.0, 0.0, 1.0]])

	H_P_H_i = np.dot(np.dot(H,P_noise_cov), np.transpose(H)) + Q
	P = H_P_H_i


	J1 = np.array([[1.0, 0.0, -delta_trans * np.sin(particle_prev['theta'] + delta_rot1)],\
						[0.0, 1.0, delta_trans * np.cos(particle_prev['theta'] + delta_rot1)],\
						[0.0, 0.0, 1.0]])

	J2  =  np.array([[np.cos(particle_prev['theta']), - np.sin(particle_prev['theta']),	  0],\
					 [np.sin(particle_prev['theta']),   np.cos(particle_prev['theta']),   0],\
					 [0.0, 								0.0, 							1.0]])

    # P size:  3x3xparticle_nos
	#measured landmark ids and ranges
	H =[]
	pred_ids = []
	h_map_fn = [] # function
	mapper_ids = []


	px = particle['x']
	py = particle['y']
	ptheta = particle['theta']

	for i in range(len(ids)):
		lm_id = ids[i]
		meas_range = ranges[i]
		lx = landmarks[lm_id][0]
		ly = landmarks[lm_id][1]


		#calculate expected range measurement
		meas_range_exp = np.sqrt( (lx - px)**2 + (ly - py)**2 ) 
		meas_bearing_exp = math.atan2((ly - py),(lx - px)) - ptheta
		# x = px + meas_range_exp* np.cos(meas_bearing_exp)
		# y = py + meas_bearing_exp* np.sin(meas_bearing_exp)
		x = (lx-px)* np.cos(-ptheta) - np.sin(-ptheta)* (ly-py)
		y = (lx-px)* np.sin(-ptheta) + (ly-py)* np.cos(-ptheta)
		H_i = np.array([[-1 ,0 , -meas_range_exp ], [0,-1,-meas_range_exp]])


		# H.append(H_i)
		h_map_fn.append([x,y])
		pred_ids.append(i)
		mapper_ids.append(lm_id)
		# H_i = np.array([[-1 ,0 , -meas_range_exp ], [0,-1,-meas_range_exp]])
		H_P_H_i = np.dot(np.dot(H_i, P ), np.transpose(H_i))
		H_P_H.append(H_P_H_i)

	predictions = {'N':len(h_map_fn), 'h_map_fn':h_map_fn, "H_P_H": H_P_H, "ID":pred_ids, "mapper_ids":mapper_ids }

	return predictions

	H_i = np.array([(px-lx)/meas_range_exp , (py-ly)/meas_range_exp , 0 ])
