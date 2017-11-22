from __future__ import division
import scipy.stats
import numpy as np
import math

def eval_sensor_model(sensor_data, particles, landmarks):
	# Computes the observation likelihood of all particles, given the
	# particle and landmark positions and sensor measurements

	sigma_r = 0.2
	sigma_phi = 0.2

	#measured landmark ids and ranges
	# ids = sensor_data['id']
	ids = range(1,48)
	ranges = sensor_data['range']
	bearing = sensor_data['bearing']

	weights = []

	#rate each particle
	# for particle in particles:
	# 	all_meas_likelihood = 1.0 #for combining multiple measurements
	# 	#loop for each observed landmark
	# 	for i in range(len(ranges)):
	# 		lm_id = ids[i]
	# 		meas_range = ranges[i]
	# 		meas_bearing = bearing[i]
	# 		lx = landmarks[lm_id][0]
	# 		ly = landmarks[lm_id][1]
	# 		px = particle['x']
	# 		py = particle['y']
	# 		ptheta = particle['theta']

	# 		#calculate expected range measurement
	# 		meas_range_exp = np.sqrt( (lx - px)**2 + (ly - py)**2 )
	# 		meas_bearing_exp = math.atan2((ly - py),(lx - px)) - ptheta
	# 		#evaluate sensor model (probability density function of normal distribution) Ranege + sensor(optional)
	# 		meas_likelihood = scipy.stats.norm.pdf(meas_range, meas_range_exp, sigma_r) * scipy.stats.norm.pdf(meas_bearing, meas_bearing_exp, sigma_phi)
			            
	# 		#combine (independent) measurements
	# 		all_meas_likelihood = all_meas_likelihood * meas_likelihood

	# 	weights.append(all_meas_likelihood)

	# #normalize weights
	# normalizer = sum(weights)


	for particle in particles:
		all_meas_likelihood = 1.0 #for combining multiple measurements

		for i in range(len(ranges)):
			lm_id = closest_landmark(landmarks, ranges[i], particle)
			meas_range = ranges[i]
			meas_bearing = bearing[i]
			lx = landmarks[lm_id][0]
			ly = landmarks[lm_id][1]
			px = particle['x']
			py = particle['y']
			ptheta = particle['theta']

			#calculate expected range measurement
			meas_range_exp = np.sqrt( (lx - px)**2 + (ly - py)**2 )
			meas_bearing_exp = math.atan2((ly - py),(lx - px)) - ptheta
			#evaluate sensor model (probability density function of normal distribution) Ranege + sensor(optional)
			meas_likelihood = scipy.stats.norm.pdf(meas_range, meas_range_exp, sigma_r) * scipy.stats.norm.pdf(meas_bearing, meas_bearing_exp, sigma_phi)
			            
			#combine (independent) measurements
			all_meas_likelihood = all_meas_likelihood * meas_likelihood

		weights.append(all_meas_likelihood)

	#normalize weights
	normalizer = sum(weights)






	if(normalizer==0):
		print(normalizer)

	if (type(normalizer)!= np.float64):
		normalizer = np.float64(normalizer)

	weights = weights / normalizer
	return weights



def euclidean_dist(p1, p2):
		return(np.sqrt( (p1[0] - p2[0])**2 + (p1[1] - p2[1])**2 ))

def closest_landmark(landmarks, range_val, particle):
	px = particle['x']
	py = particle['y']
	min = 100000000000.0
	for i in range(1, len(landmarks)+1):
		# print((landmarks[i][0], landmarks[i][1]), (px, py))
		dist = euclidean_dist((landmarks[i][0], landmarks[i][1]), (px, py))
		diff = abs(dist - range_val)
		# print(i, diff, range_val, dist)
		if diff< min:
			min = diff
			id = i
	# print("id",id, "range_val", range_val, "P", (px,py), "landmarks", landmarks[id])
	return id

def weighting_model(errors):
	normalizer = sum(errors)
	weights = []
	for error in errors:
		weights.append((normalizer- error)/normalizer)

	print(weights)
	return weights



