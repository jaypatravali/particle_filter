from __future__ import division
import scipy.stats
import numpy as np
import math


def eval_sensor_model(sensor_data, particles, landmarks):
	sigma_r = 0.2
	sigma_phi = 0.1

	ranges = sensor_data['range']
	bearing = sensor_data['bearing']
	weights = []


	new_ids, new_ranges, new_bearing = data_associate_NN(sensor_data, particles, landmarks)

	# print(new_ids)
	for index,particle in enumerate(particles):
		all_meas_likelihood = 1.0 #for combining multiple measurements
		# print("new_ranges,", new_ranges[index])

		for i in range(len(new_ranges[index])):
			# print("new_ranges,", new_ranges[index][i])
			# print("landmarks,", new_ids[index][i])

			meas_range = new_ranges[index][i]
			meas_bearing = new_bearing[index][i]
			lx = landmarks[new_ids[index][i]][0]
			ly = landmarks[new_ids[index][i]][1]
			px = particle['x']
			py = particle['y']
			ptheta = particle['theta']

			meas_range_exp = np.sqrt( (lx - px)**2 + (ly - py)**2 )
			#meas_bearing_exp = math.atan2((ly - py),(lx - px)) - ptheta
			meas_likelihood = scipy.stats.norm.pdf(meas_range, meas_range_exp, sigma_r) #* scipy.stats.norm.pdf(meas_bearing, meas_bearing_exp, sigma_phi)
			all_meas_likelihood = all_meas_likelihood * meas_likelihood
		weights.append(all_meas_likelihood)

 
	normalizer = sum(weights)
	if(normalizer==0):
		print(normalizer)
	if (type(normalizer)!= np.float64):
		normalizer = np.float64(normalizer)

	weights = weights / normalizer
	return weights


def data_associate_NN(sensor_data, particles, landmarks):
	ranges = sensor_data['range']
	bearing = sensor_data['bearing']

	new_ranges = []
	new_bearing = []
	lm_id = []
	for particle in particles:
		id_list, ranges_list, bearing_list = nearest_neighbor(landmarks, ranges, bearing, particle)
		new_bearing.append(bearing_list)
		new_ranges.append(ranges_list)
		lm_id.append(id_list)	
	return lm_id, new_ranges, new_bearing

def nearest_neighbor(landmarks, ranges, bearing, particle):
	px = particle['x']
	py = particle['y']
	ptheta = particle['theta']

	new_ranges = []
	new_bearing = []
	lm_id = []

	for i in range(len(ranges)):
		min = 100000000000.0
		id = 0
		for key in landmarks:
			if key  not in lm_id:
				pred_x = px + ranges[i]* np.cos(ptheta+ bearing[i])
				pred_y = py + ranges[i]* np.sin(ptheta+ bearing[i])

				dist = euclidean_dist((landmarks[key][0], landmarks[key][1]), (pred_x, pred_y))
				# if dist> 35:
				# 	continue
				if dist< min:
					min = dist
					id = key
		if id>0:
			# print("j", j)
			new_bearing.append(bearing[i])
			new_ranges.append(ranges[i])
			lm_id.append(id)

	return lm_id, new_ranges, new_bearing


def euclidean_dist(p1, p2):
		return(np.sqrt( (p1[0] - p2[0])**2 + (p1[1] - p2[1])**2 ))