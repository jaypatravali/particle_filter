from __future__ import division
import scipy.stats
import numpy as np
import math


def eval_sensor_model(sensor_data, particles, landmarks):
	sigma_r = 0.2
	sigma_phi = 0.015

	ids = range(1,len(landmarks)+1)

	ranges = sensor_data['range']
	bearing = sensor_data['bearing']
	weights = []

	uniform_hit = 0.001
	print_is =1
	for particle in particles:
		all_meas_likelihood = 1.0 #for combining multiple measurements
		new_ids, new_ranges, new_bearing = data_associate_NN(ranges, bearing, particle, landmarks)
		# if print_is:
		# 	print(new_ids)
		# 	print_is = 0
		for i in range(len(new_ids)):
			meas_range = new_ranges[i]
			meas_bearing = new_bearing[i]
			lx = landmarks[new_ids[i]][0]
			ly = landmarks[new_ids[i]][1]
			px = particle['x']
			py = particle['y']
			ptheta = particle['theta']

			meas_range_exp = np.sqrt( (lx - px)**2 + (ly - py)**2 )
			meas_bearing_exp = math.atan2((ly - py),(lx - px)) - ptheta
			meas_likelihood = scipy.stats.norm.pdf(meas_range, meas_range_exp, sigma_r) * scipy.stats.norm.pdf(normalize_angle(meas_bearing - meas_bearing_exp), 0,sigma_phi)
			all_meas_likelihood = all_meas_likelihood * meas_likelihood
		weights.append(uniform_hit + all_meas_likelihood)
 
	normalizer = sum(weights)
	if(normalizer==0):
		print("normalizer", normalizer)
		normalizer = len(particles)
		weights = [1.0]* len(particles)
	if (type(normalizer)!= np.float64):
		normalizer = np.float64(normalizer)
	weights = weights / normalizer

	return weights


def data_associate_NN(ranges, bearing, particle, landmarks):
	new_ranges = []
	new_bearing = []
	landmark_id = []

	for i in range(len(ranges)):
		car_landmark_detected =dict()
		car_landmark_detected['x'] = ranges[i] * np.cos(bearing[i])
		car_landmark_detected['y'] = ranges[i] * np.sin(bearing[i])
		map_landmark_detected = oplus(particle, car_landmark_detected)
		min = 100000000000.0
		lm_id = -1

		for key in landmarks:
			dist = euclidean_dist((landmarks[key][0], landmarks[key][1]), (map_landmark_detected['x'], map_landmark_detected['y']))
			if dist< min and (dist<15) and (key not in landmark_id):
				min = dist
				lm_id = key

		if lm_id>0:
			new_bearing.append(bearing[i])
			new_ranges.append(ranges[i])
			landmark_id.append(lm_id)
	return landmark_id, new_ranges, new_bearing



def normalize_angle(a):
	tmp = math.fmod(a + math.pi, 2 * math.pi)
	return tmp + math.pi if (tmp < 0) else tmp - math.pi

def oplus(pose1, pose2):
	out = dict()
	c = np.cos(pose1['x'])
	s = np.sin(pose1['y'])
	out['x'] = c * pose2['x'] - s * pose2['y'] + pose1['x']
	out['y'] = s * pose2['x'] + s * pose2['x'] + pose1['y']
	return out



def euclidean_dist(p1, p2):
		return(np.sqrt( (p1[0] - p2[0])**2 + (p1[1] - p2[1])**2 ))