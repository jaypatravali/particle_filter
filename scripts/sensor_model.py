from __future__ import division
import scipy.stats
import numpy as np
import math

def normalize_angle(a):
	tmp = math.fmod(a + math.pi, 2 * math.pi)
	return tmp + math.pi if (tmp < 0) else tmp - math.pi

def oplus(pose1, pose2):
	out = dict()
	c = np.cos(pose1['x'])
	s = np.sin(pose1['y'])
	out['x'] = c * pose2['x'] - s * pose2['y'] + pose1['x']
	out['y'] = s * pose2['x'] + c * pose2['y'] + pose1['y']
	return out

def eval_sensor_model(sensor_data, particles, landmarks):
	# Computes the observation likelihood of all particles, given the particle and landmark positions and sensor measurements

	sigma_r = 0.4
	sigma_phi = 0.03
	ranges = sensor_data['range']
	bearing = sensor_data['bearing']
	weights = []

	# for particle in particles:
	# 	all_meas_likelihood = 1.0 #for combining multiple measurements

	# 	for i in range(len(ranges)):
	# 		lm_id = closest_landmark(landmarks, ranges[i], particle)
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

	# if(normalizer==0):
	# 	print(normalizer)

	# if (type(normalizer)!= np.float64):
	# 	normalizer = np.float64(normalizer)

	# weights = weights / normalizer
	# return weights

	uniform_hit = 0.001
	for particle in particles:
		all_meas_likelihood = 1.0 #for combining multiple measurements

		for i in range(len(ranges)):
			lm_id = closest_landmark(landmarks, ranges[i], bearing[i], particle)
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
			meas_likelihood = scipy.stats.norm.pdf(meas_range, meas_range_exp, sigma_r) * scipy.stats.norm.pdf(normalize_angle(meas_bearing - meas_bearing_exp), 0,sigma_phi)
		 	#combine (independent) measurements
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


def euclidean_dist(p1, p2):
		return(np.sqrt( (p1[0] - p2[0])**2 + (p1[1] - p2[1])**2 ))


def closest_landmark(landmarks, range_val, range_bearing, particle):
	# Landmark posution in the car frame.
	car_landmark_detected =dict()

	car_landmark_detected['x'] = range_val * np.cos(range_bearing)
	car_landmark_detected['y'] = range_val * np.sin(range_bearing)

	# Lamdark postion in the map frame.
	map_landmark_detected = oplus(particle, car_landmark_detected)
	min_dist = 100000000000.0

	for key in landmarks:
		dist = euclidean_dist((landmarks[key][0], landmarks[key][1]), (map_landmark_detected['x'], map_landmark_detected['y']))
		if dist< min_dist:
			min_dist = dist
			lm_id = key
	return lm_id


def weighting_model(errors):
	normalizer = sum(errors)
	weights = []
	for error in errors:
		weights.append((normalizer- error)/normalizer)

	print(weights)
	return weights

