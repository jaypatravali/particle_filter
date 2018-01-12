from __future__ import division
import scipy.stats
import numpy as np
import math
import time

sqrt_2pi = math.sqrt(2*math.pi)


def normalize_angle(a):
	tmp = math.fmod(a + math.pi, 2 * math.pi)
	return tmp + math.pi if (tmp < 0) else tmp - math.pi

def oplus(pose1, pose2):
	out = []
	c = np.cos(pose1[2])
	s = np.sin(pose1[2])
	out += [c * pose2[0] - s * pose2[1] + pose1[0]]
	out += [s * pose2[0] + c * pose2[1] + pose1[1]]
	return out

def norm_pdf(x, exp_x, sigma):
	global sqrt_2pi
	return math.exp(-((x- exp_x)/sigma)**2)/(sqrt_2pi*sigma)

def eval_sensor_model(sensor_data, particles, landmarks):
	# Computes the observation likelihood of all particles, given the particle and landmark positions and sensor measurements

	sigma_r = 0.4
	sigma_phi = 0.03
	ranges = sensor_data['range']
	bearing = sensor_data['bearing']
	weights = []

	indices = range(len(ranges))
	endpoints = []
	for i in indices:
		range_val = ranges[i]
		range_bearing = bearing[i]
		endpoints += [[range_val * np.cos(range_bearing), range_val * np.sin(range_bearing) ]]

	uniform_hit = 0.001
	for particle in particles:
		all_meas_likelihood = 1.0 #for combining multiple measurements
		all_meas_loglikelihood = 0.0

		for i in indices:
			lm_id = closest_landmark_endpoint(landmarks, endpoints[i], particle)
			meas_range = ranges[i]
			meas_bearing = bearing[i]
			lx = landmarks[lm_id][0]
			ly = landmarks[lm_id][1]
			px, py, ptheta = particle

			#calculate expected range measurement
			meas_range_exp = np.sqrt( (lx - px)**2 + (ly - py)**2 )
			meas_bearing_exp = math.atan2((ly - py),(lx - px)) - ptheta
			#evaluate sensor model (probability density function of normal distribution) Ranege + sensor(optional)
			# meas_likelihood = scipy.stats.norm.pdf(meas_range, meas_range_exp, sigma_r) * scipy.stats.norm.pdf(normalize_angle(meas_bearing - meas_bearing_exp), 0,sigma_phi)
			# meas_likelihood = norm_pdf(meas_range, meas_range_exp, sigma_r) * norm_pdf(normalize_angle(meas_bearing - meas_bearing_exp), 0,sigma_phi)
		 	#combine (independent) measurements
			all_meas_loglikelihood = all_meas_loglikelihood - ((meas_range - meas_range_exp)/sigma_r)**2 - ((meas_bearing - meas_bearing_exp)/sigma_phi)**2 
			# all_meas_likelihood = all_meas_likelihood * meas_likelihood

		weights.append(uniform_hit + math.exp(all_meas_loglikelihood))


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
		return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2) 

def euclidean_dist2(p1, p2):
		return (p1[0] - p2[0])**2 + (p1[1] - p2[1])**2 

def closest_landmark(landmarks, range_val, range_bearing, particle):
	# Landmark posution in the car frame.
	car_landmark_detected = []

	car_landmark_detected = [range_val * np.cos(range_bearing), range_val * np.sin(range_bearing) ]
	# Lamdark postion in the map frame.
	map_landmark_detected = oplus(particle, car_landmark_detected)

	min_dist2 = 100000000000.0
	for key in landmarks:
		dist2 = euclidean_dist2(landmarks[key], map_landmark_detected)
		if dist2 < min_dist2:
			min_dist2 = dist2
			lm_id = key
	return lm_id

def closest_landmark_endpoint(landmarks, endpoint, particle):
#	car_landmark_detected = [range_val * np.cos(range_bearing), range_val * np.sin(range_bearing) ]
	# Lamdark postion in the map frame.
	map_landmark_detected = oplus(particle, endpoint)

	min_dist2 = 100000000000.0
	for key in landmarks:
		dist2 = euclidean_dist2(landmarks[key], map_landmark_detected)
		if dist2 < min_dist2:
			min_dist2 = dist2
			lm_id = key
	return lm_id	


def weighting_model(errors):
	normalizer = sum(errors)
	weights = []
	for error in errors:
		weights.append((normalizer- error)/normalizer)

	print(weights)
	return weights

