from nearest_neighbors  import  nearest_neighbors_search
from joint_compatibility_BB import joint_compatibility_BB
import numpy as np
from compute_compatibility import compute_compatibility
from get_ground_truth import ground_truth, mapped_ground_truth
from models import observation_model,  prediction_model

def data_association(sensor_data, particles, particles_prev, landmarks, method, cov_noise, odometry):
	""" Performs Data Association between Landmark Measurements
	and Predictions
	"""
	if method =='NN':
		errors = compute_associations_NN(sensor_data, particles, particles_prev, landmarks, method, cov_noise, odometry)
	elif method =='JCBB':
		errors = compute_associations_JCBB(sensor_data, particles, particles_prev, landmarks, method, cov_noise, odometry)

	return errors

def compute_associations_NN(sensor_data, particles, particles_prev, landmarks, method, cov_noise, odometry):
	errors = []
	for particle, particle_prev in zip(particles,particles_prev):
		observations = observation_model(sensor_data, particle)
		predictions  = prediction_model(sensor_data, particle, particle_prev, cov_noise, odometry, landmarks)
		# Individual Compatibility Test
		compatibility = compute_compatibility(observations, predictions)
		# compatibility["IC"] = np.eye(observations["M"], observations["M"])

		hypothesis = nearest_neighbors_search(observations, predictions, compatibility)
		# Evaluation
		gt_list= ground_truth(landmarks, observations, sensor_data)
		# print("Hypothesis: ", hypothesis)
		# print("Ground Truth      ", gt_list)
		mapped_gt_list= mapped_ground_truth(landmarks, predictions, hypothesis)
		# print("Mapped Hypothesis ", mapped_gt_list)
		error = error_check(gt_list, mapped_gt_list)
		# print(error)
		if (error>0):
			print("Mapped Hypothesis ", mapped_gt_list)
			print("Ground Truth      ", gt_list)
			print(compatibility['d2'])

		errors.append(max(error, 0.00001))

	return errors

def compute_associations_JCBB(sensor_data, particles, particles_prev, landmarks, method, cov_noise, odometry):
	errors = []
	for particle, particle_prev in zip(particles,particles_prev):
		observations = observation_model(sensor_data, particle)
		predictions  = prediction_model(sensor_data, particle, particle_prev, cov_noise, odometry, landmarks)
		# Individual Compatibility Test
		compatibility = compute_compatibility(observations, predictions)
		# compatibility['IC'] = np.eye(observations['M'], predictions['N'])
		hypothesis = joint_compatibility_BB(observations, predictions, compatibility)

		# Evaluation
		gt_list= ground_truth(landmarks, observations, sensor_data)
		print("Hypothesis: ", hypothesis)
		print("Ground Truth      ", gt_list)
		mapped_gt_list= mapped_ground_truth(landmarks, predictions, hypothesis)
		print("Mapped Hypothesis ", mapped_gt_list)
		error = error_check(gt_list, mapped_gt_list)
		print(error)
		# particle['errors'] = errors
		errors.append(max(error, 0.00001))

	return errors

def error_check(gt_list, mapped_gt_list ):
	total = 0
	for i in range(len(gt_list)):
		if gt_list[i] is not mapped_gt_list[i]:
			total = total +1
	return total
