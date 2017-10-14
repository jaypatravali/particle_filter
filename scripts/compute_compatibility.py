from scipy.stats import chi2
from scipy.spatial.distance import mahalanobis
import numpy as np

def compute_compatibility(observations, predictions):
	""" 
	Individual Compatibility Test
	"""
	compatibility = dict()
	compatibility = { 'd2': None, 'IC': None}
	compatibility['d2'] = np.zeros(shape=(observations['M'], predictions['N']))
	compatibility['IC'] = np.zeros(shape=(observations['M'], predictions['N']))

	# Compute Individual Squared Mahalanobis Distances
	for i in range(observations['M']):
		z = observations['z'][i]
		R = observations['R_covariance'][i]
		# R = [1]
		for j in range(predictions['N']):
			C = np.add(predictions['H_P_H'][i],R)
			C_inverse = np.linalg.inv(C)
			# C_inverse = [1]
			# print(z,R,C, predictions['h_map_fn'][j])
			compatibility['d2'][i][j] = mahalanobis(z, predictions['h_map_fn'][j], C_inverse)

	# Check Mahalanobis Distance against critical values from a Chi2 Distribution.
	for i in range(observations['M']):
		for j in range(predictions['N']):
			if (compatibility['d2'][i][j]  < chi2.isf(q= 0.01,df = 2)):
				compatibility['IC'][i][j]  = 1
			else: 
				compatibility['IC'][i][j]  = 0

	return compatibility
