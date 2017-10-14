from scipy.stats import chi2
import numpy as np

def nearest_neighbors_search(observations, predictions, compatibility):
	""" Computes Nearest Neighbor Search
		TODO: Use KD Tree 
	"""
	H = []  # hypothesis
	for i in range(observations['M']):
	    d2_min = compatibility['d2'][i][0]
	    nearest = 0
	    for j in range(1, predictions['N']):
	        d2_ij = compatibility['d2'][i][j]
	        if d2_ij < d2_min:
	            nearest = j
	            d2_min = d2_ij
	    if d2_min <= chi2.isf(q= 0.01,df = 2):
	        H.append(nearest)
	    else:
	        H.append(0)
	return H