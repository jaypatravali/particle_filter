import numpy as np
from joint_compatibility_test import jointly_compatible, joint_mahalanobis
visited  =0	
Best = []
def joint_compatibility_BB(observations, predictions, compatibility):
	""" 
	Implements the Joint Compatibility Branch and Bound 
	Refer: Neira et al, 2001.
	"""
	global visited
	global Best
	Best = [0]*observations['M']
	visited = 0
	JCBB_recursive (predictions, observations, compatibility, [], 0)
	print('Visited Nodes', visited)

	global Best
	return Best

def JCBB_recursive(predictions, observations, compatibility, H, i): # Find Pairings for observation Ei
	"""
	Recursive Search for Branch and Bound
	"""

	global visited
	visited = visited + 1
	if i+1 > observations['M']:  # if reached  leaf node
		# print(H)
		# if pairings(H) > pairings(Best):
		global Best
		Best = H
	else:
		# Check only those leaves
		individually_compatible = non_zero_indices(compatibility['IC'][i].tolist(), lambda x:x)
		for j in individually_compatible:
			# H_i_j =[]
			# H_i_j.append(j)
			H.append(j)
			if jointly_compatible(predictions, observations, H):
				JCBB_recursive(predictions, observations, compatibility, H , i + 1) # We accept (Ei, Fj) pairing
			else:
				H= H[:-1]
		# global Best
		# if (pairings(H) + observations['M'] - i >= pairings(Best)):   # Condition check for Null hypothesis
		# 	JCBB_recursive(predictions, observations, compatibility, H.append(0), i + 1) #star node: Ei not paired

def pairings(H):
	p = len(non_zero_indices(H,lambda x:x))
	return p

def non_zero_indices(a, func):
	"""Returns Non Zero Indices of a list"""
	return [i for i, e in enumerate(a) if e != 0]
