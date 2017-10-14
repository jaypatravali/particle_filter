from scipy.spatial.distance import mahalanobis
from scipy.stats import chi2
from scipy.linalg import block_diag
import numpy as np

def jointly_compatible(predictions, observations, H):
	"""
	Bool check for Joint Compatibility
	"""
	d2 = joint_mahalanobis (predictions, observations, H)
	dof = 2*len(H)
	# if (d2>0.05):
	# 	return False
	# else:
	# 	return True

	return d2 < chi2.isf(q= 0.01, df = dof)

def joint_mahalanobis(predictions, observations,H):
	"""
	Compute joint distance for k-th hypothesis
	"""
	Z = []
	h_fn = []
	H_P_H_joint= []
	R_i= []
	C_i_inv = []
	# idx_list = np.nonzero(H)[0].tolist()
	for i,j in enumerate(H):
		print(i,j)
		zk = observations['z'][i]
		hk = predictions['h_map_fn'][j]
		Z.extend(zk)
		h_fn.extend(hk)
		# R_i = block_diag(R_i,observations['R_covariance'][j])

		Ck = np.add(predictions['H_P_H'][j], observations['R_covariance'][j])
		C_inv_K = np.linalg.inv(Ck)
		C_i_inv = block_diag(C_i_inv, C_inv_K)

	C_i_inv = np.delete(C_i_inv,0,0)
	print("C_i_inv", C_i_inv)

	joint_distance = mahalanobis (Z , h_fn, C_i_inv)
	return joint_distance


	# print(Z, "hk", h_fn)
	# sizing = len(Z)
	# C_inv_K = np.eye(sizing, sizing)

	# # look for all non zero values in H
	# # i,j = find(H)
	# # i = row value
	# # j=
	# # iter_list = non_zero_indices(H, lambda x:x)
	# idx_list = np.nonzero(H)[0].tolist()
	# for i in idx_list:
	# 	print("helo inside", observations, predictions )
	# 	zk = observations['z'][i]
	# 	hk = predictions['h_map_fn'][idx_list[i]]
	# 	#Rk = observations['R_covariance'][]
	# 	Z.append(zk)
    #
	# 	h_fn.append(hk)
	# 	# for j in
	# 	# C_inv_K = np.linalg.inv(Ck)
    #
	# # Ck = np.add(predictions['H_P_H'], Rk)
	# # C_inv_K = np.linalg.inv(Ck)
	# # C_inv_K = C_inv_K.tolist()
	# print(Z,h_fn)
	# C_inv_K = 1
	# joint_distance = mahalanobis (Z , h_fn, C_inv_K)
	# return joint_distance