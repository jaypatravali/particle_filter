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


