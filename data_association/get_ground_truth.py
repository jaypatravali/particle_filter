def ground_truth(landmarks, observations, sensor_data):
	GT= []
	keys= []

		# for i in range(observations['M']):
		#     GT.append(predictions["mapper_ids"][i])
		#     # if len(f) >= 1:
		#     #     GT.append(f[i])
		#     # else:
		#     #     GT.append(0)
	return observations["GT"]

def mapped_ground_truth(landmarks, predictions, hypothesis):
	GT= []
	keys= []
	for i in range(len(hypothesis)):
	    GT.append(predictions["mapper_ids"][hypothesis[i]])
	return GT