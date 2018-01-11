import numpy as np
import math
np.random.seed(123)
from sensor_model  import euclidean_dist

def sample_odometry_motion_model(odometry, particles, add_noise, timestep, sim_odometry = dict()):
	# Samples new particle positions, based on old positions, the odometry
	# measurements and the motion noise
	# (probabilistic motion models slide 27)
	delta_rot1 = odometry['r1']
	delta_trans = odometry['t']
	delta_rot2 = odometry['r2']

	if add_noise:
	    # the motion noise parameters: [alpha1, alpha2, alpha3, alpha4]
		noise = [0.05, 0.05, 0.05, 0.05]
		# noise = [0.0, 0.0, 0.0, 0.0]
	
		odom_noise =  [0.01,0,0,0.01]
	else:
		noise = [0,0,0,0]
		odom_noise =  [0,0,0,0]

	if sim_odometry:
		sim_odometry, delta_trans, delta_rot1, delta_rot2 = noisy_odometry(sim_odometry, delta_rot1,delta_trans, delta_rot2, timestep, odom_noise)


	# standard deviations of motion noise
	sigma_delta_rot1 = noise[0] * abs(delta_rot1) + noise[1] * delta_trans
	sigma_delta_trans = noise[2] * delta_trans + noise[3] * (abs(delta_rot1) + abs(delta_rot2))
	sigma_delta_rot2 = noise[0] * abs(delta_rot2) + noise[1] * delta_trans
	# "move" each particle according to the odometry measurements plus sampled noise
	# to generate new particle set

	new_particles = []
	
	for particle in particles:
		new_particle = dict()

		#sample noisy motions
		noisy_delta_rot1 = delta_rot1 + np.random.normal(0, sigma_delta_rot1)
		noisy_delta_trans = delta_trans + np.random.normal(0, sigma_delta_trans)
		noisy_delta_rot2 = delta_rot2 #+ np.random.normal(0, sigma_delta_rot2)

		#calculate new particle pose
		new_particle['x'] = particle['x'] + noisy_delta_trans * np.cos(particle['theta'] + noisy_delta_rot1)
		new_particle['y'] = particle['y'] + noisy_delta_trans * np.sin(particle['theta'] + noisy_delta_rot1)
		new_particle['theta'] = particle['theta'] + noisy_delta_rot1 + noisy_delta_rot2
		new_particles.append(new_particle)

	return new_particles

def sample_odometry_motion_model_v2(odometry, particles, add_noise, timestep, sim_odometry = dict()):
	# Samples new particle positions, based on old positions, the odometry
	# measurements and the motion noise
	# (probabilistic motion models slide 27)
	delta_rot1 = odometry['r1']
	delta_trans = odometry['t']
	delta_rot2 = odometry['r2']

	if add_noise:
	    # the motion noise parameters: [alpha1, alpha2, alpha3, alpha4]
		noise = [0.025, 0.025, 0.025, 0.025]

	# standard deviations of motion noise
	sigma_delta_rot1 = noise[0] * abs(delta_rot1) + noise[1] * delta_trans
	sigma_delta_trans = noise[2] * delta_trans + noise[3] * (abs(delta_rot1) + abs(delta_rot2))
	sigma_delta_rot2 = noise[0] * abs(delta_rot2) + noise[1] * delta_trans
	
	for particle in particles:

		#sample noisy motions
		noisy_delta_rot1 = delta_rot1 + np.random.normal(0, sigma_delta_rot1)
		noisy_delta_trans = delta_trans + np.random.normal(0, sigma_delta_trans)
		noisy_delta_rot2 = delta_rot2 #+ np.random.normal(0, sigma_delta_rot2)

		#calculate new particle pose
		particle['x'] = particle['x'] + noisy_delta_trans * np.cos(particle['theta'] + noisy_delta_rot1)
		particle['y'] = particle['y'] + noisy_delta_trans * np.sin(particle['theta'] + noisy_delta_rot1)
		particle['theta'] = particle['theta'] + noisy_delta_rot1 + noisy_delta_rot2

	return particles

def sample_velocity_motion_model(odometry, particles):
	# (probabilistic motion models Table 5.3

	print(odometry)
	v = odometry['v']
	w = odometry['w']
	delta_time = odometry['dt']

	# the motion noise parameters: [alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 ]
	noise = [0.025, 0.025, 0.025, 0.025, 0.0125, 0.025]
	noise = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]


	# standard deviations of motion noise
	sigma_v = noise[0] * math.pow(v,2) + noise[1] * math.pow(w,2)
	sigma_w = noise[2] * math.pow(v,2) + noise[3] * math.pow(w,2)
	sigma_gamma = noise[4] * math.pow(v,2) + noise[5] * math.pow(w,2)

	v_w=v/w

	new_particles = []

	for particle in particles:
		new_particle = dict()
		#sample noisy motions
		noisy_v = v + np.random.normal(0, sigma_v)
		noisy_w = w + np.random.normal(0, sigma_w)
		noisy_gamma = np.random.normal(0, sigma_gamma)

		#calculate new particle pose
		new_particle['x'] = particle['x'] - v_w * np.sin(particle['theta']) + v_w * np.sin(particle['theta'] + w*delta_time)
		new_particle['y'] = particle['y'] + v_w * np.cos(particle['theta'] )- v_w * np.cos(particle['theta'] + w*delta_time)
		new_particle['theta'] = particle['theta'] +  w * delta_time + noisy_gamma * delta_time
		new_particles.append(new_particle)

	return new_particles



def noisy_odometry(sim_odometry, delta_rot1,delta_trans, delta_rot2, timestep, noise ):

	sigma_delta_rot1 = noise[0] * abs(delta_rot1) + noise[1] * delta_trans
	sigma_delta_trans = noise[2] * delta_trans + noise[3] * (abs(delta_rot1) + abs(delta_rot2))
	sigma_delta_rot2 = noise[0] * abs(delta_rot2) + noise[1] * delta_trans

	noisy_delta_rot1 = delta_rot1 + np.random.normal(0, sigma_delta_rot1)
	noisy_delta_trans = delta_trans + np.random.normal(0, sigma_delta_trans)
	noisy_delta_rot2 = delta_rot2 + np.random.normal(0, sigma_delta_rot2)

	x = sim_odometry[timestep]['x'] +  noisy_delta_trans * np.cos(sim_odometry[timestep]['theta'] + noisy_delta_rot1)
	y = sim_odometry[timestep]['y'] + noisy_delta_trans * np.sin(sim_odometry[timestep]['theta'] + noisy_delta_rot1)
	angle = sim_odometry[timestep]['theta'] + noisy_delta_rot1 

	delta_trans = euclidean_dist((sim_odometry[timestep]['x'],sim_odometry[timestep]['y']), (x,y))
	del_rot1 = math.atan2(y- sim_odometry[timestep]['y'], x- sim_odometry[timestep]['x']) - sim_odometry[timestep]['heading']
	heading = math.atan2(y- sim_odometry[timestep]['y'], x- sim_odometry[timestep]['x'])
	del_rot2 = 0
	sim_odometry[timestep+1] = {'x':x, 'y':y, 'theta':angle, 'trans':delta_trans, 'heading': heading}

	return sim_odometry, delta_trans, delta_rot1, delta_rot2
