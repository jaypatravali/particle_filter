import numpy as np

def resample_particles(particles, weights):
	# Returns a new set of particles obtained by performing
	# stochastic universal sampling, according to the particle weights.

	new_particles = []

	# distance between pointers
	step = 1.0/len(particles)
	# random start of first pointer
	u = np.random.uniform(0,step)
	# where we are along the weights
	c = weights[0]
	# index of weight container and corresponding particle
	i = 0
	new_particles = []
	#loop over all particle weights
	for particle in particles:
		#go through the weights until you find the particle
		#to which the pointer points
		while u > c:
			i = i + 1
			c = c + weights[i]

		#add that particle
		new_particles.append(particles[i])

		#increase the threshold
		u = u + step

	return new_particles

def resample_particles_LV(particles, weights):
	# Returns a new set of particles obtained by performing
	# stochastic universal sampling, according to the particle weights.

	new_particles = []

	# distance between pointers
	step = 1.0/len(particles)
	# random start of first pointer
	u = np.random.uniform(0,step)
	# where we are along the weights
	c = weights[0]
	# index of weight container and corresponding particle
	i = 0
	new_particles = []
	#loop over all particle weights
	for particle in particles:
		#go through the weights until you find the particle
		#to which the pointer points

		#increase the threshold
		U = u + step

		while U > c:
			i = i + 1
			c = c + weights[i]

		#add that particle
		new_particles.append(particles[i])



	return new_particles
