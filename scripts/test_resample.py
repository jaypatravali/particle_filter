import numpy as np
import matplotlib.pyplot as plt
from resampler import resample_particles, resample_particles_LV
from profiler_tools import profiler_tools


def main():
	profiler = profiler_tools()

	for timestep in range(1,2000):

		particles = []
		map_limits = [0, 320, 0, 350]

		for i in range(1000):
			particle = dict()
			# draw x,y and theta coordinate from uniform distribution inside map limits
			particle['x'] = np.random.uniform(map_limits[0], map_limits[1])
			particle['y'] = np.random.uniform(map_limits[2], map_limits[3])
			particle['theta'] = np.random.uniform(-np.pi, np.pi)
			particle['errors'] = []
			particle['weight'] = 1.0 / 1000
			particles.append(particle)


		weights = []

		for particle in particles:
			all_meas_likelihood = 1.0 #for combining multiple measurements
			weights.append(np.random.uniform(0,1))
		profiler.start_profiler()
		particles = resample_particles(particles, weights)
		profiler.stop_profiler(timestep, 'resampling', True)
		print("Current TimeStep: ", timestep)

		profiler.runtime_plot()

if __name__ =="__main__":
	main()