import numpy as np
import matplotlib
matplotlib.use('GTKAgg') 

import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import matplotlib.patches as patches
import cv2
class Visualization():

	def __init__(self, landmarks, map_limits, sensor_readings, odom_readings):
		self.landmarks = landmarks
		self.map_limits = map_limits
		self.sensor_readings = sensor_readings
		self.odom_readings  = odom_readings
		self.plot_setup()

	def plot_setup(self):
		lx=[]
		ly=[]
		plt.ion()
		self.fig = plt.figure()
		self.ax = self.fig.add_subplot(111)
		self.ax.axis(self.map_limits)
		for i in range (len(self.landmarks)):
			lx.append(self.landmarks[i+1][0])
			ly.append(self.landmarks[i+1][1])
		self.ax.plot(lx, ly,  'g*',markersize=5)

	def vis_particles(self, particles):
		"Particles in red"
		xs = []
		ys = []
		for particle in particles:
			xs.append(particle['x'])
			ys.append(particle['y'])
		particles_obj, = self.ax.plot(xs, ys, 'r.')
		return particles_obj

	def vis_robot_predictions(self, timestep, mean_pose):
		"Black Arrows"
		sensor_data = self.sensor_readings[timestep, 'sensor']
		ranges = sensor_data['range']
		bearing = sensor_data['bearing']
		pred_obj = []
		for i in range(len(ranges)):
			x = np.cos(bearing[i]+ mean_pose[2])* ranges[i]
			y = np.sin(bearing[i] + mean_pose[2])* ranges[i]
			obj = self.ax.arrow(mean_pose[0], mean_pose[1], x,y, shape='full', lw=1, length_includes_head=True, head_width=.1)
			pred_obj.append(obj)
		return pred_obj

	def vis_robot_trajectory(self, timestep, mean_pose ):
		"Robot Trajectory in Blue"
		self.ax.plot(mean_pose[0],mean_pose[1],  marker='o', markersize=2, color="blue",  markeredgewidth=0.0)  	

	def vis_ground_truth(self, timestep ):
		"Prints in Green-o"
		self.ax.plot(self.odom_readings[timestep][0],self.odom_readings[timestep][1],  marker='o', markersize=2, color="green",  markeredgewidth=0.0)  	

	def robot_environment(self, timestep, particles, robot_trajectory, mean_pose, create_vid =None):
		self.vis_ground_truth(timestep)
		self.vis_robot_trajectory( timestep, mean_pose )
		pred_obj = self.vis_robot_predictions(timestep, mean_pose)
		particles_obj =  self.vis_particles(particles)
		self.fig.canvas.flush_events()
		create_vid =1
		if create_vid:
			plt.savefig('../videos2/{}.jpg'.format(timestep),dpi=900)
		particles_obj.remove()
		[i.remove() for i in pred_obj]

	def debugger(self, timestep, mean_pose, particles):
		plt.clf()
		plt.ion()
		lx=[]
		ly=[]
		map_limits = [mean_pose[0]-30, mean_pose[0]+30, mean_pose[1]-30, mean_pose[1]+30]
		plt.axis(map_limits)
		for i in range (1, len(self.landmarks)+1):
			# print((self.landmarks[i][0], self.landmarks[i][1]), (mean_pose[0], mean_pose[1]))
			dist = euclidean_dist((self.landmarks[i][0], self.landmarks[i][1]), (mean_pose[0], mean_pose[1]))			
			if  dist < 30:
				pos_lx  = self.landmarks[i][0]
				pos_ly =  self.landmarks[i][1]
				lx.append(pos_lx)
				ly.append(pos_ly)
				# plt.text(x, y, s, fontdict=None, **kwargs)
				bbox_props = dict(boxstyle="circle,pad=0.5", fc="green", ec="g", lw=1, alpha=0.5)
				plt.text(pos_lx, pos_ly, str(i), bbox=bbox_props)

		self.vis_particles(particles)


		plt.plot(lx, ly,  'g.',markersize=5)

		sensor_data = self.sensor_readings[timestep, 'sensor']
		ranges = sensor_data['range']
		bearing = sensor_data['bearing']

		for i in range(len(ranges)):
			LOCAL_X = np.cos(bearing[i]+ mean_pose[2])* ranges[i]
			LOCAL_Y = np.sin(bearing[i] + mean_pose[2])* ranges[i]
			x  = mean_pose[0]  + LOCAL_X
			y  = mean_pose[1]  + LOCAL_Y
			plt.plot(x, y, marker='o', markersize=5, color="red")
		plt.quiver(mean_pose[0], mean_pose[1], np.cos(mean_pose[2]), np.sin(mean_pose[2]), angles='xy',scale_units='xy')

		create_vid = 0
		if create_vid:
			plt.savefig('../videos/{}.jpg'.format(timestep))
		plt.pause(0.01)


def euclidean_dist(p1, p2):
		return(np.sqrt( (p1[0] - p2[0])**2 + (p1[1] - p2[1])**2 ))

