import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import cv2
class Visualization():

	def __init__(self, landmarks, map_limits, sensor_readings, odom_readings):
		self.landmarks = landmarks
		self.map_limits = map_limits
		self.sensor_readings = sensor_readings
		self.odom_readings  = odom_readings
		self.robot_trajectory = []

	def plot_landmarks(self):
		lx=[]
		ly=[]
		# img = cv2.imread('../img2.png')
		# cv2.flip(img,1)
		# cv2.imwrite('../img2.png', img)
		# img=mpimg.imread('../img2.png')

		#plt.axis(self.map_limits)
		for i in range (len(self.landmarks)):
			lx.append(self.landmarks[i+1][0])
			ly.append(self.landmarks[i+1][1])
		plt.plot(lx, ly,  'g*',markersize=5)
		# img_limits =  [0, 300, 0, 300 ]
		# plt.imshow(img , origin="right" , extent = self.map_limits)
        # raw_input("Press Enter to continue...")

	def vis_particles(self, particles):
		"Particles in red"
		xs = []
		ys = []
		for particle in particles:
			xs.append(particle['x'])
			ys.append(particle['y'])

		plt.plot(xs, ys, 'r.')

	def vis_robot_predictions(self, timestep, mean_pose):
		"Black Arrows"
		sensor_data = self.sensor_readings[timestep, 'sensor']
		ranges = sensor_data['range']
		bearing = sensor_data['bearing']

		for i in range(len(ranges)):
			x = np.cos(bearing[i]+ mean_pose[2])* ranges[i]
			y = np.sin(bearing[i] + mean_pose[2])* ranges[i]
			plt.arrow(mean_pose[0], mean_pose[1], x,y, shape='full', lw=1, length_includes_head=True, head_width=.1)

	def vis_robot_trajectory(self, timestep, mean_pose ):
		"Robot Trajectory in Blue"
		self.robot_trajectory.append(mean_pose)
		for i in range(timestep):
			plt.plot(self.robot_trajectory[i][0],self.robot_trajectory[i][1],  marker='o', markersize=5, color="blue")  	

	def vis_ground_truth(self, timestep ):
		"Prints in Green-o"
		for i in range(timestep):
			plt.plot(self.odom_readings[i][0],self.odom_readings[i][1],  marker='o', markersize=2, color="green")  	

	def robot_environment(self, timestep, particles, mean_pose):
		plt.clf()
		plt.ion()
		self.plot_landmarks()
		self.vis_particles(particles)
		self.vis_ground_truth(timestep)
		self.vis_robot_trajectory( timestep, mean_pose )
		self.vis_robot_predictions(timestep, mean_pose)
		plt.pause(0.00001)








 
