import numpy as np
import sys
import matplotlib.pyplot as plt
from motion_model import sample_motion_model
from sensor_model import eval_sensor_model, weighting_model
from resampler import resample_particles, resample_particles_LV
from plotter import plot_state, mean_pose
from read_data import read_world, read_sensor_data, read_odom
from plot_trajectories import plot_trajectories, plot_trajectories_v2, plot_trajectories_v3, plot_on_maps
from data_association import data_association
from visualization import Visualization

# add random seed for generating comparable pseudo random numbers
np.random.seed(123)



class Particle_Filter():
    def __init__(self,  init_mode, data_type, play):
        # initialize the particles
        self.init_mode = init_mode
        self.data_type = data_type
        self.play = play
        self.map_limits = [0, 320, 0, 350]

    def process_disk(self, num_particles):
        curr_pose_x = []
        curr_pose_y = []
        sensor_readings, odom_readings  = self.read_data_disk()
        particles, landmarks = self.initialize_particles_landmarks(num_particles)
        vis = Visualization(landmarks, self.map_limits, sensor_readings, odom_readings)
        for timestep in range(len(sensor_readings) / 2):
            # plot_state(particles, landmarks, map_limits)
            # curr_mean = mean_pose(particles)
            # curr_pose_x.append(curr_mean[0])
            # curr_pose_y.append(curr_mean[1])
            new_particles = sample_motion_model(sensor_readings[timestep, 'odometry'], particles)
            curr_mean = mean_pose(new_particles)
            curr_pose_x.append(curr_mean[0])
            curr_pose_y.append(curr_mean[1])
            # self.vis.robot_environment(timestep, new_particles, curr_mean)
            vis.robot_environment(timestep, new_particles, curr_mean)

            # predict particles by sampling from motion model with odometry info
            # if timestep==0:
            #     new_particles =particles
            #     errors = data_association(sensor_readings[timestep, 'sensor'], new_particles, particles, landmarks, args.DA, cov_noise, sensor_readings[timestep, 'odometry'])
            # else: 
            #     errors = data_association(sensor_readings[timestep, 'sensor'], new_particles, particles, landmarks, args.DA, cov_noise, sensor_readings[timestep, 'odometry'])

            weights = eval_sensor_model(sensor_readings[timestep, 'sensor'], new_particles, landmarks)
            
            # weights = weighting_model(errors)

            particles = resample_particles(new_particles, weights)
            print("Current TimeStep: ", timestep)
            # raw_input("Press Enter to continue...")
        plot_trajectories(odom_readings, curr_pose_x,curr_pose_y ,landmarks, self.map_limits)
        plot_on_maps(curr_pose_x, curr_pose_y)
        plt.show('hold')

    def process_realtime(self):
        particles, landmarks = self.initialize_particles_landmarks(num_particles)
    
        while(len(sensor_readings)>0):
            sensor_readings, odom_readings =  self.read_data_realtime()

            # plot_state(particles, landmarks, map_limits)
            # curr_mean = mean_pose(particles)
            # curr_pose_x.append(curr_mean[0])
            # curr_pose_y.append(curr_mean[1])
            new_particles = sample_motion_model(sensor_readings['odometry'], particles)

            curr_mean = mean_pose(new_particles)
            curr_pose_x.append(curr_mean[0])
            curr_pose_y.append(curr_mean[1])

            # predict particles by sampling from motion model with odometry info
            # if timestep==0:
            #     new_particles =particles
            #     errors = data_association(sensor_readings[timestep, 'sensor'], new_particles, particles, landmarks, args.DA, cov_noise, sensor_readings[timestep, 'odometry'])
            # else: 
            #     errors = data_association(sensor_readings[timestep, 'sensor'], new_particles, particles, landmarks, args.DA, cov_noise, sensor_readings[timestep, 'odometry'])

            weights = eval_sensor_model(sensor_readings[ 'sensor'], new_particles, landmarks)
            # weights = weighting_model(errors)
            particles = resample_particles(new_particles, weights)
            print("Current TimeStep: ", timestep)
            raw_input("Press Enter to continue...")

        plot_trajectories(odom_readings, curr_pose_x,curr_pose_y ,landmarks, self.map_limits)
        plot_on_maps(curr_pose_x, curr_pose_y)
        plt.show('hold')




    def initialize_particles_landmarks(self,  num_particles): 

        print "Reading landmark positions"
        landmarks = read_world("../map/landmarks_sim.dat")

        # randomly initialize the particles inside the map limits
        particles = []

        if self.init_mode =='rand' and self.data_type=='car':
            for i in range(num_particles):
                particle = dict()
                # draw x,y and theta coordinate from uniform distribution inside map limits
                particle['x'] = np.random.uniform(self.map_limits[0], self.map_limits[1])
                particle['y'] = np.random.uniform(self.map_limits[2], self.map_limits[3])
                particle['theta'] = np.random.uniform(-np.pi, np.pi)
                particle['errors'] = []
                particles.append(particle)

        if self.init_mode =='set' and self.data_type=='car':
            for i in range(num_particles):
                particle = dict()
                # draw x,y and theta coordinate from car starting position
                particle['x'], particle['y'] = 177.03757970060997,72.711216426459998
                particle['theta'] = 1.3540745849092297
                particle['errors'] = []
                particles.append(particle)
           
        if self.init_mode =='rand' and self.data_type=='sim':
            for i in range(num_particles):
                particle = dict()
                particle['x'] = np.random.uniform(self.map_limits[0], self.map_limits[1])
                particle['y'] = np.random.uniform(self.map_limits[2], self.map_limits[3])
                particle['theta'] = np.random.uniform(-np.pi, np.pi)
                particle['errors'] = []
                particles.append(particle)
           
        if self.init_mode =='set' and self.data_type=='sim':
            for i in range(num_particles):
                particle = dict()
                # Initial Position known
                particle['x'] = 160.52209863 
                particle['y'] = 7.05611139535
                particle['theta'] = 0
                particle['errors'] = []
                particles.append(particle)
        return particles, landmarks


    def read_data_disk(self):
        if self.data_type =='sim':
            print("Reading sensor data: {0} ".format(self.data_type)) 
            sensor_readings = read_sensor_data("../map/sensor_data_sim.dat")
            print "Reading Ground truth Odometry"
            odom_readings = read_odom("../map/odom_trajectory.dat")

        if self.data_type =='car':
            print("Reading sensor data: {0} from -> {1}".format(self.data_type, self.play)) 
            sensor_readings = read_sensor_data("../map/sensor_data_car.dat")
            print "Reading Ground truth Odometry"
            odom_readings = read_odom("../map/odom_trajectory_car.dat")
        return sensor_readings, odom_readings

    def read_data_realtime(self):
        print("Reading sensor data: {0} from -> {1}".format(self.data_type, self.play)) 
        sensor_readings, odom_readings = self.extractor.get 
        return sensor_readings, odom_readings





# def Visualizer(self, timestep, particle, mean_pose):
#     vis = Visualization(self.landmarks, self.map_limits, self.sensor_readings, self.odom_readings)
#     vis.robot_environment(timestep, new_particles, curr_mean)


# class Fitler_Setup():

#     def __init__(self, init_mode='set', data_type='sim', reader ='disk'):
#         self.init_mode = init_mode
#         self.data_type = data_type
#         self.play = play
#         self.map_limits = [0, 320, 0, 350]

#     def initialize_particles(self,  num_particles, map_limits): 
#         # randomly initialize the particles inside the map limits
#         particles = []

#         if self.init_mode =='rand' and self.data_type=='car':
#             for i in range(num_particles):
#                 particle = dict()
#                 # draw x,y and theta coordinate from uniform distribution inside map limits
#                 particle['x'] = np.random.uniform(map_limits[0], map_limits[1])
#                 particle['y'] = np.random.uniform(map_limits[2], map_limits[3])
#                 particle['theta'] = np.random.uniform(-np.pi, np.pi)
#                 particle['errors'] = []
#                 particles.append(particle)

#         if self.init_mode =='set' and self.data_type=='car':
#             for i in range(num_particles):
#                 particle = dict()
#                 # draw x,y and theta coordinate from car starting position
#                 particle['x'], particle['y'] = 177.03757970060997,72.711216426459998
#                 particle['theta'] = 1.3540745849092297
#                 particle['errors'] = []
#                 particles.append(particle)
           
#         if self.init_mode =='rand' and self.data_type=='sim':
#             for i in range(num_particles):
#                 particle = dict()
#                 particle['x'] = np.random.uniform(map_limits[0], map_limits[1])
#                 particle['y'] = np.random.uniform(map_limits[2], map_limits[3])
#                 particle['theta'] = np.random.uniform(-np.pi, np.pi)
#                 particle['errors'] = []
#                 particles.append(particle)
           
#         if self.init_mode =='set' and self.data_type=='sim':
#             for i in range(num_particles):
#                 particle = dict()
#                 # Initial Position known
#                 particle['x'] = 160.52209863 
#                 particle['y'] = 7.05611139535
#                 particle['theta'] = 0
#                 particle['errors'] = []
#                 particles.append(particle)
#         return particles


#     def read_data_disk(self):
#         print "Reading landmark positions"
#         landmarks = read_world("../map/landmarks_sim.dat")

#         if self.data_type =='sim':
#             print("Reading sensor data: " + self.data_type) 
#             sensor_readings = read_sensor_data("../map/sensor_data_sim.dat")
#             print "Reading Ground truth Odometry"
#             odom_readings = read_odom("../map/odom_trajectory.dat")

#         if self.data_type =='car':
#             print("Reading sensor data: " + self.data_type, "from ->", + self.play) 
#             sensor_readings = read_sensor_data("../map/sensor_data_sim.dat")
#             print "Reading Ground truth Odometry"
#             odom_readings = read_odom("../map/odom_trajectory.dat")
#         return landmarks, sensor_readings, odom_readings

#     def read_data_realtime(self):
#         print "Reading landmark positions"
#         landmarks = read_world("../map/landmarks_sim.dat")
#         print("Reading sensor data: ", + self.data_type, "from ->", + self.play) 
#         sensor_readings, odom_readings = self.extractor.get 
#         return landmarks, sensor_readings, odom_readings