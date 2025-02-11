import numpy as np
import matplotlib.pyplot as plt
from motion_model import sample_odometry_motion_model, sample_velocity_motion_model, sample_odometry_motion_model_v2
# from sensor_model import eval_sensor_model # weighting_model
from sensor_model import eval_sensor_model

from resampler import resample_particles, resample_particles_LV
from plotter import plot_state, mean_pose, max_weight_pose, robust_mean
from read_data import read_world, read_sensor_data, read_odom
from plot_trajectories import plot_trajectories, plot_trajectories_v2, plot_trajectories_v3, plot_on_maps
from data_association import data_association
from visualization2 import Visualization
from seg_utils import seg_pipeline
from profiler_tools import profiler_tools
from termcolor import cprint, colored
np.random.seed(123)

class Particle_Filter():
    def __init__(self,  init_mode, data_type, play, add_noise, cam_type, motion_model):
        # initialize the particles
        self.init_mode = init_mode
        self.data_type = data_type
        self.play = play
        self.map_limits = [0, 320, 0, 350]
        self.add_noise = add_noise
        self.cam_type  = cam_type
        self.motion_model = motion_model
        self.robot_traj = []


    # def process_disk(self, num_particles):
    #     curr_pose_x = []
    #     curr_pose_y = []
    #     sensor_readings, odom_readings  = self.read_data_disk()
    #     particles, landmarks = self.initialize_particles_landmarks(num_particles)
    #     vis = Visualization(landmarks, self.map_limits, sensor_readings, odom_readings)
    #     sim_odometry = dict()
    #     if self.data_type is 'sim':
    #         sim_odometry[0] = {'x':160.52209863 , 'y':7.05611139535,  'theta':0, 'heading':1.2490457723982544, 'trans':0.01 }        
    #     for timestep in range(len(sensor_readings) / 2):

    #         new_particles = sample_odometry_motion_model(sensor_readings[timestep, 'odometry'], particles,  self.add_noise, timestep, sim_odometry)
    #         curr_mean = mean_pose(new_particles)
    #         curr_pose_x.append(curr_mean[0])
    #         curr_pose_y.append(curr_mean[1])
    #         self.robot_traj.append(curr_mean)
    #         if timestep>10:
    #             vis.robot_environment(timestep, new_particles, self.robot_traj, curr_mean, create_vid=False)
    #         # noisy_meas = self.add_noise_measurement(sensor_readings[timestep, 'sensor'])
    #         weights = eval_sensor_model(sensor_readings[timestep, 'sensor'], new_particles, landmarks)
    #         particles = resample_particles(new_particles, weights)
    #         print("Current TimeStep: ", timestep)
            # curr_mean = robust_mean( weights, new_particles)   # curr_mean = max_weight_pose( weights, new_particles) # curr_mean = weighted_average_pose(new_particles, weights)
            # curr_mean = max_weight_pose(weights,new_particles)

    def process_disk(self, num_particles):
        curr_pose_x = []
        curr_pose_y = []
        sensor_readings, odom_readings  = self.read_data_disk()
        particles, landmarks = self.initialize_particles_landmarks(num_particles)

        vis = Visualization(landmarks, self.map_limits, sensor_readings, odom_readings)
        sim_odometry = dict()
        if self.data_type is 'sim':
            sim_odometry[0] = {'x':160.52209863 , 'y':7.05611139535,  'theta':0, 'heading':1.2490457723982544, 'trans':0.01 }
        profiler = profiler_tools()
        weights  = 1.0/len(particles)

        for timestep in range(1 ,1600):
            profiler.start_profiler()
            new_particles = sample_odometry_motion_model(sensor_readings[timestep, 'odometry'], particles, self.add_noise, timestep, sim_odometry)
            profiler.stop_profiler(timestep, 'motion_model')
            # new_particles = sample_velocity_motion_model(sensor_readings[timestep, 'odometry'], particles)

            profiler.start_profiler()
            weights = eval_sensor_model(sensor_readings[timestep, 'sensor'], new_particles, landmarks)
            profiler.stop_profiler(timestep, 'sensor_model')

            profiler.start_profiler()
            particles = resample_particles(new_particles, weights)
            profiler.stop_profiler(timestep, 'resampling')

            profiler.start_profiler()
            curr_mean = mean_pose(new_particles)

            curr_pose_x.append(curr_mean[0])
            curr_pose_y.append(curr_mean[1])
            self.robot_traj.append(curr_mean)


            if timestep>7750:
            	vis.robot_environment(timestep, new_particles, self.robot_traj, curr_mean, create_vid=False)
            


            profiler.stop_profiler(timestep, 'visualization', True)
            print colored("Current TimeStep: {}".format(timestep), "red")
            # raw_input("Press Enter to continue...")

        profiler.runtime_plot()
        plot_trajectories(odom_readings, curr_pose_x,curr_pose_y ,landmarks, self.map_limits, sim_odometry)
        plot_on_maps(odom_readings, curr_pose_x, curr_pose_y)
        plt.show('hold')

    def add_noise_measurement(self, sensor_data):

        sigma_r = 0
        sigma_phi = 0.0

        ranges = sensor_data['range']
        bearing = sensor_data['bearing']
        weights = []

        noisy_range = []
        noisy_bearing =[]
        for i in range(len(ranges)):
            noisy_range.append( ranges[i] + np.random.normal(0, sigma_r))
            noisy_bearing.append(bearing[i] + np.random.normal(0, sigma_phi))
        sensor_data['range'] = noisy_range
        sensor_data['bearing'] =noisy_bearing
        return sensor_data

    def process_realtime(self, num_particles):
        curr_pose_x = []
        curr_pose_y = []
        particles, landmarks = self.initialize_particles_landmarks(num_particles)
        sensor_readings, odom_readings  = self.read_data_disk()
        vis = Visualization(landmarks, self.map_limits, sensor_readings, odom_readings)
        print(self.cam_type)
        seg_obj = seg_pipeline.Segmentation_Pipeline(cam_type='pg', realtime= True)
        particles = sample_odometry_motion_model(sensor_readings[0, 'odometry'], particles, self.add_noise, 0 )

        for timestep in range(1, len(sensor_readings) / 2):
            # plot_state(particles, landmarks, map_limits)
            # curr_mean = mean_pose(particles)
            # curr_pose_x.append(curr_mean[0])
            # curr_pose_y.append(curr_mean[1])
            new_particles = sample_odometry_motion_model(sensor_readings[timestep, 'odometry'], particles, self.add_noise, timestep)
            curr_mean = mean_pose(new_particles)
            curr_pose_x.append(curr_mean[0])
            curr_pose_y.append(curr_mean[1])
            # vis.robot_environment(timestep, new_particles, curr_mean)
            vis.debugger(timestep,  curr_mean , particles)
            # seg_obj.start_process_realtime(timestep)
            weights = eval_sensor_model(sensor_readings[timestep, 'sensor'], new_particles, landmarks)
            # weights = weighting_model(errors)
            particles = resample_particles(new_particles, weights)
            # print("Current TimeStep: ", timestep)
            # raw_input("Press Enter to continue...")
        # plot_trajectories(odom_readings, curr_pose_x,curr_pose_y ,landmarks, self.map_limits)
        # plot_on_maps(curr_pose_x, curr_pose_y)
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
                particle['weight'] = 1.0 / num_particles
                particles.append(particle)

        if self.init_mode =='set' and self.data_type=='car':
            for i in range(num_particles):
                particle = dict()
                # draw x,y and theta coordinate from car starting position
                """zed straight"""
                #particle['x'], particle['y'] = 177.03757970060997,72.711216426459998
                #particle['theta'] = 1.3540745849092297  # ([106.67983715984272, 259.16103693684363], -1.4253166861206177)
                """pg loop"""
                # particle['x'], particle['y'] = 106.67983715984272, 259.16103693684363
                # particle['theta'] =  -1.4253166861206177  

                """pg corner"""
                particle['x'], particle['y'] = 126.36463623279457, 197.40703201640946
                particle['theta'] = 3.077658478440323

                """zed loop"""
                # particle['x'], particle['y'] =  112.21525525427808, 249.03124386039127 
                # particle['theta'] =  -0.8590560204044709 
                particle['errors'] = []
                particle['weight'] = 1.0 / num_particles

                particles.append(particle)
           
        if self.init_mode =='rand' and self.data_type=='sim':
            for i in range(num_particles):
                particle = dict()
                particle['x'] = np.random.uniform(self.map_limits[0], self.map_limits[1])
                particle['y'] = np.random.uniform(self.map_limits[2], self.map_limits[3])
                particle['theta'] = np.random.uniform(-np.pi, np.pi)
                particle['errors'] = []
                particle['weight'] = 1.0 / num_particles
                particles.append(particle)
           
        if self.init_mode =='set' and self.data_type=='sim':
            for i in range(num_particles):
                particle = dict()
                # Initial Position known
                particle['x'] = 160.52209863 
                particle['y'] = 7.05611139535
                particle['theta'] = 0
                particle['errors'] = []
                particle['weight'] = 1.0 / num_particles              
                particles.append(particle)
        return particles, landmarks

    def read_data_disk(self):
        if self.data_type =='sim':
            print("Reading sensor data: {0} ".format(self.data_type)) 
            sensor_readings = read_sensor_data("../map/sensor_data_sim.dat")
            print "Reading Ground truth Odometry"
            odom_readings = read_odom("../map/odom_trajectory.dat")

        if self.data_type =='car' and self.cam_type =='zed':
            print("Reading sensor data: {0} from -> {1}".format(self.data_type, self.play)) 
            sensor_readings = read_sensor_data("../map/sensor_data_car_zed.dat")
            print "Reading Ground truth Odometry"
            odom_readings = read_odom("../map/odom_trajectory_car_zed.dat")

        if self.motion_model =='odometry' and self.data_type =='car' and self.cam_type =='pg':
            print("Reading sensor data: {0} from -> {1}".format(self.data_type, self.play))
            print("../map/cluster_sensor_data_car_pg.dat") 
            sensor_readings = read_sensor_data("../map/sensor_data_car_pg.dat")
            print "Reading Ground truth Odometry"
            odom_readings = read_odom("../map/odom_trajectory_car_pg.dat")


        if self.motion_model =='odometry' and self.data_type =='car' and self.cam_type =='pg_cluster':
            print colored("Reading sensor data: {0} from -> {1} of type **{2}**".format(self.data_type, self.play, self.cam_type), "blue")
            sensor_readings = read_sensor_data("../map/cluster_sensor_data_car_pg.dat")
            print "Reading Ground truth Odometry"
            odom_readings = read_odom("../map/odom_trajectory_car_pg.dat")

        if self.motion_model =='velocity' and self.cam_type =='pg':
            print("Reading sensor data: {0} from -> {1}".format(self.data_type, self.play)) 
            sensor_readings = read_sensor_data("../map/velocity_sensor_data_car_pg.dat", self.motion_model)
            print "Reading Ground truth Odometry"
            odom_readings = read_odom("../map/odom_trajectory_car_pg.dat")

        return sensor_readings, odom_readings

    def read_data_realtime(self):
        print("Reading sensor data: {0} from -> {1}".format(self.data_type, self.play)) 
        sensor_readings, odom_readings = self.extractor.get 
        return sensor_readings, odom_readings

