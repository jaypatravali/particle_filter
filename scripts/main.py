import numpy as np
import matplotlib.pyplot as plt
from motion_model import sample_motion_model
from sensor_model import eval_sensor_model, weighting_model
from resampler import resample_particles, resample_particles_LV
from plotter import plot_state, mean_pose
from read_data import read_world, read_sensor_data, read_odom
from plot_trajectories import plot_trajectories, plot_trajectories_v2, plot_trajectories_v3, plot_on_maps
import argparse
from data_association import data_association

parser = argparse.ArgumentParser(description='Change parameters')
parser.add_argument('--particles', default=1, type=int, metavar='N',
                          help='number of particles')

parser.add_argument('--DA', default="NN", type=str, metavar='N',
                          help='Search Algorithm: Choose NN or JCBB')

parser.add_argument('--noise', default="False", type=bool, metavar='N',
                          help='Add Measurement and Motion Noise?   ')

args = parser.parse_args()


# add random seed for generating comparable pseudo random numbers
np.random.seed(123)

# plot preferences, interactive plotting mode
# plt.axis([0, 220, 0, 320])
# plt.ion()
# plt.show()


def initialize_particles(num_particles, map_limits):
    # randomly initialize the particles inside the map limits
    particles = []

    for i in range(num_particles):
        particle = dict()
        # draw x,y and theta coordinate from uniform distribution inside map limits

        # particle['x'] = np.random.uniform(map_limits[0], map_limits[1])
        # particle['y'] = np.random.uniform(map_limits[2], map_limits[3])
        # particle['theta'] = np.random.uniform(-np.pi, np.pi)

        # Initial Position known
        particle['x'] = 160.52209863 
        particle['y'] = 7.05611139535
        particle['theta'] = 0.0
        particle['errors'] = []

        # particle intialization with distribution mean at initial state unit covraince
        particles.append(particle)
    return particles


def main():
    # implementation of a particle filter for robot pose estimation

    print "Reading landmark positions"
    landmarks = read_world("../map/landmarks_sim.dat")

    print "Reading sensor data"
    sensor_readings = read_sensor_data("../map/sensor_data_sim.dat")

    print "Reading Ground truth Odometry"
    odom_readings = read_odom("../map/odom_trajectory.dat")


    # initialize the particles
    map_limits = [0, 220, 0, 350]
    particles = initialize_particles(args.particles, map_limits)

    curr_pose_x = []
    curr_pose_y = []

    cov_noise = np.array([[0.01, 0, 0],[0,0.01,0],[0,0,0.01]])

    for timestep in range(len(sensor_readings) / 2):
        # plot the current state
        # plot_state(particles, landmarks, map_limits)
        new_particles = sample_motion_model(sensor_readings[timestep, 'odometry'], particles)

        # predict particles by sampling from motion model with odometry info
        if timestep==0:
            new_particles =particles
            errors = data_association(sensor_readings[timestep, 'sensor'], new_particles, particles, landmarks, args.DA, cov_noise, sensor_readings[timestep, 'odometry'])
        else: 
            errors = data_association(sensor_readings[timestep, 'sensor'], new_particles, particles, landmarks, args.DA, cov_noise, sensor_readings[timestep, 'odometry'])

        # calculate importance weights according to sensor model
        weights = eval_sensor_model(sensor_readings[timestep, 'sensor'], new_particles, landmarks)
        
        # weights = weighting_model(errors)

        # resample new particle set according to their importance weights
        particles = resample_particles(new_particles, weights)

        cx, cy, _ = mean_pose(particles)
        curr_pose_x.append(cx)
        curr_pose_y.append(cy)
        # plot_trajectories_v3( sensor_readings[timestep, 'odometry'],cx,cy ,landmarks, map_limits)
        print("Current TimeStep: ", timestep)
        raw_input("Press Enter to continue...")
    # plot_trajectories(odom_readings, curr_pose_x,curr_pose_y ,landmarks, map_limits)
    # plot_on_maps(curr_pose_x, curr_pose_y)
    plt.show('hold')

if __name__ == "__main__":
    main()
