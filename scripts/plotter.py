import numpy as np
import matplotlib.pyplot as plt


def mean_pose(particles):
    # calculate the mean pose of a particle set.
    #
    # for x and y, the mean position is the mean of the particle coordinates
    #
    # for theta, we cannot simply average the angles because of the wraparound 
    # (jump from -pi to pi). Therefore, we generate unit vectors from the 
    # angles and calculate the angle of their average 

    # save x and y coordinates of particles
    xs = []
    ys = []

    # save unit vectors corresponding to particle orientations 
    vxs_theta = []
    vys_theta = []

    for particle in particles:
        xs.append(particle['x'])
        ys.append(particle['y'])

        #make unit vector from particle orientation
        vxs_theta.append(np.cos(particle['theta']))
        vys_theta.append(np.sin(particle['theta']))

    #calculate average coordinates
    mean_x = np.mean(xs)
    mean_y = np.mean(ys)
    mean_theta = np.arctan2(np.mean(vys_theta), np.mean(vxs_theta))
    return [mean_x, mean_y, mean_theta]


# def weighted_average_pose(weights, particles):
#     """ Implements time-window averaged pose"""

#     sort_weights = sorted(weights)

#     sorted_weights[-1]

#     #calculate average coordinates
#     mean_x = np.mean(xs)
#     mean_y = np.mean(ys)
#     mean_theta = np.arctan2(np.mean(vys_theta), np.mean(vxs_theta))
#     return [mean_x, mean_y, mean_theta]


def robust_mean(weights, particles):
    """ Implements time-window averaged pose"""


    sorted_idx = sorted(range(len(weights)), key=lambda k: weights[k])
    xs = []
    ys = []

    # save unit vectors corresponding to particle orientations 
    vxs_theta = []
    vys_theta = []

    for i in range(1,5):
        xs.append(particles[sorted_idx[-i]]['x'])
        ys.append(particles[sorted_idx[-i]]['y'])
        #make unit vector from particle orientation
        vxs_theta.append(np.cos(particles[sorted_idx[-i]]['theta']))
        vys_theta.append(np.sin(particles[sorted_idx[-i]]['theta']))

    mean_x = np.mean(xs)
    mean_y = np.mean(ys)
    mean_theta = np.arctan2(np.mean(vys_theta), np.mean(vxs_theta))
    return [mean_x, mean_y, mean_theta]




def max_weight_pose(weights, particles):
    """ Implements time-window averaged pose"""


    sorted_idx = sorted(range(len(weights)), key=lambda k: weights[k])

    mean_x = particles[sorted_idx[-1]]['x']
    mean_y = particles[sorted_idx[-1]]['y']
    mean_theta = particles[sorted_idx[-1]]['theta']
    return [mean_x, mean_y, mean_theta]



def plot_state(particles, landmarks, map_limits):
    # Visualizes the state of the particle filter.
    #
    # Displays the particle cloud, mean position and landmarks.
    
    xs = []
    ys = []

    for particle in particles:
        xs.append(particle['x'])
        ys.append(particle['y'])

    # landmark positions
    lx=[]
    ly=[]

    for i in range (len(landmarks)):
        lx.append(landmarks[i+1][0])
        ly.append(landmarks[i+1][1])

    # mean pose as current estimate
    estimated_pose = mean_pose(particles)

    # plot filter state
    plt.clf()
    plt.plot(xs, ys, 'r.')
    plt.plot(lx, ly, 'go',markersize=5)
    plt.quiver(estimated_pose[0], estimated_pose[1], np.cos(estimated_pose[2]), np.sin(estimated_pose[2]), angles='xy',scale_units='xy')
    plt.axis(map_limits)

    plt.pause(0.01)



