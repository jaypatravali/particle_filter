import numpy as np
import matplotlib.pyplot as plt

def plot_trajectories_v3(sensor_data , curr_mean, landmarks, map_limits):
    # landmark positions
    lx=[]
    ly=[]

    for i in range (len(landmarks)):
        lx.append(landmarks[i+1][0])
        ly.append(landmarks[i+1][1])

    plt.figure(2)
    plt.clf()
    plt.plot(lx, ly, 'g*',markersize=5)

    ranges = sensor_data['range']
    bearing = sensor_data['bearing']


    for i in range(len(ranges)):
        x = np.cos(bearing[i])* ranges[i]
        y = np.sin(bearing[i])* ranges[i]
        print(x,y)
        plt.arrow(curr_mean[0],curr_mean[1], x,y, shape='full', lw=3, length_includes_head=True, head_width=.01)


    plt.plot(curr_mean[0],curr_mean[1],  marker='o', markersize=5, color="red")
    # plt.quiver(curr_mean[0], curr_mean[1], np.cos(curr_mean[2]), np.sin(curr_mean[2]), angles='xy',scale_units='xy', width= 0.005)

    plt.axis(map_limits)
    plt.ion()
    # plt.show()

    plt.pause(0.00001)

def plot_trajectories(odom_readings, curr_pose_x, curr_pose_y,landmarks, map_limits, sim_odometry):

    odomx = []
    odomy = []
    for i in range(len(curr_pose_x)):
        odomx.append(odom_readings[i][0])
        odomy.append(odom_readings[i][1])

    # landmark positions
    lx=[]
    ly=[]

    for key in  landmarks:
        lx.append(landmarks[key][0])
        ly.append(landmarks[key][1])

    plt.figure(2)


    # im = plt.imread('../map/sat_img.png')
    # implot = plt.imshow(im)

    curr_pose_x[0] = odomx[0]
    curr_pose_y[0]  = odomy[0]
    plt.plot(lx, ly, 'go',markersize=5)

    plt.plot(curr_pose_x[0], curr_pose_y[0],  marker='x', markersize=5, color="black")

    sx=[]
    sy=[]
    for key in sim_odometry:
        sx.append(sim_odometry[key]['x'])
        sy.append(sim_odometry[key]['y'])

    if sim_odometry:
        p1, = plt.plot(curr_pose_x,curr_pose_y,  marker='.', markersize=3, color="red")
        p2, = plt.plot(odomx,odomy, marker='.', markersize=3, color="blue")
        p3, = plt.plot(sx,sy, marker='.', markersize=3, color="orange")
        plt.legend([p1,p2,p3], ['PF trajectory', 'Ground truth', 'Noisy Odometry'])
    else:
        p1, = plt.plot(curr_pose_x,curr_pose_y,  marker='.', markersize=3, color="red")
        p2, = plt.plot(odomx,odomy, marker='.', markersize=3, color="blue")
        plt.legend([p1,p2], ['PF trajectory', 'Odometry Ground truth'])
    plt.axis(map_limits)

    err_sum = 0
    #plot differences
    for i in range(len(curr_pose_x)):
        err_x =  odomx[i] - curr_pose_x[i]
        err_y =  odomy[i] - curr_pose_y[i]
        abs_arr = np.sqrt(err_x**2 + err_y**2)
        err_sum  = err_sum + abs_arr

    mean = err_sum/len(curr_pose_x)

    bracket_dev = []

    for i in range(len(curr_pose_x)):
        err_x =  odomx[i] - curr_pose_x[i]
        err_y =  odomy[i] - curr_pose_y[i]
        abs_arr = np.sqrt(err_x**2 + err_y**2)
        bracket_dev.append((abs_arr-mean)**2)

    std_dev = np.sqrt(sum(bracket_dev)/ len(bracket_dev))
    print(std_dev, mean)



def plot_trajectories_v2(o1, curr_pose_x, curr_pose_y,landmarks, map_limits):


    # landmark positions
    lx=[]
    ly=[]

    for i in range (len(landmarks)):
        lx.append(landmarks[i+1][0])
        ly.append(landmarks[i+1][1])

    plt.figure(2)


    # im = plt.imread('../map/sat_img.png')
    # implot = plt.imshow(im)

    plt.plot(lx, ly, 'g*',markersize=5)

    p1, = plt.plot(curr_pose_x,curr_pose_y,  marker='o', markersize=5, color="red")
    p2, = plt.plot(o1[0],o1[1], marker='x', markersize=7, color="blue")

    plt.legend([p1,p2], ['PF trajectory', 'Odometry Ground truth'])
    # plt.legend([p1], ['PF trajectory'])

    # plt.axis(map_limits)

import gmplot
import numpy as np
import utm
import math

def plot_on_maps(odom_readings, curr_pose_x, curr_pose_y):

    gmap = gmplot.GoogleMapPlotter(48.013497, 7.834188, 15)

    # Landmarks

    filename = '../map/landmarks.txt' 
    f = open(filename)

    lmx = []
    lmy = []

    for line in f:
        line_s  = line.split('\n')
        line_spl  = line_s[0].split()
        lx, ly = float(line_spl[1]),float(line_spl[2])
        lmx.append(lx)
        lmy.append(ly)

    gmap.scatter(lmx, lmy, 'k', marker=True)

    # #odom 
    # filename = '../map/odom_latlon.txt' 

    # f = open(filename)

    # odomx = []
    # odomy = []

    # for line in f:
    #     line_s  = line.split('\n')
    #     line_spl  = line_s[0].split()
    #     odx, ody  = float(line_spl[0]),float(line_spl[1])
    #     odomx.append(odx)
    #     odomy.append(ody)

    #PF Predictions 
    ro1 = 413210.76645718445
    ro2 = 5318363.85196627
    to1 = 413058.940761621
    to2 = 5318228.741414887
    angle = math.atan((ro2 -to2)/(ro1-to1))
    c = np.cos(angle)
    s = np.sin(angle)

    odomx = []
    odomy = []

    for i in range(len(curr_pose_x)):
        x = odom_readings[i][0]
        y = odom_readings[i][1]
        rx = x*c - y*s
        ry = x*s + y*c
        rtx = rx + to1
        rty = ry + to2

        x,y = utm.to_latlon(rtx,rty, 32, zone_letter='U')
        odomx.append(x)
        odomy.append(y)


    gmap.plot(odomx, odomy, 'green', edge_width=3)

    re_poseX = []
    re_poseY = []

    for i in range(len(curr_pose_x)):
        x = curr_pose_x[i]
        y = curr_pose_y[i]
        rx = x*c - y*s
        ry = x*s + y*c
        rtx = rx + to1
        rty = ry + to2

        x,y = utm.to_latlon(rtx,rty, 32, zone_letter='U')
        re_poseX.append(x)
        re_poseY.append(y)

    gmap.plot(re_poseX, re_poseY, 'red', edge_width=3)

    gmap.draw("../results/mymap.html")




