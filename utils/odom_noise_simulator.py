import matplotlib.pyplot as plt
import numpy as np

import math

def read_sensor_data(filename):
	sensor_readings = dict()

	lm_ids =[]
	ranges=[]
	bearings=[]

	first_time = True
	timestamp = 0
	f = open(filename)

	for line in f:
	    
	    line_s = line.split('\n') # remove the new line character
	    line_spl = line_s[0].split(' ') # split the line
	    
	    if (line_spl[0]=='ODOMETRY'):
	        
	        sensor_readings[timestamp,'odometry'] = {'r1':float(line_spl[1]),'t':float(line_spl[2]),'r2':float(line_spl[3])}
	        
	        if first_time: 
	            first_time = False
	            
	        else: 
	            sensor_readings[timestamp,'sensor'] = {'id':lm_ids,'range':ranges,'bearing':bearings}                
	            lm_ids=[]
	            ranges = []
	            bearings = []

	        timestamp = timestamp+1
	       
	    if(line_spl[0]=='SENSOR'):
	        
	        lm_ids.append(int(line_spl[1]))    
	        ranges.append(float(line_spl[2]))
	        bearings.append(float(line_spl[3]))
                          
	    sensor_readings[timestamp-1,'sensor'] = {'id':lm_ids,'range':ranges,'bearing':bearings}            
	return sensor_readings



def noisy_odometry(sim_odometry, delta_rot1,delta_trans, delta_rot2, timestep):
	noise = [0.01,0,0,0.01]

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

def sample_odometry_motion_model(odometry, particle, sim_odometry, timestep):
	delta_rot1 = odometry['r1']
	delta_trans = odometry['t']
	delta_rot2 = odometry['r2']

	# the motion noise parameters: [alpha1, alpha2, alpha3, alpha4]
	noise = [0.1, 0.1, 0.05, 0.05]
	print("befre", delta_trans, delta_rot1, delta_rot2 )


	sim_odometry, delta_trans, delta_rot1, delta_rot2 = noisy_odometry(sim_odometry, delta_rot1,delta_trans, delta_rot2, timestep)

	print("after", delta_trans, delta_rot1, delta_rot2 )
	# standard deviations of motion noise
	sigma_delta_rot1 = noise[0] * abs(delta_rot1) + noise[1] * delta_trans
	sigma_delta_trans = noise[2] * delta_trans +   noise[3] * (abs(delta_rot1) + abs(delta_rot2))
	sigma_delta_rot2 = noise[0] * abs(delta_rot2) + noise[1] * delta_trans

	#sample noisy motions
	noisy_delta_rot1 = delta_rot1 + np.random.normal(0, sigma_delta_rot1)
	noisy_delta_trans = delta_trans + np.random.normal(0, sigma_delta_trans)
	noisy_delta_rot2 = delta_rot2 + np.random.normal(0, sigma_delta_rot2)

	#calculate new particle pose
	x = particle['x'] + noisy_delta_trans * np.cos(particle['theta'] + noisy_delta_rot1)
	y = particle['y'] + noisy_delta_trans * np.sin(particle['theta'] + noisy_delta_rot1)
	angle = particle['theta'] + noisy_delta_rot1 + noisy_delta_rot2
	particle['x'] = x
	particle['y'] = y
	particle['theta'] = angle
	
	return particle, sim_odometry

def euclidean_dist(p1, p2):
	return(np.sqrt( (p1[0] - p2[0])**2 + (p1[1] - p2[1])**2 ))


def main():
	p = []
	q = []
	plt.ion()
	plt.clf()

	f1 =  open('../map/odom_trajectory.dat')

	for i in f1:
		l = i.split('\n')
		line = l[0].split()
		x= float(line[0])
		y = float(line[1])
		p.append(x)
		q.append(y)
	plt.plot(p,q, marker='.', markersize=3, color="blue")

	sensor_readings = read_sensor_data("../map/sensor_data_sim.dat")

	particle = dict()
	particle['x'] = 160.52209863 
	particle['y'] = 7.05611139535
	particle['theta'] = 0


	sim_odometry = dict()
	sim_odometry[0] = {'x':160.52209863 , 'y':7.05611139535,  'theta':0, 'heading':1.2490457723982544, 'trans':0.01 }
	
	px = []
	py = []
	qx= []
	qy = []

	print(len(sensor_readings) )
	for timestep in range(len(sensor_readings) / 2):
		particle, sim_odometry = sample_odometry_motion_model(sensor_readings[timestep, 'odometry'], particle, sim_odometry, timestep)
		px.append(particle['x']) 
		py.append(particle['y'])
		qx.append(sim_odometry[timestep]['x']) 
		qy.append(sim_odometry[timestep]['y'])

	plt.plot(px,py, marker='.', markersize=3, color="red")
	plt.plot(qx,qy, marker='.', markersize=3, color="orange")

	plt.show('hold')
	plt.axis([0,300,0,400])
	# plt.show(0.3)


if __name__ == "__main__":
	try:  
		main()
	except KeyboardInterrupt:
		print("Qutting...")

