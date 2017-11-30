import matplotlib.pyplot as plt

p = []
q = []
plt.ion()
plt.clf()
f =  open('landmarks_pred_car.dat')
for i in f:
	l = i.split('\n')
	line = l[0].split()
	x= float(line[0])
	y = float(line[1])
	plt.plot(x,y,	'g*',markersize=5)

f1 =  open('odom_trajectory_car.dat')

for i in f1:
	l = i.split('\n')
	line = l[0].split()
	x= float(line[0])
	y = float(line[1])
	plt.plot(x,y,	'r*',markersize=2)



plt.axis([100,300,0,400])
plt.show(0.3)
 
