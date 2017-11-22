import numpy as np
import math
import matplotlib.pyplot as plt
x= [413058.940761621,  413210.76645718445, 413038.1568685182, 412838.7656277865]

y = [5318228.741414887, 5318363.85196627, 5318629.931794832, 5318476.536999348]

#o1, o2 = 413058.940761621, 5318228.741414887
#ro1,ro2 = 413210.76645718445,  5318363.85196627
#od1, od2 = 160.52209863, 7.05611139535





ox = []
oy = []

for i in range(len(x)):	
	ox.append(x[i] - x[0])
	oy.append(y[i] - y[0])


print(ox, oy)

angle = math.atan(oy[1]/ox[1])

print(angle)

print(oy[1]/ox[1])

angle =  -angle
c = np.cos(angle)
s = np.sin(angle)

Tx = np.array([[c, -s, -x[0]], [s,c, - y[0]],[0,0,1]])

R  = np.array([[c, -s], [s,c]])

P = np.array([[ox[1]], [oy[1]]])

RP = np.dot(R,P)

print("R", R.shape)

print("P", P.shape)


print("RP", RP.shape, RP)

rx= []
ry= []

for i in range(len(x)):
	rx.append(ox[i]*c - oy[i]*s)
	ry.append(ox[i]*s + oy[i]*c)


print(rx,ry)




plt.plot(rx,ry,  marker='o', markersize=3, color="red")

plt.show()
plt.ion()
