import gmplot
import numpy as np
import utm

gmap = gmplot.GoogleMapPlotter(48.013497, 7.834188, 15)

# Landmarks

filename = '../map/landmarks.txt' 

landmarks = dict()
landmarks_latlon = dict()

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


filename = '../map/odom_latlon.txt' 

odom = dict()

f = open(filename)

odomx = []
odomy = []


for line in f:
	line_s  = line.split('\n')
	line_spl  = line_s[0].split()
	odx, ody  = float(line_spl[0]),float(line_spl[1])
	odomx.append(odx)
	odomy.append(ody)


gmap.plot(odomx, odomy, 'green', edge_width=3)


gmap.draw("mymap.html")
