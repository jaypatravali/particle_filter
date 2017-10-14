import utm

filename = '../map/landmarks.txt' 

landmarks = dict()

f = open(filename)

for line in f:

	line_s  = line.split('\n')
	line_spl  = line_s[0].split()
	print( float(line_spl[1]),float(line_spl[2]))
	print(utm.from_latlon(float(line_spl[1]),float(line_spl[2])))