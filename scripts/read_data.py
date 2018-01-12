import utm
#global origin

def read_world(filename):
    # Reads the world definition and returns a list of landmarks, our 'map'.
    # 
    # The returned dict contains a list of landmarks each with the
    # following information: {id, [x, y]}

    landmarks = dict()

    f = open(filename)
    
    for line in f:

        line_s  = line.split('\n')    
        line_spl  = line_s[0].split(' ')
        landmarks[int(line_spl[0])] = [float(line_spl[1]),float(line_spl[2])]      

    return landmarks

def read_sensor_data(filename, motion_model=None):
    # Reads the odometry and sensor readings from a file.
    #
    # The data is returned in a dict where the u_t and z_t are stored
    # together as follows:
    # 
    # {odometry,sensor}
    #
    # where "odometry" has the fields r1, r2, t which contain the values of
    # the identically named motion model variables, and sensor is a list of
    # sensor readings with id, range, bearing as values.
    #
    # The odometry and sensor values are accessed as follows:
    # odometry_data = sensor_readings[timestep, 'odometry']
    # sensor_data = sensor_readings[timestep, 'sensor']

    sensor_readings = dict()

    lm_ids =[]
    ranges=[]
    bearings=[]

    first_time = True
    timestamp = 0
    f = open(filename)
    if motion_model == 'velocity':
        for line in f:
            
            line_s = line.split('\n') # remove the new line character
            line_spl = line_s[0].split(' ') # split the line
            
            if (line_spl[0]=='ODOMETRY'):
                
                sensor_readings[timestamp,'odometry'] = {'v':float(line_spl[1]),'w':float(line_spl[2]),'dt':float(line_spl[3])}
                
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
    

    else:
        for line in f:
            line_s = line.split('\n') # remove the new line character
            line_spl = line_s[0].split(' ') # split the line
                
            # print(line_spl)
            # raw_input("Press Enter to continue...")

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



def read_odom(filename):
    # Reads odom ground truth

    odom = dict()

    f = open(filename)
    count = 0
    for line in f:

        line_s  = line.split('\n')    
        line_spl  = line_s[0].split()
        odom[int(count)] = [float(line_spl[0]),float(line_spl[1])]      
        count = count + 1
    return odom