import sys

fixed_dir  = "/export/patraval" 
log_dir = "/robo_car_loop2"
cam_dir = "/pg_cam/"
root= fixed_dir+log_dir +cam_dir
fL = open(root + "left.txt" ,'w') 
fR = open(root + "right.txt" ,'w') 


def create_file_path(total):
	for i in range(total):
		fL.write(root+ "left/{}.png\n".format(i))
		fR.write(root+ "right/{}.png\n".format(i))

def main():
	print("Enter total image files")
	create_file_path(int(sys.argv[1]))

if __name__ == "__main__":
    main()
