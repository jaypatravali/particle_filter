import cv2



def compute_disparity():
	imgL = cv2.imread('/export/patraval/robo_car_images2/zed_front/left/38.png', 0)
	imgR = cv2.imread('/export/patraval/robo_car_images2/zed_front/right/38.png', 0 )

	# imgL = cv2.imread('../../left', 0)
	# imgR = cv2.imread('../../right', 0)


	# imgL = cv2.resize(imgL,(1280, 720), interpolation = cv2.INTER_CUBIC)
	# imgR = cv2.resize(imgR,(1280, 720), interpolation = cv2.INTER_CUBIC)

	# stereo = cv2.StereoBM_create(numDisparities=16, blockSize=15)
	window_size = 3
	min_disp = 16
	num_disp = 112-min_disp

	# stereo = cv2.StereoSGBM_create(minDisparity=-39, numDisparities= 112,blockSize = 15 )

	# stereo = cv2.StereoSGBM_create(minDisparity = min_disp,
	# numDisparities = num_disp,
	# blockSize = 16,
	# P1 = 8*3*window_size**2,
	# P2 = 32*3*window_size**2,
	# disp12MaxDiff = 1,
	# uniquenessRatio = 10,
	# speckleWindowSize = 100,
	# speckleRange = 32
	# )

	stereo = cv2.StereoBM_create(16, 15)

	disparity = stereo.compute(imgL, imgR)
	# plt.imshow(disparity,'gray')
	# # plt.imshow(imgL,'imgL')
	# plt.show()	
	# cv2.imshow('wow', disparity)
	# cv2.waitKey(0)
	cv2.imwrite("yolo.png", disparity)




def img_resize(list1, list2):
	dir1 = '/export/patraval/robo_car_images/pg_cam/rect/re_left5/'
	dir2= '/export/patraval/robo_car_images/pg_cam/rect/re_right5/'

	height, width = 540, 960

	# height, width = 568, 1024


	for i in range(len(list1)):
		left = cv2.imread(list1[i])
		res_l = cv2.resize(left,(width,height), interpolation = cv2.INTER_CUBIC)
		cv2.imwrite( dir1+"{}.png".format(i) , res_l)
		
		right = cv2.imread(list2[i])
		res_r = cv2.resize(right,(width, height), interpolation = cv2.INTER_CUBIC)
		cv2.imwrite( dir2+"{}.png".format(i) , res_r)



def crop_img(list1, list2):
	dir1 = '/export/patraval/robo_car_images/pg_cam/rect/crop_left/'
	dir2= '/export/patraval/robo_car_images/pg_cam/rect/crop_right/'

	height, width = 540, 960

	# height, width = 568, 1024


	for i in range(len(list1)):
		left = cv2.imread(list1[i])
		crop_img1 = left[0:1024, 0:2048] 
		cv2.imwrite( dir1+"{}.png".format(i) , crop_img1)
		
		right = cv2.imread(list2[i])
		crop_img2 = right[0:1024, 0:2048] 
		cv2.imwrite( dir2+"{}.png".format(i) , crop_img2)



def crop_img_resize(list1, list2):
	dir1 = '/export/patraval/robo_car_loop2/pg_cam/rect/left_res/'
	dir2 = '/export/patraval/robo_car_loop2/pg_cam/rect/right_res/'

	height, width = 384, 768

	print("Check if Directories are okay [Y/N]")
	if raw_input()!="Y":
		exit()
	print("All good!")
	for i in range(len(list1)):
		left = cv2.imread(list1[i])
		crop_img1 = left[0:1024, 0:2048] 
		res_l = cv2.resize(crop_img1,(width, height), interpolation = cv2.INTER_CUBIC)
		cv2.imwrite( dir1+"{}.png".format(i) , res_l)
		
		right = cv2.imread(list2[i])
		crop_img2 = right[0:1024, 0:2048] 
		res_r = cv2.resize(crop_img2,(width, height), interpolation = cv2.INTER_CUBIC)
		cv2.imwrite( dir2+"{}.png".format(i) , res_r)
		print("Files done: ", i)

def unpack_files():
	left = []
	right = []
	disparity = []

	left = [line.rstrip('\n') for line in open('/export/patraval/robo_car_loop2/pg_cam/rect/left_res.txt')]
	right = [line.rstrip('\n') for line in open('/export/patraval/robo_car_loop2/pg_cam/rect/right_res.txt')]

	return left, right, disparity

def main():
	left_list, right_list, disparity_list =  unpack_files()
	#img_resize(left_list, right_list)
	# crop_img(left_list, right_list)
	crop_img_resize(left_list, right_list)

	# compute_disparity()

if __name__ == "__main__":
    main()





# def  writer(l_img, r_img, f1, f2):
# 	height, width = 384, 768
# 	print(height, width)
# 	left = cv2.imread(l_img)
# 	res_l = cv2.resize(left,(width,height), interpolation = cv2.INTER_CUBIC)
# 	cv2.imwrite(l_img, res_l)
# 	f1.write(l_img + "\n")
# 	right = cv2.imread(r_img)
# 	res_r = cv2.resize(right,(width, height), interpolation = cv2.INTER_CUBIC)
# 	cv2.imwrite(r_img, res_r)
# 	f2.write(r_img + "\n")


# def processor(left_cam, right_cam):

# 	left_imgs = []
# 	right_imgs = []
# 	f1 = open("../left.txt",'w') 
# 	f2 = open("../right.txt",'w') 
# 	for i in range(480):
# 	    left_img =  '/export/patraval/robo_car_images/zed_front/left/' + "{}".format(i) + '.png'
# 	    right_img = '/export/patraval/robo_car_images/zed_front/right/' + "{}".format(i) + '.png'
# 	    left_img_write =  '/export/patraval/robo_car_images/zed_front/left/' + "{}".format(i) + '.png'
# 	    right_img_write = '/export/patraval/robo_car_images/zed_front/right/' + "{}".format(i) + '.png'
# 	    writer(left_img, right_img,left_img_write, right_img_write  f1, f2)

# 	f1.close()
# 	f2.close()




# def main():
# 	left_cam = '/export/patraval/robo_car_images/pg_cam/rect/left/'
# 	right_cam= '/export/patraval/robo_car_images/pg_cam/rect/right/'
# 	# processor(left_cam, right_cam)


