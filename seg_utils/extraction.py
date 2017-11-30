import numpy as np
import cv2
from matplotlib import pyplot as plt
from read_disparity import read_disparity

class Extraction():

	def __init__(self, cam_type):
		self.sorted_idx = []
		if cam_type is 'zed':
			self.out_file = '/export/patraval/robo_car_new_loop_all/zed_front/snip_loop/left_test.txt'
			self.net_blend_file = '/export/patraval/robo_car_new_loop_all/zed_front/snip_loop/network_blend.txt'
			self.disparity_file = '/export/patraval/robo_car_new_loop_all/zed_front/snip_loop/disparity.txt'
			self.resize_height, self.resize_width = 640, 1280
		elif cam_type is 'pg':
			# self.out_file = '/export/patraval/robo_car_loop2/pg_cam/snip_loop/left_test.txt'
			# self.net_blend_file = '/export/patraval/robo_car_loop2/pg_cam/snip_loop/network_blend.txt'
			# self.disparity_file = '/export/patraval/robo_car_loop2/pg_cam/snip_loop/disparity.txt'
			self.out_file = '/export/patraval/robo_car_loop2/pg_cam/snip_loop/left_test.txt'
			self.net_blend_file = '/export/patraval/robo_car_loop2/pg_cam/snip_loop/network_blend.txt'
			self.disparity_file = '/export/patraval/robo_car_loop2/pg_cam/snip_loop/disparity.txt'


			self.resize_height, self.resize_width = 1024, 2048
		self.out_list, self.overlay_list, self.disparity_list = self.unpack_files()	

	def execute_extraction(self, index):
		src, self.overlay, disparity = self.read_img(self.out_list[index], self.overlay_list[index], self.disparity_list[index],0 )
		self.original = src.copy()
		img = self.process_images(src, disparity)
		img = self.threshold(img, 0) #Binary threshold
		img = self.gaussian_blur(img) 
		img = self.dilation(img, 0)
		self.pre_filtering(src, img, 0)

		src, img, self.overlay = self.img_resize(src, img, self.overlay)
		src_clone = src.copy()
		self.image = src_clone
		contours = self.get_contours(src_clone, img, 0)
		contours = self.filter_by_area(src, contours, 0)
		final_cont, disparity_list, centre_list  =  self.filter_by_disparity(src, disparity, contours, 0)
	
		final_cont, disparity_list, centre_list = self.expensive_disparity(final_cont, disparity_list, centre_list)
		# final_cont, disparity_list, centre_list = self.expensive_x(final_cont, disparity_list, centre_list)

		low_points = self.lowest_point_get(final_cont)
		disparity_list = [i/256 for i in disparity_list]
		points  = self.compose_point_list(src_clone, disparity_list, low_points, centre_list, 0 )
		self.extractor_out = src_clone.copy()
		# self.sorted_idx = self.sort_left_to_right(points, disparity_list)
		return points


	def expensive_x(self, final_cont, disparity_list, centre_list):
		new_d = []
		new_c = []
		new_cont = []
		for i in range(len(disparity_list)):
			if centre_list[i][0] <  740:
				new_d.append(disparity_list[i])
				new_c.append(centre_list[i])
				new_cont.append(final_cont[i])

		return  new_cont, new_d, new_c

	def expensive_disparity(self, final_cont, disparity_list, centre_list):
		new_d = []
		new_c = []
		new_cont = []

		for i in range(len(disparity_list)):
			if disparity_list[i] > 1356.8:
				new_d.append(disparity_list[i])
				new_c.append(centre_list[i])
				new_cont.append(final_cont[i])

		return  new_cont, new_d, new_c


	def img_resize(self,src, img, overlay):
		res_img = cv2.resize(img,(self.resize_width, self.resize_height), interpolation = cv2.INTER_CUBIC)
		res_overlay = cv2.resize(overlay,(self.resize_width, self.resize_height), interpolation = cv2.INTER_CUBIC)
		res_src = cv2.resize(src,(self.resize_width, self.resize_height), interpolation = cv2.INTER_CUBIC)
		return res_src, res_img, res_overlay

	def display_final_image(self, vehicle_coords, pop_index, points):
		for i in range(len(vehicle_coords)):
			if i not in pop_index:
				cv2.arrowedLine(self.image, (points[i][0]+40, points[i][1]+5), (points[i][0]+20, points[i][1]+5), (0,0,255), 1)
				# print(vehicle_coords[i][0:2], " D: ", points[i][2])

		self.image = cv2.resize(self.image,(640, 320), interpolation = cv2.INTER_CUBIC)
		self.extractor_out = cv2.resize(self.extractor_out,(640, 320), interpolation = cv2.INTER_CUBIC)
		vis = np.concatenate((self.image, self.extractor_out), axis=0)
		self.overlay = cv2.resize( self.overlay,(960, 640), interpolation = cv2.INTER_CUBIC)
		vis = np.concatenate(( self.overlay, vis), axis=1)
		cv2.imshow("Real-Time",vis )
		# cv2.imshow("Real-Time Overlay",res_overlay)
		cv2.waitKey(0)


	def sort_left_to_right(self,  points, disparity_list):
		x_vals =  np.array(points)[0]
		sorted_idx = sorted(range(len(x_vals)), key=lambda k: x_vals[k])
		sorted_disp = []
		sorted_points = []
		for i in range(len(sorted_idx)):
			sorted_points.append(sorted_idx[i])
			sorted_disp.append(sorted_idx[i])
		return sorted_points, sorted_disp



	def filter_by_disparity(self, src, disparity, contours, show):
		disparity_list  =[]
		centre_list =[] 
		for i,j in enumerate(contours):
			cx, cy = self.compute_moment(j)
			disp_value =  disparity[cy, cx]*256
			centre_list.append([cx,cy])
			disparity_list.append(disp_value)
			cv2.putText(src, str(round(disp_value,2)), (cx,cy), cv2.FONT_HERSHEY_SIMPLEX, 6, (255,255, 255), 1)

		self.display([src], ["filter_by disp"], show)	    
		# print(" Before disparity_list", [int(i) for i in disparity_list] )
		# print("centre_list", centre_list)

		pop_val = []
		for i in range(len(disparity_list)-1):
			for j in range(1,len(disparity_list)):
				ub = disparity_list[j]+100
				lb = disparity_list[j]-100
				val = disparity_list[j]
				# print(disparity_list[i],disparity_list[j])
				if i == j:
					break
				if (disparity_list[i] == val)or ((disparity_list[i] < ub) and (disparity_list[i]> lb)):
					pop = self.vertical_check(i, j, centre_list, contours, src)
					pop_val.append(pop)

		filter_d = []
		center_d = []
		disparity_d = []
		for i in range(len(contours)):
			if i not in pop_val:
				filter_d.append(contours[i])
				center_d.append(centre_list [i])
				disparity_d.append(disparity_list[i])
		return filter_d, disparity_d, center_d

	def vertical_check(self, i, j, centre_list, filtered, src):
		# Now simple left right check. Later add template matching
		i_left_x, i_left_y = filtered[i][filtered[i][:,:,0].argmin()][0]
		i_right_x, i_right_y = filtered[i][filtered[i][:,:,0].argmax()][0]
		j_left_x, j_left_y = filtered[j][filtered[j][:,:,0].argmin()][0]
		j_right_x, j_right_y = filtered[j][filtered[j][:,:,0].argmax()][0]
		cv2.circle(src, (i_left_x, i_left_y ), 2, (110,255,0) ,1) 
		cv2.circle(src, (i_right_x, i_right_y), 1, (110,255,0) ,1) 
		cv2.circle(src, (j_left_x, j_left_y), 2, (110,255,0) ,1) 
		cv2.circle(src, (j_right_x, j_right_y), 2, (110,255,0) ,1) 

		same_pole = False
		pop = None
		if (centre_list[j][0] >i_left_x and centre_list[j][0] < i_right_x ):
			same_pole = True
			# print("first if" , centre_list[j][0] ,i_left_x , i_right_x )
		if (centre_list[i][0] >j_left_x and centre_list[i][0] < j_right_x ):
			same_pole = True
			# print("second if", centre_list[j][0] ,i_left_x , i_right_x )

		if (same_pole==True):
			if ( i_left_y < j_left_y):
				pop =i
			else:
				pop = j

		# print("pop, ", pop)
		self.display([src], ["left-right-bounds"], 0)
		return pop					

	def compose_point_list(self, src, disparity_list, low_points, centre_list, show):
		points = []
		for i in range(len(disparity_list)):
			X = centre_list[i][0]
			Y = (centre_list[i][1] + low_points[i][1])/2
			D = disparity_list[i]
			points.append([X,Y,D])
			cv2.putText(src, str(round(D,2)), (X,Y), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255, 255), 1)
		self.display([src], ["composer"], show)	    
		return points



	def lowest_point_get(self, contours):
		point_list= []
		for cnt in contours:
			p1 =  cnt[cnt[:,:,1].argmax()][0]
			point_list.append(p1)
		return point_list

	def lowest_point_compare_(self, i, j, filtered):
		p1 =  filtered[i][filtered[i][:,:,1].argmax()][0]
		p2 =  filtered[j][filtered[j][:,:,1].argmax()][0]
	
	def unpack_files(self):
		out_list, overlay_list, disparity_list = [],[],[]
		out_list = [line.rstrip('\n') for line in open(self.out_file)]
		overlay_list = [line.rstrip('\n') for line in open(self.net_blend_file)]
		disparity_list = [line.rstrip('\n') for line in open(self.disparity_file)]
		return out_list, overlay_list, disparity_list

	def read_img(self,  test_output, overlay_file, disparity_file, show):
		img = cv2.imread(test_output)
		overlay = cv2.imread(overlay_file)
		disparity, vis_disparity = read_disparity(disparity_file)
		self.display([img, overlay, vis_disparity],['src', 'overlay', 'vis_disparity'], show)
		return img, overlay, disparity

	def process_images(self, img, disparity):
		imgray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
		height, width = img.shape[:2]
		# print(height, width)
		# res = cv2.resize(disparity,(width, height), interpolation = cv2.INTER_CUBIC)

		return imgray

	def gaussian_blur(self,img):
		return cv2.blur(img,(3,3));

	def threshold(self, img, show):
		ret, poles = cv2.threshold(img,0, 255,  cv2.THRESH_BINARY)
		self.display([poles], ["poles"], show)
		return poles

	def display(self, img_list, string_list, show):
		"Display Images along with Image Names"
		if show is 0:
			return None
		for img, name in zip(img_list, string_list):
			cv2.imshow(name, img)
		cv2.waitKey(0)

	def dilation(self, img, show):
	 	kernel = np.ones((3,3),np.uint8)	
		dilation = cv2.dilate(img,kernel,iterations = 1)
		self.display([dilation], ['dilation'] ,show)
		return dilation

	def canny_edges(self):
		edges = cv2.Canny(imgray,20,200)
		cv2.imshow('edges',edges)
		cv2.waitKey(0)
		plt.subplot(121),plt.imshow(imgray,cmap = 'gray')
		plt.title('Original Image'), plt.xticks([]), plt.yticks([])
		plt.subplot(122),plt.imshow(edges,cmap = 'gray')
		plt.title('Edge Image'), plt.xticks([]), plt.yticks([])
		plt.show()

	def compute_moments(self, src, contours, show):
		centroid  = []
		for cnt in contours:
			M = cv2.moments(cnt)
			cx = int(M['m10']/M['m00'])
			cy = int(M['m01']/M['m00'])	
	 		centroid.append((cx,cy))
			cv2.circle(src, (cx,cy), 1, (0,255,0) ,1) 
		self.display([src], ["centroid"], show)
		return centroid

	def compute_moment(self, cnt):
		M = cv2.moments(cnt)
		cx = int(M['m10']/M['m00'])
		cy = int(M['m01']/M['m00'])	
		return cx, cy

	def pre_filtering(self, src, img, show):
		im2, contours, hierarchy = cv2.findContours(img,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
		contour_dirty = []
		for cnt in contours:
			area = cv2.contourArea(cnt)
			if area < 100:
				contour_dirty.append(cnt)

		# area_list.sort(reverse=True)
		cv2.drawContours(src, contour_dirty, -1, (0,0,0), -1)
		cv2.drawContours(img, contour_dirty, -1, (0,0,0), -1)

		self.display([src], ["pre-filter-area"], show)

	def get_contours(self, src, img, show):
		im2, contours, hierarchy = cv2.findContours(img,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
		cv2.drawContours(src, contours, -1, (0,0,255), 1)
		self.display([src], ["contours"], show)

		# # add approximation --
		# for cnt in contours:
		# 	epsilon = 0.1*cv2.arcLength(cnt,True)
		# 	approx = cv2.approxPolyDP(cnt,epsilon,True)
		# 	cv2.drawContours(src, approx, -1, (0,0,255), 1)
		# 	display([src], ["approx"], show)
		#Convex Hull
		# cnt =contours[0]
		# hull = cv2.convexHull(cnt, returnPoints = False)
		# cv2.drawContours(src, [hull], -1, (0,0,255), 1)
		# display([src], ["hull"], show)

		return contours

	def fitting_shapes(self, img, cnt):
		#Line
		rows,cols = img.shape[:2]
		[vx,vy,x,y] = cv2.fitLine(cnt, cv2.DIST_L2,0,0.01,0.01)
		lefty = int((-x*vy/vx) + y)
		righty = int(((cols-x)*vy/vx)+y)
		cv2.line(img,(cols-1,righty),(0,lefty),(0,255,0),2)

	def extremities(self, src, contours, show):
		# leftmost = tuple(cnt[cnt[:,:,0].argmin()][0])
		# rightmost = tuple(cnt[cnt[:,:,0].argmax()][0])
		for cnt in contours:
			topmost = tuple(cnt[cnt[:,:,1].argmin()][0])
			bottommost = tuple(cnt[cnt[:,:,1].argmax()][0])
			cv2.circle(src, topmost, 1, (0,255,255) ,2) 
			cv2.circle(src, bottommost, 1, (0,255,255) ,2) 
		self.display([src], ["extremities"], show)

	def filter_by_area(self, src, contours, show):
		contour_filtered = []
		area_list = []
		for cnt in contours:
			area = cv2.contourArea(cnt)
			area_list.append(area)
			if area > 70:
				contour_filtered.append(cnt)
		# area_list.sort(reverse=True)
		cv2.drawContours(src, contour_filtered, -1, (0,0,255), 1)
		self.display([src], ["filter-area"], show)
		return contour_filtered




	def area_all( self, src, contours, centroid, show):
		for cnt,point in zip(contours,centroid):
			area = cv2.contourArea(cnt)
			cv2.putText(src, str(area), point, cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255, 255), 1) 
		self.display([src], ["all all"], show)


	def mouse_pointer(self, img, show):

		if show is 0:
		    return None
		cv2.namedWindow("image")
		cv2.setMouseCallback("image", click_and_tell)
		while(1):
		    cv2.imshow('image',img)
		    if cv2.waitKey(20) & 0xFF == 27:
		        break
		cv2.destroyAllWindows()


	def click_and_tell(self, event, x, y, flags, param):

		if event == cv2.EVENT_LBUTTONDOWN:
			print("x,y", y,x)