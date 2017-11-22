import numpy as np
import json
import os, sys, getopt
import glob
import utm  
import math
import cv2
from matplotlib import pyplot as plt
from point_extraction import point_transformation



def unpack_files():
	left = []
	right = []
	disparity = []
	left = [line.rstrip('\n') for line in open('/export/patraval/robo_car_new_loop_all/zed_front/left.txt')]
	right = [line.rstrip('\n') for line in open('/export/patraval/robo_car_new_loop_all/zed_front/right.txt')]
	return left, right, disparity

def read_img(show):
	overlay = '/home/patraval/caffe/models/adapnet/example_outputs/Network output_screenshot_2.png'
	test_output = '/home/patraval/caffe/models/adapnet/example_outputs/Label output_screenshot_21.09.2017.png'	
	disparity = '/export/patraval/cityscapes/disparity_trainvaltest/disparity/val/frankfurt/frankfurt_000000_016005_disparity.png'
	img = cv2.imread(test_output)
	overlay = cv2.imread(overlay)
	disparity1 = cv2.imread(disparity)
	disparity = cv2.imread(disparity, -1)
	display([img, overlay, disparity],['src', 'overlay', 'disparity'], show)
	return img,  disparity

def process_images(img, disparity):
	imgray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
	height, width = img.shape[:2]
	# print(height, width)
	res = cv2.resize(disparity,(width, height), interpolation = cv2.INTER_CUBIC)

	return imgray, res

def gaussian_blur(img):
	return cv2.blur(img,(3,3));

def threshold(img, show):
	ret, poles = cv2.threshold(img,0, 255,  cv2.THRESH_BINARY)
	display([poles], ["poles"], show)
	return poles

def display(img_list, string_list, show):
	"Display Images along with Image Names"
	if show is 0:
		return None
	for img, name in zip(img_list, string_list):
		cv2.imshow(name, img)
	cv2.waitKey(0)

def dilation(img, show):
 	kernel = np.ones((5,5),np.uint8)	
	dilation = cv2.dilate(img,kernel,iterations = 1)
	display([dilation], ['dilation'] ,show)
	return dilation

def canny_edges():
	edges = cv2.Canny(imgray,20,200)
	cv2.imshow('edges',edges)
	cv2.waitKey(0)
	plt.subplot(121),plt.imshow(imgray,cmap = 'gray')
	plt.title('Original Image'), plt.xticks([]), plt.yticks([])
	plt.subplot(122),plt.imshow(edges,cmap = 'gray')
	plt.title('Edge Image'), plt.xticks([]), plt.yticks([])
	plt.show()

def compute_moments(src, contours, show):
	centroid  = []
	for cnt in contours:
		M = cv2.moments(cnt)
		cx = int(M['m10']/M['m00'])
		cy = int(M['m01']/M['m00'])	
 		centroid.append((cx,cy))
		cv2.circle(src, (cx,cy), 1, (0,255,0) ,1) 
	display([src], ["centroid"], show)
	return centroid

def compute_moment( cnt):
	M = cv2.moments(cnt)
	cx = int(M['m10']/M['m00'])
	cy = int(M['m01']/M['m00'])	
	return cx, cy


def get_contours(src, img, show):
	im2, contours, hierarchy = cv2.findContours(img,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
	cv2.drawContours(src, contours, -1, (0,0,255), 1)
	display([src], ["contours"], show)

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

def fitting_shapes(img, cnt):
	#Line
	rows,cols = img.shape[:2]
	[vx,vy,x,y] = cv2.fitLine(cnt, cv2.DIST_L2,0,0.01,0.01)
	lefty = int((-x*vy/vx) + y)
	righty = int(((cols-x)*vy/vx)+y)
	cv2.line(img,(cols-1,righty),(0,lefty),(0,255,0),2)

def extremities(src, contours, show):
	# leftmost = tuple(cnt[cnt[:,:,0].argmin()][0])
	# rightmost = tuple(cnt[cnt[:,:,0].argmax()][0])
	for cnt in contours:
		topmost = tuple(cnt[cnt[:,:,1].argmin()][0])
		bottommost = tuple(cnt[cnt[:,:,1].argmax()][0])
		cv2.circle(src, topmost, 1, (0,255,255) ,2) 
		cv2.circle(src, bottommost, 1, (0,255,255) ,2) 
	display([src], ["extremities"], show)

def filter_by_area(src, contours, show):
	contour_filtered = []
	area_list = []

	for cnt in contours:
		area = cv2.contourArea(cnt)
		area_list.append(area)

		if area > 300:
			contour_filtered.append(cnt)

	# area_list.sort(reverse=True)


	cv2.drawContours(src, contour_filtered, -1, (0,0,255), 1)
	display([src], ["filter"], show)
	return contour_filtered

def area_all(src, contours, centroid, show):
	for cnt,point in zip(contours,centroid):
		area = cv2.contourArea(cnt)
		cv2.putText(src, str(area), point, cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255, 255), 1) 
	display([src], ["all all"], show)


def template_matching():

	result = cv2.matchTemplate(scanline, patch, cv2.TM_CCOEFF)

	# template = cv2.imread('template.jpg',0)
	# w, h = template.shape[::-1]
	# # methods = ['cv2.TM_CCOEFF', 'cv2.TM_CCOEFF_NORMED', 'cv2.TM_CCORR',
	# #             'cv2.TM_CCORR_NORMED', 'cv2.TM_SQDIFF', 'cv2.TM_SQDIFF_NORMED']
	#    img = img2.copy()
	#    # Apply template Matching
	#    res = cv2.matchTemplate(img,template,method)
	#    min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(res)
	#    # If the method is TM_SQDIFF or TM_SQDIFF_NORMED, take minimum
	#    if method in [cv2.TM_SQDIFF, cv2.TM_SQDIFF_NORMED]:
	#        top_left = min_loc
	#    else:
	#        top_left = max_loc
	#    bottom_right = (top_left[0] + w, top_left[1] + h)
	#    cv2.rectangle(img,top_left, bottom_right, 255, 2)
	#    plt.subplot(121),plt.imshow(res,cmap = 'gray')
	#    plt.title('Matching Result'), plt.xticks([]), plt.yticks([])
	#    plt.subplot(122),plt.imshow(img,cmap = 'gray')
	#    plt.title('Detected Point'), plt.xticks([]), plt.yticks([])

def search_image(img):
	return None


def mouse_pointer(img, show):

	if show is 0:
	    return None
	cv2.namedWindow("image")
	cv2.setMouseCallback("image", click_and_tell)
	while(1):
	    cv2.imshow('image',img)
	    if cv2.waitKey(20) & 0xFF == 27:
	        break
	cv2.destroyAllWindows()


def click_and_tell(event, x, y, flags, param):

	if event == cv2.EVENT_LBUTTONDOWN:
		print("x,y", y,x)



def lowest_point_compare_(i, j, filtered):
	p1 =  filtered[i][filtered[i][:,:,1].argmax()][0]
	p2 =  filtered[j][filtered[j][:,:,1].argmax()][0]


def lowest_point_get(contours):
	point_list= []
	for cnt in contours:
		p1 =  cnt[cnt[:,:,1].argmax()][0]
		point_list.append(p1)
	return point_list

def filter_by_disparity(src, disparity, filtered, show):
	disparity_list  =[]
	centre_list =[] 
	for i,j in enumerate(filtered):
		cx, cy = compute_moment(j)
		disp_value =  disparity[cy, cx]
		centre_list.append([cx,cy])
		disparity_list.append(disp_value)
		cv2.putText(src, str(disp_value), (cx,cy), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255, 255), 1)

	display([src], ["filter_by disp"], show)	    

	print(disparity_list)

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
				pop = vertical_check(i, j, centre_list, filtered, src)
				pop_val.append(pop)

	filter_d = []
	center_d = []
	disparity_d = []
	for i in range(len(filtered)):
		if i not in pop_val:
			filter_d.append(filtered[i])
			center_d.append(centre_list [i])
			disparity_d.append(disparity_list[i])
	return filter_d, disparity_d, center_d

def vertical_check(i, j, centre_list, filtered, src):
	# Now simple left right check. Later add template matching
	i_left_x, i_left_y = filtered[i][filtered[i][:,:,0].argmin()][0]
	i_right_x, i_right_y = filtered[i][filtered[i][:,:,0].argmax()][0]
	j_left_x, j_left_y = filtered[j][filtered[j][:,:,0].argmin()][0]
	j_right_x, j_right_y = filtered[j][filtered[j][:,:,0].argmax()][0]
	cv2.circle(src, (i_left_x, i_left_y ), 1, (110,255,0) ,1) 
	cv2.circle(src, (i_right_x, i_right_y), 1, (110,255,0) ,1) 
	cv2.circle(src, (j_left_x, j_left_y), 1, (110,255,0) ,1) 
	cv2.circle(src, (j_right_x, j_right_y), 1, (110,255,0) ,1) 

	same_pole = False
	if (centre_list[j][0] >i_left_x or centre_list[j][0] < i_right_x ):
		same_pole = True

	if (centre_list[i][0] >j_left_x or centre_list[i][0] < j_right_x ):
		same_pole = True

	if (same_pole==True):
		if ( i_left_y < j_left_y):
			pop =i
		else:
			pop = j

	print(pop)
	display([src], ["left right"], 1)
	return pop					

def compose_point_list(src, disparity_list, low_points, centre_list, show):
	points = []
	for i in range(len(disparity_list)):
		X = centre_list[i][0]
		Y = (centre_list[i][1] + low_points[i][1])/2
		D = disparity_list[i]
		points.append([X,Y,D])
		cv2.putText(src, str(D), (X,Y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255, 255), 1)
	display([src], ["composer"], show)	    
	return points



def main():
	src, disparity = read_img(0)
	img, disparity = process_images(src, disparity)
	img = threshold(img, 0) #Binary threshold
	img = gaussian_blur(img) 
	img = dilation(img, 0)
	src_clone = src.copy()
	contours = get_contours(src_clone, img, 0)
	display([src_clone], ["all "],  0)
	# fitting_shapes(img,  contours)
	# centroid = compute_moments(src,  contours, 0)
	# extremities(src,  contours, 1)
	# area_all(src, contours, centroid, 1)
	# mouse_pointer(disparity, 1)
	if len(contours)> 6:
		filtered = filter_by_area(src, contours, 0)
	final_cont, disparity_list, centre_list  =  filter_by_disparity(src, disparity, filtered, 1)
	low_points = lowest_point_get(final_cont)
	print(disparity_list)
	points  = compose_point_list(src_clone, disparity_list, low_points, centre_list, 1 )
	# point_transformation(points)


if __name__ == "__main__":
    main()
