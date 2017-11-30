from extraction import Extraction
from state_transition import State_Transition
import sys
import cv2



class Segmentation_Pipeline():
	def __init__(self,  cam_type='zed', realtime=False):
		if cam_type is 'zed':
			odom_file =	'/export/patraval/robo_car_new_loop_all/zed_front/gps/fix.txt'
			self.initial_offset = 6070
			self.offset_length = 6634
		elif cam_type is 'pg': 
			odom_file = '/export/patraval/robo_car_loop2/pg_cam/gps/fix.txt'
			self.initial_offset = 5953
			self.offset_length = 6522
		
		self.initial_offset = 1312 #1417
		self.offset_length =  1360 #6634

		# self.initial_offset = 3510
		# self.offset_length = 6634
		print(self.initial_offset, 		self.offset_length)
		self.transformer = State_Transition(odom_file,  cam_type, self.initial_offset, realtime)
		self.extractor = Extraction(cam_type)


	def start_process_disk(self):

		for i in range(self.initial_offset, self.offset_length):
			points  = self.extractor.execute_extraction(i)
			self.transformer.point_transformation(points, i)
			print("Processed: ", i)
			# self.extractor.display_final_image(self.transformer.vehicle_coords_base, self.transformer.pop_index, points)
			# raw_input("Press Enter to continue...")

	def start_process_realtime2(self, index=None):
		for index in range(self.initial_offset, self.offset_length):
			points  = self.extractor.execute_extraction(index)
			self.transformer.point_transformation(points, index)
			print("Processed: ", index)
			self.extractor.display_final_image(self.transformer.vehicle_coords_base, self.transformer.pop_index, points)


	def start_process_realtime(self, index):
		points  = self.extractor.execute_extraction(index+self.initial_offset)
		self.transformer.point_transformation(points, index+self.initial_offset)
		# print("Processed: ", index+70+6000)
		self.extractor.display_final_image(self.transformer.vehicle_coords_base, self.transformer.pop_index, points)



def main():

	# seg_obj = Segmentation_Pipeline(realtime=True)
	# seg_obj.start_process_realtime2()

	seg_obj = Segmentation_Pipeline(cam_type='pg')
	# seg_obj.start_process_disk()
	seg_obj.start_process_realtime2()

if __name__ == "__main__":
	try:  
		main()
	except KeyboardInterrupt:
		print("Qutting...")
		cv2.destroyAllWindows()
		sys.exit()



# def start_process_realtime(self):
# 	for i in range(70, len(self.extractor.out_list)):
# 		points  = self.extractor.execute_extraction(i)
# 		self.transformer.point_transformation(points, i+6000)
# 		print("Processed: ", i+ 6000)
# 		self.extractor.display_final_image(self.transformer.vehicle_coords_base, self.transformer.pop_index, points)
# 		# print("yo", self.transformer.sensor_readings, self.transformer.odom_readings)
# 		# return self.transformer.sensor_readings, self.transformer.odom_readings