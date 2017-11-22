from extraction import Extraction
from state_transition import State_Transition
import sys
import cv2



class Segmentation_Pipeline():
	def __init__(self, realtime=False):
		self.transformer = State_Transition('/export/patraval/robo_car_new_loop_all/zed_front/gps/fix.txt', realtime)
		self.extractor = Extraction()

	def start_process_disk(self):
		for i in range(70, len(self.extractor.out_list)):
			points  = self.extractor.execute_extraction(i)
			self.transformer.point_transformation(points, i+6000)
			print("Processed: ", i+ 6000)
			# self.extractor.display_final_image(self.transformer.vehicle_coords_base, self.transformer.pop_index, points)
			# raw_input("Press Enter to continue...")

	def start_process_realtime(self):
		for i in range(70, len(self.extractor.out_list)):
			points  = self.extractor.execute_extraction(i)
			self.transformer.point_transformation(points, i+6000)
			print("Processed: ", i+ 6000)
			self.extractor.display_final_image(self.transformer.vehicle_coords_base, self.transformer.pop_index, points)
			# print("yo", self.transformer.sensor_readings, self.transformer.odom_readings)
			# return self.transformer.sensor_readings, self.transformer.odom_readings

def main():
	seg_obj = Segmentation_Pipeline(True)
	seg_obj.start_process_realtime()


if __name__ == "__main__":
	try:  
		main()
	except KeyboardInterrupt:
		print("Qutting...")
		cv2.destroyAllWindows()
		sys.exit()
