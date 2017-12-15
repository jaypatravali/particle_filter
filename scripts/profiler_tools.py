import time
import matplotlib.pyplot as plt


class profiler_tools(object):

	def __init__(self):
		self.codeblock_counter = []
		self.timestep_counter = []	
		self.code_block_num = 0
 		self.codeblock_dict= dict()

	def clear_registers(self):
		self.code_block = 0
	
	def start_profiler(self):
		self.start = time.time()
	
	def stop_profiler(self, timestep,  string, final=None):
		stop = time.time()
		time_elapsed = stop - self.start
		print("Time Elapsed for {}".format(string) , time_elapsed)
		# raw_input("Press Enter to continue...")

		if string  not in self.codeblock_dict:
			self.codeblock_dict[string] = []
		self.codeblock_dict[string].append(time_elapsed)
		if final:
			self.timestep_counter.append(timestep)

	def runtime_plot(self):
		plt.figure(3)
		# plt.plot( self.timestep_counter, self.time_counter, 'r--', )

		plot_list = []
		string_list = []
		symbol_list= [ 'r--' , 'g--', 'b--']
		for key, symbol in zip(self.codeblock_dict, symbol_list):
			p1, = plt.plot(self.timestep_counter, self.codeblock_dict[key], symbol )
			plot_list.append(p1)
			string_list.append(key)

		plt.legend(plot_list, string_list)
		plt.pause(0.00001)
