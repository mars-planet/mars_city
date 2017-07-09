from __future__ import division, print_function
from collections import OrderedDict

import sys
import csv
import time
import subprocess

import matplotlib.pyplot as plt

class PVC(object):
	pass

class APC(object):
	def __init__(self):

		#  ecg_q and rrint_status_q need to be continuously populated
		# note that all values are int

		# ecg data as an OrderedDict
		# key:value = timestamp:(ecg(4113), ecg(4114))
		self.ecg_dict = OrderedDict()

		# used as anomaly quality indicator
		# key:value = timestamp:rr_quality
		self.rrint_status_dict = OrderedDict()

		
		# # holds detected RR intervals
		# self.RRint_time = []

		# # holds detected QRS width
		# self.QRSwidth = []

		# # holds detected QRS area
		# self.QRSarea = []


	def findRR(self, approx_RR):
		approx_RR -= 10
		for i in xrange(20):
			if (approx_RR + i) in self.rrint_status_dict:
				return (approx_RR + i)
		return None

	def popluate_aux_structures(self, init_timestamp):
		# window size in number of ecg samples - 30 seconds
		__ecgPUrange = 256*30
		# window overlap in number of ecg samples - 5 seconds
		__ecgOverlap = 256*5

		ecg_buffer = []

		# time.sleep(300)

		try:
			while self.ecg_dict:
				ecg_cur_window = []
				ecg_cur_window.extend(ecg_buffer)
				ecg_buffer = []

				for i in xrange(__ecgPUrange - len(ecg_buffer)):
					try:
						ecg_tuple = self.ecg_dict[init_timestamp+i]
						ecg_cur_window.append((init_timestamp+i, ecg_tuple[0], ecg_tuple[1]))
						if i > (__ecgPUrange - len(ecg_buffer)) - __ecgOverlap:
							ecg_buffer.append((init_timestamp+i, ecg_tuple[0], ecg_tuple[1]))
					except:
						break
				
				# update init_timestamp
				init_timestamp = init_timestamp + __ecgPUrange - len(ecg_buffer)
				
				with open('filebuf.csv', 'w') as f:
					for i in xrange(len(ecg_cur_window)):
						print(ecg_cur_window[i][1], ecg_cur_window[i][2], sep=',', file=f)
					f.close()

				# wrsamp -F 256 -i filebuf.csv -o filebuf -s ',' 0 1
				subprocess.check_call(["wrsamp", "-F", "256", "-i", "filebuf.csv", "-o", "filebuf", "-s", ",", "0", "1"])

				# ecgpuwave -r filebuf -a filebufann
				subprocess.check_call(["ecgpuwave", "-r", "filebuf", "-a", "ann"])

				with open('filebuf.txt', 'w') as f:
					# rdann -r filebuf -a ann -v
					subprocess.check_call(["rdann", "-r", "filebuf", "-a", "ann"], stdout=f)
					f.close()

				with open('filebuf.txt', 'r') as f:
					beatlist = list(csv.reader(f, delimiter=' '))
					prev_index = int([j for j in beatlist[0] if j != ''][1])
					for i in xrange(1, len(beatlist)):
						line = [j for j in beatlist[i] if j != '']
						cur = line[2]
						if cur == 'N':
							next_index = int([j for j in beatlist[i+1] if j != ''][1])
							approx_R_peak = ecg_cur_window[int(line[1]) - 1][0]
							R_peak_timestamp = self.findRR(approx_R_peak)
							print(R_peak_timestamp)
							# sys.exit()
						prev_index = int(line[1])

				
				# sys.exit()
		except Exception as e:
			print(e)



# method to populate ecg and rrint_status dicts for testing
def populate_dicts(apcObj):
	with open('ecg_APC.txt', 'r') as f:
		testip = list(csv.reader(f, delimiter='\t'))
		for i in testip:
			apcObj.ecg_dict[int(i[0])] = (int(i[1]), int(i[1]))
		f.close()

	with open('rrinterval_status_APC.txt', 'r') as f:
		testip = list(csv.reader(f, delimiter='\t'))
		for i in testip:
			apcObj.rrint_status_dict[int(i[0])] = int(i[1])
		f.close()

def main():
	# with open('ecg_APC.txt', 'r') as f:
	# 	testip = list(csv.reader(f, delimiter='\t'))
	# 	newarr = [i[1] for i in testip]

	# 	plt.plot(range(len(newarr)), newarr)
	# 	# plt.plot(detectionTimeList, [1350] * len(detectionTimeList), 'ro')
	# 	plt.show()
	
	apcObj = APC()
	populate_dicts(apcObj)
	apcObj.popluate_aux_structures(383021266184)

if __name__ == '__main__':
	main()
