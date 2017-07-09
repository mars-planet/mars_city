from __future__ import division, print_function
from collections import OrderedDict
from math import atan2, degrees, sqrt

import sys
import csv
import time
import subprocess

import matplotlib.pyplot as plt

class PVC(object):
	pass

class APC(object):
	def __init__(self):
		# to center the data - Hexoskin range from -13.1072 to +13.1072 mV
		self.ecg_baseline = 1350
		self.ecg_halfrange = 26.2144/2

		# ecg_q and rrint_status_q need to be continuously populated
		# VERYIMP --> Enter centered data

		# ecg data as an OrderedDict
		# key:value = timestamp:(ecg(4113), ecg(4114))
		self.ecg_dict = OrderedDict()

		# used as anomaly quality indicator
		# key:value = timestamp:rr_quality
		self.rrint_status_dict = OrderedDict()

		
		# # holds detected RR intervals
		self.RRint_dict = OrderedDict()

		# holds detected QRS width
		self.QRSwidth_dict = OrderedDict()

		# holds detected QRS area
		self.QRSarea_dict = OrderedDict()

		# holds VECG data
		self.vecg_dict = OrderedDict()


	def findRPeak(self, approx_RR):
		approx_RR -= 10
		for i in xrange(20):
			if (approx_RR + i) in self.rrint_status_dict:
				return (approx_RR + i)
		# if not found, return original
		return approx_RR + 10

	def __ecgpuwave(self, ecg_cur_window):
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

	def __ecg_cur_window(self, init_timestamp, ecg_buffer, ecg_cur_window=[]):
		# window size in number of ecg samples - 30 seconds
		__ecgPUrange = 256*30
		# window overlap in number of ecg samples - 5 seconds
		__ecgOverlap = 256*5

		ecg_cur_window.extend(ecg_buffer)
		ecg_buffer = []
		lenecg = len(ecg_buffer)

		for i in xrange(__ecgPUrange - lenecg):
			try:
				ecg_tuple = self.ecg_dict[init_timestamp+i]
				ecg_cur_window.append((init_timestamp+i, ecg_tuple[0], ecg_tuple[1]))
				if i > (__ecgPUrange - lenecg) - __ecgOverlap:
					ecg_buffer.append((init_timestamp+i, ecg_tuple[0], ecg_tuple[1]))
			except:
				break

		# update init_timestamp
		init_timestamp = init_timestamp + __ecgPUrange - lenecg
		
		return ecg_cur_window, ecg_buffer, init_timestamp

	def populate_dicts(self, ecg_cur_window, prev_R_peak=None):
		with open('filebuf.txt', 'r') as f:
			beatlist = list(csv.reader(f, delimiter=' '))
			prev_index = int([j for j in beatlist[0] if j != ''][1])
			
			for i in xrange(1, len(beatlist)):
				line = [j for j in beatlist[i] if j != '']
				cur = line[2]

				# if qrs detected
				if cur == 'N':
					next_index = int([j for j in beatlist[i+1] if j != ''][1])
					approx_R_peak = ecg_cur_window[int(line[1]) - 1][0]
					# fetch using RRint_status data
					R_peak_timestamp = self.findRPeak(approx_R_peak)

					if prev_R_peak:
						self.RRint_dict[R_peak_timestamp] = R_peak_timestamp - prev_R_peak
						# update prev_R_peak
						prev_R_peak = R_peak_timestamp

						# enter QRS interval
						qrs_interval = next_index - prev_index
						self.QRSwidth_dict[R_peak_timestamp] = qrs_interval

						# enter QRS area
						abssum = 0
						for j in xrange(prev_index, next_index + 1):
							abssum += abs(ecg_cur_window[j - 1][1]) + abs(ecg_cur_window[j - 1][2])
						self.QRSarea_dict[R_peak_timestamp] = abssum

						# enter vecg data
						maxampl, mindex = 0, 0
						for j in xrange(prev_index, next_index + 1):
							ampl = sqrt((ecg_cur_window[j - 1][1]**2) + (ecg_cur_window[j - 1][2]**2))
							if ampl > maxampl:
								maxampl, mindex = ampl, j
						vcg_ang = degrees(atan2(ecg_cur_window[mindex - 1][2], ecg_cur_window[mindex - 1][1]))
						if vcg_ang < 0:
							vcg_ang = 360 + vcg_ang
						self.vecg_dict[R_peak_timestamp] = vcg_ang

					else:
						prev_R_peak = R_peak_timestamp
				# update prev_index
				prev_index = int(line[1])

	def popluate_aux_structures(self, init_timestamp, ):
		
		ecg_buffer = []
		# time.sleep(300)

		try:
			while self.ecg_dict:
				# get the current window
				ecg_cur_window, ecg_buffer, init_timestamp = self.__ecg_cur_window(init_timestamp, ecg_buffer, [])
				# use the ecgpuwave command
				self.__ecgpuwave(ecg_cur_window)
				# populate the other self. dicts
				self.populate_dicts(ecg_cur_window)
		except Exception as e:
			print(len(self.RRint_dict))
			print(len(self.QRSwidth_dict))
			print(len(self.QRSarea_dict))
			print(len(self.vecg_dict))
			print(e)

# method to populate ecg and rrint_status dicts for testing
def populate_dicts(apcObj):
	with open('ecg_APC.txt', 'r') as f:
		testip = list(csv.reader(f, delimiter='\t'))
		for i in testip:
			apcObj.ecg_dict[int(i[0])] = ((int(i[1]) - apcObj.ecg_baseline)/apcObj.ecg_halfrange, 
										  (int(i[1]) - apcObj.ecg_baseline)/apcObj.ecg_halfrange)
		f.close()

	with open('rrinterval_status_APC.txt', 'r') as f:
		testip = list(csv.reader(f, delimiter='\t'))
		for i in testip:
			apcObj.rrint_status_dict[int(i[0])] = int(i[1])
		f.close()

def main():
	apcObj = APC()
	populate_dicts(apcObj)
	apcObj.popluate_aux_structures(383021266184)


	# with open('ecg_APC.txt', 'r') as f:
	# 	testip = list(csv.reader(f, delimiter='\t'))
	# 	newarr = [(int(i[1]) - 1350)/13.1072 for i in testip]

	# 	plt.plot(range(len(newarr)), newarr)
	# 	plt.plot([key - int(testip[0][0]) for key in apcObj.RRint_dict], 
	# 			 [newarr[key - int(testip[0][0])] for key in apcObj.RRint_dict], 'ro')
	# 	plt.show()

if __name__ == '__main__':
	main()
