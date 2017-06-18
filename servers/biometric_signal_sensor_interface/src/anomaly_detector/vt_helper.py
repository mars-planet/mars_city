from __future__ import division, print_function
from threading import Thread
from time import sleep
from fractions import gcd

import os
import sys
import csv
import queue
import ctypes

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

from anomaly_detector import AnomalyDetector
from ventricular_tachycardia import get_Ampl

import bdac

"""
The purpose of this file is to indicate the proper way to call
the methods of the Ventricular Tachycardia class through the
method of the Anomaly Detector class.
A lot of things are not up to quality and need to be improved
while implementing
"""

class BeatAnalyzer(object):
	def __init__(self, ecg_dict, init_hexo_time):
		self.ecg_dict = ecg_dict

		self.i = int(init_hexo_time)

		self.m = 0
		self.n = 0
		self.mn = 0
		self.ot = 0
		self.it = 0
		self.vv = 0
		self.v = 0
		self.rval = 0

		self.OSEA = ctypes.CDLL('osea.so')
		self.OSEA.BeatDetectAndClassify.argtypes = (ctypes.c_int, ctypes.POINTER(ctypes.c_int), ctypes.POINTER(ctypes.c_int))
		self.OSEA.ResetBDAC.argtypes = ()

	def ResetBDAC(self):
		self.OSEA.ResetBDAC()

	def BeatDetectAndClassify(self, sample_val):
	    beatType = ctypes.c_int()
	    beatMatch = ctypes.c_int()
	    result = self.OSEA.BeatDetectAndClassify(ctypes.c_int(sample_val), ctypes.byref(beatType), ctypes.byref(beatMatch))
	    return int(result), int(beatType.value)

	def getVec(self):
		try:
			self.i += 1
			return int(int(self.ecg_dict[self.i - 1] * 1.4) / 2)
		except:
			return -1

	def beat_next_sample(self, ifreq, ofreq, init):
		vout = -1
		if init:
			i = gcd(ifreq, ofreq)
			self.m = int(ifreq/i)
			self.n = int(ofreq/i)
			self.mn = int(self.m*self.n)
			self.vv = int(self.getVec())
			self.v = int(self.getVec())
			self.rval = self.v
		else:
			while self.ot > self.it:
				self.vv = self.v
				self.v = int(self.getVec())
				self.rval = self.v
				if self.it > self.mn:
					self.it -= self.mn
					self.ot -= self.mn
				self.it += self.n
			vout = int(self.vv + int((self.ot%self.n)) * (self.v-self.vv)/self.n)
			self.ot += self.m
		return int(self.rval), int(vout)


# creating a class is necessary as it is easier to use the same
# data in different threads
# the other option is to use global variables which in not advisable
class VTBeatDetector(object):
	"""
	holds various data
	"""
	def __init__(self):
		# one could make all of these as private variables and
		# write methods to access them
		
		# hexoskin timestamps as key, raw ecg values as value
		self.ecg_dict = {}
		# hexoskin timestamps as key, (rr_interval, rr_quality) tuple as value
		self.rr_dict = {}
		# hexoskin timestamps as key, (hr, hr_quality) tuple as value
		self.hr_dict = {}

		# dict of active vt threads
		self.vt_dict = {}
		# a mod counter
		self.vt_dict_count = 0

		# read these from config file - already set up
		self.hr_low = 40
		self.hr_high = 70

		# read these from config file - already set up
		# config file will give time in sec, multiply by 256
		self.PVC_window = 10 * 256
		self.PVC_number = 4

		# config file will give time in sec, multiply by 256
		self.Unknown_window = 10 * 256
		self.Unknown_number = 4

		# config file will give time in sec, multiply by 256
		self.ectopic_window = 10 * 256
		self.ectopic_number = 4
		# read from config file
		self.ectopic_beat_thresh = 10

	def collect_data(self):
		"""
		this should be implemented as:
		create three threads to call Hexoskin's API and popluate
		the respective data structures in real time
		Instead of calling Hexoskin's API, it could call
		other methods if data is already being collected elsewhere
		"""
		# read ecg data
		with open ('ecg_full.txt', 'r') as ipfile:
			ip = (csv.reader(ipfile, delimiter='\t'))
			for i in ip:
				self.ecg_dict[int(i[0])] = int(i[1])

		# read rr_interval data
		with open('rrinterval_full.txt', 'r') as ipfile1:
			ip1 = list(csv.reader(ipfile1, delimiter='\t'))
			with open('rrinterval_status_full.txt', 'r') as ipfile2:
				ip2 = list(csv.reader(ipfile2, delimiter='\t'))
				for i in xrange(len(ip1)):
					self.rr_dict[int(ip2[i][0])] = (float(ip1[i][1]), int(ip2[i][1]))

		# read heartrate data
		with open('heartrate_full.txt', 'r') as ipfile1:
			ip1 = list(csv.reader(ipfile1, delimiter='\t'))
			with open('hr_quality_full.txt', 'r') as ipfile2:
				ip2 = list(csv.reader(ipfile2, delimiter='\t'))
				for i in xrange(len(ip1)):
					self.hr_dict[int(ip1[i][0])] = (float(ip1[i][1]), int(ip2[i][1]))

	def delete_data(self)					:
		"""
		since the data structures need to be of a limited size,
		data needs to be periodically deleted. Maybe something like
		start deleting continously with a lag of 0.5 seconds after initial
		5 min buffer where nothing is deleted
		create a three more threads maybe?
		"""
		raise NotImplementedError

	def __get_key(self, rr_flag, hexo_time, flag):
		try:
			# look in rr_dict
			if rr_flag:
				# if flag, look in forward direction for next timestamp - 20 seconds
				if flag:
					for i in xrange(hexo_time, hexo_time + (256*20)):
						if i in self.rr_dict:
							return i
				# if not flag, look in backward direction for prev timestamp - 20 seconds
				else:
					for i in xrange(hexo_time, hexo_time - (256*20), -1):
						if i in self.rr_dict:
							return i
			# look in hr_dict
			else:
				# look in forward direction for next timestamp - 20 seconds
				if flag:
					for i in xrange(hexo_time, hexo_time + (256*20)):
						if i in self.hr_dict:
							return i
				# if not flag, look in backward direction for prev timestamp - 20 seconds
				else:
					for i in xrange(hexo_time, hexo_time - (256*20), -1):
						if i in self.hr_dict:
							return i
		except:
			raise KeyError

	def get_six_second_data(self, start_hexo_time):
		# start_hexo_time is an int indicating start of six second window
		
		# dataframes must be sorted
		six_second_df = None
		__ampl = 0
		# construct ecg data frame and find amplitude of previous six seconds
		if start_hexo_time in self.ecg_dict:
			six_second_dict = {timestamp:self.ecg_dict[timestamp] for timestamp in xrange(start_hexo_time, start_hexo_time + (256*6))}
			six_second_dict = sorted(six_second_dict.items())
			six_second_df = pd.DataFrame(six_second_dict, columns=["hexoskin_timestamps", "ecg_val"])

			if (start_hexo_time - (256*6)) in self.ecg_dict:
				prev_six_second_dict = {timestamp:self.ecg_dict[timestamp] for timestamp in xrange(start_hexo_time - (256*6), start_hexo_time)}
				prev_six_second_dict = sorted(prev_six_second_dict.items())
				prev_six_second_df = pd.DataFrame(prev_six_second_dict, columns=["hexoskin_timestamps", "ecg_val"])
				__ampl = get_Ampl(prev_six_second_df)
			else:
				__ampl = get_Ampl(six_second_df)

		# construct rr_int and rr_status dataframes
		six_second_rr_df, six_second_rrint_stat_df = None, None
		# construct rr_intervals and rr_interval_status data frames
		__startindex = self.__get_key(True, start_hexo_time, 1)
		__endindex = self.__get_key(True, start_hexo_time + (256*6) - 1, 0)

		six_second_rr_dict = {timestamp:self.rr_dict[timestamp][0] for timestamp in xrange(__startindex, __endindex + 1) if timestamp in self.rr_dict}
		six_second_rr_dict = sorted(six_second_rr_dict.items())
		six_second_rrint_stat_dict = {timestamp:self.rr_dict[timestamp][1] for timestamp in xrange(__startindex, __endindex + 1) if timestamp in self.rr_dict}
		six_second_rrint_stat_dict = sorted(six_second_rrint_stat_dict.items())

		six_second_rr_df = pd.DataFrame(six_second_rr_dict, columns=["hexoskin_timestamps", "rr_int"])
		six_second_rrint_stat_df = pd.DataFrame(six_second_rrint_stat_dict, columns=["hexoskin_timestamps", "rr_status"])

		return six_second_df, six_second_rr_df, six_second_rrint_stat_df, __ampl

	def analyze_six_second(self, start_hexo_time):
		# start_hexo_time is an int indicating start of six second window
		six_second_df, six_second_rr_df, six_second_rrint_stat_df, ampl = self.get_six_second_data(start_hexo_time)
		
		AD = AnomalyDetector()
		th = Thread(target=AD.vt_anomaly_detect, args=[six_second_df, six_second_rr_df, six_second_rrint_stat_df, ampl])
		th.start()
		self.vt_dict[self.vt_dict_count] = AD
		self.vt_dict_count = (self.vt_dict_count + 1) % 1000
		# print(AD.vt_result)
		# th.join()

	def ping_AD_dict(self):
		"""
		ping the vt_dict periodically
		i.e. check the return value for all the keys in the dict
		if the value of vt_result is False, do nothing
		if the value of vt_result is True, get next six second data - 
			take current timestamp, add (256*6) and call analyze_six_second
		if the value of vt_result is an int, store it in the database
		
		finally delete the key: value from the dict
		"""
		while True:
			sleep(1)
			print(len(self.vt_dict))

	def heart_rate_analyzer(self, init_hexo_time):
		while True:
			try:
				__timestamp = self.__get_key(False, init_hexo_time, 1)
				if not self.hr_low <= self.hr_dict[__timestamp][0] <= self.hr_high:
					if self.hr_dict[__timestamp][1] == 0:
						# print(self.hr_dict[__timestamp], __timestamp)
						self.analyze_six_second(__timestamp)
				# find next timestamp
				init_hexo_time = __timestamp + 1
				# delay to be set according to how fast the data is being collected
				# could be reset dynamically
				# sleep(0.5)
			except:
				return

	def beat_classf_analyzer(self, init_hexo_time):
		"""
		High number of PVC's within a certain time limit can indicate a VT onset.
		Many 'unknown' beat types in a short time interval indicate that something is
		wrong with the heart signal because QRS-complexes cannot be detected.
		"""
		Beats = BeatAnalyzer(self.ecg_dict, init_hexo_time)
		ADCGain, ADCZero = 200, 1024
		ip_freq, op_freq = 256, 200

		Beats.ResetBDAC()
		samplecount = 0

		beatTypeList, detectionTimeList = [], []
		PVC_q = queue.Queue(maxsize=self.PVC_number)
		Unknown_q = queue.Queue(maxsize=self.Unknown_number)

		nextval, ecgval = Beats.beat_next_sample(ip_freq, op_freq, 1)
		while nextval != -1:
			nextval, ecgval = Beats.beat_next_sample(ip_freq, op_freq, 0)
			samplecount += 1

			lTemp = ecgval - ADCZero
			lTemp *= 200
			lTemp /= ADCGain
			ecgval = lTemp

			delay, beatType = Beats.BeatDetectAndClassify(int(ecgval))
			if delay != 0:
				DetectionTime = samplecount - delay

				DetectionTime *= ip_freq
				DetectionTime /= op_freq

				# print(beatType, DetectionTime)
				# detectionTimeList.append(DetectionTime)

				if beatType == 5:
					end = int(init_hexo_time + DetectionTime)
					PVC_q.put(end)
					if PVC_q.full():
						begin = PVC_q.get()
						if (end - begin) <= self.PVC_window:
							self.analyze_six_second(end)

				if beatType == 13:
					end = int(init_hexo_time + DetectionTime)
					Unknown_q.put(end)
					if Unknown_q.full():
						begin = Unknown_q.get()
						if (end - begin) <= self.Unknown_window:
							self.analyze_six_second(end)

			# delay to be set according to how fast the data is being collected
			# could be reset dynamically
			# sleep(0.05)

			# # uncomment to visualize
			# if samplecount == 256*20:
			# 	break

		# print(samplecount)
		# # uncomment to visualize
		# temparr = [((self.ecg_dict[i] * 1.4)/2) for i in xrange(init_hexo_time, init_hexo_time+(256*25))]
		# newtemparr = [init_hexo_time + i for i in detectionTimeList]
		# plt.plot(range(init_hexo_time, init_hexo_time + (256*25)), temparr)
		# plt.plot(newtemparr, [950]*len(newtemparr), 'ro')
		# plt.show()

	def ectopic_analyzer(self, init_hexo_time):
		rrint_q = queue.Queue(maxsize=self.ectopic_number)
		rrintstatus_dict = {}
		__timestamp = self.__get_key(True, init_hexo_time, 1)
		prev_interval = self.rr_dict[__timestamp][0]
		init_hexo_time = __timestamp
		try:
			while True:
				init_hexo_time += 1
				__timestamp = self.__get_key(True, init_hexo_time, 1)
				# calc threshold rr_intervals value
				__thresh_value = (self.ectopic_beat_thresh/100) * prev_interval
				# check if current beat is within given percentage of previous beat
				if not ((prev_interval - __thresh_value) <= self.rr_dict[__timestamp][0] <= (prev_interval + __thresh_value)):
					rrint_q.put(__timestamp)
					rrintstatus_dict[__timestamp] = self.rr_dict[__timestamp][1]
					if rrint_q.full():
						num_of_zeroone = len([1 for key in rrintstatus_dict if (rrintstatus_dict[key] == 0 or rrintstatus_dict[key] == 1)])
						begin = rrint_q.get()
						end = __timestamp
						del rrintstatus_dict[begin]
						if num_of_zeroone >= int(self.ectopic_number/2) and (end-begin) >= self.ectopic_window:
							# print(begin, rrintstatus_dict)
							self.analyze_six_second(end)

				# set vars for next iteration
				prev_interval = self.rr_dict[__timestamp][0]
				init_hexo_time = __timestamp

				# delay to be set according to how fast the data is being collected
				# could be reset dynamically
				# sleep(0.05)
		except:
			return		

	def beat_analyze(self, init_hexo_time):
		# init_hexo_time is the timestamp to begin with
		
		# start heart_rate analysis thread
		th1 = Thread(target=self.heart_rate_analyzer, args=[init_hexo_time])
		th1.start()

		# start PVC and Unknown beat classification thread
		th2 = Thread(target=self.beat_classf_analyzer, args=[init_hexo_time])
		th2.start()
		
		# start ectopic beats analysis thread
		th3 = Thread(target=self.ectopic_analyzer, args=[init_hexo_time])
		th3.start()
		
def main():
	VTBD = VTBeatDetector()
	VTBD.collect_data()
	# VTBD.delete_data()
	# VTBD.analyze_six_second(383021233184)
	VTBD.beat_analyze(383021233184)
	# VTBD.ping_AD_dict()

if __name__ == '__main__':
	main()