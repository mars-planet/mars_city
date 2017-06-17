from __future__ import division, print_function
from threading import Thread
from time import sleep

import os
import sys
import csv

import numpy as np
import pandas as pd

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

	def __get_key_rr(self, hexo_time, flag):
		try:
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
		__startindex = self.__get_key_rr(start_hexo_time, 1)
		__endindex = self.__get_key_rr(start_hexo_time + (256*6) - 1, 0)

		six_second_rr_dict = {timestamp:self.rr_dict[timestamp][0] for timestamp in xrange(__startindex, __endindex + 1) if timestamp in self.rr_dict}
		six_second_rr_dict = sorted(six_second_rr_dict.items())
		six_second_rrint_stat_dict = {timestamp:self.rr_dict[timestamp][1] for timestamp in xrange(__startindex, __endindex + 1) if timestamp in self.rr_dict}
		six_second_rrint_stat_dict = sorted(six_second_rrint_stat_dict.items())

		six_second_rr_df = pd.DataFrame(six_second_rr_dict, columns=["hexoskin_timestamps", "rr_int"])
		six_second_rrint_stat_df = pd.DataFrame(six_second_rrint_stat_dict, columns=["hexoskin_timestamps", "rr_status"])

		return six_second_df, six_second_rr_df, six_second_rrint_stat_df, __ampl

	def analyze_six_second(self, start_hexo_time, count):
		# start_hexo_time is an int indicating start of six second window
		six_second_df, six_second_rr_df, six_second_rrint_stat_df, ampl = self.get_six_second_data(start_hexo_time)
		
		AD = AnomalyDetector()
		th = Thread(target=AD.vt_anomaly_detect, args=(six_second_df, six_second_rr_df, six_second_rrint_stat_df, ampl))
		th.start()
		self.vt_dict[count] = AD
		# print(AD.vt_result)
		# th.join()

	def ping_AD_dict(self):
		while True:
			sleep(1)
			print(self.vt_dict[1].vt_result)

	def beat_analyze(self, init_hexo_time):
		# init_hexo_time is the timestamp to begin with
		pass


def main():
	VTBD = VTBeatDetector()
	VTBD.collect_data()
	# VTBD.delete_data()
	VTBD.analyze_six_second(383021233184, 1)
	VTBD.ping_AD_dict()

if __name__ == '__main__':
	main()