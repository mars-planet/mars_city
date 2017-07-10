from __future__ import division, print_function
from collections import OrderedDict
from math import atan2, degrees, sqrt

import sys
import csv
import time
import subprocess

import matplotlib.pyplot as plt


class APC(object):
	def __init__(self, ecg_baseline, ecg_halfrange):
		# to center the data - Hexoskin range from -13.1072 to +13.1072 mV
		self.ecg_baseline = ecg_baseline
		self.ecg_halfrange = ecg_halfrange

		self.init_timestamp = None

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

		# anomalyDict - use popitem() to remove
		self.anomaly_dict = OrderedDict()

	def print_func(self):
		print(self.RRint_dict)
		print(self.QRSwidth_dict)
		print(self.QRSarea_dict)
		print(self.vecg_dict)

	def get_window(self, minus, plus, timestamp, in_dict):
		localdict = {0: self.RRint_dict,
					 1: self.QRSwidth_dict,
					 2: self.QRSarea_dict,
					 3: self.vecg_dict}[in_dict]
		
		window = []
		timestamp_copy = timestamp + 1

		while len(window) < minus:
			if timestamp in localdict:
				window.append((timestamp, localdict[timestamp]))
			timestamp -= 1
			if timestamp < timestamp_copy - (256 * 30):
				break

		window.reverse()

		while len(window) < (minus + plus):
			if timestamp_copy in localdict:
				window.append((timestamp_copy, localdict[timestamp_copy]))
			timestamp_copy += 1
			if timestamp_copy > timestamp + (256 * 50):
				break

		if len(window) == (minus + plus):
			return window
		else:
			raise ValueError

	def find_overlaps(self, window):
		ocount = 0
		for i in xrange(len(window)):
			starti, endi = window[i][0], window[i][1]
			
			for j in xrange(i + 1, len(window)):
				startj, endj = window[j][0], window[j][1]

				if (startj <= starti <= endj) or\
				   (starti <= startj <= endi):
				   ocount += 1
		return ocount

	def absolute_arrhythmia(self):
		# skip the initial 20 seconds
		self.init_timestamp += 20*256
		next_timestamp = self.init_timestamp

		while self.RRint_dict:
			cur_window = self.get_window(6, 4, next_timestamp, 0)

			mean_RR = sum([i[1] for i in cur_window])/10

			maxDiff = 0
			for i in xrange(10):
				curDiff = abs(cur_window[i][1] - mean_RR)
				if curDiff > maxDiff:
					maxDiff = curDiff

			RRwindows = []
			one_percent = 0.01 * maxDiff
			for i in xrange(10):
				curDiff = abs(cur_window[i][1] - mean_RR)
				RRwindows.append((curDiff - one_percent, curDiff + one_percent))

			numofoverlaps = self.find_overlaps(RRwindows)
			print(numofoverlaps)


			next_timestamp = cur_window[6][0] + 1

def main():
	pass

if __name__ == '__main__':
	main()
