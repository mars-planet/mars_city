from __future__ import division, print_function
from collections import OrderedDict
from threading import Thread

import os
import sys
import csv
import time
import ConfigParser

import matplotlib.pyplot as plt

class RespiratoryAD(object):
	def __init__(self, config):
		# get the tidal volume deltas
		self.tidal_volume_delta = config.getfloat('Respiratory AD', 'tidal_volume_delta')
		self.tidal_volume_window_delta = config.getfloat('Respiratory AD', 'tidal_volume_window_delta')

		# get the minute ventilation deltas
		self.minute_ventilation_delta = config.getfloat('Respiratory AD', 'minute_ventilation_delta')
		self.minute_ventilation_window_delta = config.getfloat('Respiratory AD', 'minute_ventilation_window_delta')

		# raw respiratory data as an OrderedDict
		# key:value = timestamp:(thoracic, abdominal)
		self.raw_resp_dict = OrderedDict()

		# tidal volume (vt) dict
		# key:value = timestamp:tidal volume
		self.tidal_volume_dict = OrderedDict()

		# minute ventilation dict
		# key:value = timestamp:minute ventilation
		self.minute_ventilation_dict = OrderedDict()

		# dict of tidal volume anomalies
		# key:value = timestamp:'window'/'not_window'
		self.tidal_volume_anomaly_dict = OrderedDict()

		# dict of minute ventilation anomalies
		# key:value = timestamp:'window'/'not_window'
		self.minute_ventilation_anomaly_dict = OrderedDict()

	def delete_DS(self):
		pass

	def minute_ventilation_anomaly(self):
		# check if i+1 is out of (ith +/- delta) range
		# check if i+1 is out of (i-5 to ith window +/- window delta range)

		# get the initial timestamp
		begin = self.minute_ventilation_dict.popitem(last=False)[0]

		# assuming window size to be 5
		window = []
		# skip 10 timestamps
		count = 0
		while count < 10:
			if begin + 1 in self.minute_ventilation_dict:
				count += 1
				if count >= 6:
					window.append((begin+1, self.minute_ventilation_dict[begin+1]))
			begin += 1

		# set prev and begin
		prev = self.minute_ventilation_dict[begin]
		begin += 1

		while len(self.minute_ventilation_dict) > 100:
			if begin in self.minute_ventilation_dict:
				cur = self.minute_ventilation_dict[begin]

				# check for non window delta
				percent = (self.minute_ventilation_delta/100) * prev
				if not ((prev - percent) < cur < (prev + percent)):
					self.minute_ventilation_anomaly_dict[begin] = 'not_window'

				# check for window delta
				windowmean = sum([i[1] for i in window])/len(window)
				percent = (self.minute_ventilation_window_delta/100) * windowmean
				if not ((windowmean - percent) < cur < (windowmean + percent)):
					self.minute_ventilation_anomaly_dict[begin] = 'window'
				
				# update window
				window.pop(0)
				window.append((begin, cur))
				# update prev
				prev = self.minute_ventilation_dict[begin]

			# update begin
			begin += 1

			if begin == 383021389273:
				print(self.minute_ventilation_anomaly_dict)

		return

	def tidal_volume_anomaly(self):
		# check if i+1 is out of (ith +/- delta) range
		# check if i+1 is out of (i-5 to ith window +/- window delta range)

		# get the initial timestamp
		begin = self.tidal_volume_dict.popitem(last=False)[0]

		# assuming window size to be 5
		window = []
		# skip 10 timestamps
		count = 0
		while count < 10:
			if begin + 1 in self.tidal_volume_dict:
				count += 1
				if count >= 6:
					window.append((begin+1, self.tidal_volume_dict[begin+1]))
			begin += 1

		# set prev and begin
		prev = self.tidal_volume_dict[begin]
		begin += 1

		while len(self.tidal_volume_dict) > 100:
			if begin in self.tidal_volume_dict:
				cur = self.tidal_volume_dict[begin]

				# check for non window delta
				percent = (self.tidal_volume_delta/100) * prev
				if not ((prev - percent) < cur < (prev + percent)):
					self.tidal_volume_anomaly_dict[begin] = 'not_window'

				# check for window delta
				windowmean = sum([i[1] for i in window])/len(window)
				percent = (self.tidal_volume_window_delta/100) * windowmean
				if not ((windowmean - percent) < cur < (windowmean + percent)):
					self.tidal_volume_anomaly_dict[begin] = 'window'
				
				# update window
				window.pop(0)
				window.append((begin, cur))
				# update prev
				prev = self.tidal_volume_dict[begin]

			# update begin
			begin += 1

			# if begin == 383021389273:
			# 	print(self.tidal_volume_anomaly_dict)

		return

	def populate_DS(self):
		with open('vt.txt', 'r') as f:
			testip = list(csv.reader(f, delimiter='\t'))
			for i in testip:
				self.tidal_volume_dict[int(float(i[0]))] = float(i[1])
			f.close()

		with open('minuteventilation.txt', 'r') as f:
			testip = list(csv.reader(f, delimiter='\t'))
			for i in testip:
				self.minute_ventilation_dict[int(float(i[0]))] = float(i[1])
			f.close()


def main():
	config = ConfigParser.RawConfigParser()
	dirname = dir_path = os.path.dirname(os.path.realpath(__file__))
	cfg_filename = os.path.join(dirname, 'anomaly_detector.cfg')
	config.read(cfg_filename)

	respObj = RespiratoryAD(config)

	th1 = Thread(target=respObj.populate_DS, args=[])
	th1.start()

	th1.join()

	th2 = Thread(target=respObj.tidal_volume_anomaly, args=[])
	th2.start()

	th3 = Thread(target=respObj.minute_ventilation_anomaly, args=[])
	th3.start()


if __name__ == '__main__':
	main()
