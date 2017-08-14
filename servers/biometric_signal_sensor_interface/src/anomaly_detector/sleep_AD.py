from __future__ import division, print_function
from collections import OrderedDict
from copy import deepcopy
# from threading import Thread

import os
import sys
import csv
import time
# import ConfigParser

import matplotlib.pyplot as plt

class SleepAD(object):
	def __init__(self):
		# key:value = timestamp:sleep_phase
		self.sleep_phase_dict = OrderedDict()
		# key:value = timestamp:sleep_position
		self.sleep_posn_dict = OrderedDict()

		# indicates start of sleep using first null
		# of sleep_phase as reference
		self.start_sleep_act = 0;
		# indicates start of sleep using last null
		# of sleep_phase as reference
		self.end_sleep_act = 0;
		
		# The sleep efficiency is the proportion
		# of time asleep over the 'time in bed'.
		# The sleep efficiency is normally over 95%
		# A value under 85% is generally associated
		# as a bad night
		self.sleep_efficieny = 0

		# The time to fall asleep (sleep latency) is
		# the time between the sleep activity onset to
		# the first sleepphase detections
		self.sleep_latency = 0

		# The sleep percent is the proportion of time
		# asleep over the sleep time
		# (sleep time = 'time in bed' - sleep latency)
		self.sleep_percent = 0

		# Time from sleep onset to wake up sleepphase
		self.sleep_period = 0

		# Number of sleepposition changes detected
		self.sleep_position_changes = 0

		# The Total Sleep Time is the time spent in
		# any sleepphase (i.e. not awake)
		self.sleep_total_time = 0

		# The total time in the NREM sleepphase during
		# the sleep period, which is defined as the time
		# between the sleep onset and the last awakening
		self.sleep_non_REM_time = 0

		# The total time in the REM sleepphase during the
		# sleep period, which is defined as the time between
		# the sleep onset and the last awakening
		self.sleep_REM_time = 0

		# The total time in the awake vigilance state during
		# the sleep period, which is defined as the time between
		# the sleep onset and the last awakening
		self.sleep_wake_time = 0

		# number of times sleep_phase is 6
		self.woke_up_count = 0

	def calc_woke_up_count(self):
		__total_times = 0
		__sleep_phase_dict = deepcopy(self.sleep_phase_dict)
		while __sleep_phase_dict:
			temp = __sleep_phase_dict.popitem(last=False)
			if temp[1] == 6:
				__total_times += 1
		self.woke_up_count = __total_times;
		return

	def get_metrics(self):
		# get all of these variables
		# refer <https://api.hexoskin.com/docs/resource/metric/>
		# self.sleep_efficieny
		# self.sleep_latency
		# self.sleep_percent
		# self.sleep_period
		# self.sleep_position_changes
		# self.sleep_total_time
		# self.sleep_non_REM_time
		# self.sleep_REM_time
		# self.sleep_wake_time
		pass

	def populate_DS(self):
		with open('sleepphase.txt', 'r') as f:
			testip = list(csv.reader(f, delimiter=','))
			flag = False
			for i in testip:
				if i[1] == "null":
					if not flag:
						flag = True
						self.start_sleep_act = int(float(i[0]))
						continue
					else:
						self.end_sleep_act = int(float(i[0]))
						continue
				self.sleep_phase_dict[int(float(i[0]))] = float(i[1])
			f.close()
		# print(self.sleep_phase_dict)
		# print(self.start_sleep_act)
		# print(self.end_sleep_act)

		with open('sleepposition.txt', 'r') as f:
			testip = list(csv.reader(f, delimiter=','))
			for i in testip:
				if i[1] == "null":
					continue
				if self.start_sleep_act <= int(float(i[0])) <= self.end_sleep_act:
					self.sleep_posn_dict[int(float(i[0]))] = float(i[1])
			f.close()
		# print(self.sleep_posn_dict)

def main():
	SleepObj = SleepAD()
	SleepObj.populate_DS()
	SleepObj.get_metrics()
	SleepObj.calc_woke_up_count()

if __name__ == '__main__':
	main()
