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
	def __init__(self, ecg_baseline, ecg_halfrange):
		# to center the data - Hexoskin range from -13.1072 to +13.1072 mV
		self.ecg_baseline = ecg_baseline
		self.ecg_halfrange = ecg_halfrange

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

	def print_func(self):
		print(self.RRint_dict)
		print(self.QRSwidth_dict)
		print(self.QRSarea_dict)
		print(self.vecg_dict)

def main():
	pass

if __name__ == '__main__':
	main()
