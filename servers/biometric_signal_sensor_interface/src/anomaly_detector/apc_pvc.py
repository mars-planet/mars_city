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
		# key:value = timestamp:(RRintervalstatus, PVC_from)
		self.anomaly_dict = OrderedDict()

		# references for the above dicts
		self.QRSarea_REF = None
		self.vecg_REF = None

	def delete_method(self):
		# method to maintain data structures' size

		# initial wait time
		time.sleep(60*10)
		while rrint_status_dict:
			for i in xrange(256):
				self.ecg_dict.popitem(last=False)

			self.rrint_status_dict.popitem(last=False)
			self.RRint_dict.popitem(last=False)
			self.QRSwidth_dict.popitem(last=False)
			self.QRSarea_dict.popitem(last=False)
			self.vecg_dict.popitem(last=False)
			time.sleep(20)

		return

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
		# timestamp is part of backward search
		timestamp_copy = timestamp + 1

		# look in backward direction
		while len(window) < minus:
			# start from the timestamp sent
			if timestamp in localdict:
				window.append((timestamp, localdict[timestamp]))
			timestamp -= 1
			# break if not found
			if timestamp < timestamp_copy - (256 * 30):
				break

		# arrange the window in correct order
		window.reverse()

		# look in forward direction
		while len(window) < (minus + plus):
			if timestamp_copy in localdict:
				window.append((timestamp_copy, localdict[timestamp_copy]))
			timestamp_copy += 1
			# look in forward direction
			if timestamp_copy > timestamp + (256 * 50):
				break

		if len(window) == (minus + plus):
			return window
		else:
			raise ValueError

	def find_overlaps(self, window):
		ocount = 0
		# simple O(n^2) overlap check
		for i in xrange(len(window)):
			starti, endi = window[i][0], window[i][1]
			
			for j in xrange(i + 1, len(window)):
				startj, endj = window[j][0], window[j][1]
				# if starting point of either lies within
				# the other window, then it is an overlap
				if (startj <= starti <= endj) or\
				   (starti <= startj <= endi):
				   ocount += 1
		return ocount

	def final_APC_test(self, timestamp):
		# get QRSwidth, QRSarea and vecg lists
		QRSwidth_list = self.get_window(6, 0, timestamp, 1)
		QRSarea_list = self.get_window(6, 0, timestamp, 2)
		vecg_list = self.get_window(6, 0, timestamp, 3)

		# get the current values
		curQRSwidth = QRSwidth_list.pop()[1]
		curQRSarea = QRSarea_list.pop()[1]
		curvecg = vecg_list.pop()[1]

		# sort to get median
		QRSwidthREF = sorted(QRSwidth_list, key=lambda x: x[1])[2][1]
		QRSareaREF = sorted(QRSarea_list, key=lambda x: x[1])[2][1]
		vecgREF = sorted(vecg_list, key=lambda x: x[1])[2][1]

		# calculate the DIFFS
		QRSwidthDIFF = (abs(curQRSwidth - QRSwidthREF)/QRSwidthREF)*100
		QRSareaDIFF = (abs(curQRSarea - QRSareaREF)/QRSareaREF)*100
		vecgDIFF = abs(curvecg - vecgREF)

		# classify
		if (QRSwidthDIFF < 5) or\
		   ((QRSwidthDIFF < 10) and (QRSareaDIFF < 25)) or\
		   ((vecgDIFF < 25) and (QRSareaDIFF < 50)):
		    print("APC apc test")
		    nearestRR = self.get_window(1, 0, timestamp, 0)[0][0]
		    if nearestRR in self.rrint_status_dict:
		    	self.anomaly_dict[nearestRR] = (self.rrint_status_dict[nearestRR], 0)
		else:
			print("PVC apc test")
			nearestRR = self.get_window(1, 0, timestamp, 0)[0][0]
			if nearestRR in self.rrint_status_dict:
				self.anomaly_dict[nearestRR] = (self.rrint_status_dict[nearestRR], 1)

		return

	def single_premature_heartbeat(self, timestamp):
		seven_beats = self.get_window(7, 0, timestamp, 0)
		ith_rr = seven_beats[6][1]
		iminusone_rr = seven_beats[5][1]
		avg_first_five_rr = sum([seven_beats[i][1] for i in xrange(5)])/5

		# percentage difference averaged over previous intervals
		RR_diff = (abs(ith_rr - iminusone_rr)/avg_first_five_rr)*100

		# RR_diff is greater than 15% and additional conditions
		if RR_diff > 15 and\
			((iminusone_rr < (0.9*avg_first_five_rr)) or
			 ((ith_rr > (1.1*avg_first_five_rr)) and
			  (iminusone_rr <= avg_first_five_rr)) or
			 ((ith_rr/iminusone_rr) > 1.2)):
			self.final_APC_test(timestamp)
		else:
			if (iminusone_rr < (0.75*avg_first_five_rr)) and\
			   (ith_rr < (0.75*avg_first_five_rr)):
				print("most likely V single premature heartbeat")
				nearestRR = self.get_window(1, 0, timestamp, 0)[0][0]
				if nearestRR in self.rrint_status_dict:
					self.anomaly_dict[nearestRR] = (self.rrint_status_dict[nearestRR], 1)
			else:
				print("N or V beat single premature heartbeat")

		return

	def checkAorV(self, temp_beatlist):
		# the QRSarea_REF this time is the QRSarea of the first beat
		# after the first pathological pause
		QRSarea_REF = self.get_window(1, 0, temp_beatlist[0][0], 2)[0][1]
		# the vecg_REF this time is the vecg of the first beat
		# after the first pathological pause
		vecg_REF = self.get_window(1, 0, temp_beatlist[0][0], 3)[0][1]
		
		# get the QRSareas
		QRSarea_list = self.get_window(len(temp_beatlist), 0, temp_beatlist[len(temp_beatlist)-1][0] + 1, 2)
		# get the vecg values
		vecg_list = self.get_window(len(temp_beatlist), 0, temp_beatlist[len(temp_beatlist)-1][0] + 1, 3)

		for i in xrange(len(temp_beatlist)):
			QRSarea = QRSarea_list[i][1]
			QRSarea_DIFF = (abs(QRSarea - QRSarea_REF)/QRSarea_REF)*100

			vecg = vecg_list[i][1]
			vecg_DIFF = abs(vecg - vecg_REF)

			# if QRSArea difference less than 50% and angle less than 25 deg
			if QRSarea_DIFF < 50 and vecg_DIFF < 25:
				# APC detected
				print("APC pathologic pause")
				timestamp = temp_beatlist[i][0]
				nearestRR = self.get_window(1, 0, timestamp, 0)[0][0]
				if nearestRR in self.rrint_status_dict:
					self.anomaly_dict[nearestRR] = (self.rrint_status_dict[nearestRR], 0)
			else:
				# PVC detected
				print("PVC pathologic pause")
				timestamp = temp_beatlist[i][0]
				nearestRR = self.get_window(1, 0, timestamp, 0)[0][0]
				if nearestRR in self.rrint_status_dict:
					self.anomaly_dict[nearestRR] = (self.rrint_status_dict[nearestRR], 1)

		return

	def pathologic_pause_test(self, timestamp):
		# get rrint with this timestamp or previous to it
		rrint = self.get_window(1, 0, timestamp, 0)
		# if it is more than 1.5 seconds
		if rrint[0][1] > 1.5 * 256:
			# get next 25 RR intervals
			# an approximation for next 15 seconds
			rrint_list = self.get_window(0, 25, timestamp, 0)

			temp_beatlist = []
			for i in xrange(len(rrint_list)):
				timestamp_cur = rrint_list[i][0]
				# if is more than 1.5 seconds and
				# lies within 15 second interval
				if (rrint_list[i][1] > (1.5 * 256)) and\
					((timestamp_cur - timestamp) < (15*256)) and\
						len(temp_beatlist) > 0:
					# label all beats in between
					self.checkAorV(temp_beatlist)
					break
				else:
					temp_beatlist.append(rrint_list[i])
		else:
			# check for single premature heartbeat
			self.single_premature_heartbeat(timestamp)
		return

	def supraventricular_tachycardia(self, timestamp):
		# get rrint with this timestamp or previous to it
		rrint = self.get_window(1, 0, timestamp, 0)
		# if it is less than 0.5 seconds
		if rrint[0][1] < 0.5 * 256:
			if not (self.QRSarea_REF and self.vecg_REF):
				# get previous 5 QRSarea
				prev_five_QRSarea = self.get_window(5, 0, timestamp, 2)
				prev_five_QRSarea = sorted(prev_five_QRSarea, key=lambda x: x[1])
				median_QRSarea = prev_five_QRSarea[2][1]
				# set the QRSarea_REF to be the median
				self.QRSarea_REF = median_QRSarea
				print(self.QRSarea_REF, "median QRS")

				# get previous 5 vecg
				prev_five_vecg = self.get_window(5, 0, timestamp, 3)
				prev_five_vecg = sorted(prev_five_vecg, key=lambda x: x[1])
				median_vecg = prev_five_vecg[2][1]
				# set the vecg_REF to be the median
				self.vecg_REF = median_vecg
				print(self.vecg_REF, "median vecg")
			else:
				QRSarea = self.get_window(1, 0, timestamp, 2)[0][1]
				QRSarea_DIFF = (abs(QRSarea - self.QRSarea_REF)/self.QRSarea_REF)*100

				vecg = self.get_window(1, 0, timestamp, 3)[0][1]
				vecg_DIFF = abs(vecg - self.vecg_REF)

				# if QRSArea difference less than 50% and angle less than 25 deg
				if QRSarea_DIFF < 50 and vecg_DIFF < 25:
					# APC detected
					print("APC supraventricular tachycardia")
					nearestRR = self.get_window(1, 0, timestamp, 0)[0][0]
					if nearestRR in self.rrint_status_dict:
						self.anomaly_dict[nearestRR] = (self.rrint_status_dict[nearestRR], 0)
				else:
					# PVC detected
					print("PVC supraventricular tachycardia")
					nearestRR = self.get_window(1, 0, timestamp, 0)[0][0]
					if nearestRR in self.rrint_status_dict:
						self.anomaly_dict[nearestRR] = (self.rrint_status_dict[nearestRR], 1)
		else:
			self.pathologic_pause_test(timestamp)

		return

	def absolute_arrhythmia(self):
		# initial wait time
		# time.sleep(180)

		# skip the initial 20 seconds
		self.init_timestamp += 20*256
		next_timestamp = self.init_timestamp

		while self.RRint_dict:
			# get 10 seconds of RR interval - 5th is current
			# if the indexing starts from 0
			cur_window = self.get_window(6, 4, next_timestamp, 0)

			# calculate mean of RR intervals
			mean_RR = sum([i[1] for i in cur_window])/10

			# find max RR interval to normalize the next
			maxDiff = 0
			for i in xrange(10):
				curDiff = abs(cur_window[i][1] - mean_RR)
				if curDiff > maxDiff:
					maxDiff = curDiff

			# calculate the +1% and -1% windows for each RR interval
			RRwindows = []
			one_percent = 0.01 * maxDiff
			for i in xrange(10):
				curDiff = abs(cur_window[i][1] - mean_RR)
				RRwindows.append((curDiff - one_percent, curDiff + one_percent))

			# find the number of overlaps
			numofoverlaps = self.find_overlaps(RRwindows)

			if numofoverlaps <= 6:
				# further check
				self.supraventricular_tachycardia(next_timestamp)
			else:
				# fill in db entry
				# might be N or V beat
				print("N or V beat from absolute arrythmia")
				pass

			# move the window further
			# since 5th is current in 0 based indexing,
			# next is 6th, move it to that
			next_timestamp = cur_window[6][0] + 1
			# time.sleep(15)

		return
