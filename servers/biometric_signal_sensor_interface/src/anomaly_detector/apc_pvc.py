from __future__ import division, print_function

import sys
import csv
import time
import subprocess

is_py2 = sys.version[0] == '2'
if is_py2:
    import Queue as queue
else:
    import queue as queue

import matplotlib.pyplot as plt

class PVC(object):
	pass

class APC(object):
	def __init__(self):

		# circular array size - circular counter
		self.cc = 10000

		# both of ecg_q and rrint_status_q need to be continuously populated

		# ecg data as a queue
		# each element is a tuple (timestamp, ecg(4113), ecg(4114))
		self.ecg_q = queue.Queue()

		# used as anomaly quality indicator
		# each element is a tuple (timestamp, rr_quality)
		self.rrint_status_q = queue.Queue()

		populate_queue(self)

		# holds detected RR peak timestamp
		self.RRtime = []

		# holds detected QRS width
		self.QRSwidth = []

		self.popluate_aux_structures()

	def popluate_aux_structures(self):
		# window size in number of ecg samples
		__ecgPUrange = 256*30
		# window overlap in number of ecg samples
		__ecgOverlap = 256*5

		ecg_buffer = []

		# time.sleep(300)

		while not self.ecg_q.empty():
			ecg_cur_window = []
			ecg_cur_window.extend(ecg_buffer)
			ecg_buffer = []

			for i in xrange(__ecgPUrange - len(ecg_buffer)):
				if not self.ecg_q.empty():
					ecg_tuple = self.ecg_q.get()
					ecg_cur_window.append(ecg_tuple)
					if i > (__ecgPUrange - len(ecg_buffer)) - __ecgOverlap:
						ecg_buffer.append(ecg_tuple)
				else:
					break
			
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
						print(next_index - prev_index)
						print(next_index, prev_index)
						self.RRtime.append(ecg_cur_window[int(line[1]) - 1][0])
						print(self.RRtime)
						sys.exit()
					prev_index = int(line[1])

			
			sys.exit()



# method to populate ecg and rrint_status queues for testing
def populate_queue(apcObj):
	with open('ecg_APC.txt', 'r') as f:
		testip = list(csv.reader(f, delimiter='\t'))
		for i in testip:
			apcObj.ecg_q.put((int(i[0]), int(i[1]), int(i[1])))

	with open('rrinterval_status_APC.txt', 'r') as f:
		testip = list(csv.reader(f, delimiter='\t'))
		for i in testip:
			apcObj.rrint_status_q.put((int(i[0]), int(i[1])))

def main():
	with open('ecg_APC.txt', 'r') as f:
		testip = list(csv.reader(f, delimiter='\t'))
		newarr = [i[1] for i in testip]

		plt.plot(range(len(newarr)), newarr)
		# plt.plot(detectionTimeList, [1350] * len(detectionTimeList), 'ro')
		plt.show()
	apcObj = APC()

if __name__ == '__main__':
	main()
