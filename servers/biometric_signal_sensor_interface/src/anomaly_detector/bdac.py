from __future__ import division, print_function
from fractions import gcd

import ctypes
import csv

_OSEA = ctypes.CDLL('osea.so')
_OSEA.BeatDetectAndClassify.argtypes = (ctypes.c_int, ctypes.POINTER(ctypes.c_int), ctypes.POINTER(ctypes.c_int))
_OSEA.ResetBDAC.argtypes = ()

ip = None

def ResetBDAC():
	global _OSEA
	_OSEA.ResetBDAC()

def BeatDetectAndClassify(sample_val):
    global _OSEA
    beatType = ctypes.c_int()
    beatMatch = ctypes.c_int()
    result = _OSEA.BeatDetectAndClassify(ctypes.c_int(sample_val), ctypes.byref(beatType), ctypes.byref(beatMatch))
    return int(result), int(beatType.value)

def getVec():
	global ip
	
	if not hasattr(getVec, "i"):
		getVec.i = 0

	try:
		getVec.i += 1
		return int(ip[getVec.i - 1])
	except:
		return -1

def NextSample(ifreq, ofreq, init):
	if not all(hasattr(NextSample, attr) for attr in ["m", "n", "mn", "ot", "it", "vv", "v", "rval"]):
	    NextSample.m = 0
	    NextSample.n = 0
	    NextSample.mn = 0
	    NextSample.ot = 0
	    NextSample.it = 0
	    NextSample.vv = 0
	    NextSample.v = 0
	    NextSample.rval = 0

	vout = -1
	if init:
		i = gcd(ifreq, ofreq)
		NextSample.m = int(ifreq/i)
		NextSample.n = int(ofreq/i)
		NextSample.mn = int(NextSample.m*NextSample.n)
		NextSample.vv = int(getVec())
		NextSample.v = int(getVec())
		NextSample.rval = NextSample.v
	else:
		while NextSample.ot > NextSample.it:
			NextSample.vv = NextSample.v
			NextSample.v = int(getVec())
			NextSample.rval = NextSample.v
			if NextSample.it > NextSample.mn:
				NextSample.it -= NextSample.mn
				NextSample.ot -= NextSample.mn
			NextSample.it += NextSample.n
		vout = int(NextSample.vv + int((NextSample.ot%NextSample.n)) * (NextSample.v-NextSample.vv)/NextSample.n)
		NextSample.ot += NextSample.m
	return int(NextSample.rval), int(vout)

def AnalyzeBeatTypeSixSecond(ecg, ADCGain, ADCZero, ip_freq, op_freq):
	global ip
	global _OSEA

	ResetBDAC()
	samplecount = 0

	beatTypeList = []
	detectionTimeList = []
	ip = ecg

	nextval, ecgval = NextSample(ip_freq, op_freq, 1)
	while nextval != -1:
		nextval, ecgval = NextSample(ip_freq, op_freq, 0)
		samplecount += 1

		lTemp = ecgval - ADCZero
		lTemp *= 200
		lTemp /= ADCGain
		ecgval = lTemp

		delay, beatType = BeatDetectAndClassify(int(ecgval))

		if delay != 0:
			DetectionTime = samplecount - delay

			DetectionTime *= ip_freq
			DetectionTime /= op_freq
			# print(DetectionTime, beatType)
			beatTypeList.append(beatType)
			detectionTimeList.append(DetectionTime)

	return beatTypeList, detectionTimeList

def main():
	with open('testinput.txt', 'r') as f:
		testip = list(csv.reader(f, delimiter=' '))
		newarr = [i[0] for i in testip]
		AnalyzeBeatTypeSixSecond(newarr, 200, 1024, 360, 200)

if __name__ == '__main__':
	main()