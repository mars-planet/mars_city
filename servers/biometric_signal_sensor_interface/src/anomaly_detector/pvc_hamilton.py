from __future__ import division, print_function
from collections import OrderedDict
from fractions import gcd

import ctypes
import csv
import time

# import matplotlib.pyplot as plt


class BeatAnalyzer(object):
    # this is a slightly modified copy of the
    # class BDAC from bdac.py
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
        self.OSEA.BeatDetectAndClassify.argtypes =\
            (ctypes.c_int, ctypes.POINTER(ctypes.c_int),
             ctypes.POINTER(ctypes.c_int))
        self.OSEA.ResetBDAC.argtypes = ()

    def ResetBDAC(self):
        self.OSEA.ResetBDAC()

    def BeatDetectAndClassify(self, sample_val):
        beatType = ctypes.c_int()
        beatMatch = ctypes.c_int()
        result = self.OSEA.\
            BeatDetectAndClassify(ctypes.c_int(sample_val),
                                  ctypes.byref(beatType),
                                  ctypes.byref(beatMatch))
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
            self.m = int(ifreq / i)
            self.n = int(ofreq / i)
            self.mn = int(self.m * self.n)
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
            vout = int(self.vv + int((self.ot % self.n)) *
                       (self.v - self.vv) / self.n)
            self.ot += self.m
        return int(self.rval), int(vout)


class PVC(object):
    def __init__(self):
        # key:value = timestamp:ecg(4113)
        self.ecg_dict = OrderedDict()

        # key:value = timestamp:rrinterval_status
        self.rrquality_dict = OrderedDict()

        # use popitem() to get anomaly data
        # key:value = timestamp:(RRintervalstatus, PVC_from)
        self.anomaly_dict = OrderedDict()

    def delete_method(self):
        # method to maintain data structures' size

        # initial wait time
        time.sleep(60*10)
        while self.rrquality_dict:
            for i in xrange(256):
                self.ecg_dict.popitem(last=False)

            self.rrquality_dict.popitem(last=False)
            time.sleep(20)

        return

    def get_nearest_Rpeak(self, timestamp):
        # look in backward direction
        for i in xrange(256*10):
            # start from the timestamp sent
            if (timestamp-i) in self.rrquality_dict:
                return (timestamp-i)

        # look in forward direction
        for i in xrange(256*10):
            # start from the timestamp sent
            if (timestamp+i) in self.rrquality_dict:
                return (timestamp+i)

        return None

    def beat_classf_analyzer(self, init_hexo_time):
        """
        PVC indicator
        """
        Beats = BeatAnalyzer(self.ecg_dict, init_hexo_time)
        ADCGain, ADCZero = 200, 1024
        ip_freq, op_freq = 256, 200

        Beats.ResetBDAC()
        samplecount = 0

        # beatTypeList, detectionTimeList = [], []

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
                # detectionTimeList.\
                #     append(self.get_nearest_Rpeak
                #            (init_hexo_time + int(DetectionTime)))

                # 5 is PVC beatType
                if beatType == 5:
                    print("PVC", DetectionTime)
                    timestamp = self.\
                        get_nearest_Rpeak(init_hexo_time + int(DetectionTime))
                    self.anomaly_dict[timestamp] =\
                        (self.rrquality_dict[timestamp], 2)

                time.sleep(0.05)

        # print(len(detectionTimeList))
        # # uncomment to visualize
        # with open('ecg_APC.txt', 'r') as f:
        #     testip = list(csv.reader(f, delimiter='\t'))
        #     newarr = [int(i[1]) for i in testip]
        #     xcoordinates = [int(i[0]) for i in testip]
        #     plt.plot(xcoordinates, newarr)
        #     newtemparr = [i for i in detectionTimeList]
        #     testip = dict(testip)
        #     ycoordinates = [testip[str(i)] for i in newtemparr]
        #     plt.plot(newtemparr, ycoordinates, 'ro')
        #     plt.show()

    def populate_data(self):
        with open('ecg_APC.txt', 'r') as f:
            testip = list(csv.reader(f, delimiter='\t'))
            for i in testip:
                self.ecg_dict[int(i[0])] = int(i[1])
            f.close()

        with open('rrinterval_status_APC.txt', 'r') as f:
            testip = list(csv.reader(f, delimiter='\t'))
            for i in testip:
                self.rrquality_dict[int(i[0])] = int(i[1])
            f.close()


def main():
    pvcObj = PVC()
    pvcObj.populate_data()
    pvcObj.beat_classf_analyzer(383021266184)
    # pvcObj.delete_method()


if __name__ == '__main__':
    main()
