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
    def __init__(self, config, init_val):
        # set the percent change for get_window
        self.window_delta = 0.003
        # the size of the window to be used in the classifier
        self.resp_classf_win_size = 2

        # the first val of the resp data
        self.init_val = init_val

        # get the tidal volume deltas
        self.tidal_volume_delta =\
            config.getfloat('Respiratory AD', 'tidal_volume_delta')
        self.tidal_volume_window_delta =\
            config.getfloat('Respiratory AD', 'tidal_volume_window_delta')

        # get the minute ventilation deltas
        self.minute_ventilation_delta =\
            config.getfloat('Respiratory AD',
                            'minute_ventilation_delta')
        self.minute_ventilation_window_delta =\
            config.getfloat('Respiratory AD',
                            'minute_ventilation_window_delta')

        # get resp up and down thresholds - in raw units from the baseline
        # upper bounds
        self.up_thresh = config.getint('Respiratory AD', 'up_thresh')
        self.down_thresh = config.getint('Respiratory AD', 'down_thresh')
        # lower bounds
        self.up_low = config.getint('Respiratory AD', 'up_low')
        self.down_low = config.getint('Respiratory AD', 'down_low')

        # get respiratory variation threshold for resp_variation() method
        self.resp_variation_thresh =\
            config.getint('Respiratory AD', 'resp_variation_thresh')

        # raw respiratory data as an OrderedDict
        # key:value = timestamp:(thoracic, abdominal)
        self.raw_resp_dict = OrderedDict()

        # breathing rate dict
        # key:value = timestamp:breathing rate
        self.breathing_rate_dict = OrderedDict()

        # breathing rate status dict
        # key:value = timestamp:breathing rate quality
        self.breathing_rate_status_dict = OrderedDict()

        # inspiration dict
        # key:value = timestamp:inspiration
        self.inspiration_dict = OrderedDict()

        # expiration dict
        # key:value = timestamp:expiration
        self.expiration_dict = OrderedDict()

        # tidal volume (vt) dict
        # key:value = timestamp:tidal volume
        self.tidal_volume_dict = OrderedDict()

        # minute ventilation dict
        # key:value = timestamp:minute ventilation
        self.minute_ventilation_dict = OrderedDict()

        # dict of tidal volume anomalies
        # in general Key:value =\
        #     timestamp:(int(breathing rate status mean), 'string')
        # -1 for breathing rate status if not applicable
        # mean is mean of breathing rate status in that period
        # key:value = timestamp:(-1, 'tidal_window'/'tidal_not_window')
        # key:value =\
        #     timestamp:(-1, 'minute_vent_window'/'minute_vent_not_window')
        # key:value = timestamp:(-1, 'insp-exp'/'exp-inp')
        self.resp_anomaly_dict = OrderedDict()

    def delete_DS(self):
        # method to maintain data structures' size

        # initial wait time
        time.sleep(60*10)
        while self.raw_resp_dict:
            for i in xrange(200):
                self.raw_resp_dict.popitem(last=False)

            self.breathing_rate_dict.popitem(last=False)
            self.breathing_rate_status_dict.popitem(last=False)
            self.inspiration_dict.popitem(last=False)
            self.expiration_dict.popitem(last=False)
            self.tidal_volume_dict.popitem(last=False)
            self.minute_ventilation_dict.popitem(last=False)
            time.sleep(20)

        return

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
                    window.append((begin+1,
                                   self.minute_ventilation_dict
                                   [begin+1]))
            begin += 1

        # set prev and begin
        prev = self.minute_ventilation_dict[begin]
        begin += 1

        while len(self.minute_ventilation_dict) > 100:
            time.sleep(20)
            if begin in self.minute_ventilation_dict:
                cur = self.minute_ventilation_dict[begin]

                # check for non window delta
                percent = (self.minute_ventilation_delta/100) * prev
                if not ((prev - percent) < cur < (prev + percent)):
                    self.resp_anomaly_dict[begin] =\
                        (-1, 'minute_vent_not_window')

                # check for window delta
                windowmean = sum([i[1] for i in window])/len(window)
                percent =\
                    (self.minute_ventilation_window_delta/100) * windowmean
                if not ((windowmean - percent) < cur < (windowmean + percent)):
                    self.resp_anomaly_dict[begin] = (-1, 'minute_vent_window')

                # update window
                window.pop(0)
                window.append((begin, cur))
                # update prev
                prev = self.minute_ventilation_dict[begin]

            # update begin
            begin += 1

            # if begin == 383021389273:
            #   print(self.minute_ventilation_anomaly_dict)

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
            time.sleep(20)
            if begin in self.tidal_volume_dict:
                cur = self.tidal_volume_dict[begin]

                # check for non window delta
                percent = (self.tidal_volume_delta/100) * prev
                if not ((prev - percent) < cur < (prev + percent)):
                    self.resp_anomaly_dict[begin] =\
                        (-1, 'tidal_volume_not_window')

                # check for window delta
                windowmean = sum([i[1] for i in window])/len(window)
                percent = (self.tidal_volume_window_delta/100) * windowmean
                if not ((windowmean - percent) < cur < (windowmean + percent)):
                    self.resp_anomaly_dict[begin] = (-1, 'tidal_volume_window')

                # update window
                window.pop(0)
                window.append((begin, cur))
                # update prev
                prev = self.tidal_volume_dict[begin]

            # update begin
            begin += 1

            # if begin == 383021389273:
            #   print(self.tidal_volume_anomaly_dict)

        return

    def resp_variation(self):
        # finds gap between inspiration and expiration
        inspiration = self.init_val
        while True:
            if inspiration in self.inspiration_dict:
                break
            inspiration += 1

        timestamp = []
        while len(self.inspiration_dict) > 100 and\
                len(self.expiration_dict) > 100:
            time.sleep(20)
            expiration = inspiration
            # find expiration
            while True:
                if expiration in self.expiration_dict:
                    break
                expiration += 1

            # if gap is greater than 3 seconds
            if (expiration - inspiration) > (256*self.resp_variation_thresh):
                print('exp, insp')
                self.resp_anomaly_dict[inspiration] = (-1, 'exp-insp')
                timestamp.append(inspiration)

            # find inspiration
            inspiration = expiration
            while True:
                if inspiration in self.inspiration_dict:
                    break
                inspiration += 1

            # if gap is greater than 3 seconds
            if (inspiration - expiration) > (256*self.resp_variation_thresh):
                print('insp, exp')
                self.resp_anomaly_dict[expiration] = (-1, 'insp-exp')
                timestamp.append(expiration)

            # if inspiration == 383021388517:
            #   break

        # dicts = dict(zip(list(self.raw_resp_dict.keys()),
        #                  [i[0] for i in
        #                   list(self.raw_resp_dict.values())]))
        # plt.plot(list(self.raw_resp_dict.keys()),
        #          [i[0] for i in list(self.raw_resp_dict.values())])
        # plt.plot(timestamp,
        #          [self.raw_resp_dict[i][0] for i in
        #           list(self.raw_resp_dict.keys()) if i in timestamp], 'ro')
        # plt.show()
        # print(timestamp)
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

        with open('resp.txt', 'r') as f:
            testip = list(csv.reader(f, delimiter='\t'))
            for i in testip:
                self.raw_resp_dict[int(i[0])] = (int(i[1]), int(i[2]))
            f.close()

        with open('breathingrate.txt', 'r') as f:
            testip = list(csv.reader(f, delimiter='\t'))
            for i in testip:
                self.breathing_rate_dict[int(i[0])] = int(i[1])
            f.close()

        with open('br_quality.txt', 'r') as f:
            testip = list(csv.reader(f, delimiter='\t'))
            for i in testip:
                self.breathing_rate_status_dict[int(i[0])] = int(i[1])
            f.close()

        with open('inspiration.txt', 'r') as f:
            testip = list(csv.reader(f, delimiter='\t'))
            for i in testip:
                self.inspiration_dict[int(i[0])] = int(i[1])
            f.close()

        with open('expiration.txt', 'r') as f:
            testip = list(csv.reader(f, delimiter='\t'))
            for i in testip:
                self.expiration_dict[int(i[0])] = int(i[1])
            f.close()

        return


def main():
    config = ConfigParser.RawConfigParser()
    dirname = os.path.dirname(os.path.realpath(__file__))
    cfg_filename = os.path.join(dirname, 'anomaly_detector.cfg')
    config.read(cfg_filename)

    respObj = RespiratoryAD(config, 383021140185)

    th1 = Thread(target=respObj.populate_DS, args=[])
    th1.start()

    th1.join()

    th2 = Thread(target=respObj.tidal_volume_anomaly, args=[])
    th2.start()

    th3 = Thread(target=respObj.minute_ventilation_anomaly, args=[])
    th3.start()

    th4 = Thread(target=respObj.resp_variation, args=[])
    th4.start()

    # th5 = Thread(target=respObj.resp_classf, args=[])
    # th5.start()

    # th6 = Thread(target=respObj.delete_DS, args=[])
    # th6.start()

    # print(respObj.resp_anomaly_dict)


if __name__ == '__main__':
    main()
