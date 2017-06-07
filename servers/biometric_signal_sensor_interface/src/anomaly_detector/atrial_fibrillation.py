from __future__ import division, print_function

import numpy as np

from math import floor


class AtrialFibrillation(object):
    """
    implements the methods for Atrial Fibrillation Anomaly Detection
    based on 'A Simple Method to Detect Atrial Fibrillation Using RR Intervals'
    by Jie Lian et. al.
    Input data from rr_interval.txt and hr_quality.txt
    """
    def __init__(self, rr_intervals, hr_quality_indices, win_size):
        self.rr_intervals = rr_intervals
        self.hr_quality_indices = hr_quality_indices
        self.win_size = win_size
        # throws key error if win_size is wrongly set
        self.nec_cutoff_threshold = {32: 23, 64: 40, 128: 65}[self.win_size]
        self.num_of_squares = 56
        # set from the cited paper - has to be a square grid
        self.x_offset = -8
        self.y_offset = 20

    def __calc_hr_quality(self):
        """
        returns percentage of non-zero hr_quality indices in the given range
        """
        num_of_nonzeros = \
            self.hr_quality_indices['quality_ind'].astype(bool).sum(axis=0)
        return int((num_of_nonzeros/len(self.hr_quality_indices))*100)

    def __plot_map(self, __2Dgrid):
        import matplotlib.pyplot as plt
        plt.imshow(__2Dgrid, cmap='gist_heat', interpolation='nearest')
        plt.show()

    def get_anomaly(self):
        data_reliability = self.__calc_hr_quality()

        start_hexo_timestamp = self.rr_intervals['hexoskin_timestamps'][0]
        end_hexo_timestamp = self.\
            rr_intervals['hexoskin_timestamps'][len(self.rr_intervals)-1]

        self.rr_intervals = self.rr_intervals.\
            reindex(columns=['rr_int', 'hexoskin_timestamps'])
        # rdr is difference between rr intervals as described in the paper
        self.rr_intervals.columns = ['rr_int', 'rdr']
        self.rr_intervals['rdr'] = self.rr_intervals['rdr'].astype(np.float64)

        for i in xrange(1, len(self.rr_intervals)):
            self.rr_intervals['rdr'][i] = self.rr_intervals['rr_int'][i] -\
                self.rr_intervals['rr_int'][i-1]

        __2Dgrid = np.zeros((self.num_of_squares,
                            self.num_of_squares), dtype=np.int8)

        exceptioncnt = 0
        for i in xrange(1, len(self.rr_intervals)):
            # convert seconds to milliseconds and find cell coordinates
            __xindex = int(floor((self.rr_intervals['rr_int'][i] * 1000)
                                 / 25)) + self.x_offset
            __yindex = int(floor((self.rr_intervals['rdr'][i] * 1000)
                                 / 25)) + self.y_offset
            try:
                __2Dgrid[__xindex][__yindex] += 1
            except IndexError:
                # some pints may lie outside the grid due to bad quality data
                exceptioncnt += 1

        # self.__plot_map(__2Dgrid)
        __num_of_nonzero = np.count_nonzero(__2Dgrid)

        if __num_of_nonzero >= self.nec_cutoff_threshold:
            return {'start_hexo_timestamp': start_hexo_timestamp,
                    'end_hexo_timestamp': end_hexo_timestamp,
                    'num_of_NEC': __num_of_nonzero,
                    'data_reliability': data_reliability,
                    'window_size': self.win_size}
        else:
            return -1
