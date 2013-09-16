#!/usr/bin/python
"""
Implements the Aouda Suit Server Interface.
"""
from __future__ import division, print_function

import os
import sys
from datetime import datetime, timedelta

from numpy import nan

from preprocessing import read_data, extract_hr_acc


class Aouda(object):
    """
    Implements the Aouda Server Interface.
    """

    def __init__(self, filename=None):
        print('Constructing Aouda')
        self.resolution = 1000  # in millisecs
        print('Loading Data')
        self.data = self._load_data(filename)
        print('Finished constructing Aouda')

    def _load_data(self, filename):
        """
        Loads hr and acc data into a pandas.DataFrame from a dataset file.
        """
        if filename is None:
            dirname = os.path.dirname(__file__)
            filename = os.path.join(dirname, 'dataset.dat')
        data = extract_hr_acc(read_data(filename))
        return data.resample('%sL' % self.resolution)

    def get_data(self, period):
        """
        Returns an array with all datapoints
        from last query until datetime.until().
        The format is:
        [
         [hr1, acc_x1, acc_y1, acc_z1,
          hr2, acc_x2, acc_y2, acc_z2,
          ...
          hrN, acc_xN, acc_yN, acc_zN],
         [timestamp1, timestamp1, timestamp1, timestamp1,
          timestamp2, timestamp2, timestamp2, timestamp2,
          ...
          timestampN, timestampN, timestampN, timestampN],
        ]
        """
        until = datetime.now()
        since = until - timedelta(seconds=period)
        filtered_data = self.data[self.data.index >= since]
        filtered_data = filtered_data[filtered_data.index <= until]
        if len(filtered_data) > 0:
            ret_val = [[], []]
            for index, row in filtered_data.iterrows():
                ret_val[0].append(row['hr'])
                ret_val[0].append(row['acc_x'])
                ret_val[0].append(row['acc_y'])
                ret_val[0].append(row['acc_z'])
                ret_val[1].extend([index.strftime('%Y-%m-%d %H:%M:%S.%f')] * 4)

            return ret_val
        else:
            print("No more data on dataset.")
            sys.exit()

    def _get_instantaneos_values(self):
        """
        Returns a row from the data such that row.index <= datetime.now().
        """
        current_row = self.data[self.data.index <= datetime.now()]
        if len(current_row) > 0:
            current_row = current_row.ix[-1]
        return current_row

    def read_heart_rate(self):
        """
        Returns instantaneous heart rate.
        """
        heart_rate = self._get_instantaneos_values()
        if len(heart_rate) > 0:
            heart_rate = heart_rate['hr']
        else:
            heart_rate = nan
        return heart_rate

    def read_acc_magn(self):
        """
        Returns instantaneous acceleration magnitude.
        """
        acc_magn = self._get_instantaneos_values()
        if len(acc_magn) > 0:
            acc_magn = acc_magn['acc']
        else:
            acc_magn = nan
        return acc_magn
