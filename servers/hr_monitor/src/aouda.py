#!/usr/bin/python
"""
Implements the Aouda Suit Server Interface.
"""
from __future__ import division, print_function

import os
import sys
from datetime import datetime, timedelta

from preprocessing import read_data, extract_hr_acc


class Aouda(object):
    """
    Implements the Aouda Server Interface.
    """

    # for sqlite use conn_str='sqlite:///hr_monitor.db'
    def __init__(self, filename=None):
        print('Constructing Aouda')
        self.resolution = 1000  # in millisecs
        print('Loading Data')
        self.data = self._load_data(filename)
        print('Finished constructing Aouda')

    def _load_data(self, filename):
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
        [[hr, timestamp1],
         [acc_x, timestamp1], [acc_y, timestamp1], [acc_z, timestamp1],
         [hr, timestamp2],
         [acc_x, timestamp2], [acc_y, timestamp2], [acc_z, timestamp2],
         ...
         [hr, timestampN],
         [acc_x, timestampN], [acc_y, timestampN], [acc_z, timestampN],
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
