"""
Implements the Aouda Suit Server Interface.
"""
from __future__ import division, print_function

from datetime import datetime, timedelta
from collections import namedtuple

from numpy import nan

from preprocessing import read_data, extract_hr_acc


class NoMoreDataError(Exception):
    """
    Exception to be thrown where there's no more data available in the dataset.
    """
    pass


class Aouda(object):
    """
    Implements the Aouda Server Interface.
    """

    def __init__(self, dataframe=None, filename=None,
                 resolution=None, base_datetime=None):
        print('Constructing Aouda')
        if resolution is not None:
            self.resolution = resolution  # in millisecs
        else:
            self.resolution = 1000  # in millisecs
        self.base_datetime = base_datetime
        print('Loading Data')
        if dataframe is not None:
            self.data = dataframe.copy()
        else:
            self.data = self._load_data(filename)
        self.data = self.data.resample('%sL' % self.resolution, how='mean')
        print('Finished constructing Aouda')

    def _load_data(self, filename):
        """
        Loads hr and acc data into a pandas.DataFrame from a dataset file.
        """
        data = extract_hr_acc(read_data(filename, self.base_datetime))
        return data

    def get_data(self, period):
        """
        Returns an array with all datapoints in the last period seconds.
        """
        until = datetime.now()
        since = until - timedelta(seconds=period)
        filtered_data = self.data[self.data.index >= since]
        filtered_data = filtered_data[filtered_data.index <= until]
        if len(filtered_data) > 0:
            ret_val = []
            for index, row in filtered_data.iterrows():
                datum = Aouda.DP(index, row['hr'], row['acc_x'],
                              row['acc_y'], row['acc_z'])
                ret_val.append(datum)
            return ret_val
        else:
            raise NoMoreDataError()

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


Aouda.DP = namedtuple('DP', ['timestamp', 'hr', 'acc_x', 'acc_y', 'acc_z'])
