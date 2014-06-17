"""
Implements the Aouda Suit Server Interface.
"""
from __future__ import division, print_function

from collections import namedtuple
from datetime import datetime, timedelta
import gc

import Oger as og
import numpy as np
import pandas as pd


class NoDataAvailableError(Exception):
    """
    Exception to be thrown where there's no more data available in the dataset.
    """
    pass


class Aouda(object):
    """
    Implements the Aouda Server Interface.
    """

    def __init__(self, dataframe=None,
                 base_datetime=None,
                 shift_data=False):
        print('Constructing Aouda')
        self.base_datetime = base_datetime
        print('Loading Data')
        if dataframe is not None:
            self.data = dataframe.copy()
        self.shift_data = shift_data
        print('Finished constructing Aouda')

    def _load_data(self, filename):
        """
        Loads hr and acc data into a pandas.DataFrame from a dataset file.
        """
        periods = 1000
        data = og.datasets.mackey_glass(sample_len=periods,
                                        n_samples=10,
                                        seed=50).reshape((10, periods))
        data = data + np.abs(data.min())
        data = data.transpose()
        return pd.DataFrame(data=data,
                            index=pd.date_range(datetime.now(),
                                                periods=periods,
                                                freq='500L'),
                            columns=['timestamp', 'ecg_v1', 'ecg_v2', 'o2',
                                     'temperature', 'air_flow', 'hr',
                                     'acc_x', 'acc_y', 'acc_z'])

    def _shift_data(self, from_datetime):
        if self.shift_data and self.data.index[-1] <= from_datetime:
            delta = (from_datetime - self.data.index[0]).total_seconds()
            print('Shifting data % s seconds.' % delta)
            self.data = self.data.shift(periods=int(delta), freq='S')
            gc.collect()

    def get_data(self, period):
        """
        Returns an array with all datapoints in the last period seconds.
        """
        until = datetime.now()
        self._shift_data(until)
        since = until - timedelta(seconds=period)
        filtered_data = self.data[self.data.index >= since]
        filtered_data = filtered_data[filtered_data.index <= until]
        ret_val = []
        try:
            if len(filtered_data) > 0:
                for index, row in filtered_data.iterrows():
                    datum = Aouda.DP(index, row['hr'], row['acc_x'],
                                  row['acc_y'], row['acc_z'])
                    ret_val.append(datum)
                return ret_val, until
            else:
                raise NoDataAvailableError()
        finally:
            del filtered_data

    def _get_instantaneos_values(self):
        """
        Returns a row from the data such that row.index <= datetime.now().
        """
        self._shift_data(datetime.now())
        current_row = self.data[self.data.index <= datetime.now()]
        if len(current_row) > 0:
            current_row = current_row.ix[-1]
        return current_row

    def read_ecg_v1(self):
        """
        Returns instantaneous readings of the V1 ECG contact.
        """
        ecg_v1 = self._get_instantaneos_values()
        if len(ecg_v1) > 0:
            ecg_v1 = ecg_v1['ecg_v1']
        else:
            ecg_v1 = np.nan
        return ecg_v1

    def read_ecg_v2(self):
        """
        Returns instantaneous readings of the V2 ECG contact.
        """
        ecg_v2 = self._get_instantaneos_values()
        if len(ecg_v2) > 0:
            ecg_v2 = ecg_v2['ecg_v2']
        else:
            ecg_v2 = np.nan
        return ecg_v2

    def read_o2(self):
        """
        Returns instantaneous O2 in blood values.
        """
        o2 = self._get_instantaneos_values()
        if len(o2) > 0:
            o2 = o2['o2']
        else:
            o2 = np.nan
        return o2

    def read_temperature(self):
        """
        Returns instantaneous temperature.
        """
        temperature = self._get_instantaneos_values()
        if len(temperature) > 0:
            temperature = temperature['temperature']
        else:
            temperature = np.nan
        return temperature

    def read_air_flow(self):
        """
        Returns instantaneous air flow values.
        """
        air_flow = self._get_instantaneos_values()
        if len(air_flow) > 0:
            air_flow = air_flow['air_flow']
        else:
            air_flow = np.nan
        return air_flow

    def read_heart_rate(self):
        """
        Returns instantaneous heart rate.
        """
        heart_rate = self._get_instantaneos_values()
        if len(heart_rate) > 0:
            heart_rate = heart_rate['hr']
        else:
            heart_rate = np.nan
        return heart_rate

    def read_acc_magn(self):
        """
        Returns instantaneous acceleration magnitude.
        """
        acc_magn = self._get_instantaneos_values()
        if len(acc_magn) > 0:
            acc_magn = acc_magn['acc']
        else:
            acc_magn = np.nan
        return acc_magn


Aouda.DP = namedtuple('DP', ['timestamp', 'ecg_v1', 'ecg_v2', 'o2',
                             'temperature', 'air_flow', 'hr',
                             'acc_x', 'acc_y', 'acc_z'])
