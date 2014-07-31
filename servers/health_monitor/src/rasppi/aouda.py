"""
Implements the Aouda Suit Server Interface.
"""
from __future__ import division, print_function

from collections import namedtuple
from datetime import datetime
import gc

import Oger as og
import ehealth as eh
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

    def __init__(self, simulate=False, dataframe=None, base_datetime=None,
                 shift_data=False, log_function=None):
        if log_function is None:
            log_function = print
        self.log_function = log_function
        self.log_function('Constructing Aouda')
        self.simulate = simulate
        if simulate:
            self.base_datetime = base_datetime
            self.log_function('Loading Data')
            if dataframe is not None:
                self.data = dataframe.copy()
            self.shift_data = shift_data
            self.data = self._load_data()
        else:
            self.ehealth = eh.eHealthClass()
            self.ehealth.initPositionSensor()
            self.ehealth.initPulsioximeter()
        self.log_function('Finished constructing Aouda')

    def _load_data(self):
        """
        Loads hr and acc data into a pandas.DataFrame from a dataset file.
        """
        periods = 1000
        data = og.datasets.mackey_glass(sample_len=periods,
                                        n_samples=9,
                                        seed=50)
        data = np.asanyarray(data).reshape((9, periods))
        data = data + np.abs(data.min())
        data = data.transpose()
        return pd.DataFrame(data=data,
                            index=pd.date_range(datetime.now(),
                                                periods=periods,
                                                freq='500L'),
                            columns=['ecg_v1', 'ecg_v2', 'o2',
                                     'temperature', 'air_flow', 'hr',
                                     'acc_x', 'acc_y', 'acc_z'])

    def _shift_data(self, from_datetime):
        if self.shift_data and self.data.index[-1] <= from_datetime:
            delta = (from_datetime - self.data.index[0]).total_seconds()
            self.log_function('Shifting data % s seconds.' % delta)
            self.data = self.data.shift(periods=int(delta), freq='S')
            gc.collect()

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
        if self.simulate:
            ecg_v1 = self._get_instantaneos_values()
            if len(ecg_v1) > 0:
                ecg_v1 = ecg_v1['ecg_v1']
            else:
                ecg_v1 = np.nan
        else:
            ecg_v1 = self.ehealth.getECG()
        return ecg_v1

    def read_ecg_v2(self):
        """
        Returns instantaneous readings of the V2 ECG contact.
        """
        if self.simulate:
            ecg_v2 = self._get_instantaneos_values()
            if len(ecg_v2) > 0:
                ecg_v2 = ecg_v2['ecg_v2']
            else:
                ecg_v2 = np.nan
        else:
            ecg_v2 = np.nan  # self.ehealth.getECG()
        return ecg_v2

    def read_o2(self):
        """
        Returns instantaneous O2 in blood values.
        """
        if self.simulate:
            o2 = self._get_instantaneos_values()
            if len(o2) > 0:
                o2 = o2['o2']
            else:
                o2 = np.nan
        else:
            o2 = self.ehealth.getOxygenSaturation()
            self.ehealth.setupPulsioximeterForNextReading()
        return o2

    def read_temperature(self):
        """
        Returns instantaneous temperature.
        """
        if self.simulate:
            temperature = self._get_instantaneos_values()
            if len(temperature) > 0:
                temperature = temperature['temperature']
            else:
                temperature = np.nan
        else:
            temperature = self.ehealth.getTemperature()
        return temperature

    def read_air_flow(self):
        """
        Returns instantaneous air flow values.
        """
        if self.simulate:
            air_flow = self._get_instantaneos_values()
            if len(air_flow) > 0:
                air_flow = air_flow['air_flow']
            else:
                air_flow = np.nan
        else:
            air_flow = self.ehealth.getAirFlow()
        return air_flow

    def read_heart_rate(self):
        """
        Returns instantaneous heart rate.
        """
        if self.simulate:
            heart_rate = self._get_instantaneos_values()
            if len(heart_rate) > 0:
                heart_rate = heart_rate['hr']
            else:
                heart_rate = np.nan
        else:
            heart_rate = self.ehealth.getBPM()
            self.ehealth.setupPulsioximeterForNextReading()
        return heart_rate

    def read_acceleration(self):
        """
        Returns instantaneous acceleration magnitude.
        """
        if self.simulate:
            acc = self._get_instantaneos_values()
            if len(acc) > 0:
                acc_x = acc['acc_x']
                acc_y = acc['acc_y']
                acc_z = acc['acc_z']
            else:
                acc_x = np.nan
                acc_y = np.nan
                acc_z = np.nan
        else:
            acc = self.ehealth.getBodyAcceleration()
            acc = eh.floatArray_frompointer(acc)
            acc_x = acc[0]
            acc_y = acc[1]
            acc_z = acc[2]
        return (acc_x, acc_y, acc_z)


Aouda.DP = namedtuple('DP', ['timestamp', 'ecg_v1', 'ecg_v2', 'o2',
                             'temperature', 'air_flow', 'hr',
                             'acc_x', 'acc_y', 'acc_z'])
