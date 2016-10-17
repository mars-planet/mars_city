"""
Implements the Aouda Suit Server Interface.
"""
from __future__ import division, print_function

from collections import deque
from collections import namedtuple
from datetime import datetime, timedelta
import gc

import numpy as np
import pandas as pd


def _minmax_scale(data, min_x=-1, max_x=1):
    minimum = np.min(data)
    maximum = np.max(data)
    data_std = (data - minimum) / (maximum - minimum)
    return data_std * (max_x - min_x) + min_x


def _load_data():
    """
    Loads hr and acc data into a pandas.DataFrame from a dataset file.
    """
    import Oger as og
    import csv
    now = datetime.now()
    now -= timedelta(microseconds=now.microsecond)
    periods = 500
    o2 = og.datasets.mackey_glass(sample_len=periods,
                                  n_samples=1,
                                  seed=1)[0][0].flatten()
    o2 = _minmax_scale(o2, 96, 99)
    timestamp = pd.date_range(now, periods=periods, freq='500L')
    temperature = og.datasets.mackey_glass(sample_len=periods,
                                           n_samples=1,
                                           seed=3)[0][0].flatten()
    temperature = _minmax_scale(temperature, 36, 37.5)
    air_flow = og.datasets.mackey_glass(sample_len=periods,
                                        n_samples=1,
                                        seed=5)[0][0].flatten()
    air_flow = _minmax_scale(air_flow, 0, 100)
    heart_rate = og.datasets.mackey_glass(sample_len=periods,
                                          n_samples=1,
                                          seed=7)[0][0].flatten()
    heart_rate = _minmax_scale(heart_rate, 60, 70)
    data = zip(*[o2, temperature, air_flow, heart_rate])
    data = pd.DataFrame(data, columns=['o2', 'temperature',
                                       'air_flow', 'heart_rate'],
                        index=timestamp)
    data = data.resample('250L', fill_method='pad')
    last_row = data.xs(data.index[-1])
    last_row.name = data.index[-1] + timedelta(seconds=0.25)
    data = data.append(last_row)
    acc_magn = og.datasets.mackey_glass(sample_len=2 * periods,
                                        n_samples=1,
                                        seed=11)[0][0].flatten()
    acc_magn = _minmax_scale(acc_magn, 0, 5)
    data['acc_magn'] = acc_magn
    data = data.resample('10L', fill_method='pad')
    with open('ecg_v1.csv', 'rb') as f:
        reader = csv.reader(f)
        reader.next()
        ecg_v1 = [float(r[0]) for r in reader]
    last_rows = data.iloc[-24:]
    last_rows['index'] = last_rows.index + timedelta(seconds=0.01)
    last_rows.set_index('index', inplace=True)
    data = data.append(last_rows)
    data['ecg_v1'] = ecg_v1
    return data


class Aouda(object):
    """
    Implements the Aouda Server Interface.
    """

    def __init__(self, simulate=False, shift_data=False, log_function=None,
                 air_flow_threshold=500):
        if log_function is None:
            log_function = print
        self.log_function = log_function
        self.log_function('Constructing Aouda')
        self.air_flow_threshold = air_flow_threshold
        self.simulate = simulate
        if simulate:
            self.log_function('Loading Data')
            self.shift_data = shift_data
            self.data = _load_data()
        else:
            import ehealth as eh
            self.ehealth = eh.eHealthClass()
            self.ehealth.initPositionSensor()
            self.ehealth.initPulsioximeter(1)
        self.log_function('Finished constructing Aouda')
        self.ecg_v1_buffer = deque(maxlen=10000)

    def _shift_data(self, from_datetime):
        if self.shift_data and self.data.index[-1] <= from_datetime:
            delta = (from_datetime - self.data.index[0]).total_seconds()
            self.log_function('Shifting data % s seconds.' % delta)
            self.data = self.data.shift(periods=int(delta), freq='S')
            gc.collect()

    def _get_instantaneous_values(self):
        """
        Returns a row from the data such that row.index <= datetime.now().
        """
        self._shift_data(datetime.now())
        current_row = self.data[self.data.index <= datetime.now()]
        if len(current_row) > 0:
            current_row = current_row.ix[-1]
        return current_row

    def read_acceleration(self):
        """
        Returns instantaneous acceleration magnitude.
        """
        if self.simulate:
            acc = self._get_instantaneous_values()
            if len(acc) > 0:
                acc_x = acc_y = acc_z = acc['acc_magn']
            else:
                acc_x = np.nan
                acc_y = np.nan
                acc_z = np.nan
        else:
            try:
                import ehealth as eh
                acc = self.ehealth.getBodyAcceleration()
                acc = eh.floatArray_frompointer(acc)
            except:
                acc = (np.nan, np.nan, np.nan)
            acc_x = acc[0]
            acc_y = acc[1]
            acc_z = acc[2]
        return (acc_x, acc_y, acc_z)

    def read_air_flow(self):
        """
        Returns instantaneous air flow values.
        """
        if self.simulate:
            air_flow = self._get_instantaneous_values()
            if len(air_flow) > 0:
                air_flow = air_flow['air_flow']
            else:
                air_flow = np.nan
        else:
            try:
                air_flow = self.ehealth.getAirFlow()
            except:
                air_flow = np.nan
            if air_flow > self.air_flow_threshold:
                air_flow = 0
        return air_flow

    def read_heart_rate(self):
        """
        Returns instantaneous heart rate.
        """
        if self.simulate:
            heart_rate = self._get_instantaneous_values()
            if len(heart_rate) > 0:
                heart_rate = heart_rate['heart_rate']
            else:
                heart_rate = np.nan
        else:
            try:
                heart_rate = self.ehealth.getBPM()
                self.ehealth.setupPulsioximeterForNextReading()
            except:
                heart_rate = np.nan
        return heart_rate

    def read_ecg_v1(self):
        """
        Returns instantaneous readings of the V1 ECG contact.
        """
        if self.simulate:
            ecg_v1 = self._get_instantaneous_values()
            if len(ecg_v1) > 0:
                ecg_v1 = ecg_v1['ecg_v1']
            else:
                ecg_v1 = np.nan
        else:
            try:
                ecg_v1 = self.ehealth.getECG()
            except:
                ecg_v1 = np.nan
        return ecg_v1

    def read_o2(self):
        """
        Returns instantaneous O2 in blood values.
        """
        if self.simulate:
            o2 = self._get_instantaneous_values()
            if len(o2) > 0:
                o2 = o2['o2']
            else:
                o2 = np.nan
        else:
            try:
                o2 = self.ehealth.getOxygenSaturation()
                self.ehealth.setupPulsioximeterForNextReading()
            except:
                o2 = np.nan
        return o2

    def read_temperature(self):
        """
        Returns instantaneous temperature.
        """
        if self.simulate:
            temperature = self._get_instantaneous_values()
            if len(temperature) > 0:
                temperature = temperature['temperature']
            else:
                temperature = np.nan
        else:
            try:
                temperature = self.ehealth.getTemperature()
            except:
                temperature = np.nan
        return temperature


Aouda.DP = namedtuple('DP', ['timestamp', 'ecg_v1', 'ecg_v2', 'o2',
                             'temperature', 'air_flow', 'heart_rate',
                             'acc_x', 'acc_y', 'acc_z'])
