#! /usr/bin/env python
from __future__ import division, print_function

from datetime import datetime, timedelta
import sys
import unittest
import numpy as np

sys.path.append("../../")

from src.rasppi.aouda import Aouda


class AoudaTests(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        super(AoudaTests, cls).setUpClass()
        cls.instance = Aouda(simulate=True)
        cls.resolution = 1000
        cls.data = cls.instance.data.copy()

    def tearDown(self):
        if self.instance.data.empty:
            self.instance.data = self.data.copy()

    def test_read_acceleration(self):
        acc_x, acc_y, acc_z = self.instance.read_acceleration()
        self.assertFalse(np.isnan(acc_x))
        self.assertFalse(np.isnan(acc_y))
        self.assertFalse(np.isnan(acc_z))

    def test_read_acceleration_no_enough_data(self):
        self.instance.data.drop(self.instance.data.index, inplace=True)
        acc_x, acc_y, acc_z = self.instance.read_acceleration()
        self.assertTrue(np.isnan(acc_x))
        self.assertTrue(np.isnan(acc_y))
        self.assertTrue(np.isnan(acc_z))

    def test_read_air_flow(self):
        air_flow = self.instance.read_air_flow()
        self.assertFalse(np.isnan(air_flow))

    def test_read_air_flow_no_enough_data(self):
        self.instance.data.drop(self.instance.data.index, inplace=True)
        air_flow = self.instance.read_air_flow()
        self.assertTrue(np.isnan(air_flow))

    def test_read_heart_rate(self):
        heart_rate = self.instance.read_heart_rate()
        self.assertFalse(np.isnan(heart_rate))

    def test_read_heart_rate_no_enough_data(self):
        self.instance.data.drop(self.instance.data.index, inplace=True)
        heart_rate = self.instance.read_heart_rate()
        self.assertTrue(np.isnan(heart_rate))

    def test_read_ecg_v1(self):
        ecg_v1 = self.instance.read_ecg_v1()
        self.assertFalse(np.isnan(ecg_v1))

    def test_read_ecg_v1_no_enough_data(self):
        self.instance.data.drop(self.instance.data.index, inplace=True)
        ecg_v1 = self.instance.read_ecg_v1()
        self.assertTrue(np.isnan(ecg_v1))

    def test_read_o2(self):
        o2 = self.instance.read_o2()
        self.assertFalse(np.isnan(o2))

    def test_read_o2_no_enough_data(self):
        self.instance.data.drop(self.instance.data.index, inplace=True)
        o2 = self.instance.read_o2()
        self.assertTrue(np.isnan(o2))

    def test_read_temperature(self):
        temperatur = self.instance.read_temperature()
        self.assertFalse(np.isnan(temperatur))

    def test_read_temperature_no_enough_data(self):
        self.instance.data.drop(self.instance.data.index, inplace=True)
        temperatur = self.instance.read_temperature()
        self.assertTrue(np.isnan(temperatur))


if __name__ == '__main__':
    unittest.main()
