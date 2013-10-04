#! /usr/bin/env python
from __future__ import division, print_function

import sys
import unittest
import os
from datetime import datetime, timedelta

sys.path.append("../../")

from numpy import isnan

from src.preprocessing import read_data, extract_hr_acc
from src.aouda import Aouda, NoDataAvailableError


class AoudaTests(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        super(AoudaTests, cls).setUpClass()
        dirname = os.path.dirname(__file__)
        cls.filename = os.path.join(dirname, 'small_dataset.dat')
        cls.resolution = 1000
        cls.base_datetime = datetime.now() - timedelta(seconds=15)
        cls.data = extract_hr_acc(read_data(cls.filename, cls.base_datetime))

    def setUp(self):
        unittest.TestCase.setUp(self)
        self.data = AoudaTests.data.copy()

    def tearDown(self):
        unittest.TestCase.tearDown(self)
        del self.data

    def test_init_filename(self):
        edata = self.data
        inst = Aouda(filename=AoudaTests.filename,
                     base_datetime=AoudaTests.base_datetime)
        self.assertEqual(len(inst.data), len(edata),
                        'exp: %s; act: %s' % (len(inst.data), len(edata)))
        for i in range(len(inst.data)):
            self.assertItemsEqual(
                        inst.data.ix[i].tolist(), edata.ix[i].tolist(),
                        'exp: %s; act: %s' % (edata.ix[i].tolist(),
                                              inst.data.ix[i].tolist()))

    def test_init_dataframe(self):
        edata = self.data
        inst = Aouda(dataframe=self.data)

        self.assertTrue(inst.data.eq(edata).all().all(),
                        'exp: %s; act: %s' % (edata, inst.data))

    def test_get_data(self):
        inst = Aouda(dataframe=self.data)
        period = 2

        data, until = inst.get_data(period)

        since = until - timedelta(seconds=period)
        edata = self.data[self.data.index >= since][self.data.index <= until]

        self.assertEqual(len(edata), len(data),
                        'exp: %s; act: %s' % (len(edata), len(data)))
        for i in range(len(data)):
            self.assertEqual(data[i].timestamp, edata.index[i],
                            'exp: %s; act: %s' % (data[i].timestamp,
                                                  edata.index[i]))
            self.assertEqual(data[i].hr, edata.ix[i]['hr'],
                            'exp: %s; act: %s' % (data[i].hr,
                                                  edata.ix[i]['hr']))
            self.assertEqual(data[i].acc_x, edata.ix[i]['acc_x'],
                            'exp: %s; act: %s' % (data[i].acc_x,
                                                  edata.ix[i]['acc_x']))
            self.assertEqual(data[i].acc_y, edata.ix[i]['acc_y'],
                            'exp: %s; act: %s' % (data[i].acc_y,
                                                  edata.ix[i]['acc_y']))
            self.assertEqual(data[i].acc_z, edata.ix[i]['acc_z'],
                            'exp: %s; act: %s' % (data[i].acc_z,
                                                  edata.ix[i]['acc_z']))

    def test_get_data_no_more_data(self):
        inst = Aouda(dataframe=self.data.shift(-10, 'T'))
        self.assertRaises(NoDataAvailableError, inst.get_data, 2)

    def test_get_data_shift_data(self):
        edata = self.data.shift(-10, 'T')
        inst = Aouda(dataframe=edata, shift_data=True)
        period = 2

        data, until = inst.get_data(period)
        delta = (until - edata.index[0]).total_seconds()
        edata = edata.shift(periods=int(delta), freq='S')
        since = until - timedelta(seconds=period)
        edata = edata[edata.index >= since][edata.index <= until]

        self.assertEqual(len(edata), len(data),
                        'exp: %s; act: %s' % (len(edata), len(data)))
        for i in range(len(data)):
            self.assertEqual(data[i].timestamp, edata.index[i],
                            'exp: %s;\nact: %s' % (data[i].timestamp,
                                                  edata.index[i]))
            self.assertEqual(data[i].hr, edata.ix[i]['hr'],
                            'exp: %s; act: %s' % (data[i].hr,
                                                  edata.ix[i]['hr']))
            self.assertEqual(data[i].acc_x, edata.ix[i]['acc_x'],
                            'exp: %s; act: %s' % (data[i].acc_x,
                                                  edata.ix[i]['acc_x']))
            self.assertEqual(data[i].acc_y, edata.ix[i]['acc_y'],
                            'exp: %s; act: %s' % (data[i].acc_y,
                                                  edata.ix[i]['acc_y']))
            self.assertEqual(data[i].acc_z, edata.ix[i]['acc_z'],
                            'exp: %s; act: %s' % (data[i].acc_z,
                                                  edata.ix[i]['acc_z']))

    def test_read_heart_rate(self):
        inst = Aouda(dataframe=self.data)

        until = datetime.now()
        heart_rate = inst.read_heart_rate()

        eheart_rate = self.data[self.data.index <= until]['hr'][-1]

        self.assertEqual(heart_rate, eheart_rate,
                        'exp: %s; act: %s' % (eheart_rate, heart_rate))

    def test_read_heart_rate_no_enough_data(self):
        inst = Aouda(dataframe=self.data.shift(10, 'T'))
        heart_rate = inst.read_heart_rate()
        self.assertTrue(isnan(heart_rate))

    def test_read_acc_magn(self):
        inst = Aouda(dataframe=self.data)

        until = datetime.now()
        acc_magn = inst.read_acc_magn()

        eacc_magn = self.data[self.data.index <= until]['acc'][-1]

        self.assertEqual(acc_magn, eacc_magn,
                        'exp: %s; act: %s' % (eacc_magn, acc_magn))

    def test_read_acc_magn_no_enough_data(self):
        inst = Aouda(dataframe=self.data.shift(10, 'T'))
        acc_magn = inst.read_acc_magn()
        self.assertTrue(isnan(acc_magn))


if __name__ == '__main__':
    unittest.main()
