#! /usr/bin/env python
from __future__ import division, print_function

import sys
import unittest
import os
from datetime import datetime, timedelta

from pandas import notnull
from numpy import log

sys.path.append("../../")

from src.preprocessing import read_data, extract_hr_acc


class PreprocessingTests(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        super(PreprocessingTests, cls).setUpClass()
        dirname = os.path.dirname(__file__)
        cls.filename = os.path.join(dirname, 'small_dataset.dat')
        cls.resolution = 1000
        cls.base_datetime = datetime.now() - timedelta(seconds=15)

    def test_read_data(self):
        frame = read_data(path=PreprocessingTests.filename,
                          base_datetime=PreprocessingTests.base_datetime)

        columns = set(frame.columns.tolist())
        self.assertIn('activityID', columns)
        self.assertIn('hr', columns)
        self.assertIn('IMU_Chest_Magnitude', columns)
        self.assertIn('IMU_Chest_x', columns)
        self.assertIn('IMU_Chest_y', columns)
        self.assertIn('IMU_Chest_z', columns)
        self.assertLess(PreprocessingTests.base_datetime, frame.index[0])

    def test_read_data_no_base_datetime(self):
        frame = read_data(path=PreprocessingTests.filename)

        columns = set(frame.columns.tolist())
        self.assertIn('activityID', columns)
        self.assertIn('hr', columns)
        self.assertIn('IMU_Chest_Magnitude', columns)
        self.assertIn('IMU_Chest_x', columns)
        self.assertIn('IMU_Chest_y', columns)
        self.assertIn('IMU_Chest_z', columns)
        self.assertLess(datetime.now(), frame.index[0])

    def test_extract_hr_acc(self):
        data = read_data(path=PreprocessingTests.filename,
                          base_datetime=PreprocessingTests.base_datetime)
        frame = extract_hr_acc(data)

        columns = set(frame.columns.tolist())
        self.assertIn('hr', columns)
        self.assertIn('acc', columns)
        self.assertIn('acc_x', columns)
        self.assertIn('acc_y', columns)
        self.assertIn('acc_z', columns)
        self.assertIn('ratio', columns)
        self.assertIn('ratio_log', columns)
        self.assertItemsEqual(data.index.tolist(), frame.index.tolist())
        self.assertEqual(len(frame), len(notnull(frame)))

        data['eratio'] = (data.hr / data.IMU_Chest_Magnitude)
        data['eratio_log'] = log(data['eratio'])

        data = data.fillna(method='ffill').fillna(method='bfill')
        self.assertItemsEqual(data.hr.tolist(), frame.hr.tolist())
        self.assertItemsEqual(data.IMU_Chest_Magnitude.tolist(),
                              frame.acc.tolist())
        self.assertItemsEqual(data.IMU_Chest_x.tolist(), frame.acc_x.tolist())
        self.assertItemsEqual(data.IMU_Chest_y.tolist(), frame.acc_y.tolist())
        self.assertItemsEqual(data.IMU_Chest_z.tolist(), frame.acc_z.tolist())

        self.assertItemsEqual(data.eratio.tolist(), frame.ratio.tolist())
        self.assertItemsEqual(data.eratio_log.tolist(),
                              frame.ratio_log.tolist())


if __name__ == '__main__':
    unittest.main()
