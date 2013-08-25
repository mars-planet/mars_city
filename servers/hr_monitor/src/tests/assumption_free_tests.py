from __future__ import division

import sys
import os
import gc
import unittest

import numpy as np

from src.preprocessing import extract_hr_acc, read_data
from src.assumption_free import AssumptionFreeAA


class AssumptionFreeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        """
        Reads a test data set into memory
        """
        dirname = os.path.dirname(__file__)
        filename = os.path.join(dirname, 'dataset2.dat')
        cls._df = extract_hr_acc(read_data(filename))


    def setUp(self,
              window_size=1000, lead_window_factor=3,
              lag_window_factor=30, word_size=10,
              recursion_level=2, resampling_rate='10ms',
              resampling_method='mean'):
        """
        Copies the data set to an test's instance variable
        and instantiates an anomaly detector.
        """
        self._data = self._df.resample(rule=resampling_rate,
                                      how=resampling_method)
        # fill NAs forward
        self._data = self._data.fillna(method='ffill')
        # fill NAs backwards (fill any NAs at the start of all series)
        self._data = self._data.fillna(method='bfill')
        self._instance = AssumptionFreeAA(window_size=window_size,
                                         lead_window_factor=lead_window_factor,
                                         lag_window_factor=lag_window_factor,
                                         word_size=word_size,
                                         recursion_level=recursion_level)


    def tearDown(self):
        """
        Deletes instance's data.
        """
        del self._data
        del self._instance
        gc.collect()


    def runTest(self):
        self._scores = []
        self._bitmaps = []
        for i in xrange(len(self._data)):
            if np.isnan(self._data.ratio[i]):
                print self._data.ix[i]
                sys.exit()
            result = self._instance.detect([self._data.ratio[i]])
            if len(result) != 0:
                self._scores.append(result[0].score)
                self._bitmaps.append((result[0].bitmp1, result[0].bitmp2))
            else:
                self._scores.append(0.)
                self._bitmaps.append(([], []))
        self.assertTrue(True)


#if __name__ == '__main__':
#    unittest.main()