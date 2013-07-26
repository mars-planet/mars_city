from __future__ import division

import os
import gc
import unittest

import matplotlib.pyplot as plt

from src.preprocessing import extract_hr_acc, read_data
from src.assumption_free import AssumptionFreeAA


class AssumptionFreeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        """
        Reads a test data set into memory
        """
        dirname = os.path.dirname(__file__)
        filename = os.path.join(dirname, 'dataset.dat')
        cls._df = extract_hr_acc(read_data(filename))


    def setUp(self,
              window_size=1000, lead_window_factor=3,
              lag_window_factor=30, word_size=10,
              recursion_level=2, resampling_rate='10ms',
              resampling_method='mean'):
        """
        Copies the data set to an test's instance variable
        and instanciates an anomaly detector.
        """
        self._data = self._df.resample(rule=resampling_rate,
                                      how=resampling_method)
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


    def test_detect_anomalies(self):
        self._scores = []
        self._bitmaps = []
        for i in xrange(len(self._data)):
            result = self._instance.detect_anomalies([self._data.ratio[i]])
            if len(result) != 0:
                self._scores.append(result[0].score)
                self._bitmaps.append((result[0].bitmp1, result[0].bitmp2))
            else:
                self._scores.append(0.)
                self._bitmaps.append(([], []))


        # TODO: remove plots
        plt.figure()
        plt.subplot(2,1,1)
        (self._data.acc/self._data.acc.max()).plot()
        (self._data.hr/self._data.hr.max()).plot()
        self._data.ratio_log.plot()
        plt.legend(loc='best')
        plt.subplot(2,1,2)
        plt.plot(self._data.index[:len(self._scores)], self._scores)

        self.assertTrue(True)


if __name__ == '__main__':
    unittest.main()
