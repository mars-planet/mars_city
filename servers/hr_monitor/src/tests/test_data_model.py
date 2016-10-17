#! /usr/bin/env python
from __future__ import division

import sys
import unittest
from datetime import datetime, timedelta
from random import uniform, randint, choice

sys.path.append("../")
sys.path.append("../../")

from src.data_model import Datapoint, Alarm


class DatapointTests(unittest.TestCase):

    def test_no_millisec(self):
        etimestamp = datetime.now()
        emillisecs = etimestamp.microsecond / 1000
        ehr = randint(30, 240)
        eacc_x = uniform(0, 16)
        eacc_y = uniform(0, 16)
        eacc_z = uniform(0, 16)
        eacc_magn = (eacc_x ** 2 + eacc_y ** 2 + eacc_z ** 2) ** 0.5

        dp = Datapoint(etimestamp, ehr, eacc_x, eacc_y, eacc_z)

        self.assertEqual(etimestamp, dp.timestamp)
        self.assertEqual(emillisecs, dp.millisecond)
        self.assertAlmostEqual(ehr, dp.hr)
        self.assertAlmostEqual(eacc_x, dp.acc_x)
        self.assertAlmostEqual(eacc_y, dp.acc_y)
        self.assertAlmostEqual(eacc_z, dp.acc_z)
        self.assertAlmostEqual(eacc_magn, dp.acc_magn)

    def test_millisec(self):
        etimestamp = datetime.now()
        emillisecs = etimestamp.microsecond / 1000
        ehr = randint(30, 240)
        eacc_x = uniform(0, 16)
        eacc_y = uniform(0, 16)
        eacc_z = uniform(0, 16)
        eacc_magn = (eacc_x ** 2 + eacc_y ** 2 + eacc_z ** 2) ** 0.5

        dp = Datapoint(etimestamp, ehr, eacc_x, eacc_y, eacc_z, emillisecs)

        self.assertEqual(etimestamp, dp.timestamp)
        self.assertEqual(emillisecs, dp.millisecond)
        self.assertAlmostEqual(ehr, dp.hr)
        self.assertAlmostEqual(eacc_x, dp.acc_x)
        self.assertAlmostEqual(eacc_y, dp.acc_y)
        self.assertAlmostEqual(eacc_z, dp.acc_z)
        self.assertAlmostEqual(eacc_magn, dp.acc_magn)

    def test_repr(self):
        etimestamp = datetime.now()
        emillisecs = etimestamp.microsecond / 1000
        ehr = randint(30, 240)
        eacc_x = uniform(0, 16)
        eacc_y = uniform(0, 16)
        eacc_z = uniform(0, 16)
        eacc_magn = (eacc_x ** 2 + eacc_y ** 2 + eacc_z ** 2) ** 0.5

        dp = Datapoint(etimestamp, ehr, eacc_x, eacc_y, eacc_z, emillisecs)

        self.assertEqual(dp.__repr__(),
                         "<Datapoint('%s','%s','%s','%s','%s','%s')>"
                         % (etimestamp, ehr, eacc_x, eacc_y, eacc_z,
                            eacc_magn))


class AlarmTests(unittest.TestCase):

    def test_timestamp_millisec(self):
        etimestamp = datetime.now()
        emillisecs = etimestamp.microsecond / 1000
        ealarm_lvl = randint(30, 240)
        esgmt_begin = uniform(0, 16)
        esgmt_end = uniform(0, 16)
        chars = ''.join(chr(x) for x in range(65, 122))
        size = 20
        ebitmp1 = ''.join(choice(chars) for x in range(size))
        ebitmp2 = ''.join(choice(chars) for x in range(size))

        alarm = Alarm(ealarm_lvl, esgmt_begin, esgmt_end,
                      ebitmp1, ebitmp2, etimestamp, emillisecs)

        self.assertEqual(etimestamp, alarm.timestamp)
        self.assertEqual(emillisecs, alarm.millisecond)
        self.assertAlmostEqual(ealarm_lvl, alarm.alarm_lvl)
        self.assertAlmostEqual(esgmt_begin, alarm.sgmt_begin)
        self.assertAlmostEqual(esgmt_end, alarm.sgmt_end)
        self.assertAlmostEqual(ebitmp1, alarm.bitmp1)
        self.assertAlmostEqual(ebitmp2, alarm.bitmp2)

    def test_timestamp_no_millisec(self):
        etimestamp = datetime.now()
        emillisecs = etimestamp.microsecond / 1000
        ealarm_lvl = randint(30, 240)
        esgmt_begin = uniform(0, 16)
        esgmt_end = uniform(0, 16)
        chars = ''.join(chr(x) for x in range(65, 122))
        size = 20
        ebitmp1 = ''.join(choice(chars) for x in range(size))
        ebitmp2 = ''.join(choice(chars) for x in range(size))

        alarm = Alarm(ealarm_lvl, esgmt_begin, esgmt_end,
                      ebitmp1, ebitmp2, etimestamp)

        self.assertEqual(etimestamp, alarm.timestamp)
        self.assertEqual(emillisecs, alarm.millisecond)
        self.assertAlmostEqual(ealarm_lvl, alarm.alarm_lvl)
        self.assertAlmostEqual(esgmt_begin, alarm.sgmt_begin)
        self.assertAlmostEqual(esgmt_end, alarm.sgmt_end)
        self.assertAlmostEqual(ebitmp1, alarm.bitmp1)
        self.assertAlmostEqual(ebitmp2, alarm.bitmp2)

    def test_no_timestamp_millisec(self):
        etimestamp = datetime.now()
        emillisecs = etimestamp.microsecond / 1000
        ealarm_lvl = randint(30, 240)
        esgmt_begin = uniform(0, 16)
        esgmt_end = uniform(0, 16)
        chars = ''.join(chr(x) for x in range(65, 122))
        size = 20
        ebitmp1 = ''.join(choice(chars) for x in range(size))
        ebitmp2 = ''.join(choice(chars) for x in range(size))

        alarm = Alarm(ealarm_lvl, esgmt_begin, esgmt_end,
                      ebitmp1, ebitmp2, millisecond=emillisecs)

        self.assertAlmostEqual(
                float((etimestamp
                       - timedelta(microseconds=etimestamp.microsecond))
                      .strftime('%s.%f')),
                float(alarm.timestamp.strftime('%s.%f')),
                delta=1)
        self.assertEqual(emillisecs, alarm.millisecond)
        self.assertAlmostEqual(ealarm_lvl, alarm.alarm_lvl)
        self.assertAlmostEqual(esgmt_begin, alarm.sgmt_begin)
        self.assertAlmostEqual(esgmt_end, alarm.sgmt_end)
        self.assertAlmostEqual(ebitmp1, alarm.bitmp1)
        self.assertAlmostEqual(ebitmp2, alarm.bitmp2)

    def test_no_timestamp_no_millisec(self):
        etimestamp = datetime.now()
        emillisecs = etimestamp.microsecond / 1000
        ealarm_lvl = randint(30, 240)
        esgmt_begin = uniform(0, 16)
        esgmt_end = uniform(0, 16)
        chars = ''.join(chr(x) for x in range(65, 122))
        size = 20
        ebitmp1 = ''.join(choice(chars) for x in range(size))
        ebitmp2 = ''.join(choice(chars) for x in range(size))

        alarm = Alarm(ealarm_lvl, esgmt_begin, esgmt_end, ebitmp1, ebitmp2)

        self.assertAlmostEqual(
                float((etimestamp
                       - timedelta(microseconds=etimestamp.microsecond))
                      .strftime('%s.%f')),
                float(alarm.timestamp.strftime('%s.%f')),
                delta=1)
        self.assertAlmostEqual(emillisecs, alarm.millisecond, delta=1)
        self.assertAlmostEqual(ealarm_lvl, alarm.alarm_lvl)
        self.assertAlmostEqual(esgmt_begin, alarm.sgmt_begin)
        self.assertAlmostEqual(esgmt_end, alarm.sgmt_end)
        self.assertAlmostEqual(ebitmp1, alarm.bitmp1)
        self.assertAlmostEqual(ebitmp2, alarm.bitmp2)

    def test_repr(self):
        etimestamp = datetime.now()
        emillisecs = etimestamp.microsecond / 1000
        ealarm_lvl = randint(30, 240)
        esgmt_begin = uniform(0, 16)
        esgmt_end = uniform(0, 16)
        chars = ''.join(chr(x) for x in range(65, 122))
        size = 20
        ebitmp1 = ''.join(choice(chars) for x in range(size))
        ebitmp2 = ''.join(choice(chars) for x in range(size))

        alarm = Alarm(ealarm_lvl, esgmt_begin, esgmt_end,
                      ebitmp1, ebitmp2, etimestamp, emillisecs)

        self.assertEqual(alarm.__repr__(),
                         "<Alarm('%s','%s', '%s', '%s')>"
                         % (etimestamp, ealarm_lvl,
                            esgmt_begin, esgmt_end))


if __name__ == '__main__':
    unittest.main()
