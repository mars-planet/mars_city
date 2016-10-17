#! /usr/bin/env python
from __future__ import division, print_function

import sys
import unittest
from datetime import datetime, timedelta
import base64
from collections import namedtuple
import string
import random

import numpy as np
from sqlalchemy import create_engine
from sqlalchemy.orm import sessionmaker
import os

sys.path.append("../../")

from src.hr_monitor import HRMonitor, DuplicatedDatapointError
from src.data_model import Datapoint, Alarm, Base


class HRMonitorTests(unittest.TestCase):

    def setUp(self):
        self.engine = create_engine('sqlite:///:memory:')
        Base.metadata.drop_all(self.engine)
        Base.metadata.create_all(self.engine)
        self.Session = sessionmaker(bind=self.engine, autocommit=False)

    def tearDown(self):
        try:
            self.Session.close_all()
            del self.Session
        except:
            pass
        try:
            self.engine.dispose()
            del self.engine
        except:
            pass

    def _insert_datapoints(self):
        now = datetime.now()
        timestamps = [now - timedelta(seconds=i) for i in range(17, 1, -1)]
        hrs = [x * 10 for x in range(5, 21)]
        acc_xs = [x / 16 for x in range(16)]
        acc_ys = [x / 16 for x in range(16, 0, -1)]
        acc_zs = [x / 10 for x in range(16)]
        session = self.Session()
        for i in range(16):
            datum = Datapoint(timestamp=timestamps[i], hr=hrs[i],
                              acc_x=acc_xs[i], acc_y=acc_ys[i],
                              acc_z=acc_zs[i])
            session.add(datum)
        session.commit()
        session.close()
        Data = namedtuple('Data',
                          ['timestamps', 'hrs', 'acc_xs', 'acc_ys', 'acc_zs'])
        return Data(timestamps, hrs, acc_xs, acc_ys, acc_zs)

    def _insert_alarms(self):
        now = datetime.now()
        timestamps = [now - timedelta(seconds=i) for i in range(17, 1, -1)]
        scores = [x / 16 for x in range(16)]
        bitmp1s = [''.join(random.choice(string.ascii_uppercase
                                         + string.digits)
                           for x in range(20))
                   for _ in range(16)]

        bitmp2s = [''.join(random.choice(string.ascii_uppercase
                                         + string.digits)
                           for x in range(20))
                   for _ in range(16)]

        session = self.Session()
        for i in range(16):
            alarm = Alarm(timestamp=timestamps[i], alarm_lvl=scores[i],
                          bitmp1=bitmp1s[i], bitmp2=bitmp2s[i],
                          sgmt_begin=timestamps[i],
                          sgmt_end=timestamps[i])
            session.add(alarm)
        session.commit()
        session.close()
        Alarms = namedtuple('Alarms',
                            ['timestamps', 'scores', 'bitmp1s', 'bitmp2s'])
        return Alarms(timestamps, scores, bitmp1s, bitmp2s)

    def test_init_conn_str(self):
        eword_size = 10
        ewindow_factor = 100
        elead_window_factor = 3
        elag_window_factor = 30
        ewindow_size = ewindow_factor * eword_size
        elead_window_size = elead_window_factor * ewindow_size
        elag_window_size = elag_window_factor * ewindow_size
        euniverse_size = elead_window_size + elag_window_size
        eresolution = 1000
        eengine = None
        econn_str = 'sqlite:///hr_monitor_tests.db'

        self.tearDown()

        inst = HRMonitor(word_size=eword_size,
                         window_factor=ewindow_factor,
                         lead_window_factor=elead_window_factor,
                         lag_window_factor=elag_window_factor,
                         resolution=eresolution,
                         engine=eengine,
                         conn_str=econn_str)

        self.assertEqual(eresolution, inst.resolution)
        self.assertAlmostEqual(float(datetime.now().strftime('%s.%f')),
                               float(inst.last_alarm_timestamp
                                         .strftime('%s.%f')),
                               2)
        self.assertAlmostEqual(float(datetime.now().strftime('%s.%f')),
                               float(inst.last_detection_timestamp
                                         .strftime('%s.%f')),
                               2)

        eword_size = 10
        ewindow_factor = 100
        elead_window_factor = 3
        elag_window_factor = 30
        self.assertEqual(eword_size, inst.detector._word_size)
        self.assertEqual(ewindow_size, inst.detector._window_size)
        self.assertEqual(euniverse_size, inst.detector.universe_size)
        self.assertEqual(elead_window_size, inst.detector._lead_window.maxlen)
        self.assertEqual(elag_window_size, inst.detector._lag_window.maxlen)
        self.assertEqual(elag_window_size, inst.detector._lag_window.maxlen)

        engine = create_engine(econn_str)
        conn = engine.connect()
        tables_created = (engine.dialect.has_table(conn, "alarms")
                          and engine.dialect.has_table(conn, "datapoints"))
        self.assertTrue(tables_created)

        engine.dispose()
        del engine
        os.remove('hr_monitor_tests.db')

    def test_init_engine(self):
        eword_size = 10
        ewindow_factor = 100
        elead_window_factor = 3
        elag_window_factor = 30
        ewindow_size = ewindow_factor * eword_size
        elead_window_size = elead_window_factor * ewindow_size
        elag_window_size = elag_window_factor * ewindow_size
        euniverse_size = elead_window_size + elag_window_size
        eresolution = 1000
        econn_str = 'sqlite:///:memory:'
        eengine = create_engine(econn_str)

        inst = HRMonitor(word_size=eword_size,
                         window_factor=ewindow_factor,
                         lead_window_factor=elead_window_factor,
                         lag_window_factor=elag_window_factor,
                         resolution=eresolution,
                         engine=eengine,
                         conn_str=econn_str)

        self.assertEqual(eengine, inst.engine)
        self.assertEqual(eresolution, inst.resolution)
        self.assertAlmostEqual(float(datetime.now().strftime('%s.%f')),
                               float(inst.last_alarm_timestamp
                                         .strftime('%s.%f')),
                               2)
        self.assertAlmostEqual(float(datetime.now().strftime('%s.%f')),
                               float(inst.last_detection_timestamp
                                         .strftime('%s.%f')),
                               2)

        eword_size = 10
        ewindow_factor = 100
        elead_window_factor = 3
        elag_window_factor = 30
        self.assertEqual(eword_size, inst.detector._word_size)
        self.assertEqual(ewindow_size, inst.detector._window_size)
        self.assertEqual(euniverse_size, inst.detector.universe_size)
        self.assertEqual(elead_window_size, inst.detector._lead_window.maxlen)
        self.assertEqual(elag_window_size, inst.detector._lag_window.maxlen)
        self.assertEqual(elag_window_size, inst.detector._lag_window.maxlen)

        eengine.dispose()
        del eengine

    def test_register_datapoint(self):
        etimestamp = datetime.now()
        ehr = 140
        eacc_x = 0.5
        eacc_y = -0.5
        eacc_z = 1
        inst = HRMonitor(engine=self.engine)
        inst.register_datapoint(etimestamp, ehr, eacc_x, eacc_y, eacc_z)

        session = self.Session()
        query = session.query(Datapoint)
        data = query.all()
        session.close()
        self.assertEqual(1, len(data))
        self.assertEqual(etimestamp, data[0].timestamp)
        self.assertEqual(ehr, data[0].hr)
        self.assertEqual(eacc_x, data[0].acc_x)
        self.assertEqual(eacc_y, data[0].acc_y)
        self.assertEqual(eacc_z, data[0].acc_z)

        del inst

    def test_register_datapoint_duplicate_datapoint(self):
        etimestamp = datetime.now()
        ehr = 140
        eacc_x = 0.5
        eacc_y = -0.5
        eacc_z = 1
        inst = HRMonitor(engine=self.engine)
        inst.register_datapoint(etimestamp, ehr, eacc_x, eacc_y, eacc_z)
        self.assertRaises(DuplicatedDatapointError,
                          inst.register_datapoint,
                          etimestamp, ehr,
                          eacc_x, eacc_y, eacc_z)

        session = self.Session()
        query = session.query(Datapoint)
        data = query.all()
        session.close()
        self.assertEqual(1, len(data))
        self.assertEqual(etimestamp, data[0].timestamp)
        self.assertEqual(ehr, data[0].hr)
        self.assertEqual(eacc_x, data[0].acc_x)
        self.assertEqual(eacc_y, data[0].acc_y)
        self.assertEqual(eacc_z, data[0].acc_z)

        del inst

    def test_generate_alarms(self):
        inst = HRMonitor(word_size=2,
                         window_factor=2,
                         lead_window_factor=2,
                         lag_window_factor=2,
                         engine=self.engine)

        self._insert_datapoints()
        inst.last_alarm_timestamp -= timedelta(seconds=17)
        inst.last_detection_timestamp -= timedelta(seconds=17)
        inst._generate_alarms()

        session = self.Session()
        query = session.query(Alarm)
        data = query.all()
        session.close()

        escore = 0.125
        ebitmp1 = np.array([[0.,  0.,  0.,  0.],
                            [0.,  0.,  0.,  0.],
                            [0.,  0.,  0.,  0.],
                            [1.,  0.,  0.,  0.]])
        ebitmp1 = base64.b64encode(ebitmp1.tostring())
        ebitmp2 = np.array([[0.,  0.,  0.,  1.],
                            [0.,  0.,  0.,  0.],
                            [0.,  0.,  0.,  0.],
                            [0.,  0.,  0.,  0.]])
        ebitmp2 = base64.b64encode(ebitmp2.tostring())

        self.assertEqual(1, len(data))
        self.assertEqual(escore, data[0].alarm_lvl,
                         'exp: %s; act: %s' % (escore, data[0].alarm_lvl))
        self.assertEqual(ebitmp1, data[0].bitmp1,
                         'exp: %s; act: %s' % (ebitmp1, data[0].bitmp1))
        self.assertEqual(ebitmp2, data[0].bitmp2,
                         'exp: %s; act: %s' % (ebitmp2, data[0].bitmp2))

    def test_generate_alarms_duplicate_timestamp(self):
        inst = HRMonitor(word_size=2,
                         window_factor=2,
                         lead_window_factor=2,
                         lag_window_factor=2,
                         engine=self.engine)

        data = self._insert_datapoints()
        inst.last_alarm_timestamp -= timedelta(seconds=17)
        inst.last_detection_timestamp -= timedelta(seconds=17)
        session = self.Session()
        timestamp = (data.timestamps[-1]
                     - timedelta(microseconds=data.timestamps[-1].microsecond))
        session.add(Alarm(timestamp=timestamp, alarm_lvl=0,
                          bitmp1='', bitmp2='',
                          sgmt_begin=timestamp,
                          sgmt_end=timestamp))
        session.commit()
        inst._generate_alarms()

        session = self.Session()
        query = session.query(Alarm)
        data = query.all()
        session.close()

        escore = 0.125
        ebitmp1 = np.array([[0.,  0.,  0.,  0.],
                            [0.,  0.,  0.,  0.],
                            [0.,  0.,  0.,  0.],
                            [1.,  0.,  0.,  0.]])
        ebitmp1 = base64.b64encode(ebitmp1.tostring())
        ebitmp2 = np.array([[0.,  0.,  0.,  1.],
                            [0.,  0.,  0.,  0.],
                            [0.,  0.,  0.,  0.],
                            [0.,  0.,  0.,  0.]])
        ebitmp2 = base64.b64encode(ebitmp2.tostring())

        self.assertEqual(2, len(data))
        self.assertEqual(escore, data[1].alarm_lvl,
                         'exp: %s; act: %s' % (escore, data[1].alarm_lvl))
        self.assertEqual(ebitmp1, data[1].bitmp1,
                         'exp: %s; act: %s' % (ebitmp1, data[1].bitmp1))
        self.assertEqual(ebitmp2, data[1].bitmp2,
                         'exp: %s; act: %s' % (ebitmp2, data[1].bitmp2))

    def test_get_avg_hr(self):
        inst = HRMonitor(word_size=2,
                         window_factor=2,
                         lead_window_factor=2,
                         lag_window_factor=2,
                         engine=self.engine)

        data = self._insert_datapoints()
        eavg_hr = sum(data.hrs) / len(data.hrs)
        avg_hr = inst.get_avg_hr(len(data.hrs) + 10)

        self.assertAlmostEqual(eavg_hr, avg_hr, 2,
                               'exp: %s; act: %s' % (eavg_hr, avg_hr))

    def test_get_avg_hr_no_data(self):
        inst = HRMonitor(word_size=2,
                         window_factor=2,
                         lead_window_factor=2,
                         lag_window_factor=2,
                         engine=self.engine)

        avg_hr = inst.get_avg_hr(10)

        self.assertTrue(np.isnan(avg_hr))

    def test_get_avg_acc(self):
        inst = HRMonitor(word_size=2,
                         window_factor=2,
                         lead_window_factor=2,
                         lag_window_factor=2,
                         engine=self.engine)

        data = self._insert_datapoints()
        acc = [(data.acc_xs[i] ** 2
                + data.acc_ys[i] ** 2
                + data.acc_zs[i] ** 2) ** 0.5 for i in range(len(data.acc_xs))]
        eavg_acc = sum(acc) / len(acc)
        avg_acc = inst.get_avg_acc(len(acc) + 10)

        self.assertAlmostEqual(eavg_acc, avg_acc, 2,
                               'exp: %s; act: %s' % (eavg_acc, avg_acc))

    def test_get_avg_acc_no_data(self):
        inst = HRMonitor(word_size=2,
                         window_factor=2,
                         lead_window_factor=2,
                         lag_window_factor=2,
                         engine=self.engine)

        avg_acc = inst.get_avg_acc(10)

        self.assertTrue(np.isnan(avg_acc))

    def test_get_current_alarms(self):
        inst = HRMonitor(word_size=2,
                         window_factor=2,
                         lead_window_factor=2,
                         lag_window_factor=2,
                         engine=self.engine)

        ealarms = self._insert_alarms()
        alarms = inst.get_current_alarms(len(ealarms.timestamps) + 10)

        self.assertEqual(len(ealarms.scores), len(alarms),
                         'exp: %s; act: %s'
                         % (len(ealarms.scores), len(alarms)))
        for i in range(len(alarms)):
            self.assertEqual(ealarms.timestamps[i], alarms[i].timestamp,
                            'exp: %s; act: %s'
                            % (ealarms.timestamps[i], alarms[i].timestamp))

            self.assertEqual(ealarms.scores[i], alarms[i].alarm_lvl,
                            'exp: %s; act: %s'
                            % (ealarms.scores[i], alarms[i].alarm_lvl))

            self.assertEqual(ealarms.bitmp1s[i], alarms[i].bitmp1,
                            'exp: %s; act: %s'
                            % (ealarms.bitmp1s[i], alarms[i].bitmp1))

            self.assertEqual(ealarms.bitmp2s[i], alarms[i].bitmp2,
                            'exp: %s; act: %s'
                            % (ealarms.bitmp2s[i], alarms[i].bitmp2))


if __name__ == '__main__':
    unittest.main()
