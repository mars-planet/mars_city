#! /usr/bin/env python
from __future__ import division

from datetime import datetime, timedelta
from random import uniform, randint
import sys
import unittest

sys.path.append("../../")
import src.data_model as dm


class ModuleFunctionsTests(unittest.TestCase):
    def test_datapoint_class(self):
        expected = type("A", (object,), {})
        actual = dm.datapoint_class(expected)
        self.assertEqual(expected, actual)

    def test_get_datapoint_class(self):
        expected = type("A", (object,), {})
        dm.datapoint_class(expected)
        actual = dm.get_datapoint_class("A")
        self.assertEqual(expected, actual)

    def test_camel_to_underscore(self):
        expected = "camel_case"
        actual = dm.camel_to_underscore("CamelCase")
        self.assertEqual(expected, actual)

        expected = "camel_camel_case"
        actual = dm.camel_to_underscore("CamelCamelCase")
        self.assertEqual(expected, actual)

        expected = "camel2_camel2_case"
        actual = dm.camel_to_underscore("Camel2Camel2Case")
        self.assertEqual(expected, actual)

        expected = "get_http_response_code"
        actual = dm.camel_to_underscore("getHTTPResponseCode")
        self.assertEqual(expected, actual)


class EcgV1DatapointTests(unittest.TestCase):

    def test_no_millisec(self):
        etimestamp = datetime.now()
        emillisecs = etimestamp.microsecond / 1000
        esource_id = 1
        eecg_v1 = uniform(0, 16)

        dp = dm.EcgV1Datapoint(etimestamp, esource_id, eecg_v1)

        self.assertEqual(etimestamp, dp.timestamp)
        self.assertEqual(emillisecs, dp.millisecond)
        self.assertAlmostEqual(esource_id, dp.source_id)
        self.assertAlmostEqual(eecg_v1, dp.ecg_v1)

    def test_millisec(self):
        etimestamp = datetime.now()
        emillisecs = etimestamp.microsecond / 1000
        esource_id = 1
        eecg_v1 = uniform(0, 16)

        dp = dm.EcgV1Datapoint(etimestamp, esource_id, eecg_v1, emillisecs)

        self.assertEqual(etimestamp, dp.timestamp)
        self.assertEqual(emillisecs, dp.millisecond)
        self.assertAlmostEqual(esource_id, dp.source_id)
        self.assertAlmostEqual(eecg_v1, dp.ecg_v1)

    def test_repr(self):
        etimestamp = datetime.now()
        emillisecs = etimestamp.microsecond / 1000
        esource_id = 1
        eecg_v1 = uniform(0, 16)

        dp = dm.EcgV1Datapoint(etimestamp, esource_id, eecg_v1, emillisecs)

        self.assertEqual(dp.__repr__(),
                         "<EcgV1Datapoint"
                         "(source_id=%s, timestamp=%s, ecg_v1=%s)>" %
                         (esource_id, etimestamp, eecg_v1))


class EcgV2DatapointtTests(unittest.TestCase):

    def test_no_millisec(self):
        etimestamp = datetime.now()
        emillisecs = etimestamp.microsecond / 1000
        esource_id = 1
        eecg_v2 = uniform(0, 16)

        dp = dm.EcgV2Datapoint(etimestamp, esource_id, eecg_v2)

        self.assertEqual(etimestamp, dp.timestamp)
        self.assertEqual(emillisecs, dp.millisecond)
        self.assertAlmostEqual(esource_id, dp.source_id)
        self.assertAlmostEqual(eecg_v2, dp.ecg_v2)

    def test_millisec(self):
        etimestamp = datetime.now()
        emillisecs = etimestamp.microsecond / 1000
        esource_id = 1
        eecg_v2 = uniform(0, 16)

        dp = dm.EcgV2Datapoint(etimestamp, esource_id, eecg_v2, emillisecs)

        self.assertEqual(etimestamp, dp.timestamp)
        self.assertEqual(emillisecs, dp.millisecond)
        self.assertAlmostEqual(esource_id, dp.source_id)
        self.assertAlmostEqual(eecg_v2, dp.ecg_v2)

    def test_repr(self):
        etimestamp = datetime.now()
        emillisecs = etimestamp.microsecond / 1000
        esource_id = 1
        eecg_v2 = uniform(0, 16)

        dp = dm.EcgV2Datapoint(etimestamp, esource_id, eecg_v2, emillisecs)

        self.assertEqual(dp.__repr__(),
                         "<EcgV2Datapoint"
                         "(source_id=%s, timestamp=%s, ecg_v2=%s)>" %
                         (esource_id, etimestamp, eecg_v2))


class O2DatapointTests(unittest.TestCase):

    def test_no_millisec(self):
        etimestamp = datetime.now()
        emillisecs = etimestamp.microsecond / 1000
        esource_id = 1
        eo2 = uniform(0, 16)

        dp = dm.O2Datapoint(etimestamp, esource_id, eo2)

        self.assertEqual(etimestamp, dp.timestamp)
        self.assertEqual(emillisecs, dp.millisecond)
        self.assertAlmostEqual(esource_id, dp.source_id)
        self.assertAlmostEqual(eo2, dp.o2)

    def test_millisec(self):
        etimestamp = datetime.now()
        emillisecs = etimestamp.microsecond / 1000
        esource_id = 1
        eo2 = uniform(0, 16)

        dp = dm.O2Datapoint(etimestamp, esource_id, eo2, emillisecs)

        self.assertEqual(etimestamp, dp.timestamp)
        self.assertEqual(emillisecs, dp.millisecond)
        self.assertAlmostEqual(esource_id, dp.source_id)
        self.assertAlmostEqual(eo2, dp.o2)

    def test_repr(self):
        etimestamp = datetime.now()
        emillisecs = etimestamp.microsecond / 1000
        esource_id = 1
        eo2 = uniform(0, 16)

        dp = dm.O2Datapoint(etimestamp, esource_id, eo2, emillisecs)

        self.assertEqual(dp.__repr__(),
                         "<O2Datapoint"
                         "(source_id=%s, timestamp=%s, o2=%s)>" %
                         (esource_id, etimestamp, eo2))


class TemperatureDatapointTests(unittest.TestCase):

    def test_no_millisec(self):
        etimestamp = datetime.now()
        emillisecs = etimestamp.microsecond / 1000
        esource_id = 1
        etemperature = uniform(0, 16)

        dp = dm.TemperatureDatapoint(etimestamp, esource_id, etemperature)

        self.assertEqual(etimestamp, dp.timestamp)
        self.assertEqual(emillisecs, dp.millisecond)
        self.assertAlmostEqual(esource_id, dp.source_id)
        self.assertAlmostEqual(etemperature, dp.temperature)

    def test_millisec(self):
        etimestamp = datetime.now()
        emillisecs = etimestamp.microsecond / 1000
        esource_id = 1
        etemperature = uniform(0, 16)

        dp = dm.TemperatureDatapoint(etimestamp, esource_id,
                                     etemperature, emillisecs)

        self.assertEqual(etimestamp, dp.timestamp)
        self.assertEqual(emillisecs, dp.millisecond)
        self.assertAlmostEqual(esource_id, dp.source_id)
        self.assertAlmostEqual(etemperature, dp.temperature)

    def test_repr(self):
        etimestamp = datetime.now()
        emillisecs = etimestamp.microsecond / 1000
        esource_id = 1
        etemperature = uniform(0, 16)

        dp = dm.TemperatureDatapoint(etimestamp, esource_id,
                                     etemperature, emillisecs)

        self.assertEqual(dp.__repr__(),
                         "<TemperatureDatapoint"
                         "(source_id=%s, timestamp=%s, temperature=%s)>" %
                         (esource_id, etimestamp, etemperature))


class AirFlowDatapointTests(unittest.TestCase):

    def test_no_millisec(self):
        etimestamp = datetime.now()
        emillisecs = etimestamp.microsecond / 1000
        esource_id = 1
        eair_flow = uniform(0, 16)

        dp = dm.AirFlowDatapoint(etimestamp, esource_id, eair_flow)

        self.assertEqual(etimestamp, dp.timestamp)
        self.assertEqual(emillisecs, dp.millisecond)
        self.assertAlmostEqual(esource_id, dp.source_id)
        self.assertAlmostEqual(eair_flow, dp.air_flow)

    def test_millisec(self):
        etimestamp = datetime.now()
        emillisecs = etimestamp.microsecond / 1000
        esource_id = 1
        eair_flow = uniform(0, 16)

        dp = dm.AirFlowDatapoint(etimestamp, esource_id, eair_flow, emillisecs)

        self.assertEqual(etimestamp, dp.timestamp)
        self.assertEqual(emillisecs, dp.millisecond)
        self.assertAlmostEqual(esource_id, dp.source_id)
        self.assertAlmostEqual(eair_flow, dp.air_flow)

    def test_repr(self):
        etimestamp = datetime.now()
        emillisecs = etimestamp.microsecond / 1000
        esource_id = 1
        eair_flow = uniform(0, 16)

        dp = dm.AirFlowDatapoint(etimestamp, esource_id, eair_flow, emillisecs)

        self.assertEqual(dp.__repr__(),
                         "<AirFlowDatapoint"
                         "(source_id=%s, timestamp=%s, air_flow=%s)>" %
                         (esource_id, etimestamp, eair_flow))


class HeartRateDatapointTests(unittest.TestCase):

    def test_no_millisec(self):
        etimestamp = datetime.now()
        emillisecs = etimestamp.microsecond / 1000
        esource_id = 1
        eheart_rate = uniform(0, 16)

        dp = dm.HeartRateDatapoint(etimestamp, esource_id, eheart_rate)

        self.assertEqual(etimestamp, dp.timestamp)
        self.assertEqual(emillisecs, dp.millisecond)
        self.assertAlmostEqual(esource_id, dp.source_id)
        self.assertAlmostEqual(eheart_rate, dp.heart_rate)

    def test_millisec(self):
        etimestamp = datetime.now()
        emillisecs = etimestamp.microsecond / 1000
        esource_id = 1
        eheart_rate = uniform(0, 16)

        dp = dm.HeartRateDatapoint(etimestamp, esource_id,
                                   eheart_rate, emillisecs)

        self.assertEqual(etimestamp, dp.timestamp)
        self.assertEqual(emillisecs, dp.millisecond)
        self.assertAlmostEqual(esource_id, dp.source_id)
        self.assertAlmostEqual(eheart_rate, dp.heart_rate)

    def test_repr(self):
        etimestamp = datetime.now()
        emillisecs = etimestamp.microsecond / 1000
        esource_id = 1
        eheart_rate = uniform(0, 16)

        dp = dm.HeartRateDatapoint(etimestamp, esource_id,
                                   eheart_rate, emillisecs)

        self.assertEqual(dp.__repr__(),
                         "<HeartRateDatapoint"
                         "(source_id=%s, timestamp=%s, heart_rate=%s)>" %
                         (esource_id, etimestamp, eheart_rate))


class AccelerationDatapointTests(unittest.TestCase):

    def test_no_millisec(self):
        etimestamp = datetime.now()
        emillisecs = etimestamp.microsecond / 1000
        esource_id = 1
        eacc_x = uniform(0, 16)
        eacc_y = uniform(0, 16)
        eacc_z = uniform(0, 16)
        eacc_magn = (eacc_x ** 2 + eacc_y ** 2 + eacc_z ** 2) ** 0.5

        dp = dm.AccelerationDatapoint(etimestamp, esource_id,
                                      eacc_x, eacc_y, eacc_z)

        self.assertEqual(etimestamp, dp.timestamp)
        self.assertEqual(emillisecs, dp.millisecond)
        self.assertAlmostEqual(esource_id, dp.source_id)
        self.assertAlmostEqual(eacc_x, dp.acc_x)
        self.assertAlmostEqual(eacc_y, dp.acc_y)
        self.assertAlmostEqual(eacc_z, dp.acc_z)
        self.assertAlmostEqual(eacc_magn, dp.acc_magn)

    def test_millisec(self):
        etimestamp = datetime.now()
        emillisecs = etimestamp.microsecond / 1000
        esource_id = 1
        eacc_x = uniform(0, 16)
        eacc_y = uniform(0, 16)
        eacc_z = uniform(0, 16)
        eacc_magn = (eacc_x ** 2 + eacc_y ** 2 + eacc_z ** 2) ** 0.5

        dp = dm.AccelerationDatapoint(etimestamp, esource_id,
                                      eacc_x, eacc_y, eacc_z,
                                      emillisecs)

        self.assertEqual(etimestamp, dp.timestamp)
        self.assertEqual(emillisecs, dp.millisecond)
        self.assertAlmostEqual(esource_id, dp.source_id)
        self.assertAlmostEqual(eacc_x, dp.acc_x)
        self.assertAlmostEqual(eacc_y, dp.acc_y)
        self.assertAlmostEqual(eacc_z, dp.acc_z)
        self.assertAlmostEqual(eacc_magn, dp.acc_magn)

    def test_repr(self):
        etimestamp = datetime.now()
        emillisecs = etimestamp.microsecond / 1000
        esource_id = 1
        eacc_x = uniform(0, 16)
        eacc_y = uniform(0, 16)
        eacc_z = uniform(0, 16)
        eacc_magn = (eacc_x ** 2 + eacc_y ** 2 + eacc_z ** 2) ** 0.5

        dp = dm.AccelerationDatapoint(etimestamp, esource_id,
                                      eacc_x, eacc_y, eacc_z,
                                      emillisecs)

        self.assertEqual(dp.__repr__(),
                         "<AccelerationDatapoint"
                         "(source_id=%s, timestamp=%s, "
                         "acc_magn=%s, acc_x=%s, acc_y=%s, acc_z=%s)>" %
                         (esource_id, etimestamp,
                          eacc_magn, eacc_x, eacc_y, eacc_z))


class AlarmTests(unittest.TestCase):

    def test_timestamp_millisec(self):
        etimestamp = datetime.now()
        emillisecs = etimestamp.microsecond / 1000
        ealarm_lvl = randint(30, 240)
        esgmt_begin = uniform(0, 16)
        esgmt_end = uniform(0, 16)
        esource_id = 1

        alarm = dm.Alarm(ealarm_lvl, esgmt_begin, esgmt_end, esource_id,
                         etimestamp, emillisecs)

        self.assertEqual(etimestamp, alarm.timestamp)
        self.assertEqual(emillisecs, alarm.millisecond)
        self.assertAlmostEqual(ealarm_lvl, alarm.alarm_lvl)
        self.assertAlmostEqual(esgmt_begin, alarm.sgmt_begin)
        self.assertAlmostEqual(esgmt_end, alarm.sgmt_end)

    def test_timestamp_no_millisec(self):
        etimestamp = datetime.now()
        emillisecs = etimestamp.microsecond / 1000
        ealarm_lvl = randint(30, 240)
        esgmt_begin = uniform(0, 16)
        esgmt_end = uniform(0, 16)
        esource_id = 1

        alarm = dm.Alarm(ealarm_lvl, esgmt_begin, esgmt_end, esource_id,
                         etimestamp)

        self.assertEqual(etimestamp, alarm.timestamp)
        self.assertEqual(emillisecs, alarm.millisecond)
        self.assertAlmostEqual(ealarm_lvl, alarm.alarm_lvl)
        self.assertAlmostEqual(esgmt_begin, alarm.sgmt_begin)
        self.assertAlmostEqual(esgmt_end, alarm.sgmt_end)

    def test_no_timestamp_millisec(self):
        etimestamp = datetime.now()
        emillisecs = etimestamp.microsecond / 1000
        ealarm_lvl = randint(30, 240)
        esgmt_begin = uniform(0, 16)
        esgmt_end = uniform(0, 16)
        esource_id = 1

        alarm = dm.Alarm(ealarm_lvl, esgmt_begin, esgmt_end, esource_id,
                         millisecond=emillisecs)

        self.assertAlmostEqual(
                ((etimestamp - timedelta(microseconds=etimestamp.microsecond))
                 - datetime(1970, 1, 1)).total_seconds(),
                (alarm.timestamp - datetime(1970, 1, 1)).total_seconds(),
                delta=1)
        self.assertEqual(emillisecs, alarm.millisecond)
        self.assertAlmostEqual(ealarm_lvl, alarm.alarm_lvl)
        self.assertAlmostEqual(esgmt_begin, alarm.sgmt_begin)
        self.assertAlmostEqual(esgmt_end, alarm.sgmt_end)

    def test_no_timestamp_no_millisec(self):
        etimestamp = datetime.now()
        emillisecs = etimestamp.microsecond / 1000
        ealarm_lvl = randint(30, 240)
        esgmt_begin = uniform(0, 16)
        esgmt_end = uniform(0, 16)
        esource_id = 1

        alarm = dm.Alarm(ealarm_lvl, esgmt_begin, esgmt_end, esource_id)

        self.assertAlmostEqual(
                ((etimestamp - timedelta(microseconds=etimestamp.microsecond))
                 - datetime(1970, 1, 1)).total_seconds(),
                (alarm.timestamp - datetime(1970, 1, 1)).total_seconds(),
                delta=1)
        self.assertAlmostEqual(emillisecs, alarm.millisecond, delta=10)
        self.assertAlmostEqual(ealarm_lvl, alarm.alarm_lvl)
        self.assertAlmostEqual(esgmt_begin, alarm.sgmt_begin)
        self.assertAlmostEqual(esgmt_end, alarm.sgmt_end)

    def test_repr(self):
        etimestamp = datetime.now()
        emillisecs = etimestamp.microsecond / 1000
        ealarm_lvl = randint(30, 240)
        esgmt_begin = uniform(0, 16)
        esgmt_end = uniform(0, 16)
        esource_id = 1

        alarm = dm.Alarm(ealarm_lvl, esgmt_begin, esgmt_end, esource_id,
                         etimestamp, emillisecs)

        self.assertEqual(alarm.__repr__(),
                         "<Alarm("
                         "source_id=%s, timestamp=%s, alarm_lvl=%s, "
                         "sgmt_begin=%s, sgmt_end=%s)>"
                         % (esource_id, etimestamp, ealarm_lvl,
                            esgmt_begin, esgmt_end))


class SuitTests(unittest.TestCase):

    def test_no_millisec(self):
        etimestamp = datetime.now()
        emillisecs = etimestamp.microsecond / 1000
        esource_id = 1
        eecg_v1 = uniform(0, 16)

        dp = dm.EcgV1Datapoint(etimestamp, esource_id, eecg_v1)

        self.assertEqual(etimestamp, dp.timestamp)
        self.assertEqual(emillisecs, dp.millisecond)
        self.assertAlmostEqual(esource_id, dp.source_id)
        self.assertAlmostEqual(eecg_v1, dp.ecg_v1)

    def test_millisec(self):
        etimestamp = datetime.now()
        emillisecs = etimestamp.microsecond / 1000
        esource_id = 1
        eecg_v1 = uniform(0, 16)

        dp = dm.EcgV1Datapoint(etimestamp, esource_id, eecg_v1, emillisecs)

        self.assertEqual(etimestamp, dp.timestamp)
        self.assertEqual(emillisecs, dp.millisecond)
        self.assertAlmostEqual(esource_id, dp.source_id)
        self.assertAlmostEqual(eecg_v1, dp.ecg_v1)

    def test_repr(self):
        etimestamp = datetime.now()
        emillisecs = etimestamp.microsecond / 1000
        esource_id = 1
        eecg_v1 = uniform(0, 16)

        dp = dm.EcgV1Datapoint(etimestamp, esource_id, eecg_v1, emillisecs)

        self.assertEqual(dp.__repr__(),
                         "<EcgV1Datapoint(source_id=%s, timestamp=%s, "
                         "ecg_v1=%s)>" % (esource_id, etimestamp, eecg_v1))


if __name__ == '__main__':
    unittest.main()
