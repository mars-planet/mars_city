
#!/usr/bin/python

from __future__ import division
import sys
from datetime import datetime, timedelta

import PyTango
from sqlalchemy import create_engine
from sqlalchemy.orm import sessionmaker
from numpy import mean

from data_model import Datapoint, Alarm, Base
from assumtion_free import AssumptionFreeAA as Detector

class Device(PyTango.DeviceClass):
    cmd_list = {
        'register_datapoint' : [[PyTango.ArgType.DevVar_FloatArray,
                                 "[hr, acc_x, acc_y, acc_z]"],
                                [PyTango.ArgType.DevVoid]],

        'get_avg_hr' : [[PyTango.ArgType.DevLong, "Period"],
                        [PyTango.ArgType.DevFloat,
                         "Average HR in last [Period] seconds"]],

        'get_avg_acc' : [[PyTango.ArgType.DevLong, "Period"],
                         [PyTango.ArgType.DevFloat,
                          "Average acc. magn. in last [Period] seconds"]],

        'get_current_alarms': [[PyTango.ArgType.DevLong, "Period"],
                               [PyTango.ArgType.DevVar_FloatArray,
                                "Alarm levels of the last [Period] seconds"]],
                }

    attr_list = { }


    def __init__(self, name):
        PyTango.DeviceClass.__init__(self, name)
        self.set_type("TestDevice")


class HRMonitor(PyTango.Device_4Impl):


    def __init__(self, cl, name, conn_str='sqlite:///hr_monitor.db'):
        PyTango.Device_4Impl.__init__(self, cl, name)
        self.info_stream('In HRMonitor.__init__')
        HRMonitor.init_device(self)

        self.engine = create_engine(conn_str)
        with self.engine.connect() as conn:
            if not self.engine.dialect.has_table(conn, "alarms"):
                Base.metadata.create_all(self.engine)
        self.Session = sessionmaker(bind=self.engine)

        self.resolution = 2000 # in millisecs
        self.detector = Detector(window_size=500, lead_window_factor=3,
                                lag_window_factor=30, word_size=10,
                                recursion_level=2)


    def __del__(self):
        self.engine.dispose()
        del self.engine
        self.Session.close_all()
        del self.Session


    def _get_avgs(self, period):
        session = self.Session()
        query = session.query(Datapoint)
        query = query.filter(Datapoint.timestamp
                            > datetime.now() - timedelta(seconds=period)
                            )
        avgs = mean([[x.hr, x.acc_magn] for x in query.all()], axis=0)
        session.close()

        return {'hr': avgs[0], 'acc': avgs[1]}


############### BEGIN register_datapoint ###############################################
    def is_register_datapoint_allowed(self, req_type):
        return self.get_state() == PyTango.DevState.ON


    def register_datapoint(self, args):
        dp = Datapoint(datetime.now(), *args)

        universe_size = self.resolution * self.detector.universe_size
        universe_size = timedelta(milliseconds=universe_size)
        session = self.Session()
        query = session.query(Datapoint)
        query = query.filter(Datapoint.timestamp > dp.timestamp - universe_size)
        datapoints = query.all()
        session.close()

        datapoints += dp
        anomaly_score = self.detector.detect_anomalies(datapoints)[0]
        session = self.Session()
        session.add(dp)
        session.add(Alarm(dp.timestamp, anomaly_score,
                          datapoints[0].timestamp, dp.timestamp))
        session.commit()


############### END register_datapoint #################################################


############### BEGIN get_avg_hr ###############################################
    def is_get_avg_hr_allowed(self, req_type):
        return self.get_state() == PyTango.DevState.ON


    def get_avg_hr(self, period):
        return self._get_avgs(period)['hr']
############### END get_avg_hr #################################################


############### BEGIN get_acc_avg ##############################################
    def is_get_acc_avg_allowed(self, req_type):
        return self.get_state() == PyTango.DevState.ON


    def get_acc_avg(self, period):
        return self._get_avgs(period)['acc']
############### END get_acc_avg ################################################


############### BEGIN get_current_alarms #######################################
    def is_get_current_alarms_allowed(self, req_type):
        return self.get_state() == PyTango.DevState.ON


    def get_current_alarms(self, period):
        session = self.Session()
        query = session.query(Alarm)
        query = query.filter(Alarm.timestamp
                            > datetime.now() - timedelta(seconds=period)
                            )
        results = query.all()
        session.close()

        return results
############### END get_current_alarms #########################################


    def init_device(self):
        self.info_stream('In Python init_device method')
        self.set_state(PyTango.DevState.ON)


if __name__ == '__main__':
    util = PyTango.Util(sys.argv)
    util.add_class(Device, HRMonitor)

    U = PyTango.Util.instance()
    U.server_init()
    U.server_run()
