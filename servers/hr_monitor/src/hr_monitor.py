#!/usr/bin/python

from __future__ import division, print_function

from datetime import datetime, timedelta
from collections import deque
from threading import Thread, Lock
from random import randint

from numpy import mean, nan, isnan
from pandas import DataFrame
from sqlalchemy import create_engine
from sqlalchemy.orm import sessionmaker
from sqlalchemy.exc import IntegrityError

from data_model import Datapoint, Alarm, Base
from assumption_free import AssumptionFreeAA as Detector


class HRMonitor(object):

    # for sqlite use conn_str='sqlite:///hr_monitor.db'
    def __init__(self, engine=None,
                 conn_str='mysql+mysqldb://root@localhost/hr_monitor'):
        print('Constructing HRMonitor')
        if engine is not None:
            self.engine = engine
        else:
            print('Setting up SQLite DB')
            engn = create_engine(conn_str)
            conn = engn.connect()
            if not engn.dialect.has_table(conn, "alarms"):
                print('Creating Tables')
                Base.metadata.create_all(engn)
            conn.close()
            engn.dispose()
            self.engine = create_engine(conn_str,
                                        isolation_level="SERIALIZABLE")
        self.Session = sessionmaker(bind=self.engine, autocommit=False)
        self.resolution = 1000  # in millisecs
        self.last_alarm_timestamp = datetime.now()
        self.last_detection_timestamp = datetime.now()
        print('Constructing Detector')
        self.detector = Detector(word_size=5, window_factor=2,
                                 lead_window_factor=2, lag_window_factor=4,
                                 recursion_level=2)
        print('Finished constructing HRMonitor')
        self.timestamps = deque(maxlen=self.detector.universe_size)
        self.lock = Lock()

    def __del__(self):
        print('Cleaning up HRMonitor')
        self.Session.close_all()
        del self.Session
        self._sqlconn.close()
        del self._sqlconn
        self.engine.dispose()
        del self.engine

    def _get_avgs(self, period):
        current = datetime.now()
        current -= timedelta(microseconds=current.microsecond)
        init = current - timedelta(seconds=period)
        try:
            session = self.Session()
            query = session.query(Datapoint)
            query = query.filter(Datapoint.timestamp >= init)
            data = [[x.hr, x.acc_magn] for x in query.all()]
        finally:
            session.close()
        if len(data) > 0:
            avgs = mean(data, axis=0)
        else:
            avgs = [nan, nan]
        print("current: %s; init: %s; avgs: %s"
              % (current, init, avgs))
        if any(isnan(avgs)):
            print(data)
        return {'hr': avgs[0], 'acc': avgs[1]}

    def register_datapoint(self, *args):
        timestamp, hr, acc_x, acc_y, acc_z = args
        dp = Datapoint(timestamp=timestamp, hr=hr,
                       acc_x=acc_x, acc_y=acc_y, acc_z=acc_z)
        session = self.Session()
        try:
            session.add(dp)
            session.commit()
            t = Thread(target=self._generate_alarms)
            t.start()
        finally:
            session.close()

    def _generate_alarms(self):
        timerange = datetime.now() - self.last_alarm_timestamp
        timerange = timerange.total_seconds() * 1000
        if timerange >= self.resolution:
            self.last_alarm_timestamp = datetime.now()
            session = self.Session()
            try:
                query = session.query(Datapoint)
                query = query.filter(Datapoint.timestamp
                                     >= self.last_detection_timestamp)
                datapoints = query.all()
                datapoints = DataFrame(
                       {
                        'timestamp': [d.timestamp for d in datapoints],
                        'ratio': [d.hr / d.acc_magn for d in datapoints]
                       })
                datapoints.set_index('timestamp', inplace=True)
                datapoints = datapoints.resample('%sms' % self.resolution)
                datapoints = datapoints.fillna(method='ffill')
                session.close()
                if len(datapoints) > 0:
                    first_timestamp = datapoints.index.min()
                    last_timestamp = datapoints.index.max()
                    self.timestamps.extend(datapoints.index.tolist())
                    self.lock.acquire()
                    self.last_detection_timestamp = last_timestamp
                    analysis = self.detector.detect(datapoints.ratio)
                    self.lock.release()
                    if analysis:
                        session = self.Session()
                        if self.timestamps:
                            first_timestamp = self.timestamps.popleft()
                        analysis = analysis[0]
                        bitmp1 = analysis.bitmp1.flatten().tostring()
                        bitmp2 = analysis.bitmp2.flatten().tostring()
                        session.add(
                                Alarm(timestamp=last_timestamp,
                                      alarm_lvl=analysis.score,
                                      bitmp1=bitmp1, bitmp2=bitmp2,
                                      sgmt_begin=first_timestamp,
                                      sgmt_end=last_timestamp))
                        try:
                            session.commit()
                        except IntegrityError:
                            session.rollback()
                            musecs = randint(0, 10 ** 6)
                            session.add(
                                    Alarm(timestamp=last_timestamp
                                            + timedelta(microseconds=musecs),
                                          alarm_lvl=analysis.score,
                                          bitmp1=bitmp1, bitmp2=bitmp2,
                                          sgmt_begin=first_timestamp,
                                          sgmt_end=last_timestamp))
                            session.commit()
            finally:
                session.close()
                del session

    def get_avg_hr(self, period):
        return self._get_avgs(period)['hr']

    def get_avg_acc(self, period):
        return self._get_avgs(period)['acc']

    def get_current_alarms(self, period):
        current = datetime.now()
        current -= timedelta(microseconds=current.microsecond)
        init = current - timedelta(seconds=period)
        try:
            session = self.Session()
            query = session.query(Alarm)
            query = query.filter(Alarm.timestamp >= init)
            results = query.all()
        finally:
            session.close()
        print("current: %s; init: %s; results: %s"
              % (current, init, results))
        return results


if __name__ == '__main__':
    import os
    from collections import namedtuple
    from preprocessing import read_data, extract_hr_acc

    dirname = os.path.dirname(__file__)
    dbfilename = os.path.join(dirname, 'hr_monitor.db')
    engine = create_engine('sqlite:///' + dbfilename)
    Base.metadata.drop_all(engine)
    Base.metadata.create_all(engine)

    hr_mon = HRMonitor(engine, commit_on_request=False)

    dirname = os.path.join(dirname, 'tests')
    filename = os.path.join(dirname, 'dataset.dat')
    print(filename)
    data = extract_hr_acc(read_data(filename))

    DP = namedtuple("DP", ["timestamp", "hr", "acc_x", "acc_y", "acc_z"])
    i = 0
    percentiles = int(len(data) / 1000)
    for index, row in data.iterrows():
        if i % percentiles == 0:
            print("%s%%" % (i / percentiles))
        datapoint = DP(timestamp=index.to_datetime(), hr=row['hr'],
                       acc_x=row['acc_x'], acc_y=row['acc_y'],
                       acc_z=row['acc_z'])
        hr_mon.register_datapoint(datapoint)
        i += 1

    del hr_mon
