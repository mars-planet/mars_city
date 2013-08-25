#!/usr/bin/python

from __future__ import division, print_function
from datetime import datetime, timedelta

from numpy import mean, nan
from pandas import DataFrame
from sqlalchemy import create_engine
from sqlalchemy.orm import sessionmaker


from data_model import Datapoint, Alarm, Base
from assumption_free import AssumptionFreeAA as Detector


class HRMonitor(object):

    def __init__(self, engine=None, conn_str='sqlite:///hr_monitor.db',
                 commit_on_request=True):
        print('Constructing HRMonitor')
        if engine is not None:
            self.engine = engine
        else:
            print('Setting up SQLite DB')
            self.engine = create_engine(conn_str)
            conn = self.engine.connect()
            if not self.engine.dialect.has_table(conn, "alarms"):
                print('Creating Tables')
                Base.metadata.create_all(self.engine)
        self.Session = sessionmaker(bind=self.engine)
        self._current_session = None
        self._commit_on_request = commit_on_request
        self.resolution = 2000  # in millisecs
        print('Constructing Detector')
        self.detector = Detector(word_size=10, window_factor=3,
                                 lead_window_factor=3, lag_window_factor=12,
                                 recursion_level=2)
        print('Finished constructing HRMonitor')

    def __del__(self):
        print('Cleaning up HRMonitor')
        self._get_session().commit()
        self.Session.close_all()
        del self.Session
        self.engine.dispose()
        del self.engine

    def _get_session(self):
        if not self._current_session:
            self._current_session = self.Session()
        return self._current_session

    def _session_commit(self):
        if self._commit_on_request:
            self._current_session.commit()
            self._current_session = None

    def _session_close(self):
        if self._commit_on_request:
            self._current_session.close()
            self._current_session = None

    def _get_avgs(self, period):
        session = self._get_session()
        query = session.query(Datapoint)
        query = query.filter(Datapoint.timestamp
                            > datetime.now() - timedelta(seconds=period)
                            )
        data = [[x.hr, x.acc_magn] for x in query.all()]
        if len(data) > 0:
            avgs = mean(data, axis=0)
        else:
            avgs = [nan, nan]
        self._session_close()
        return {'hr': avgs[0], 'acc': avgs[1]}

    def register_datapoint(self, args):
        dp = Datapoint(*args)
        session = self._get_session()
        session.add(dp)
        self._session_commit()
        query = session.query(Datapoint)
        query = query.filter(Datapoint.timestamp
                             > self.detector.last_timestamp)
        datapoints = query.all()
        self._session_close()
        timerange = (dp.timestamp - datapoints[0].timestamp).total_seconds()
        timerange *= 1000
        if datapoints and  timerange >= self.resolution:
            datapoints.append(dp)
            first_dp = datapoints[0]
            datapoints = DataFrame({
                            'timestamp': [d.timestamp for d in datapoints],
                            'ratio': [d.hr / d.acc_magn for d in datapoints]
                        })
            datapoints.set_index('timestamp', inplace=True)
            datapoints = datapoints.resample('%sms' % (self.resolution))
            analysis = self.detector.detect(datapoints.ratio, dp.timestamp)
            if analysis:
                analysis = analysis[0]
                session = self._get_session()
                session.add(Alarm(dp.timestamp, analysis.score,
                                  first_dp.timestamp, dp.timestamp))
                self._session_commit()
                self._session_close()

    def get_avg_hr(self, period):
        return self._get_avgs(period)['hr']

    def get_avg_acc(self, period):
        return self._get_avgs(period)['acc']

    def get_current_alarms(self, period):
        session = self._get_session()
        query = session.query(Alarm)
        query = query.filter(Alarm.timestamp
                            > datetime.now() - timedelta(seconds=period)
                            )
        results = query.all()
        self._session_close()
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
