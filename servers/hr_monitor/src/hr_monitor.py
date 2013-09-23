#!/usr/bin/python
"""
Implements the HR Monitor Server Interface.
"""
from __future__ import division, print_function

from datetime import datetime, timedelta
from collections import deque
from threading import Thread, Lock
from random import randint
import base64

from numpy import mean, nan
from pandas import DataFrame
from sqlalchemy import create_engine
from sqlalchemy.orm import sessionmaker
from sqlalchemy.exc import IntegrityError

from data_model import Datapoint, Alarm, Base
from assumption_free import AssumptionFreeAA as Detector


class DuplicatedDatapointError(Exception):
    """
    Signals a duplicated datapoint tried to be inserted in the database.
    """
    pass


class HRMonitor(object):
    """
    Implements the HR Monitor Server Interface.
    """

    # for sqlite use conn_str='sqlite:///hr_monitor.db'
    def __init__(self, word_size=5, window_factor=2, lead_window_factor=2,
                 lag_window_factor=4, resolution=1000, engine=None,
                 conn_str='mysql+mysqldb://root@localhost/hr_monitor'):
        """
        word_size,
        window_factor,
        lead_window_factor,
        lag_window_factor: parameters passed to the anomaly detector;
        resolution: determines how often should alarms be generated;
        engine: allows to inject an sqlalchemy.engine instance;
        conn_str: database's connection string.
        """
        print('Constructing HRMonitor')
        if engine is not None:
            self.engine = engine
        else:
            self._setup_database(conn_str)
            self.engine = create_engine(conn_str,
                                        isolation_level="SERIALIZABLE")
        self.Session = sessionmaker(bind=self.engine, autocommit=False)
        self.resolution = resolution  # in millisecs
        self.last_alarm_timestamp = datetime.now()
        self.last_detection_timestamp = datetime.now()
        print('Constructing Detector')
        self.detector = Detector(word_size=word_size,
                                 window_factor=window_factor,
                                 lead_window_factor=lead_window_factor,
                                 lag_window_factor=lag_window_factor)
        print('Finished constructing HRMonitor')
        self.timestamps = deque(maxlen=self.detector.universe_size)
        self.lock = Lock()

    def __del__(self):
        """
        Closes all sqlalchemy.session
        and disposes the sqlalchemy.engine instance.
        """
        print('Cleaning up HRMonitor')
        self.Session.close_all()
        del self.Session
        self.engine.dispose()
        del self.engine

    def _setup_database(self, conn_str):
        """
        Checks if the tables are already in the database,
        and creates them if necessary.
        """
        print('Setting up SQLite DB on: %s' % conn_str)
        self.engine = create_engine(conn_str)
        conn = self.engine.connect()
        if not (self.engine.dialect.has_table(conn, "alarms")
                and self.engine.dialect.has_table(conn, "datapoints")):
            print('Creating Tables')
            Base.metadata.create_all(self.engine)
        conn.close()
        self.engine.dispose()

    def _get_avgs(self, period):
        """
        Returns averages for heart rate and acceleration
        over the given [period] (in seconds).
        """
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
        return {'hr': avgs[0], 'acc': avgs[1]}

    def register_datapoint(self, timestamp, hr, acc_x, acc_y, acc_z):
        """
        Registers a new datapoint in the database and launches a new thread to
        analyze the data collected so far.
        args should be (timestamp, hr, acc_x, acc_y, acc_z).
        """
        datum = Datapoint(timestamp=timestamp, hr=hr,
                          acc_x=acc_x, acc_y=acc_y, acc_z=acc_z)
        session = self.Session()
        try:
            session.add(datum)
            session.commit()
            Thread(target=self._generate_alarms).start()
        except IntegrityError:
            raise DuplicatedDatapointError()
        finally:
            session.close()

    def _generate_alarms(self):
        """
        Analyzes the data collected so far and inserts in the database
        the scores generated (if any).
        """
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
                session.close()
                datapoints = DataFrame(
                       {
                        'timestamp': [d.timestamp for d in datapoints],
                        'ratio': [d.hr / d.acc_magn for d in datapoints]
                       })
                datapoints.set_index('timestamp', inplace=True)
                datapoints = datapoints.resample('%sL' % self.resolution,
                                                 how='mean')
                datapoints = datapoints.fillna(method='ffill')
                datapoints = datapoints.fillna(method='bfill')
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
                        bitmp1 = base64.b64encode(bitmp1)
                        bitmp2 = analysis.bitmp2.flatten().tostring()
                        bitmp2 = base64.b64encode(bitmp2)
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
        """
        Returns average heart rate over the given [period] (in seconds).
        """
        return self._get_avgs(period)['hr']

    def get_avg_acc(self, period):
        """
        Returns average acceleration over the given [period] (in seconds).
        """
        return self._get_avgs(period)['acc']

    def get_current_alarms(self, period):
        """
        Returns the alarm scores generated in the last [period] seconds.
        """
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
        return results
