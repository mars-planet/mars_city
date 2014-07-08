#!/usr/bin/python
'''
Implements the Health Monitor Server Interface.
'''
from __future__ import division, print_function

from collections import deque, Iterable
from datetime import datetime, timedelta
from random import randint
from threading import Thread, Lock

from sqlalchemy import create_engine
from sqlalchemy.exc import IntegrityError
from sqlalchemy.orm import sessionmaker
from sqlalchemy.pool import StaticPool

from assumption_free import AssumptionFreeAA as Detector
import data_model as dm
import numpy as np
import pandas as pd


class HealthMonitor(object):
    '''
    Implements the Health Monitor Server Interface.
    '''

    def __init__(self, sources,
                 word_size=5, window_factor=2, lead_window_factor=2,
                 lag_window_factor=4, resolution=1000, engine=None,
                 conn_str='sqlite://', log_function=None):
        '''
        word_size,
        window_factor,
        lead_window_factor,
        lag_window_factor: parameters passed to the anomaly detector;
        resolution: determines how often should alarms be generated;
        engine: allows to inject an sqlalchemy.engine instance;
        conn_str: database's connection string.
        '''
        if log_function is None:
            log_function = print
        self.log_function = log_function
        self.log_function('Constructing HealthMonitor')
        if not isinstance(sources, Iterable):
            raise ValueError('An iterable of sources must be provided.')
        self.sources = sources
        if engine is not None:
            self.engine = engine
        else:
            print(conn_str)
            self.engine = create_engine(conn_str,
                                        connect_args={'check_same_thread':
                                                      False},
                                        poolclass=StaticPool,
                                        isolation_level='SERIALIZABLE')
            self._setup_database()
        self.Session = sessionmaker(bind=self.engine, autocommit=False)
        self.resolution = resolution  # in millisecs
        self.last_alarm_timestamp = datetime.now()
        self.last_detection_timestamp = datetime.now()
        self.log_function('Constructing Detector')
        self.detector = Detector(word_size=word_size,
                                 window_factor=window_factor,
                                 lead_window_factor=lead_window_factor,
                                 lag_window_factor=lag_window_factor)
        self.log_function('Finished constructing HealthMonitor')
        self.timestamps = deque(maxlen=self.detector.universe_size)
        self.lock = Lock()
        self.source_entity = dm.Suit
        self.dp_entities = [dm.AccelerationDatapoint, dm.AirFlowDatapoint,
                            dm.EcgV1Datapoint, dm.EcgV2Datapoint,
                            dm.HeartRateDatapoint, dm.O2Datapoint,
                            dm.TemperatureDatapoint]

    def __del__(self):
        '''
        Closes all sqlalchemy.session
        and disposes the sqlalchemy.engine instance.
        '''
        self.log_function('Cleaning up HealthMonitor')
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

    def _setup_database(self):
        '''
        Checks if the tables are already in the database,
        and creates them if necessary.
        '''
        self.log_function('Setting up SQLite DB on: %s' % self.engine.url)
        with self.engine.connect():
            self.log_function('Creating Tables')
            dm.Base.metadata.create_all(self.engine)

    def register_datapoints(self, timestamp, source_id, **kwargs):
        '''
        Registers a new datapoint in the database and launches a new thread to
        analyze the data collected so far.
        args should be (timestamp, hr, acc_x, acc_y, acc_z).
        '''
        with self.Session() as session:
            for cls in self.dp_entities:
                Datapoint = dm.get_datapoint_class(name=cls)
                datum = Datapoint(timestamp=timestamp, source_id=source_id,
                                  **kwargs)
                try:
                    session.add(datum)
                    Thread(target=self._generate_alarms).start()
                except IntegrityError:
                    pass
            session.commit()

    # this isn't used at the moment
    def register_datasource(self, name, connection_str):
        '''
        Registers a new data source in the database.
        connection_str is a Tango connection string.
        '''
        source = self.source_entity(name=name, connection_str=connection_str,
                                    variable_classes=self.dp_entities)
        with self.Session() as session:
            session.add(source)
            session.commit()

    def _generate_alarms(self):
        '''
        Analyzes the data collected so far and inserts in the database
        the scores generated (if any).
        '''
        timerange = datetime.now() - self.last_alarm_timestamp
        timerange = timerange.total_seconds() * 1000
        if timerange >= self.resolution:
            self.last_alarm_timestamp = datetime.now()
            with self.Session() as session:
                query = session.query()
                for entity in self.dp_entities:
                    query = query.add_columns(
                                  [c.label('%s_%s' % (c.table.name, c.key))
                                   for c in entity.columns()])
                    query = query.filter(entity.timestamp
                                         >= self.last_detection_timestamp)
                datapoints = query.all()
                session.close()
                datapoints = pd.DataFrame(
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
                        session.add(
                                dm.Alarm(timestamp=last_timestamp,
                                      alarm_lvl=analysis.score,
                                      sgmt_begin=first_timestamp,
                                      sgmt_end=last_timestamp))
                        try:
                            session.commit()
                        except IntegrityError:
                            session.rollback()
                            musecs = randint(0, 10 ** 6)
                            session.add(
                                    dm.Alarm(timestamp=last_timestamp
                                            + timedelta(microseconds=musecs),
                                          alarm_lvl=analysis.score,
                                          sgmt_begin=first_timestamp,
                                          sgmt_end=last_timestamp))
                            session.commit()

    def get_alarms(self, period):
        '''
        Returns the alarm scores generated in the last [period] seconds.
        '''
        current = datetime.now()
        current -= timedelta(microseconds=current.microsecond)
        init = current - timedelta(seconds=period)
        with self.Session() as session:
            query = session.query(dm.Alarm)
            query = query.filter(dm.Alarm.timestamp >= init)
            results = query.all()

        return results
