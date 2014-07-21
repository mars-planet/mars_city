#!/usr/bin/python
'''
Implements the Health Monitor Server Interface.
'''
from __future__ import division, print_function

from collections import Iterable
from datetime import datetime, timedelta

from sqlalchemy import create_engine
from sqlalchemy.exc import IntegrityError
from sqlalchemy.orm import sessionmaker
from sqlalchemy.pool import StaticPool

from assumption_free import AssumptionFreeAA as Detector
import data_model as dm
import numpy as np
import pandas as pd


def _analize_datapoints(datapoints, detector):
    if not isinstance(datapoints, pd.Series):
        raise ValueError('The datapoints parameter must be an instance of '
                         'pandas.DataFrame insted of: %s' %
                         datapoints.__class__.__name__)
    if not datapoints.isnull().all():
        datapoints = datapoints.dropna()
        sgmt_begin = datapoints.index.min()
        sgmt_end = datapoints.index.max()

        analysis_result = detector.detect(np.asarray(datapoints))
        if analysis_result:
            return analysis_result, sgmt_begin, sgmt_end


class HealthMonitor(object):
    '''
    Implements the Health Monitor Server Interface.
    '''

    def __init__(self, sources,
                 word_size=5, window_factor=2, lead_window_factor=2,
                 lag_window_factor=4, engine=None,
                 conn_str='sqlite://', log_function=None):
        '''
        word_size,
        window_factor,
        lead_window_factor,
        lag_window_factor: parameters passed to the anomaly detector;
        engine: allows to inject an sqlalchemy.engine instance;
        conn_str: database's connection string.
        '''
        if log_function is None:
            log_function = print
        self.log_function = log_function
        self.log_function('Constructing HealthMonitor')
        if not isinstance(sources, Iterable):
            raise ValueError('An iterable of sources must be provided.')
        if engine is not None:
            self.engine = engine
        else:
            self.log_function(conn_str)
            self.engine = create_engine(conn_str,
                                        connect_args={'check_same_thread':
                                                      False},
                                        poolclass=StaticPool,
                                        isolation_level='SERIALIZABLE')
            self._setup_database()
        self.Session = sessionmaker(bind=self.engine, autocommit=False)
        self.log_function('Constructing Detector')
        detector = (lambda: Detector(word_size=word_size,
                                     window_factor=window_factor,
                                     lead_window_factor=lead_window_factor,
                                     lag_window_factor=lag_window_factor))
        self.log_function('Finished constructing HealthMonitor')
        self.source_entity = dm.Suit
        self.dp_entities = [dm.AccelerationDatapoint, dm.AirFlowDatapoint,
                            dm.EcgV1Datapoint, dm.EcgV2Datapoint,
                            dm.HeartRateDatapoint, dm.O2Datapoint,
                            dm.TemperatureDatapoint]
        self.sources = {s: [(entity, detector())
                            for entity in self.dp_entities]
                        for s in sources}

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

    # this isn't used at the moment
    def register_datasource(self, name, connection_str):
        '''
        Registers a new data source in the database.
        connection_str is a Tango connection string.
        '''
        source = self.source_entity(name=name, connection_str=connection_str,
                                    variable_classes=self.dp_entities)
        try:
            session = self.Session()
            session.add(source)
            session.commit()
        finally:
            session.close()

    def generate_alarms(self, source_id, **kwargs):
        '''
        Analyzes the data collected so far and inserts in the database
        the scores generated (if any).
        kwargs should be a dictionary with the variables as names and
        lists of pairs (timestamp, variable value) as values, e.g.:
        kwargs['heart_rate'] = [(2014-07-13 20:56:41.0, 0.583374),
                                (2014-07-13 20:56:41.5, 0.585754),
                                (2014-07-13 20:56:42.0, 0.583782),
                                (2014-07-13 20:56:42.5, 0.579044)]
        '''
        data = pd.DataFrame()
        for k in kwargs:
            data = data.append(pd.DataFrame(kwargs[k],
                                            columns=['timestamp', k])
                                 .set_index('timestamp'))
        data = data.sort_index()
        for entity, detector in self.sources[source_id]:
            try:
                session = self.Session()
                result = _analize_datapoints(data[entity.variable_names()[0]],
                                             detector)
                if result:
                    analysis, sgmt_begin, sgmt_end = result
                    analysis = analysis[0]
                    session.add(
                            dm.Alarm(timestamp=sgmt_end,
                                     alarm_lvl=analysis.score,
                                     sgmt_begin=sgmt_begin,
                                     sgmt_end=sgmt_end,
                                     source_id=source_id,
                                     kind=entity.variable_names()[0]))
                session.commit()
            except IntegrityError as e:
                print('IntegrityError: %s' % e)
            finally:
                session.close()

    def get_alarms(self, period, source):
        '''
        Returns the alarm scores generated in the last [period] seconds.
        '''
        current = datetime.now()
        current -= timedelta(microseconds=current.microsecond)
        init = current - timedelta(seconds=int(period))
        print('init: %s' % init)
        results = []
        try:
            session = self.Session()
            query = (session.query(dm.Alarm)
                            .filter((dm.Alarm.timestamp >= init) &
                                    (dm.Alarm.source_id == source))
                            .order_by(dm.Alarm.timestamp))
            results = query.all()
        finally:
            session.close()

        return results
