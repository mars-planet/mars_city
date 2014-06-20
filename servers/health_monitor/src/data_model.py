from __future__ import division

from collections import OrderedDict
from datetime import datetime, timedelta
import re

import inflect
from sqlalchemy import Column, DateTime, Enum, Float, Index, Integer, String
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.ext.declarative import declared_attr
from sqlalchemy.orm import reconstructor


p = inflect.engine()


def get_datapoint_class(name):
    return globals()[name]


def camel_to_underscore(name):
    s1 = re.sub('(.)([A-Z][a-z]+)', r'\1_\2', name)
    return re.sub('([a-z0-9])([A-Z])', r'\1_\2', s1).lower()


Base = declarative_base()


class Mixin(object):
    @declared_attr
    def __tablename__(cls):
        return p.plural(camel_to_underscore(cls.__name__))

    @classmethod
    def variable_names(cls, *args, **kwargs):
        return sorted(c.name for c in cls.__table__.columns)

    @classmethod
    def columns(cls, *args, **kwargs):
        return sorted(cls.__table__.columns.values())

    def __repr__(self):
        attr_repr = ', '.join('%s=%s' % (k, v)
                              for k, v in self.variables(filter_common=False))
        return ("<%s(%s)>" % (self.__class__.__name__, attr_repr))

    def variables(self, *args, **kwargs):
        return OrderedDict({k: self.__dict__[k]
                            for k in self.variable_names(filter_common)})


class DatapointMixin(Mixin):
    timestamp = Column(DateTime, primary_key=True)
    millisecond = Column(Float, primary_key=True)
    source_id = Column(Integer, primary_key=True)
    doe = Column(DateTime, default=datetime.now)

    @declared_attr
    def __table_args__(cls):
        return (Index('idx_%s' % cls.__tablename__,
                      'timestamp', 'millisecond', 'source_id'),)

    @classmethod
    def variable_names(cls, filter_common=True):
        ret_val = super(DatapointMixin).variable_names()
        if filter_common:
            ret_val.remove('timestamp')
            ret_val.remove('millisecond')
            ret_val.remove('source_id')
            ret_val.remove('doe')
        return ret_val

    @classmethod
    def columns(cls, filter_common=True):
        ret_val = [c for c in super(DatapointMixin).columns()
                        if not filter_common
                            or c.name not in ('timestamp', 'millisecond',
                                              'source_id', 'doe')]
        return ret_val

    def __init__(self, timestamp, source_id, millisecond=None):
        self.timestamp = timestamp
        self.source_id = source_id
        if millisecond:
            self.timestamp -= timedelta(microseconds=timestamp.microsecond)
            self.timestamp += timedelta(microseconds=millisecond * 1000)
            self.millisecond = millisecond
        else:
            self.millisecond = timestamp.microsecond / 1000


class SourceMixin(Mixin):
    source_id = Column(Integer, primary_key=True,
                       index=True, autoincrement=True)
    name = Column(String, nullable=False)
    doe = Column(DateTime, default=datetime.now)
    connection_str = Column(String, nullable=False)
    _variable_classes_str = Column(String, nullable=True)

    @classmethod
    def variable_names(cls, filter_common=True):
        ret_val = super(SourceMixin).variable_names()
        if filter_common:
            ret_val.remove('source_id')
            ret_val.remove('name')
            ret_val.remove('doe')
            ret_val.remove('connection_str')
        return ret_val

    @classmethod
    def columns(cls, filter_common=True):
        ret_val = [c for c in super(SourceMixin).columns()
                        if not filter_common
                            or c.name not in ('source_id', 'name',
                                              'connection_str', 'doe',
                                              'variable_classes')]
        return ret_val

    def __init__(self, name, connection_str,
                 source_id=None, variable_classes=None):
        if source_id is not None:
            self.source_id = source_id
        self.name = name
        self.connection_str = connection_str
        self.variable_classes = variable_classes
        if variable_classes:
            variable_classes_str = ';'.join(c.__name__
                                            for c in variable_classes)
            self._variable_classes_str = variable_classes_str

    @reconstructor
    def reconstruct(self):
        self.variable_classes = [globals()[cls_name]
                                 for cls_name in self._variable_classes_str
                                                     .split(';')]


class EcgV1Datapoint(DatapointMixin, Base):
    ecg_v1 = Column(Float)

    def __init__(self, timestamp, source_id, ecg_v1, millisecond=None):
        super(EcgV1Datapoint, self).__init__(timestamp, source_id, millisecond)
        self.ecg_v1 = ecg_v1


class EcgV2Datapoint(DatapointMixin, Base):
    ecg_v2 = Column(Float)

    def __init__(self, timestamp, source_id, ecg_v2, millisecond=None):
        super(EcgV2Datapoint, self).__init__(timestamp, source_id, millisecond)
        self.ecg_v2 = ecg_v2


class O2Datapoint(DatapointMixin, Base):
    o2 = Column(Float)

    def __init__(self, timestamp, source_id, o2, millisecond=None):
        super(O2Datapoint, self).__init__(timestamp, source_id, millisecond)
        self.o2 = o2


class TemperatureDatapoint(DatapointMixin, Base):
    temperature = Column(Float)

    def __init__(self, timestamp, source_id, temperature, millisecond=None):
        super(TemperatureDatapoint, self).__init__(timestamp,
                                                   source_id,
                                                   millisecond)
        self.temperature = temperature


class AirFlowDatapoint(DatapointMixin, Base):
    air_flow = Column(Float)

    def __init__(self, timestamp, source_id, air_flow, millisecond=None):
        super(AirFlowDatapoint, self).__init__(timestamp,
                                               source_id,
                                               millisecond)
        self.air_flow = air_flow


class HeartRateDatapoint(DatapointMixin, Base):
    heart_rate = Column(Float)

    def __init__(self, timestamp, source_id, heart_rate, millisecond=None):
        super(HeartRateDatapoint, self).__init__(timestamp,
                                                 source_id,
                                                 millisecond)
        self.heart_rate = heart_rate


class AccelerationDatapoint(DatapointMixin, Base):
    acc_x = Column(Float)
    acc_y = Column(Float)
    acc_z = Column(Float)
    acc_magn = Column(Float)

    def __init__(self, timestamp, source_id,
                 acc_x, acc_y, acc_z, millisecond=None):
        super(AccelerationDatapoint, self).__init__(timestamp,
                                                    source_id,
                                                    millisecond)
        self.acc_x = acc_x
        self.acc_y = acc_y
        self.acc_z = acc_z
        self.acc_magn = (acc_x ** 2 + acc_y ** 2 + acc_z ** 2) ** 0.5


class Alarm(DatapointMixin, Base):
    alarm_lvl = Column(Float, nullable=False)
    kind = Column(Enum('ecg_v1', 'ecg_v2', 'o2',
                       'temperature', 'air_flow', 'hr',
                       'acc_x', 'acc_y', 'acc_z', 'acc_magn'),
                  nullable=False)
    mixed_kind = Column(Enum('ecg_v1', 'ecg_v2', 'o2',
                             'temperature', 'air_flow', 'hr',
                             'acc_x', 'acc_y', 'acc_z', 'acc_magn'),
                        nullable=True)
    sgmt_begin = Column(DateTime, nullable=False)
    sgmt_end = Column(DateTime, nullable=False)

    def __init__(self, alarm_lvl, sgmt_begin, sgmt_end,
                 source_id, timestamp=None, millisecond=None):
        if not timestamp:
            timestamp = datetime.now()
        super(AccelerationDatapoint, self).__init__(timestamp,
                                                    source_id,
                                                    millisecond)
        self.alarm_lvl = alarm_lvl
        self.sgmt_begin = sgmt_begin
        self.sgmt_end = sgmt_end


class Suit(SourceMixin, Base):
    def __init__(self, suit_id, name, connection_str):
        super(Suit, self).__init__(suit_id, name, connection_str)
