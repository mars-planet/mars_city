from __future__ import division

from collections import OrderedDict
from datetime import datetime, timedelta
import re

import inflect
from sqlalchemy import Column, DateTime, Enum, Float, Index, String
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.ext.declarative import declared_attr
from sqlalchemy.orm import reconstructor


_inflct_engn = inflect.engine()

_datapoint_classes = {}


def datapoint_class(cls):
    _datapoint_classes[cls.__name__] = cls
    return cls


def get_datapoint_class(name):
    return _datapoint_classes[name]


def camel_to_underscore(name):
    s1 = re.sub('(.)([A-Z][a-z]+)', r'\1_\2', name)
    return re.sub('([a-z0-9])([A-Z])', r'\1_\2', s1).lower()


Base = declarative_base()


class Mixin(object):
    @declared_attr
    def __tablename__(cls):
        return _inflct_engn.plural(camel_to_underscore(cls.__name__))

    @classmethod
    def variable_names(cls, *args, **kwargs):
        return sorted(c.name for c in cls.__table__.columns)

    @classmethod
    def variable_columns(cls, *args, **kwargs):
        return sorted(cls.__table__.columns.values(), key=lambda x: x.name)

    def __repr__(self):
        attr_repr = ', '.join('%s=%s' % (k, v)
                              for k, v
                              in self.variables(filter_common=False).items())
        return ("<%s(%s)>" % (self.__class__.__name__, attr_repr))

    def __getitem__(self, key):
        if key not in self.__table__.columns:
            raise ValueError('No such column: %s' % key)
        return self.__dict__[key]

    def variables(self, filter_common=True, *args, **kwargs):
        ret_val = OrderedDict()
        for k in self.variable_names(filter_common):
            if k in self.__dict__:
                ret_val[k] = self.__dict__[k]
        return ret_val


class DatapointMixin(Mixin):
    timestamp = Column(DateTime, primary_key=True)
    millisecond = Column(Float, primary_key=True)
    source_id = Column(String, primary_key=True)
    doe = Column(DateTime, default=datetime.now)

    @declared_attr
    def __table_args__(cls):
        return (Index('idx_%s' % cls.__tablename__,
                      'timestamp', 'millisecond', 'source_id'),)

    @classmethod
    def variable_names(cls, filter_common=True):
        ret_val = super(DatapointMixin, cls).variable_names()
        ret_val.remove('millisecond')
        ret_val.remove('timestamp')
        ret_val.remove('source_id')
        ret_val.remove('doe')

        if not filter_common:
            ret_val.insert(0, 'doe')
            ret_val.insert(0, 'timestamp')
            ret_val.insert(0, 'source_id')
        return ret_val

    @classmethod
    def variable_columns(cls, filter_common=True):
        ret_val = [c for c in super(DatapointMixin, cls).variable_columns()
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
    source_id = Column(String, primary_key=True,
                       index=True, autoincrement=True)
    name = Column(String, nullable=False)
    doe = Column(DateTime, default=datetime.now)
    connection_str = Column(String, nullable=False)
    _variable_classes_str = Column(String, nullable=True)

    @classmethod
    def variable_names(cls, filter_common=True):
        ret_val = super(SourceMixin, cls).variable_names()
        ret_val.remove('source_id')
        ret_val.remove('name')
        ret_val.remove('doe')
        ret_val.remove('connection_str')
        if not filter_common:
            ret_val.insert(0, 'doe')
            ret_val.insert(0, 'connection_str')
            ret_val.insert(0, 'name')
            ret_val.insert(0, 'source_id')
        return ret_val

    @classmethod
    def variable_columns(cls, filter_common=True):
        ret_val = [c for c in super(SourceMixin, cls).variable_columns()
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
        self.variable_classes = [get_datapoint_class(cls_name)
                                 for cls_name in self._variable_classes_str
                                                     .split(';')]


@datapoint_class
class EcgV1Datapoint(DatapointMixin, Base):
    ecg_v1 = Column(Float)

    def __init__(self, timestamp, source_id, ecg_v1,
                 millisecond=None, **kwargs):
        super(EcgV1Datapoint, self).__init__(timestamp, source_id, millisecond)
        self.ecg_v1 = ecg_v1


@datapoint_class
class O2Datapoint(DatapointMixin, Base):
    o2 = Column(Float)

    def __init__(self, timestamp, source_id, o2,
                 millisecond=None, **kwargs):
        super(O2Datapoint, self).__init__(timestamp, source_id, millisecond)
        self.o2 = o2


@datapoint_class
class TemperatureDatapoint(DatapointMixin, Base):
    temperature = Column(Float)

    def __init__(self, timestamp, source_id, temperature,
                 millisecond=None, **kwargs):
        super(TemperatureDatapoint, self).__init__(timestamp,
                                                   source_id,
                                                   millisecond)
        self.temperature = temperature


@datapoint_class
class AirFlowDatapoint(DatapointMixin, Base):
    air_flow = Column(Float)

    def __init__(self, timestamp, source_id, air_flow,
                 millisecond=None, **kwargs):
        super(AirFlowDatapoint, self).__init__(timestamp,
                                               source_id,
                                               millisecond)
        self.air_flow = air_flow


@datapoint_class
class HeartRateDatapoint(DatapointMixin, Base):
    heart_rate = Column(Float)

    def __init__(self, timestamp, source_id, heart_rate,
                 millisecond=None, **kwargs):
        super(HeartRateDatapoint, self).__init__(timestamp,
                                                 source_id,
                                                 millisecond)
        self.heart_rate = heart_rate


@datapoint_class
class AccelerationDatapoint(DatapointMixin, Base):
    acc_x = Column(Float)
    acc_y = Column(Float)
    acc_z = Column(Float)
    acc_magn = Column(Float)

    def __init__(self, timestamp, source_id, acceleration=None,
                 acc_x=None, acc_y=None, acc_z=None, acc_magn=None,
                 millisecond=None, **kwargs):
        super(AccelerationDatapoint, self).__init__(timestamp,
                                                    source_id,
                                                    millisecond)
        if acceleration is not None:
            self.acc_x, self.acc_y, self.acc_z = acceleration
        elif acc_x is not None and acc_y is not None and acc_z is not None:
            self.acc_x = acc_x
            self.acc_y = acc_y
            self.acc_z = acc_z
        else:
            raise ValueError("Either acceleration or (acc_x, acc_y, acc_z) "
                             "must be provided")

        self.acc_magn = acc_magn
        if acc_magn is None:
            self.acc_magn = (self.acc_x ** 2 +
                             self.acc_y ** 2 +
                             self.acc_z ** 2) ** 0.5

    @classmethod
    def variable_names(cls, filter_common=True):
        var_names = (super(AccelerationDatapoint, cls)
                          .variable_names(filter_common))
        if filter_common:
            var_names.remove('acc_magn')
            var_names.insert(0, 'acc_magn')
        return var_names


class Alarm(DatapointMixin, Base):
    alarm_lvl = Column(Float, nullable=False)
    kind = Column(Enum('ecg_v1', 'ecg_v2', 'o2',
                       'temperature', 'air_flow', 'heart_rate',
                       'acc_x', 'acc_y', 'acc_z', 'acc_magn'),
                  primary_key=True)
    mixed_kind = Column(Enum('ecg_v1', 'ecg_v2', 'o2',
                             'temperature', 'air_flow', 'heart_rate',
                             'acc_x', 'acc_y', 'acc_z', 'acc_magn'),
                        nullable=True)
    sgmt_begin = Column(DateTime, nullable=False)
    sgmt_end = Column(DateTime, nullable=False)

    def __init__(self, alarm_lvl, sgmt_begin, sgmt_end,
                 source_id, kind, mixed_kind=None,
                 timestamp=None, millisecond=None):
        if not timestamp:
            timestamp = datetime.now()
        super(Alarm, self).__init__(timestamp, source_id, millisecond)
        self.alarm_lvl = alarm_lvl
        self.sgmt_begin = sgmt_begin
        self.sgmt_end = sgmt_end
        self.kind = kind
        self.mixed_kind = mixed_kind


class Suit(SourceMixin, Base):
    def __init__(self, suit_id, name, connection_str):
        super(Suit, self).__init__(suit_id, name, connection_str)
