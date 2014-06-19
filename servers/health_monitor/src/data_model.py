from __future__ import division

from datetime import datetime, timedelta
import re

from sqlalchemy import Column, DateTime, Enum, Float, Index, Integer, String
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.ext.declarative import declared_attr


def camel_to_underscore(name):
    s1 = re.sub('(.)([A-Z][a-z]+)', r'\1_\2', name)
    return re.sub('([a-z0-9])([A-Z])', r'\1_\2', s1).lower()


Base = declarative_base()


class TimestampSourceMixin(object):
    timestamp = Column(DateTime, primary_key=True)
    millisecond = Column(Float, primary_key=True)
    source_id = Column(Integer, primary_key=True)
    doe = Column(DateTime, default=datetime.now)

    @declared_attr
    @classmethod
    def __table_args__(cls):
        return (Index('idx_%s' % cls.__tablename__,
                      'timestamp', 'millisecond', 'source_id'),)

    @declared_attr
    @classmethod
    def __tablename__(cls):
        return camel_to_underscore(cls.__name__)

    def __init__(self, timestamp, source_id, millisecond=None):
        self.timestamp = timestamp
        if millisecond:
            self.timestamp -= timedelta(microseconds=timestamp.microsecond)
            self.timestamp += timedelta(microseconds=millisecond * 1000)
            self.millisecond = millisecond
        else:
            self.millisecond = timestamp.microsecond / 1000

    def __repr__(self):
        attr_repr = ', '.join('%s=%s' % (k, self.__dict__[k])
                                            for k in sorted(self.__dict__)
                                                if '_sa_' != k[:4])
        return ("<%s(%s)>"
                % (self.__class__.__name__, attr_repr))


class EcgV1Datapoint(TimestampSourceMixin, Base):
    ecg_v1 = Column(Float)

    def __init__(self, timestamp, source_id, ecg_v1, millisecond=None):
        super(EcgV1Datapoint, self).__init__(timestamp, source_id, millisecond)
        self.ecg_v1 = ecg_v1


class EcgV2Datapoint(TimestampSourceMixin, Base):
    ecg_v2 = Column(Float)

    def __init__(self, timestamp, source_id, ecg_v2, millisecond=None):
        super(EcgV2Datapoint, self).__init__(timestamp, source_id, millisecond)
        self.ecg_v2 = ecg_v2


class O2Datapoint(TimestampSourceMixin, Base):
    o2 = Column(Float)

    def __init__(self, timestamp, source_id, o2, millisecond=None):
        super(O2Datapoint, self).__init__(timestamp, source_id, millisecond)
        self.o2 = o2


class TemperatureDatapoint(TimestampSourceMixin, Base):
    temperature = Column(Float)

    def __init__(self, timestamp, source_id, temperature, millisecond=None):
        super(TemperatureDatapoint, self).__init__(timestamp,
                                                   source_id,
                                                   millisecond)
        self.temperature = temperature


class AirFlowDatapoint(TimestampSourceMixin, Base):
    air_flow = Column(Float)

    def __init__(self, timestamp, source_id, air_flow, millisecond=None):
        super(AirFlowDatapoint, self).__init__(timestamp,
                                               source_id,
                                               millisecond)
        self.air_flow = air_flow


class HeartRateDatapoint(TimestampSourceMixin, Base):
    heart_rate = Column(Float)

    def __init__(self, timestamp, source_id, heart_rate, millisecond=None):
        super(HeartRateDatapoint, self).__init__(timestamp,
                                                 source_id,
                                                 millisecond)
        self.heart_rate = heart_rate


class AccelerationDatapoint(TimestampSourceMixin, Base):
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


class Alarm(TimestampSourceMixin, Base):
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
            self.timestamp = datetime.now()
        else:
            self.timestamp = timestamp
        super(AccelerationDatapoint, self).__init__(timestamp,
                                                    source_id,
                                                    millisecond)
        self.alarm_lvl = alarm_lvl
        self.sgmt_begin = sgmt_begin
        self.sgmt_end = sgmt_end


class Suit(Base):
    suit_id = Column(Integer, primary_key=True)
    name = Column(String, nullable=False)
    connection_str = Column(String, nullable=False)
    doe = Column(DateTime, default=datetime.now)

    def __init__(self, suit_id, name, connection_str):
        self.suit_id = suit_id
        self.name = name
        self.connection_str = connection_str

    def __repr__(self):
        return ("<Suit('%s','%s','%s','%s')>"
                    % (self.suit_id, self.name, self.connection_str, self.doe))
