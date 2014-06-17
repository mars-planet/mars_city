from __future__ import division

from datetime import datetime, timedelta

from sqlalchemy import Column, DateTime, Float, String, Enum
from sqlalchemy.ext.declarative import declarative_base


Base = declarative_base()


class EcgV1Datapoint(Base):
    __tablename__ = 'ecg_v1_datapoints'
    timestamp = Column(DateTime, primary_key=True)
    millisecond = Column(Float, primary_key=True)
    ecg_v1 = Column(Float)
    doe = Column(DateTime, default=datetime.now)

    def __init__(self, timestamp, ecg_v1, millisecond=None):
        self.timestamp = timestamp
        if millisecond:
            self.timestamp -= timedelta(microseconds=timestamp.microsecond)
            self.timestamp += timedelta(microseconds=millisecond * 1000)
            self.millisecond = millisecond
        else:
            self.millisecond = timestamp.microsecond / 1000
        self.ecg_v1 = ecg_v1

    def __repr__(self):
        return ("<EcgV1Datapoint('%s','%s')>" % (self.timestamp, self.ecg_v1))


class EcgV2Datapoint(Base):
    __tablename__ = 'ecg_v2_datapoints'
    timestamp = Column(DateTime, primary_key=True)
    millisecond = Column(Float, primary_key=True)
    ecg_v2 = Column(Float)
    doe = Column(DateTime, default=datetime.now)

    def __init__(self, timestamp, ecg_v2, millisecond=None):
        self.timestamp = timestamp
        if millisecond:
            self.timestamp -= timedelta(microseconds=timestamp.microsecond)
            self.timestamp += timedelta(microseconds=millisecond * 1000)
            self.millisecond = millisecond
        else:
            self.millisecond = timestamp.microsecond / 1000
        self.ecg_v2 = ecg_v2

    def __repr__(self):
        return ("<EcgV2Datapoint('%s','%s')>" % (self.timestamp, self.ecg_v2))


class O2Datapoint(Base):
    __tablename__ = 'o2_datapoints'
    timestamp = Column(DateTime, primary_key=True)
    millisecond = Column(Float, primary_key=True)
    o2 = Column(Float)
    doe = Column(DateTime, default=datetime.now)

    def __init__(self, timestamp, o2, millisecond=None):
        self.timestamp = timestamp
        if millisecond:
            self.timestamp -= timedelta(microseconds=timestamp.microsecond)
            self.timestamp += timedelta(microseconds=millisecond * 1000)
            self.millisecond = millisecond
        else:
            self.millisecond = timestamp.microsecond / 1000
        self.o2 = o2

    def __repr__(self):
        return ("<O2Datapoint('%s','%s')>" % (self.timestamp, self.o2))


class TemperatureDatapoint(Base):
    __tablename__ = 'temperature_datapoints'
    timestamp = Column(DateTime, primary_key=True)
    millisecond = Column(Float, primary_key=True)
    temperature = Column(Float)
    doe = Column(DateTime, default=datetime.now)

    def __init__(self, timestamp, temperature, millisecond=None):
        self.timestamp = timestamp
        if millisecond:
            self.timestamp -= timedelta(microseconds=timestamp.microsecond)
            self.timestamp += timedelta(microseconds=millisecond * 1000)
            self.millisecond = millisecond
        else:
            self.millisecond = timestamp.microsecond / 1000
        self.temperature = temperature

    def __repr__(self):
        return ("<TemperatureDatapoint('%s','%s')>"
                % (self.timestamp, self.temperature))


class AirFlowDatapoint(Base):
    __tablename__ = 'air_flow_datapoints'
    timestamp = Column(DateTime, primary_key=True)
    millisecond = Column(Float, primary_key=True)
    air_flow = Column(Float)
    doe = Column(DateTime, default=datetime.now)

    def __init__(self, timestamp, air_flow, millisecond=None):
        self.timestamp = timestamp
        if millisecond:
            self.timestamp -= timedelta(microseconds=timestamp.microsecond)
            self.timestamp += timedelta(microseconds=millisecond * 1000)
            self.millisecond = millisecond
        else:
            self.millisecond = timestamp.microsecond / 1000
        self.air_flow = air_flow

    def __repr__(self):
        return ("<AirFlowDatapoint('%s','%s')>"
                % (self.timestamp, self.air_flow))


class HeartRateDatapoint(Base):
    __tablename__ = 'heart_rate_datapoints'
    timestamp = Column(DateTime, primary_key=True)
    millisecond = Column(Float, primary_key=True)
    heart_rate = Column(Float)
    doe = Column(DateTime, default=datetime.now)

    def __init__(self, timestamp, heart_rate, millisecond=None):
        self.timestamp = timestamp
        if millisecond:
            self.timestamp -= timedelta(microseconds=timestamp.microsecond)
            self.timestamp += timedelta(microseconds=millisecond * 1000)
            self.millisecond = millisecond
        else:
            self.millisecond = timestamp.microsecond / 1000
        self.heart_rate = heart_rate

    def __repr__(self):
        return ("<HeartRateDatapoint('%s','%s')>"
                % (self.timestamp, self.heart_rate))


class AccelerationDatapoint(Base):
    __tablename__ = 'datapoints'
    timestamp = Column(DateTime, primary_key=True)
    millisecond = Column(Float, primary_key=True)
    acc_x = Column(Float)
    acc_y = Column(Float)
    acc_z = Column(Float)
    acc_magn = Column(Float)
    doe = Column(DateTime, default=datetime.now)

    def __init__(self, timestamp, acc_x, acc_y, acc_z, millisecond=None):
        self.timestamp = timestamp
        if millisecond:
            self.timestamp -= timedelta(microseconds=timestamp.microsecond)
            self.timestamp += timedelta(microseconds=millisecond * 1000)
            self.millisecond = millisecond
        else:
            self.millisecond = timestamp.microsecond / 1000
        self.acc_x = acc_x
        self.acc_y = acc_y
        self.acc_z = acc_z
        self.acc_magn = (acc_x ** 2 + acc_y ** 2 + acc_z ** 2) ** 0.5

    def __repr__(self):
        return ("<AccelerationDatapoint('%s','%s','%s','%s','%s')>"
                % (self.timestamp,
                   self.acc_x, self.acc_y, self.acc_z, self.acc_magn))


class Alarm(Base):
    __tablename__ = 'alarms'
    timestamp = Column(DateTime, primary_key=True)
    millisecond = Column(Float, primary_key=True)
    alarm_lvl = Column(Float, nullable=False)
    kind = Column(Enum('ecg_v1', 'ecg_v2', 'o2',
                       'temperature', 'air_flow', 'hr',
                       'acc_x', 'acc_y', 'acc_z', 'acc_magn'),
                  nullable=False)
    mixed_kind = Column(Enum('ecg_v1', 'ecg_v2', 'o2',
                             'temperature', 'air_flow', 'hr',
                             'acc_x', 'acc_y', 'acc_z', 'acc_magn'),
                        nullable=True)
    bitmp1 = Column(String(length=512), nullable=False)
    bitmp2 = Column(String(length=512), nullable=False)
    sgmt_begin = Column(DateTime, nullable=False)
    sgmt_end = Column(DateTime, nullable=False)

    def __init__(self, alarm_lvl, sgmt_begin, sgmt_end, bitmp1, bitmp2,
                 timestamp=None, millisecond=None):
        if not timestamp:
            self.timestamp = datetime.now()
        else:
            self.timestamp = timestamp
        ts = self.timestamp
        if millisecond:
            self.timestamp -= timedelta(microseconds=ts.microsecond)
            self.timestamp += timedelta(microseconds=millisecond * 1000)
            self.millisecond = millisecond
        else:
            self.millisecond = ts.microsecond / 1000
        self.alarm_lvl = alarm_lvl
        self.bitmp1 = bitmp1
        self.bitmp2 = bitmp2
        self.sgmt_begin = sgmt_begin
        self.sgmt_end = sgmt_end

    def __repr__(self):
        return ("<Alarm('%s','%s', '%s', '%s')>"
                % (self.timestamp, self.alarm_lvl,
                   self.sgmt_begin, self.sgmt_end))
