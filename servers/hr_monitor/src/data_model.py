from __future__ import division

from datetime import datetime, timedelta

from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy import Column, DateTime, Float, String

Base = declarative_base()


class Datapoint(Base):
    __tablename__ = 'datapoints'
    timestamp = Column(DateTime, primary_key=True)
    millisecond = Column(Float, primary_key=True)
    hr = Column(Float)
    acc_x = Column(Float)
    acc_y = Column(Float)
    acc_z = Column(Float)
    acc_magn = Column(Float)
    doe = Column(DateTime, default=datetime.now)

    def __init__(self, timestamp, hr, acc_x, acc_y, acc_z, millisecond=None):
        if millisecond:
            self.timestamp = timestamp
            self.timestamp += timedelta(microseconds=millisecond * 1000)
        else:
            self.timestamp = timestamp
            self.millisecond = timestamp.microsecond / 1000
        self.hr = hr
        self.acc_x = acc_x
        self.acc_y = acc_y
        self.acc_z = acc_z
        self.acc_magn = (acc_x ** 2 + acc_y ** 2 + acc_z ** 2) ** 0.5

    def __repr__(self):
        return ("<Datapoint('%s','%s','%s','%s','%s','%s')>"
                % (self.timestamp, self.hr,
                   self.acc_x, self.acc_y, self.acc_z, self.acc_magn))


class Alarm(Base):
    __tablename__ = 'alarms'
    timestamp = Column(DateTime, primary_key=True)
    millisecond = Column(Float, primary_key=True)
    alarm_lvl = Column(Float, nullable=False)
    bitmp1 = Column(String(length=512), nullable=False)
    bitmp2 = Column(String(length=512), nullable=False)
    sgmt_begin = Column(DateTime, nullable=False)
    sgmt_end = Column(DateTime, nullable=False)

    def __init__(self, alarm_lvl, sgmt_begin, sgmt_end, bitmp1, bitmp2,
                 timestamp=None, millisecond=None):
        if not timestamp:
            self.timestamp = datetime.now()
            self.millisecond = self.timestamp.microsecond / 1000
        else:
            self.timestamp = timestamp
        if timestamp and millisecond:
            self.timestamp = timestamp
            self.timestamp += timedelta(microseconds=millisecond * 1000)
            self.millisecond = millisecond
        if not millisecond:
            self.millisecond = self.timestamp.microsecond / 1000
        self.alarm_lvl = alarm_lvl
        self.sgmt_begin = sgmt_begin
        self.sgmt_end = sgmt_end
        self.bitmp1 = bitmp1
        self.bitmp2 = bitmp2

    def __repr__(self):
        return ("<Alarm('%s','%s', '%s', '%s')>"
                % (self.timestamp, self.alarm_lvl,
                   self.sgmt_begin, self.sgmt_end))
