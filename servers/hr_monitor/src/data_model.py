from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy import Column, DateTime, Float
from numpy import sqrt, power
from datetime import datetime

Base = declarative_base()


class Alarm(Base):
    __tablename__ = 'alarms'

    timestamp = Column(DateTime, primary_key=True)
    alarm_lvl = Column(Float, nullable=False)
    sgmt_begin = Column(DateTime, nullable=False)
    sgmt_end = Column(DateTime, nullable=False)
    doe = Column(DateTime, default=datetime.now)

    def __init__(self, timestamp, alarm_lvl, sgmt_begin, sgmt_end):
        self.timestamp = timestamp
        self.alarm_lvl = alarm_lvl
        self.sgmt_begin = sgmt_begin
        self.sgmt_end = sgmt_end


    def __repr__(self):
        return ("<Alarm('%s','%s', '%s', '%s')>"
                % (self.timestamp, self.alarm_lvl,
                   self.sgmt_begin, self.sgmt_end))


class Datapoint(Base):
    __tablename__ = 'datapoints'

    timestamp = Column(DateTime, primary_key=True)
    hr = Column(Float)
    acc_x = Column(Float)
    acc_y = Column(Float)
    acc_z = Column(Float)
    acc_magn = Column(Float)
    doe = Column(DateTime, default=datetime.now)

    def __init__(self, timestamp, hr, acc_x, acc_y, acc_z):
        self.timestamp = timestamp
        self.hr = hr
        self.acc_x = acc_x
        self.acc_y = acc_y
        self.acc_z = acc_z
        self.acc_magn = sqrt(power(acc_x, 2) + power(acc_y, 2) + power(acc_z, 2))


    def __repr__(self):
        return ("<Datapoint('%s','%s','%s','%s','%s','%s')>"
                % (self.timestamp, self.hr,
                   self.acc_x, self.acc_y, self.acc_z, self.acc_magn))
