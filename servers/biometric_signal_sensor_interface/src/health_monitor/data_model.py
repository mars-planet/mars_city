from __future__ import division, print_function
from datetime import datetime
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy import Column, DateTime, Integer, SmallInteger, create_engine

engine = create_engine('sqlite:///anomalies.db', echo=False)
Base = declarative_base()

########################################################################


class AtrFibAlarms(Base):
    __tablename__ = 'AtrFibAlarms'
    start_hexo_timestamp = Column(Integer, primary_key=True, nullable=False)
    end_hexo_timestamp = Column(Integer, nullable=False)
    doe = Column(DateTime, nullable=False, default=datetime.now())
    num_of_NEC = Column(SmallInteger, nullable=False)
    data_reliability = Column(SmallInteger, nullable=False)
    window_size = Column(SmallInteger, nullable=False, default=64)

    def __repr__(self):
        return ("<AtrFibAlarms('%s', '%s', '%s', '%s', '%s')>"
                % (self.start_hexo_timestamp, self.end_hexo_timestamp,
                   self.doe, self.num_of_NEC, self.data_reliability))

    def __init__(self, start_hexo_timestamp, end_hexo_timestamp,
                 num_of_NEC, data_reliability, window_size):
        self.start_hexo_timestamp = start_hexo_timestamp
        self.end_hexo_timestamp = end_hexo_timestamp
        self.doe = datetime.now()
        self.num_of_NEC = num_of_NEC
        self.data_reliability = data_reliability
        self.window_size = window_size
# ----------------------------------------------------------------------------


class VenTacAlarms(Base):
    __tablename__ = 'VenTacAlarms'
    start_hexo_timestamp = Column(Integer, primary_key=True, nullable=False)
    end_hexo_timestamp = Column(Integer, nullable=False)
    doe = Column(DateTime, nullable=False, default=datetime.now())
    data_reliability = Column(SmallInteger, nullable=False)

    def __repr__(self):
        return ("<AtrFibAlarms('%s', '%s', '%s', '%s')>"
                % (self.start_hexo_timestamp, self.end_hexo_timestamp,
                   self.doe, self.data_reliability))

    def __init__(self, start_hexo_timestamp, end_hexo_timestamp,
                 data_reliability):
        self.start_hexo_timestamp = start_hexo_timestamp
        self.end_hexo_timestamp = end_hexo_timestamp
        self.doe = datetime.now()
        self.data_reliability = data_reliability

class APCAlarms(Base):
    __tablename__ = 'VenTacAlarms'
    RRPeak_hexo_timestamp = Column(Integer, primary_key=True, nullable=False)
    RR_Quality = Column(SmallInteger, nullable=False)
    doe = Column(DateTime, nullable=False, default=datetime.now())
    PVC_from = Column(SmallInteger, nullable=False)

    def __repr__(self):
        return ("<AtrFibAlarms('%s', '%s', '%s', '%s')>"
                % (self.RRPeak_hexo_timestamp, self.RR_Quality,
                   self.doe, self.PVC_from))

    def __init__(self, start_hexo_timestamp, end_hexo_timestamp,
                 data_reliability):
        self.RRPeak_hexo_timestamp = RRPeak_hexo_timestamp
        self.RR_Quality = RR_Quality
        self.doe = datetime.now()
        self.PVC_from = PVC_from

# create tables
Base.metadata.create_all(engine)
