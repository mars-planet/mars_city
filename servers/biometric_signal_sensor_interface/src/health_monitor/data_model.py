from datetime import datetime

from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy import Column, DateTime, Integer, SmallInteger

Base = declarative_base()


# Add class Datapoint(Base)

class AtrFibAlarms(Base):
    __tablename__ = 'AtrFibAlarms'
    start_hexo_timestamp = Column(Integer, primary_key=True, nullable=False)
    end_hexo_timestamp = Column(Integer, nullable=False)
    doe = Column(DateTime, nullable=False, default=datetime.now)
    num_of_NEC = Column(SmallInteger, nullable=False)
    data_reliability = Column(SmallInteger, nullable=False)
    window_size = Column(SmallInteger, nullable=False, default=64)

    def __repr__(self):
        return ("<AtrFibAlarms('%s', '%s', '%s', '%s', '%s')>"
                % (self.start_hexo_timestamp, self.end_hexo_timestamp,
                   self.doe, self.num_of_NEC, self.data_reliability))


class VenTacAlarms(Base):
    __tablename__ = 'VenTacAlarms'
    start_hexo_timestamp = Column(Integer, primary_key=True, nullable=False)
    end_hexo_timestamp = Column(Integer, nullable=False)
    doe = Column(DateTime, nullable=False, default=datetime.now)
    data_reliability = Column(SmallInteger, nullable=False)

    def __repr__(self):
        return ("<AtrFibAlarms('%s', '%s', '%s', '%s')>"
                % (self.start_hexo_timestamp, self.end_hexo_timestamp,
                   self.doe, self.data_reliability))
