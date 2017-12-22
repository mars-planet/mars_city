from __future__ import division, print_function
from datetime import datetime
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy import Column, DateTime, Integer, SmallInteger,\
    String, Float, create_engine, Sequence

engine = create_engine('sqlite:///tango.db', echo=False)
Base = declarative_base()

########################################################################

class Lookup(Base):
    __tablename__ = 'Lookup'
    timestamp = Column(DateTime)
    tango_addr = Column(String, primary_key=True)
    ip_addr = Column(String)
    ip_addr = Column(String)

    def __repr__(self):
        return ("<Lookup('%s', '%s' ,'%s')>"
                % (self.timestamp, self.tango_addr, self.ip_addr))

    def __init__(self, timestamp, tango_addr, ip_addr):
        self.timestamp = timestamp
        self.tango_addr = tango_addr
        self.ip_addr = ip_addr
# ----------------------------------------------------------------------------

class Attributes(Base):
    __tablename__ = 'Attributes'
    tango_addr = Column(String, primary_key=True)
    attr_name = Column(String, primary_key=True)
    attr_type = Column(String)

    def __repr__(self):
        return ("<Attributes('%s', '%s' ,'%s')>"
                % (self.tango_addr, self.attr_name, self.attr_type))

    def __init__(self, tango_addr, attr_name, attr_type):
        self.attr_name = attr_name 
        self.tango_addr = tango_addr
        self.attr_type = attr_type
# ----------------------------------------------------------------------------

def create_db():
    """Creates the databases"""
    Base.metadata.create_all(engine)

def main():
	create_db()

if __name__ == "__main__":
    main()