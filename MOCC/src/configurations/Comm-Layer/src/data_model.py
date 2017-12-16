from __future__ import division, print_function
from datetime import datetime
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy import Column, DateTime, Integer, SmallInteger,\
    String, Float, create_engine

engine = create_engine('sqlite:///tango.db', echo=False)
Base = declarative_base()

########################################################################

class Lookup(Base):
    __tablename__ = 'Lookup'
    timestamp = Column(DateTime)
    tango_addr = Column(String, primary_key=True)
    ip_addr = Column(String)

    def __repr__(self):
        return ("<Data('%s', '%s' ,'%s')>"
                % (self.timestamp, self.tango_addr, self.ip_addr))

    def __init__(self, timestamp, tango_addr, ip_addr):
        self.timestamp = timestamp
        self.tango_addr = tango_addr
        self.ip_addr = ip_addr
# ----------------------------------------------------------------------------

# create tables
def create_db():
	Base.metadata.create_all(engine)

def main():
	create_db()

if __name__ == "__main__":
    main(sys.argv)