from __future__ import division, print_function
from sqlalchemy import create_engine, Column, String
from sqlalchemy.ext.declarative import declarative_base

engine = create_engine('sqlite:///plan_actors.db', echo=True)
Base = declarative_base()

########################################################################


class PlanActors(Base):
    """"""
    __tablename__ = "plan_actors"

    address = Column(String, primary_key=True)
    type = Column(String)
    avail_start = Column(String)
    avail_end = Column(String)
    capabilities = Column(String)

# ----------------------------------------------------------------------
    def __init__(self, address, type, avail_start, avail_end, capabilities):
        """"""
        self.address = address
        self.type = type
        self.avail_start = avail_start
        self.avail_end = avail_end
        self.capabilities = capabilities


# create tables
Base.metadata.create_all(engine)
