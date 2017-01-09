from sqlalchemy import Column, ForeignKey, String
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.orm import relationship
from sqlalchemy import create_engine
from sqlalchemy.dialects.mysql import TIMESTAMP, BOOLEAN

Base = declarative_base()

class Category(Base):
    __tablename__ = 'Categories'
    Name = Column(String(255), nullable=False, primary_key=True)
    plans = relationship("Plan", backref="Categories")
    configs = relationship("Config", backref="Categories")
    scripts = relationship("Script", backref="Categories")

class Plan(Base):
    __tablename__ = 'Plans'
    Name = Column(String(255), nullable=False, primary_key=True)
    Path = Column(String(255), nullable=False)
    MongoDBid = Column(String(255), nullable=False)
    Category = Column(String(255), ForeignKey('Categories.Name'), nullable=False)
    Last_Modified = Column(TIMESTAMP, nullable=False)
    Last_Retrieved = Column(TIMESTAMP, nullable=False, default='0000-00-00 00:00:00')
    Validity = Column(BOOLEAN, nullable=False)
    Is_ple = Column(BOOLEAN, nullable=False)

class Config(Base):
    __tablename__ = 'Configs'
    Name = Column(String(255), nullable=False, primary_key=True)
    Path = Column(String(255), nullable=False)
    Category = Column(String(255), ForeignKey('Categories.Name'), nullable=False)
    Validity = Column(BOOLEAN, nullable=False)

class Script(Base):
    __tablename__ = 'Scripts'
    Name = Column(String(255), nullable=False, primary_key=True)
    Path = Column(String(255), nullable=False)
    Category = Column(String(255), ForeignKey('Categories.Name'), nullable=False)
    Validity = Column(BOOLEAN, nullable=False)
    Is_pst = Column(BOOLEAN, nullable=False)

engine = create_engine('mysql+pymysql://root:@localhost:3306/PlexilDatabase')

Base.metadata.create_all(engine)