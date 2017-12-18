from __future__ import division, print_function
from data_model import Lookup, create_db
from sqlalchemy import create_engine, func
from sqlalchemy.orm import sessionmaker
from datetime import datetime

# Connecting to the database
engine = create_engine('sqlite:///tango.db', echo=True) #Change to False in production
Session = sessionmaker(bind=engine)

def create():
	create_db()

def delete():
	# Create session
    s = Session()
    s.query(Lookup).delete(synchronize_session=False)
    s.commit()

def add(ts, taddr, ipaddr):
    # Create session
    s = Session()

    try:
        query = s.query(Lookup).filter(
            Lookup.timestamp.in_([ts]))
        result = query.first()

        if result:
            return -1
        else:
            af = Lookup(ts, taddr, ipaddr)
            s.add(af)

            # commit the record the database
            s.commit()
            return 0

    except:
        s.rollback()
        return -1

    finally:
        s.close()

def get(tango_address):
	return_data = []

	s = Session()
	try:
	    query = s.query(Lookup).filter_by(tango_addr=tango_address)
	    result = query.all()
	    for data in result:
	        _return = []
	        _return.append(data.timestamp)
	        _return.append(data.tango_addr)
	        _return.append(data.ip_addr)
	        return_data.append(_return)

	    s.close()
	    return return_data
	except:
	    return -1

def update(tango_address, ts):
	s = Session()

	try:
		query = s.query(Lookup).filter_by(tango_addr=tango_address).first()
		query.timestamp = ts

		s.commit()
		s.close()
		return 0
	except:
		return -1