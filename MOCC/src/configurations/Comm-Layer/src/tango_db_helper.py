from __future__ import division, print_function
from data_model import Lookup, Attributes, Commands, create_db
from sqlalchemy import create_engine, func
from sqlalchemy.orm import sessionmaker
from datetime import datetime

# Connecting to the database
engine = create_engine('sqlite:///tango.db', echo=True) #Change to False in production
Session = sessionmaker(bind=engine)

def create():
	"""Facilitates creation of the database."""
	create_db()

def delete():
	"""Deletes the contents of the database."""
	# Create session
	s = Session()
	s.query(Lookup).delete(synchronize_session=False)
	s.query(Attributes).delete(synchronize_session=False)
	s.query(Commands).delete(synchronize_session=False)
	s.commit()

def add(ts, taddr, ipaddr):
	"""Function to add an entry into tango database
	---
    tags:
      - add
    parameters:
      - name: ts
        in: timestamp
        type: string
        required: true
      - name: taddr
        in: tango address
        type: string
        required: true
      - name: ipaddr
        in: ip address
        type: string
        required: true
    responses:
        0:	Successful addition of entry into the database.
        -1: Failure. 
    """
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
	"""Function to retrieve an entry from the tango database
	---
    tags:
      - get
    parameters:
      - name: tango_address
        in: tango address
        type: string
        required: true
    responses:
        return_data:	Returns entry corresponding to the tango_address.
        -1: Failure. 
    """
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
	"""Function to update an entry of tango database
	---
    tags:
      - update
    parameters:
      - name: tango_address
        in: tango address
        type: string
        required: true
      - name: ts
        in: timestamp
        type: string
        required: true
    responses:
        0:	Successful updation of entry into the database.
        -1: Failure. 
    """
	s = Session()

	try:
		query = s.query(Lookup).filter_by(tango_addr=tango_address).first()
		query.timestamp = ts

		s.commit()
		s.close()
		return 0
	except:
		return -1

def add_attr(taddr, attr_name, attr_type):
	"""Function to add an attribute of a device into tango database
	---
    tags:
      - add_attr
    parameters:
      - name: taddr
        in: tango address
        type: string
        required: true
      - name: attr_name
        in: attribute name
        type: string
        required: true
      - name: attr_type
        in: attribute type
        type: string
        required: true
    responses:
        0:	Successful addition of entry into the database.
        -1: Failure. 
    """
    # Create session
	s = Session()

	try:
	    query = s.query(Attributes).filter(
	        Attributes.tango_addr.in_([taddr])).filter(
	        Attributes.attr_name.in_([attr_name]))
	    result = query.first()

	    if result:
	        return -1
	    else:
	        af = Attributes(taddr, attr_name, attr_type)
	        s.add(af)

	        # commit the record the database
	        s.commit()
	        return 0

	except:
	    s.rollback()
	    return -1

	finally:
		s.close()

def add_command(taddr, cmd_name, cmd_params):
	"""Function to add an attribute of a device into tango database
	---
    tags:
      - add_attr
    parameters:
      - name: taddr
        in: tango address
        type: string
        required: true
      - name: cmd_name
        in: command name
        type: string
        required: true
      - name: cmd_params
        in: command parameters
        type: string
        required: true
    responses:
        0:	Successful addition of entry into the database.
        -1: Failure. 
    """
    # Create session
	s = Session()

	try:
	    query = s.query(Commands).filter(
	        Commands.tango_addr.in_([taddr])).filter(
	        Commands.cmd_name.in_([cmd_name]))
	    result = query.first()

	    if result:
	        return -1
	    else:
	        af = Commands(taddr, cmd_name, cmd_params)
	        s.add(af)

	        # commit the record the database
	        s.commit()
	        return 0

	except:
	    s.rollback()
	    return -1

	finally:
		s.close()
