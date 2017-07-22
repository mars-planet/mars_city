from __future__ import division, print_function
import sys
sys.path.insert(0, '../health_monitor')
from data_model import AtrFibAlarms, VenTacAlarms, APCAlarms
from sqlalchemy import create_engine
from sqlalchemy.orm import sessionmaker
from datetime import datetime

# Connecting to the database
engine = create_engine('sqlite:///../health_monitor/anomalies.db', echo=False)
Session = sessionmaker(bind=engine)

#
#
# Anomaly ADD functions


def add_af(data):
    start_hexo_timestamp = data['start_hexo_timestamp']
    end_hexo_timestamp = data['end_hexo_timestamp']
    doe = datetime.now()
    num_of_NEC = data['num_of_NEC']
    data_reliability = data['data_reliability']
    window_size = data['window_size']

    # Create session
    s = Session()

    try:
        query = s.query(AtrFibAlarms).filter(
            AtrFibAlarms.start_hexo_timestamp.in_([start_hexo_timestamp]))
        result = query.first()

        if result:
            return -1
        else:
            af = AtrFibAlarms(start_hexo_timestamp, end_hexo_timestamp, doe,
                              num_of_NEC, data_reliability, window_size)
            s.add(af)

            # commit the record the database
            s.commit()
            print("Inserted row successfully")
            return 0

    except:
        s.rollback()
        return -1

    finally:
        s.close()


def add_vt(data):
    start_hexo_timestamp = data['start_hexo_timestamp']
    end_hexo_timestamp = data['end_hexo_timestamp']
    data_reliability = data['data_reliability']
    doe = datetime.now()

    # Create session
    s = Session()

    try:
        query = s.query(VenTacAlarms).filter(
            VenTacAlarms.start_hexo_timestamp.in_([start_hexo_timestamp]))
        result = query.first()

        if result:
            return -1
        else:
            vt = VenTacAlarms(start_hexo_timestamp, end_hexo_timestamp,
                              doe, data_reliability)
            s.add(vt)

            # commit the record the database
            s.commit()
            print("Inserted row successfully")
            return 0

    except:
        s.rollback()
        return -1

    finally:
        s.close()


def add_apc(data):
    RRPeak_hexo_timestamp = data['RRPeak_hexo_timestamp']
    RR_Quality = data['RR_Quality']
    PVC_from = data['PVC_from']
    doe = datetime.now()

    # Create session
    s = Session()

    try:
        query = s.query(APCAlarms).filter(
            APCAlarms.RRPeak_hexo_timestamp.in_([RRPeak_hexo_timestamp]))
        result = query.first()

        if result:
            return -1
        else:
            apc = APCAlarms(RRPeak_hexo_timestamp, RR_Quality,
                            doe, PVC_from)
            s.add(apc)

            # commit the record the database
            s.commit()
            print("Inserted row successfully")
            return 0

    except:
        s.rollback()
        return -1

    finally:
        s.close()

#
#
# Anomaly GET functions


def get_af():
    return_data = []

    s = Session()
    try:
        query = s.query(AtrFibAlarms)
        result = query.all()
        for data in result:
            _return = []
            _return.append(data.start_hexo_timestamp)
            _return.append(data.end_hexo_timestamp)
            _return.append(data.doe)
            _return.append(data.num_of_NEC)
            _return.append(data.data_reliability)
            _return.append(data.window_size)
            return_data.append(_return)

        s.close()
        return return_data
    except:
        return -1


def get_vt():
    return_data = []

    s = Session()
    try:
        query = s.query(VenTacAlarms)
        result = query.all()
        for data in result:
            return_data.append(data.start_hexo_timestamp)
            return_data.append(data.end_hexo_timestamp)
            return_data.append(data.doe)
            return_data.append(data.data_reliability)

        s.close()
        return return_data
    except:
        return -1


def get_apc():
    return_data = []

    s = Session()
    try:
        query = s.query(APCAlarms)
        result = query.all()
        for data in result:
            return_data.append(data.RRPeak_hexo_timestamp)
            return_data.append(data.RR_Quality)
            return_data.append(data.doe)
            return_data.append(data.PVC_from)

        s.close()
        return return_data
    except:
        return -1
