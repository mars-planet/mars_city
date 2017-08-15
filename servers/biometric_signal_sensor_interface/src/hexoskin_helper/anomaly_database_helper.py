from __future__ import division, print_function
import sys
import time
sys.path.insert(0, '../health_monitor')
from data_model import AtrFibAlarms, VenTacAlarms, APCAlarms, Data, RespAlarms
from sqlalchemy import create_engine, func
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
            print("Inserted AF row successfully")
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
            print("Inserted VT row successfully")
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
            print("Inserted APC row successfully")
            return 0

    except:
        s.rollback()
        return -1

    finally:
        s.close()

def add_resp(data):
    Resp_hexo_timestamp = data['Resp_hexo_timestamp']
    BRstatus_mean = data['BRstatus_mean']
    Anomaly_type = data['Anomaly_type']
    doe = datetime.now()

    # Create session
    s = Session()

    try:
        query = s.query(RespAlarms).filter(
            RespAlarms.Resp_hexo_timestamp.in_([Resp_hexo_timestamp]))
        result = query.first()

        if result:
            return -1
        else:
            resp = RespAlarms(Resp_hexo_timestamp, BRstatus_mean,
                            doe, Anomaly_type)
            s.add(resp)

            # commit the record the database
            s.commit()
            print("Inserted Resp row successfully")
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
            _return = []
            _return.append(data.start_hexo_timestamp)
            _return.append(data.end_hexo_timestamp)
            _return.append(data.doe)
            _return.append(data.data_reliability)
            return_data.append(_return)

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
            _return = []
            _return.append(data.RRPeak_hexo_timestamp)
            _return.append(data.RR_Quality)
            _return.append(data.doe)
            _return.append(data.PVC_from)
            return_data.append(_return)

        s.close()
        return return_data
    except:
        return -1

def get_resp():
    return_data = []

    s = Session()
    try:
        query = s.query(RespAlarms)
        result = query.all()
        for data in result:
            _return = []
            _return.append(data.Resp_hexo_timestamp)
            _return.append(data.BRstatus_mean)
            _return.append(data.doe)
            _return.append(data.Anomaly_type)
            return_data.append(_return)

        s.close()
        return return_data
    except:
        return -1
       


#-------------------------------------------------------------------
def add_data(time, data, _type):
    hexo_timestamp = time
    data = data
    datatype = _type

    # Create session
    s = Session()

    try:
        query = s.query(Data).filter(
            Data.hexo_timestamp.in_([hexo_timestamp]))
        result = query.first()

        if result:
            return -1
        else:
            af = Data(hexo_timestamp, data,
                              datatype)
            s.add(af)

            # commit the record the database
            s.commit()
            # print("Inserted Data row successfully")
            return 0

    except:
        s.rollback()
        return -1

    finally:
        s.close()

def get_data():
    return_data = {
    '18':[], '4113': [], '1000': [], '19':[], '4129':[], '36':[],
    '37':[], '33':[], '1001':[], '34':[], '35':[]
    }

    s = Session()

    query = s.query(Data).order_by(Data.datatype)
    result = query.all()
    for data in result:
        _return = []
        _return.append(time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(int(data.hexo_timestamp/256))))
        _return.append(data.data)
        return_data[str(data.datatype)].append(_return)

    s.close()
    return return_data


def _delete_data():
    s = Session()
    if s.query(func.count(Data.hexo_timestamp)).scalar() > 1000:
        query = s.query(Data)
        query.delete()
        s.commit()
    s.close()

def delete_data():
    s = Session()
    query = s.query(Data)
    query.delete()
    s.commit()
    s.close()


# -------------------------------------------------------------


def main(argv):
    data = {}
    data['Resp_hexo_timestamp'] = 384686420480
    data['BRstatus_mean'] = 1
    data['Anomaly_type'] = "Boo ba"
    add_resp(data)

    data = {}
    data['Resp_hexo_timestamp'] = 384686420485
    data['BRstatus_mean'] = 1
    data['Anomaly_type'] = "Boo ba"
    add_resp(data)

    data = {}
    data['Resp_hexo_timestamp'] = 384686420490
    data['BRstatus_mean'] = 1
    data['Anomaly_type'] = "Boo ba"
    add_resp(data)


if __name__ == "__main__":
    main(sys.argv)