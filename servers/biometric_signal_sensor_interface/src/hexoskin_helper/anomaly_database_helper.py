from __future__ import division, print_function
import sys
import os
sys.path.insert(0, '../health_monitor')
from data_model import AtrFibAlarms
from sqlalchemy import create_engine
from sqlalchemy.orm import sessionmaker

# Connecting to the database
engine = create_engine('sqlite:///../health_monitor/anomalies.db', echo=False)
Session = sessionmaker(bind=engine)


def add_af(data):
    start_hexo_timestamp = data['start_hexo_timestamp']  
    end_hexo_timestamp = data['end_hexo_timestamp']  
    num_of_NEC = data['num_of_NEC']  
    data_reliability = data['data_reliability']  
    window_size = data['window_size']  

    # Create session
    s = Session()

    try:
        query = s.query(AtrFibAlarms).filter(AtrFibAlarms
            .start_hexo_timestamp.in_([start_hexo_timestamp]))
        result = query.first()

        if result:
            return -1
        else:
            af = AtrFibAlarms(start_hexo_timestamp, end_hexo_timestamp,
                num_of_NEC, data_reliability, window_size)
            s.add(af)

            # commit the record the database
            s.commit()
            print("Inserted row successfully")
            return 0

    except:
        s.rollback()
        return "das"

    finally:
        s.close()


# def get(address):
#     return_data = []

#     s = Session()
#     try:
#         query = s.query(PlanActors).filter(PlanActors.address.in_([address]))
#         result = query.first()
#         return_data.append(result.address)
#         return_data.append(result.actor_type)
#         return_data.append(result.avail_start)
#         return_data.append(result.avail_end)
#         return_data.append(result.capabilities)
#         s.close()
#         return return_data
#     except:
#         return -1
