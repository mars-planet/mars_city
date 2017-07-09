from __future__ import absolute_import, division, print_function
from threading import Thread
import sys
import os
import json
import time
sys.path.insert(0, '../hexoskin_helper')
sys.path.insert(0, '../anomaly_detector')
import utility_helper as util
import resource_helper as resource
import anomaly_detector as ad
import anomaly_database_helper as db
import vt_helper as vth
import ConfigParser


__author__ = 'abhijith'


def atrial_fibrillation_helper(auth):
    '''
    Returns the rr_interval data in realtime.
            @param auth:		Authentication token
    '''
    recordID = resource.get_active_record_list(auth)[0]
    if recordID not in resource.get_active_record_list(auth):
        # record not updated in realtime.
        return -1

    AD = ad.AnomalyDetector()

    config = ConfigParser.RawConfigParser()
    dirname = os.path.dirname(os.path.realpath(__file__))
    cfg_filename = os.path.join(dirname,
                                '../anomaly_detector/anomaly_detector.cfg')
    config.read(cfg_filename)

    window_size = config.getint('Atrial Fibrillation', 'window_size')

    datatypes = [util.datatypes['rrinterval'][0],
                 util.datatypes['hr_quality'][0]]
    resource.AF_realtime(auth, recordID, AD.af_anomaly_detect,
                         window_size, datatypes)
    # Successfully finished. Astronaut docked.
    return 1


def ventricular_tachycardia_helper(auth):
    '''
    Returns the rr_interval data in realtime.
            @param auth:		Authentication token
    '''
    recordID = resource.get_active_record_list(auth)[0]
    if recordID not in resource.get_active_record_list(auth):
        # record not updated in realtime.
        return -1

    VTBD = vth.VTBeatDetector()

    datatypes = [util.raw_datatypes['ecg'][0],
                 util.datatypes['rrinterval'][0],
                 util.datatypes['rrintervalstatus'][0],
                 util.datatypes['heartrate'][0],
                 util.datatypes['hr_quality'][0]]
    # Call to get data
    th1 = Thread(target=resource.VT_realtime, args=[
                 auth, recordID, VTBD, datatypes])
    th1.start()
    # resource.VT_realtime(auth, recordID, VTBD, datatypes)

    # Call to add anomaly into the data base
    th2 = Thread(target=VTBD.ping_AD_dict)
    th2.start()
    # VTBD.ping_AD_dict()

    # Call to keep VT datastructure size under limit
    # time.sleep(120)
    # while(True):
    #     VTBD.delete_data()
    #     time.sleep(2)

    # Successfully finished. Astronaut docked.
    return 1


def get_user_name(auth):
    # Returns the JSON response string with authenticated user information
    user_info = util.account_info_helper(auth)
    user_info = json.loads(user_info.text)
    return user_info['objects'][0]['first_name']


def get_auth_token():
    # Returns the auth token to the tango device server
    return util.auth_login()


def get_rrecordid(auth):
    # Returns the real-time record id of the current session
    try:
        recordID = resource.get_active_record_list(auth)[0]
    except:
        return -1

    return recordID


def get_all_data(auth):
    # Returns the required data in real-time for GUI
    recordID = resource.get_active_record_list(auth)[0]
    if recordID not in resource.get_active_record_list(auth):
        # record not updated in realtime.
        return -1

    resource.get_all_data(auth, recordID, datatypes=[4113, 18])

def af_from_db():
    


def main(argv):
    auth = util.auth_login()
    # print(util.all_users(auth).text)
    # af = Thread(target=atrial_fibrillation_helper, args=[auth])
    # af.start()
    # vt = Thread(target=ventricular_tachycardia_helper, args=[auth])
    # vt.start()
    if argv[1] == 'af':
        atrial_fibrillation_helper(auth)
    elif argv[1] == 'vt':
        ventricular_tachycardia_helper(auth)
    elif argv[1] == 'data':
        resource.get_all_data(auth)


if __name__ == "__main__":
    main(sys.argv)
