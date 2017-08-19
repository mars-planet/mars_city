from __future__ import absolute_import, division, print_function
from threading import Thread
import sys
import json
sys.path.insert(0, '../hexoskin_helper')
sys.path.insert(0, '../anomaly_detector')
import utility_helper as util
import resource_helper as resource
import anomaly_detector as ad
import anomaly_database_helper as db
import vt_helper as vth
import apc_pvc_helper as apc_pvc
import pvc_hamilton as pvc_h
import respiration_AD as respiration
import sleep_AD as sleepAD
import ConfigParser


__author__ = 'abhijith'


def atrial_fibrillation_helper(auth):
    '''
            @param auth:		Authentication token
    '''
    recordID = resource.get_active_record_list(auth)[0]
    if recordID not in resource.get_active_record_list(auth):
        # record not updated in realtime.
        return -1

    AD = ad.AnomalyDetector()

    config = ConfigParser.RawConfigParser()

    config.read('../anomaly_detector/anomaly_detector.cfg')

    window_size = config.getint('Atrial Fibrillation', 'window_size')

    datatypes = [util.datatypes['rrinterval'][0],
                 util.datatypes['hr_quality'][0]]
    resource.AF_realtime(auth, recordID, AD.af_anomaly_detect,
                         window_size, datatypes)
    # Successfully finished. Astronaut docked.
    return 1


def ventricular_tachycardia_helper(auth):
    '''
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


def _apc_pvc_helper(auth):
    '''
            @param auth:        Authentication token
    '''
    recordID = resource.get_active_record_list(auth)[0]
    if recordID not in resource.get_active_record_list(auth):
        # record not updated in realtime.
        return -1

    APC_PVC = apc_pvc.APC_helper()
    PVCH = pvc_h.PVC()

    datatypes = [util.raw_datatypes['ecg'][0],
                 util.datatypes['rrintervalstatus'][0]]
    # Call to get data
    th1 = Thread(target=resource.APC_PVC_realtime, args=[
                 auth, recordID, [APC_PVC, PVCH], datatypes])
    th1.start()

    th4 = Thread(target=APC_PVC.apcObj.delete_method, args=[])
    th4.start()

    th5 = Thread(target=PVCH.delete_method, args=[])
    th5.start()

    # Successfully finished. Astronaut docked.
    return 1


def resp_helper(auth):
    '''
            @param auth:        Authentication token
    '''
    recordID = resource.get_active_record_list(auth)[0]
    if recordID not in resource.get_active_record_list(auth):
        # record not updated in realtime.
        return -1

    config = ConfigParser.RawConfigParser()
    config.read('../anomaly_detector/anomaly_detector.cfg')

    datatypes = [util.datatypes['vt'][0],
                 util.datatypes['minuteventilation'][0],
                 util.raw_datatypes['resp'][0],
                 util.datatypes['breathingrate'][0],
                 util.datatypes['br_quality'][0],
                 util.datatypes['inspiration'][0],
                 util.datatypes['expiration'][0]]

    resource.resp_realtime(auth, recordID, "",
        datatypes)

    # Successfully finished. Astronaut docked.
    return 1


def sleep_helper(auth):
    '''
            @param auth:        Authentication token
    '''
    # recordID = resource.get_record_list(auth)[0]['id']
    # if recordID in resource.get_active_record_list(auth):
    #     # record should not updated in realtime.
    #     return -1
    recordID = 132638
    resource.sleep_ad(auth, recordID)
    sleepAD.initiate_sleepAD()


def get_user_info(auth):
    '''
            @param auth :       Authentication token
            @return :           JSON response string with authenticated
                                user information
    '''
    user_info = util.account_info_helper(auth)
    user_info = user_info.text
    return user_info


def get_auth_token():
    '''
        @return :               The auth token to the tango device server
    '''
    return util.auth_login()


def get_rrecordid(auth):
    '''
            @param auth :       Authentication token
            @return :           The real-time record id of the current session
    '''
    try:
        recordID = resource.get_active_record_list(auth)[0]
    except:
        return -1

    return recordID


def get_all_data(auth=""):
    '''
            @param auth :       Authentication token
            @return :           The real-time record id of the current session
    '''
    auth = util.auth_login()
    # Returns the required data in real-time for GUI
    recordID = resource.get_active_record_list(auth)[0]
    if recordID not in resource.get_active_record_list(auth):
        # record not updated in realtime.
        return -1

    datatypes = [4113, 18]
    data = resource.get_all_data(auth, recordID, datatypes=datatypes)

    return_data = {}

    for i in range(0, len(datatypes)):
        _data = []
        _data.append(data[datatypes[i]][0])
        _data.append(data[datatypes[i]][int(len(data[datatypes[i]]) / 2)])
        _data.append(data[datatypes[i]][len(data[datatypes[i]]) - 1])

        return_data[datatypes[i]] = _data

    return json.dumps(return_data)


def af_from_db():
    '''
            @return :           Retrieve AF AD data from DB
    '''
    data = db.get_af()
    return_json = {}
    for _data in data:
        _data[2] = _data[2].now().strftime('%Y-%m-%d %H:%M:%S')
        return_json[_data[0]] = _data[1:]

    return json.dumps((return_json))


def vt_from_db():
    '''
            @return :           Retrieve VT AD data from DB
    '''
    data = db.get_vt()
    return_json = {}
    for _data in data:
        _data[2] = _data[2].now().strftime('%Y-%m-%d %H:%M:%S')
        return_json[_data[0]] = _data[1:]

    return json.dumps((return_json))


def apc_from_db():
    '''
            @return :           Retrieve APC/PVC AD data from DB
    '''
    data = db.get_apc()
    # print(data, "DATA")
    return_json = {}
    for _data in data:
        _data[2] = _data[2].now().strftime('%Y-%m-%d %H:%M:%S')
        return_json[_data[0]] = _data[1:]

    return json.dumps((return_json))

def resp_from_db():
    '''
            @return :           Retrieve Respiration AD data from DB
    '''
    data = db.get_resp()
    # print(data, "DATA")
    return_json = {}
    for _data in data:
        _data[2] = _data[2].now().strftime('%Y-%m-%d %H:%M:%S')
        return_json[_data[0]] = _data[1:]

    return json.dumps((return_json))

def sleep_from_db():
    '''
            @return :           Retrieve Respiration AD data from DB
    '''
    data = db.get_sleep()
    # print(data, "DATA")
    return_json = {}
    for _data in data:
        _data[2] = _data[2].now().strftime('%Y-%m-%d %H:%M:%S')
        return_json[_data[0]] = _data[1:]

    return json.dumps((return_json))


def data_from_db():
    '''
            @return :           Retrieve Biometric data from DB
    '''
    data = db.get_data()
    return json.dumps((data))


def delete_from_db():
    '''
    Delete the biometric data database
    '''
    db.delete_data()


def main(argv):
    ''''
    Main function
    '''
    auth = util.auth_login()

    # For sleep detection
    # Will be called using a CRONJOB
    sleep_helper(auth)


if __name__ == "__main__":
    main(sys.argv)
