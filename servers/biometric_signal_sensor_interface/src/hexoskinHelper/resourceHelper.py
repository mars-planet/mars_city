from __future__ import absolute_import, division, print_function
import sys, json
import utilityHelper as util

__author__ = 'abhijith'

'''
biometric resourceHelper provides all methods that retrieves the biometric
data. This helper module is called by the Tango server module providing a clear
line of abstraction.
'''


def get_record_list_helper():
    '''
    Each astronaut session with the hexoskin is a record. Record starts when he
    plugs the device to his shirt, and stops when the device is plugged into
    the system when the astronaut returns to the station.
    '''
    url = "https://api.hexoskin.com/api/record/"
    response = util.api_helper(url)

    return response


def get_active_record_list_helper():
    '''
    Param auth token, userID

    Returns the records (record ID) that are currently in progress, i.e astronaut is still
    exploring and hasn't returned to the stationed and docked in yet.
    '''
    recordList = get_record_list_helper()
    recordJSON = json.loads(str(recordList.text))
    response = []

    for record in recordJSON['objects']:
        if(record['status'] == "realtime"):
            response.append(record['id'])

    return response


def get_record_info_helper(recordID):
    '''
    Param: auth, recordID

    Returns the information regarding the passed recordID
    '''

    recordList = get_record_list_helper()
    recordJSON = json.loads(str(recordList.text))
    response = []

    for record in recordJSON['objects']:
        if(record['id'] == recordID):
            response.append(record)

    return response


def get_data_helper(recordID, datatypes):
    '''
    Param: auth token, record ID of the record/session and the datatype of the
    metric that needs to be measured.

    Returns the complete data measured for the corresponding datatype of the
    record.
    '''
    datatypes = ','.join(map(str, datatypes))
    url = "https://api.hexoskin.com/api/data/?datatype__in=" + datatypes + \
        "&record=" + str(recordID)
    response = util.api_helper(url)

    return response


def get_realtime_data_helper(recordID, datatype):
    '''
    Param: auth token, record ID of the record/session and the datatype of the
    metric that needs to be measured.

    Runs till the record is active and returns the data of the user every 5
    seconds adding to the record.
    '''
    active_records = get_active_record_list_helper()
    if recordID in active_records:
        return NotImplementedError


def get_metric_helper(recordID, datatype):
    '''
    Param: auth token, record ID of the record/session and the datatype of the
    metric that needs to be measured.

    Along with the actual data from the hexoskin, metrics are also available.
    Metrics are a way to summarize data in a single values or histograms.
    A few example of metrics are heart rate average, max activity, minimum
    breathing rate and so on.
    '''
    return NotImplementedError


def get_gps_helper(userID):
    '''
    Param auth token, userID
    '''
    return NotImplementedError


def main(argv):
    res = get_data_helper(125340, [19,33])
    print(res.text)


if __name__ == "__main__":
    main(sys.argv)
