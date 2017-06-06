from __future__ import absolute_import, division, print_function
import sys
import time
import json
import calendar
import utilityHelper as util
import pandas as pd
import numpy as np

__author__ = 'abhijith'

'''
biometric resourceHelper provides all methods that retrieves the biometric
data. This helper module is called by the Tango server module providing a
clear line of abstraction.
'''


def get_record_list(auth, limit="100", user='', deviceFilter=''):
    '''
    Each astronaut session with the hexoskin is a record. Record starts when
    he plugs the device to his shirt, and stops when the device is plugged
    into the system when the astronaut returns to the station.

    Returns the results records corresponding to the selected filters
        @param auth:            The authentication token to use for the call
        @param limit:           The limit of results to return. Passing 0 returns
                                all the records
        @param userFilter:      The ID of the user to look for
        @param deviceFilter:    The device ID to look for. Takes the form
                                HXSKIN12XXXXXXXX, where XXXXXXXX is the
                                0-padded serial number. Example : HXSKIN1200001234
        @return :               The record list
    '''
    filters = dict()
    if limit != "100":
        filters['limit'] = limit
    if user != '':
        filters['user'] = user
    if deviceFilter != '':
        filters['device'] = deviceFilter
    out = auth.api.record.list(filters)
    return out.response.result['objects']


def get_active_record_list(auth, limit="100", user='', deviceFilter=''):
    '''
    Returns the results records that are measuring in realtime corresponding
    to the selected filters
        @param auth:            The authentication token to use for the call
        @param limit:           The limit of results to return. Passing 0 returns
                                all the records
        @param userFilter:      The ID of the user to look for
        @param deviceFilter:    The device ID to look for. Takes the form
                                HXSKIN12XXXXXXXX, where XXXXXXXX is the
                                0-padded serial number. Example : HXSKIN1200001234
        @return :               The realtime measuring record list
    '''
    recordList = get_record_list_helper(auth)
    response = []

    for record in recordList:
        if(record['status'] == "realtime"):
            response.append(record['id'])

    return response


def get_record_info_helper(auth, recordID):
    '''
    Returns the information regarding the passed recordID
        @param auth:        The authentication token to use for the call
        @param recordID:    Record ID of record
        @return :           Returns the information regarding the passed
                            recordID
    '''

    recordList = get_record_list_helper(auth)
    response = []

    for record in recordList:
        if(record['id'] == recordID):
            response.append(record)

    return response


def get_record_data(auth, recordID, downloadRaw=True):
    """
    This function allows you to specify a record, and it will manage the
    retrieval of the different datatypes by itself
        @param auth:        The authentication token to use for the call
        @param recordID:    Record ID of record
        @return :           returns a dictionary containing all datatypes
                            in separate entries
    """
    record = auth.api.record.get(recordID)
    final_dat = get_data(auth=auth, recordID=recordID,
                         downloadRaw=downloadRaw)
    final_dat['info'] = record.fields
    return final_dat


def get_data(auth, recordID, start='', end='', datatypes='',
             downloadRaw=True):
    """
    This function fetches the specified data range. Called by getRangeData
    and getRecordData
        @param auth:        The authentication token to use for the call
        @param recordID:    Record ID of record
        @param start:       Start timestamp for data to be fetched
        @param end:         End timestamp for data to be fetched
        @param datatypes:   Datatypes to be fetched
                            If not passed, all raw_datatypes (based on next
                            param value) and other datatypes downloaded
        @param downloadRaw: Flag to fetch raw-datatypes also
        @return :           returns a dictionary containing all datatypes
                            in separate entries
    """
    record = auth.api.record.get(recordID)
    if start == '':
        start = record.start
    if end == '':
        end = record.end

    final_dat = {}
    if datatypes != '':
        for dataID in datatypes:
            raw_flag = 0
            d_items = util.datatypes.items()
            key = [d_key for d_key, value in d_items if value == [
                dataID]]
            if not key:
                d_items = util.raw_datatypes.items()
                key = [d_key for d_key, value in d_items if value == [dataID]]
                raw_flag = 1
            print("Downloading " + key[0])
            if raw_flag:
                data = get_unsubsampled_data_helper(
                    auth=auth, userID=record.user, start=start, end=end,
                    dataID=util.raw_datatypes[key[0]])
            else:
                data = get_unsubsampled_data_helper(
                    auth=auth, userID=record.user, start=start, end=end,
                    dataID=util.datatypes[key[0]])
            final_dat[dataID] = data
    else:
        if downloadRaw is True:
            for rawID in util.raw_datatypes:
                print("Downloading" + rawID)
                raw_dat = get_unsubsampled_data_helper(
                    auth=auth, userID=record.user, start=record.start,
                    end=record.end, dataID=util.raw_datatypes[rawID])
                final_dat[rawID] = raw_dat
        for dataID in util.datatypes:
            print("Downloading " + dataID)
            data = get_unsubsampled_data_helper(
                auth=auth, userID=record.user, start=record.start,
                end=record.end, dataID=util.datatypes[dataID])
            final_dat[dataID] = data
    return final_dat


def get_unsubsampled_data_helper(auth, userID, start, end, dataID):
    """
    All data comes in subsampled form if the number of samples
    exceeds 65535. If this is the case, fetch data
    page by page to prevent getting subsampled data.
        @param auth:        The authentication token to use for the call
        @param userID:      userID ID of record
        @param start:       Start timestamp for data to be fetched
        @param end:         End timestamp for data to be fetched
        @param datatypes:   Datatypes to be fetched
        @return :           returns a dictionary containing all datatypes
                            in separate entries
    """
    out = []
    # Number of ticks between each sample
    datSampRate = util.dataSampleRate[dataID[0]]
    if datSampRate != []:
        # Number of ticks max size not to overflow 65535 max size
        sampPerIter = 65535 * datSampRate
        a = start
        b = min(end, a + sampPerIter)
        while a < end:
            dat = auth.api.data.list(
                start=a, end=b, user=userID, datatype=dataID)
            try:
                if len(dat.response.result[0]['data'].values()[0]) > 0:
                    ts = zip(*dat.response.result[0][u'data'].values()[0])[0]
                    data = [zip(*dat.response.result[0][u'data'][str(v)])[1]
                            for v in dataID]
                    out.extend(zip(ts, *data))
            except:
                out.extend(zip([], []))

            a = min(a + sampPerIter, end)
            b = min(b + sampPerIter, end)
            # Rate limiting to stay below 5 requests per second
            time.sleep(0.2)
    else:
        dat = auth.api.data.list(start=start, end=end,
                                 user=userID, datatype=dataID)
        if dat.response.result != []:
            out.extend(dat.response.result[0]['data'][str(dataID[0])])
    out = util.convertTimestamps(out, util.TIMESTAMP)
    return out


def get_realtime_data_helper(auth, user, start, end, datatypes):
    """
    This function fetches the specified data range. Called by
    realtime_data_generator() page by page to prevent getting subsampled data.
        @param auth:        The authentication token to use for the call
        @param userID:      userID ID of record
        @param start:       Start timestamp for data to be fetched
        @param end:         End timestamp for data to be fetched
        @param datatypes:   Datatypes to be fetched
        @return :           returns a dictionary containing all datatypes in
                            separate entries
    """
    final_dat = {}

    for dataID in datatypes:
        raw_flag = 0
        key = [d_key for d_key, value in util.datatypes.items() if value == [
            dataID]]
        if not key:
            data_items = util.raw_datatypes.items()
            key = [d_key for d_key, value in data_items if value == [dataID]]
            raw_flag = 1

        if raw_flag:
            data = get_unsubsampled_data_helper(
                auth=auth, userID=user, start=start, end=end,
                dataID=util.raw_datatypes[key[0]])
        else:
            data = get_unsubsampled_data_helper(
                auth=auth, userID=user, start=start, end=end,
                dataID=util.datatypes[key[0]])
        final_dat[dataID] = data
    return final_dat


def realtime_data_generator(auth, recordID, datatypes):
    '''
    A generator function that yields the realtime data to get_realtime_data()
        @param auth:        The authentication token to use for the call
        @param recordID:    recordID ID of record
        @param datatypes:   Datatypes to be fetched
        @return :           returns a dictionary containing all datatypes in
                            separate entries
    '''
    record = auth.api.record.get(recordID)

    start_epoch = calendar.timegm(time.gmtime())
    start_epoch = start_epoch - (10)
    start = (start_epoch) * 256

    end = (calendar.timegm(time.gmtime()) - 5) * 256

    while True:
        user = record.user
        data = get_realtime_data_helper(auth, user, start, end, datatypes)
        record = auth.api.record.get(recordID)
        start = end
        end = (calendar.timegm(time.gmtime()) - 5) * 256
        time.sleep(2)
        yield data

    return


def get_realtime_data(auth, recordID, datatypes):
    '''
    Param: auth token, record ID of the record/session and the datatype of the
    metric that needs to be measured.

    
    This function fetches the specified data range. Called by
    realtime_data_generator() page by page to prevent getting subsampled data.
        @param auth:        The authentication token to use for the call
        @param recordID:    recordID ID of record
        @param datatypes:   Datatypes to be fetched
        @return :           Runs till the record is active and returns the
                            data of the user regularly, adding to the record.
                            -1, if data not being collected in realtime
    '''
    record = auth.api.record.get(recordID)
    if record.status != 'realtime':
        print("No realtime data available with this record. Already docked.")
        return -1

    exitCounter = 5
    for data in realtime_data_generator(auth, recordID, datatypes):
        if len(data[datatypes[0]]) == 0:
            exitCounter = exitCounter - 5
            if exitCounter == 0:
                break
        else:
            exitCounter = 5
            hTimestamp = []
            value = []
            for a in data[datatypes[0]]:
                hTimestamp.append(a[0])
                value.append(a[1])
            # Pandas Dataframe
            df = pd.DataFrame(np.column_stack([hTimestamp, value]))

            # Call anomaly detection endpoint here
            print(df.head())
    return


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
    auth = util.auth_login()
    #print(json.loads(str())
    # print(get_active_record_list(auth))
    #get_realtime_data(auth, 125340, [18])


if __name__ == "__main__":
    main(sys.argv)
