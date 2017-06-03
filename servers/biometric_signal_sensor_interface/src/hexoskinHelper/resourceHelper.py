from __future__ import absolute_import, division, print_function
import sys
import time
import calendar
import utilityHelper as util

__author__ = 'abhijith'

'''
biometric resourceHelper provides all methods that retrieves the biometric
data. This helper module is called by the Tango server module providing a
clear line of abstraction.
'''


def get_record_list_helper(auth, limit="100", user='', deviceFilter=''):
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


def get_active_record_list_helper(auth):
    '''
    Param auth token, userID

    Returns the records (record ID) that are currently in progress, i.e
    astronaut is still exploring and hasn't returned to the stationed
    and docked in yet.
    '''
    recordList = get_record_list_helper(auth)
    response = []

    for record in recordList:
        if(record['status'] == "realtime"):
            response.append(record['id'])

    return response


def get_record_info_helper(auth, recordID):
    '''
    Param: auth, recordID

    Returns the information regarding the passed recordID
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
    download of the different datatypes by itself
    returns a dictionary containing all datatypes in separate entries
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
                time.sleep(2)

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
    record = auth.api.record.get(recordID)

    start_epoch = calendar.timegm(time.gmtime())
    start_epoch = start_epoch - (60)
    start = (start_epoch) * 256

    end = int((start / 256) + 5) * 256

    while True:
        user = record.user
        data = get_realtime_data_helper(auth, user, start, end, datatypes)
        record = auth.api.record.get(recordID)
        start = end
        end = int((start / 256) + 5) * 256
        yield data
        time.sleep(3)

    return


def get_realtime_data(auth, recordID, datatypes):
    '''
    Param: auth token, record ID of the record/session and the datatype of the
    metric that needs to be measured.

    Runs till the record is active and returns the data of the user every 5
    seconds adding to the record.
    '''
    record = auth.api.record.get(recordID)
    if record.status != 'realtime':
        print("No realtime data available with this record. Already docked.")
        return

    for data in realtime_data_generator(auth, recordID, datatypes):
        if record.status == "complete":
            break
        print(data)


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
    print(get_active_record_list_helper(auth))
    get_realtime_data(auth, get_active_record_list_helper(auth)[0], [4113])


if __name__ == "__main__":
    main(sys.argv)
