from __future__ import absolute_import, division, print_function
import sys

__author__ = 'abhijith'

'''
biometric resourceHelper provides all methods that retrieves the biometric
data. This helper module is called by the Tango server module providing a clear
line of abstraction.
'''


def getRecordListHelper(auth, user):
    '''
    Each astronaut session with the hexoskin is a record. Record starts when he
    plugs the device to his shirt, and stops when the device is plugged into
    the system when the astronaut returns to the station.
    '''
    raise NotImplementedError


def getActiveRecordListHelper(auth, user):
    '''
    Param auth token, userID

    Returns the records that are currently in progress, i.e astronaut is still
    exploring and hasn't returned to the stationed and docked in yet.
    '''
    return NotImplementedError


def getRecordInfoHelper(auth, recordID):
    '''
    Param: auth, recordID

    Returns the information regarding the passed recordID
    '''
    raise NotImplementedError


def getBiometricDataHelper(auth, recordID, datatype):
    '''
    Param: auth token, record ID of the record/session and the datatype of the
    metric that needs to be measured.

    Returns the complete data measured for the corresponding datatype of the
    record.
    '''
    return NotImplementedError


def getRealtimeBiometricDataHelper(auth, recordID, datatype):
    '''
    Param: auth token, record ID of the record/session and the datatype of the
    metric that needs to be measured.

    Runs till the record is active and returns the data of the user every 5
    seconds adding to the record.
    '''
    return NotImplementedError


def getMetricHelper(auth, recordID, datatype):
    '''
    Param: auth token, record ID of the record/session and the datatype of the
    metric that needs to be measured.

    Along with the actual data from the hexoskin, metrics are also available.
    Metrics are a way to summarize data in a single values or histograms.
    A few example of metrics are heart rate average, max activity, minimum
    breathing rate and so on.
    '''
    return NotImplementedError


def getGPSHelper(auth, userID):
    '''
    Param auth token, userID
    '''
    return NotImplementedError


def main(argv):
    # loginHelper()
    # getHexoskinDatatypes('dummy_auth')
    pass


if __name__ == "__main__":
    main(sys.argv)
