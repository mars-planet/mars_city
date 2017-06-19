from __future__ import absolute_import, division, print_function
import sys
import os
sys.path.insert(0, '../hexoskin_helper')
sys.path.insert(0, '../anomaly_detector')
import utility_helper as util
import resource_helper as resource
import anomaly_detector as ad
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

	resource.get_realtime_data(auth, recordID, util.datatypes['hr_quality'])
	
	VTBD = VTBeatDetector()

	datatypes = [util.datatypes['ecg'][0],
	             util.datatypes['rrinterval'][0],
	             util.datatypes['rrintervalstatus'][0]]

	#Call to get data
	resource.get_realtime_data(auth, recordID, VTBD.collect_data,
	                           datatypes)

	#VTBD.delete_data()

	#VTBD.ping_AD_dict()

	# Successfully finished. Astronaut docked.
	return 1


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
	values accepted as datatypes:
	'acc': [4145, 4146, 4147],
	    'ecg': [4113],
	    'resp': [4129, 4130]
	'activity': [49],
	'cadence': [53],
	    'heartrate': [19],
	    'minuteventilation': [36],
	    'vt': [37],
	    'breathingrate': [33],
	    'hr_quality': [1000],
	    'br_quality': [1001],
	    'inspiration': [34],
	    'expiration': [35],
	    'batt': [247],
	    'step': [52],
	    'rrinterval': [18],
	    'qrs': [22],
	"""
    return resource.get_data(auth, recordID, start, end, datatypes,
                             downloadRaw)


def main(argv):
    auth = util.auth_login()
    atrial_fibrillation_helper(auth)


if __name__ == "__main__":
    main(sys.argv)
