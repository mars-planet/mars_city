from __future__ import division, print_function

import os
import sys
import ConfigParser

import numpy as np
import pandas as pd

from atrial_fibrillation import AtrialFibrillation
from ventricular_tachycardia import VentricularTachycardia


class AnomalyDetector(object):
    """
    implements methods to call various Anomaly Detection Algorithms
    """

    def __init__(self):
        config = ConfigParser.RawConfigParser()
        dirname = dir_path = os.path.dirname(os.path.realpath(__file__))
        cfg_filename = os.path.join(dirname, 'anomaly_detector.cfg')
        self.config.read(cfg_filename)

        self.vt_result = None

    def af_anomaly_detect(self, rr_intervals, hr_quality_indices):
        """
        executes the Atrial Fibrillation Anomaly detection
        Input:
            rr_intervals:           a 2D pandas dataframe -
                                    (refer rrinterval.txt from Hexoskin record)
                                    first column named "hexoskin_timestamps" -
                                    contains 'int' timestamps
                                    second column named as "rr_int" -
                                    contains 'double' interval data
            hr_quality_indices:     a 2D pandas dataframe -
                                    (refer hr_quality.txt from Hexoskin record)
                                    first column named "hexoskin_timestamps" -
                                    containts 'int' timestamps
                                    second column named as "quality_ind" -
                                    contains 'int' quality indices,
                                    with max value 127

        Output:
            returns:
            if anomaly:
                'dict' with follwing keys:
                    start_hexo_timestamp:   an integer denoting timestamp of
                                            the first record
                    end_hexo_timestamp:     an integer denoting timestamp of
                                            32/64/128 - last record
                    num_of_NEC:             a small integer, higher the number,
                                            more severe the anomaly here
                    data_reliability:       a small integer, which denotes as a
                                            percentage, the quality of the data
                                            in this window
                                            the higher the percentage, worse
                                            the quality
                    window_size:            a small integer, takes 32/64/128
                                            as values
            else:
                None

        Notes:
            based on 'A Simple Method to Detect
            Atrial Fibrillation Using RR Intervals'
            by Jie Lian et. al.
            Note the return value (if not 'None') and
            check with the data_reliability and previous
            data timestamps to set AFAlarmAttribute at
            the health_monitor server
        """

        if not (len(rr_intervals)) == self.window_size:
            raise ValueError("window length of rr_intervals\
                passed doesn't match config file")

        if not (rr_intervals['hexoskin_timestamps'][0] >=
                hr_quality_indices['hexoskin_timestamps'][0] and
                rr_intervals['hexoskin_timestamps'][len(rr_intervals)-1] <=
                hr_quality_indices
                ['hexoskin_timestamps'][len(hr_quality_indices)-1]):
                raise ValueError("first rr_interval timestamp\
                 and last rr_interval timestamp must lie within first \
                 and last timestamp of hr_quality")

        AF = AtrialFibrillation(rr_intervals, hr_quality_indices,
                                self.config)
        return AF.get_anomaly()

    def vt_anomaly_detect(self, ecg, rr_intervals,
                          rr_interval_status, prev_ampl):
        """
        creates an object and calls the Ventricular Tachycardia
        anomaly detection methods
        Input:
            ecg:                    a 2D pandas dataframe -
                                    (refer ecg.txt from Hexoskin record)
                                    first column named "hexoskin_timestamps" -
                                    contains 'int' timestamps
                                    second column named as "ecg_val" -
                                    contains 'int' raw ecg data
            rr_intervals:           a 2D pandas dataframe -
                                    (refer rrinterval.txt from Hexoskin record)
                                    first column named "hexoskin_timestamps" -
                                    contains 'int' timestamps
                                    second column named as "rr_int" -
                                    contains 'double' interval data
            rr_intervals_status:    a 2D pandas dataframe -
                                    (refer rrintervalstatus from Hexoskin API)
                                    first column named "hexoskin_timestamps" -
                                    containts 'int' timestamps
                                    second column named as "rr_status" -
                                    contains 'int' quality indices.

        Output:
            sets:
            vt_result:  this is an attribute of an object of this
                        (Anomaly Detector) class. Its value can
                        be read from the caller method. Its value
                        is set to __zero_one_count which is
                        described next.

            __zero_one_count    -   if it is the string True, it means
                                    that analysis of next 6 seconds is
                                    required
                                -   if it is False, it means that next 6
                                    second analysis is not required
                                -   if it has an integer value then it
                                    means that a VT event has been detected
                                    and it has to be stored in the anomaly
                                    database and of course next 6 second
                                    analysis is required

        Notes:
            based on the following three papers:

            'Ventricular Tachycardia/Fibrillation Detection
            Algorithm for 24/7 Personal Wireless Heart Monitoring'
            by Fokkenrood et. al.

            'Real Time detection of ventricular fibrillation
            and tachycardia' by Jekova et. al.

            'Increase in Heart Rate Precedes Episodes of
            Ventricular Tachycardia and Ventricular
            Fibrillation in Patients with Implantahle
            Cardioverter Defihrillators: Analysis of
            Spontaneous Ventricular Tachycardia Database'
            by Nemec et. al.

            Refer to readme for more details
        """
        __zero_one_count = True

        VTobj = VentricularTachycardia(ecg, rr_intervals,
                                       rr_interval_status, self.config)

        further_analyze = VTobj.analyze_six_second()
        # if initial analysis indicates that further analysis
        # is not required
        if not further_analyze:
            __zero_one_count = False
            self.vt_result = __zero_one_count

        # the print can be commented out
        print("Doing further analysis")

        # perform the preprocessing
        VTobj.signal_preprocess()

        # call the DangerousHeartActivity detector
        cur_ampl, stop_cur = VTobj.DHA_detect(prev_ampl)

        # whatever be the results of the following stages,
        # we necessarily have to analyze the next six second epoch

        # if further analysis is not required
        if stop_cur is True:
            self.vt_result = __zero_one_count

        # asystole detector
        vtvfres = VTobj.asystole_detector(cur_ampl)

        # to analyze next six second epoch
        if vtvfres == 'VT/VF':
            # A VT episode has been found
            # the print can be omitted
            print(vtvfres)
            __zero_one_count = VTobj.zero_one_count
            self.vt_result = __zero_one_count
        else:
            # not a VT episode
            # the print can be omitted
            print(vtvfres)
            self.vt_result = __zero_one_count


def main():
    AD = AnomalyDetector()
    rr_intervals = (pd.read_csv('rrinterval.txt',
                    sep="\t",
                    nrows=AD.config.getint('Atrial Fibrillation',
                                           'window_size'),
                    dtype={"hexoskin_timestamps": np.int64,
                           "rr_int": np.float64},
                    header=None,
                    names=["hexoskin_timestamps", "rr_int"]))
    hr_quality_indices = (pd.read_csv('hr_quality.txt',
                                      sep="\t",
                                      nrows=AD.config.
                                      getint('Atrial Fibrillation',
                                             'window_size')-8,
                                      dtype={"hexoskin_timestamps": np.int64,
                                             "quality_ind": np.int32},
                                      header=None,
                                      names=["hexoskin_timestamps",
                                             "quality_ind"]))
    # call the Atrial Fibrillation anomaly detection method
    # print(AD.af_anomaly_detect(rr_intervals, hr_quality_indices))

    ecg = (pd.read_csv('ecg.txt',
                       sep="\t",
                       nrows=256*6,
                       dtype={"hexoskin_timestamps": np.int64,
                              "ecg_val": np.int32},
                       header=None,
                       names=["hexoskin_timestamps", "ecg_val"]))
    """
    for testing, ensure that only the relevant timestamped
    rr_intervals are present in rrinterval.txt as it reads
    a preset 7 rows
    """
    rr_intervals = (pd.read_csv('rrinterval.txt',
                                sep="\t",
                                nrows=7,
                                dtype={"hexoskin_timestamps": np.int64,
                                       "rr_int": np.float64},
                                header=None,
                                names=["hexoskin_timestamps", "rr_int"]))
    """
    for testing, ensure that only the relevant timestamped
    rr_status are present in rr_interval_status.txt as it
    reads a preset 7 rows
    """
    rr_interval_status = (pd.read_csv('rr_interval_status.txt',
                                      sep="\t",
                                      nrows=7,
                                      dtype={"hexoskin_timestamps": np.int64,
                                             "rr_status": np.int32},
                                      header=None,
                                      names=["hexoskin_timestamps",
                                             "rr_status"]))
    # call the Ventricular Tachycardia anomaly detection method
    AD.vt_anomaly_detect(ecg, rr_intervals, rr_interval_status, 1400)


if __name__ == '__main__':
    main()
