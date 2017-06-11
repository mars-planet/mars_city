from __future__ import division, print_function

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy import signal


class VentricularTachycardia(object):
    def __init__(self, ecg, qrs, rr_intervals, config):
        # from the paper Real time detection of ventricular fibrillation and tachycardia by Jekova et. al.
        self.mV_threshold = config.getfloat('Ventricular Tachycardia', 'mV_threshold')
        self.resample_frequency = config.getint('Ventricular Tachycardia', 'resample_frequency')

        self.gain = config.getint('Ventricular Tachycardia', 'gain')
        
        # the value of the time window in seconds
        self.time_window = 6
        # the value of the sampling frequency of the ecg in Hz
        self.sample_freq = 256
        # the max amplitude of the ecg in mV
        self.mV_orig = 26.2144

        # the value of the asystole threshold in mV
        self.asystole_threshold = 0.3

        self.highpass_cutoff = config.getfloat('Ventricular Tachycardia', 'highpass_cutoff')
        self.lowpass_cutoff = config.getfloat('Ventricular Tachycardia', 'lowpass_cutoff')

        self.ecg = ecg

        if not len(self.ecg) == (self.time_window * self.sample_freq):
            raise ValueError("sufficient ecg data not collected")

        self.qrs = qrs

        if not (self.qrs['hexoskin_timestamps'][0] >= self.ecg['hexoskin_timestamps'][0] and
                self.qrs['hexoskin_timestamps'][len(qrs)-1] <= self.ecg['hexoskin_timestamps'][len(ecg)-1]):
            raise ValueError("qrs is not within ecg bounds")

        self.rr_intervals = rr_intervals

        if not (self.rr_intervals['hexoskin_timestamps'][0] >= self.ecg['hexoskin_timestamps'][0] and
                self.rr_intervals['hexoskin_timestamps'][len(rr_intervals)-1] <= self.ecg['hexoskin_timestamps'][len(ecg)-1]):
            raise ValueError("rr_intervals is not within ecg bounds")
        
    def plot_map(self, df):
        plt.plot(df['hexoskin_timestamps'], df['ecg_val'])
        plt.show()

    def normalize_and_resample(self):
        # # remove gain
        # self.ecg['ecg_val'] = self.ecg['ecg_val'] - self.gain

        # subtract mean
        self.ecg['ecg_val'] = self.ecg['ecg_val'] - (self.ecg['ecg_val'].sum()/len(self.ecg))

        # # normalize
        # if not self.mV_threshold == self.mV_orig:
        #     self.ecg['ecg_val'] = self.ecg['ecg_val'] * (self.mV_threshold/self.mV_orig)

        # resample
        if not self.resample_frequency == self.sample_freq:
            new_ecg, new_timestamps = signal.resample(self.ecg['ecg_val'], self.time_window*self.resample_frequency, self.ecg['hexoskin_timestamps'])
            # recreate dataframe
            self.ecg = pd.DataFrame(np.column_stack([new_timestamps, new_ecg]), columns=['hexoskin_timestamps', 'ecg_val'])

        # convert ecg values to mV
        self.ecg['ecg_val'] = self.ecg['ecg_val'] * 0.0064

    def filter(self):
        plt.plot(self.ecg['hexoskin_timestamps'], self.ecg['ecg_val'])

        # to scale the highpass
        __firstval = self.ecg['ecg_val'][0]

        # implement the 1.4 Hz high-pass forward-backward filter
        __nyq = 0.5 * self.resample_frequency
        __normal_highpass_cutoff = self.highpass_cutoff / __nyq
        __order = 2
        b, a = signal.butter(__order, __normal_highpass_cutoff, btype='high', analog=False)
        self.ecg['ecg_val'] = signal.filtfilt(b, a, self.ecg['ecg_val'])

        # plt.plot(self.ecg['hexoskin_timestamps'], self.ecg['ecg_val'])

        # not advised
        # self.ecg['ecg_val'] = self.ecg['ecg_val'] + __firstval

        # plt.plot(self.ecg['hexoskin_timestamps'], self.ecg['ecg_val']) 

        # implement the 30 Hz low-pass forward-backward filter
        __nyq = 0.5 * self.resample_frequency
        __normal_lowpass_cutoff = self.lowpass_cutoff / __nyq
        __order = 5
        b, a = signal.butter(__order, __normal_lowpass_cutoff, btype='low')
        self.ecg['ecg_val'] = signal.filtfilt(b, a, self.ecg['ecg_val'])

        plt.plot(self.ecg['hexoskin_timestamps'], self.ecg['ecg_val'])
        plt.plot(self.qrs['hexoskin_timestamps'], [0] * len(self.qrs), 'ro')
        plt.plot(self.rr_intervals['hexoskin_timestamps'], [0.1] * len(self.rr_intervals), 'go')

        plt.show()


    def signal_preprocess(self):
        # self.plot_map(self.ecg)
        self.normalize_and_resample()
        # self.plot_map(self.ecg)
        self.filter()
        # self.plot_map(self.ecg)

    # prev_ampl should be in normalized mV representation
    def DHA_detect(self, prev_ampl):
        # peak amplitude
        __cur_ampl = self.ecg['ecg_val'].max()
        print(__cur_ampl)

        # # avg amplitude
        # arr = self.rr_intervals['hexoskin_timestamps'] - self.rr_intervals['hexoskin_timestamps'][0] + 1
        # peak_indices = signal.find_peaks_cwt(self.ecg['ecg_val'].as_matrix(), arr.as_matrix())
        # # plt.plot([self.ecg['hexoskin_timestamps'][i] for i in peak_indices], [0.2] * len(peak_indices), 'bo')
        # __cur_ampl = sum([abs(self.ecg['ecg_val'][i]) for i in peak_indices])/len(peak_indices)
        # print(__cur_ampl)
        
        if __cur_ampl > prev_ampl:
            return __cur_ampl, True
        else:
            return __cur_ampl, False

    def asystole_classifier(self):
        # outputs asystole, QRS or VT/VF
        # if ecg['']
        pass

    def VT_VF_classifier(self):
        pass

    def asystole_detector(self, cur_ampl):
        if cur_ampl < self.asystole_threshold:
            self.asystole_classifier()
        else:
            self.VT_VF_classifier()


def main():
    import ConfigParser, os, sys
    config = ConfigParser.RawConfigParser()
    dirname = os.path.dirname(os.path.abspath(sys.argv[0]))
    cfg_filename = os.path.join(dirname, 'anomaly_detector.cfg')
    config.read(cfg_filename)

    ecg = (pd.read_csv('ecg.txt',
                    sep="\t",
                    nrows=256*6,
                    dtype={"hexoskin_timestamps": np.int64,
                           "ecg_val": np.int32},
                    header=None,
                    names=["hexoskin_timestamps", "ecg_val"]))
    qrs = (pd.read_csv('qrs.txt',
                    sep="\t",
                    nrows=6,
                    dtype={"hexoskin_timestamps": np.int64,
                           "qrs_zero": np.int32},
                    header=None,
                    names=["hexoskin_timestamps", "qrs_zero"]))
    rr_intervals = (pd.read_csv('rrinterval.txt',
                    sep="\t",
                    nrows=13,
                    dtype={"hexoskin_timestamps": np.int64,
                           "rr_int": np.float64},
                    header=None,
                    names=["hexoskin_timestamps", "rr_int"]))

    VTobj = VentricularTachycardia(ecg, qrs, rr_intervals, config)
    VTobj.signal_preprocess()
    cur_ampl, stop_cur = VTobj.DHA_detect(1)
    
    # to analyze next six second epoch
    if stop_cur == True:
        return cur_ampl

    # asystole detector
    VTobj.asystole_detector(cur_ampl)

    # to analyze next six second epoch
    return cur_ampl
    
if __name__ == '__main__':
    main()