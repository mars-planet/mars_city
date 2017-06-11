from __future__ import division, print_function

import pandas as pd
import numpy as np
from scipy import signal


class VentricularTachycardia(object):
    def __init__(self, ecg, config):
        # from the paper Real time detection of ventricular fibrillation and tachycardia by Jekova et. al.
        self.mV_threshold = config.getfloat('Ventricular Tachycardia', 'mV_threshold')
        self.resample_frequency = config.getint('Ventricular Tachycardia', 'resample_frequency')
        
        # the value of the time window in seconds
        self.time_window = 6
        # the value of the sampling frequency of the ecg in Hz
        self.sample_freq = 256
        # the max amplitude of the ecg in mV
        self.mV_orig = 26.2144

        self.highpass_cutoff = config.getfloat('Ventricular Tachycardia', 'highpass_cutoff')
        self.lowpass_cutoff = config.getfloat('Ventricular Tachycardia', 'lowpass_cutoff')

        self.ecg = ecg

        if not len(self.ecg) == (self.time_window * self.sample_freq):
            raise ValueError("sufficient ecg data not collected")
        
    def plot_map(self, df):
        import matplotlib.pyplot as plt
        plt.plot(df['hexoskin_timestamps'], df['ecg_val'])
        plt.show()

    def normalize_and_resample(self):
        # normalize
        if not self.mV_threshold == self.mV_orig:
            self.ecg['ecg_val'] = self.ecg['ecg_val'] * (self.mV_threshold/self.mV_orig)

        # resample
        if not self.resample_frequency == self.sample_freq:
            new_ecg, new_timestamps = signal.resample(self.ecg['ecg_val'], self.time_window*self.resample_frequency, self.ecg['hexoskin_timestamps'])
            # recreate dataframe
            self.ecg = pd.DataFrame(np.column_stack([new_timestamps, new_ecg]), columns=['hexoskin_timestamps', 'ecg_val'])

        # convert ecg values to mV
        self.ecg['ecg_val'] = self.ecg['ecg_val'] * 0.0064

    def filter(self):
        # implement the 1.4 Hz high-pass forward-backward filter
        __nyq = 0.5 * self.resample_frequency
        __normal_highpass_cutoff = self.highpass_cutoff / __nyq
        __order = 5
        b, a = signal.butter(__order, __normal_highpass_cutoff, btype='high')
        self.ecg['ecg_val'] = signal.filtfilt(b, a, self.ecg['ecg_val'])

        # implement the 30 Hz low-pass forward-backward filter
        __nyq = 0.5 * self.resample_frequency
        __normal_lowpass_cutoff = self.lowpass_cutoff / __nyq
        __order = 5
        b, a = signal.butter(__order, __normal_lowpass_cutoff)
        self.ecg['ecg_val'] = signal.filtfilt(b, a, self.ecg['ecg_val'])


    def signal_preprocess(self):
        # self.plot_map(self.ecg)
        self.normalize_and_resample()
        self.plot_map(self.ecg)
        self.filter()
        self.plot_map(self.ecg)
        

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

    VTobj = VentricularTachycardia(ecg, config)
    VTobj.signal_preprocess()

if __name__ == '__main__':
    main()