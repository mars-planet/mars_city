from __future__ import division, print_function

import sys
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import bdac

from scipy import signal
from detect_peaks import detect_peaks
from copy import deepcopy

# ecg is a pd dataframe with hexoskin_timestamps and ecg_val as the columns
def get_Ampl(ecg):
    # # peak amplitude absolute value
    # __cur_peak_ampl = max(abs(ecg['ecg_val']))
    # # print(__cur_peak_ampl)

    # avg peak amplitude - calc average of peaks
    # <http://nbviewer.jupyter.org/github/demotu/BMC/blob/master/notebooks/DetectPeaks.ipynb>
    peak_indices = detect_peaks(np.array(ecg['ecg_val']), mph=1450, mpd=100)
    trough_indices = detect_peaks(np.array(ecg['ecg_val']), mph=-1320, mpd=100, valley=True)
    # plt.plot(ecg['hexoskin_timestamps'], ecg['ecg_val'])
    # plt.plot([ecg['hexoskin_timestamps'][i] for i in peak_indices], [1400] * len(peak_indices), 'bo')
    # plt.plot([ecg['hexoskin_timestamps'][i] for i in trough_indices], [1410] * len(trough_indices), 'go')
    # plt.show()

    __peak_avg_ampl = abs((sum([ecg['ecg_val'][i] for i in peak_indices])/len(peak_indices)))
    __trough_avg_ampl = abs((sum([ecg['ecg_val'][i] for i in trough_indices])/len(trough_indices)))
    __mean_ampl = (__peak_avg_ampl + __trough_avg_ampl)/2
    return __mean_ampl

class VentricularTachycardia(object):
    def __init__(self, ecg, rr_intervals, rr_interval_status, config):
        # from the paper 'Real time detection of ventricular fibrillation and tachycardia' by Jekova et. al.
        # see anomaly_detector.cfg for description
        self.mV_threshold = config.getfloat('Ventricular Tachycardia', 'mV_threshold')
        self.resample_frequency = config.getint('Ventricular Tachycardia', 'resample_frequency')
        self.gain = config.getint('Ventricular Tachycardia', 'gain')
        self.hr_high = config.getint('Ventricular Tachycardia', 'hr_high')
        self.highpass_cutoff = config.getfloat('Ventricular Tachycardia', 'highpass_cutoff')
        self.lowpass_cutoff = config.getfloat('Ventricular Tachycardia', 'lowpass_cutoff')
        self.ectopic_beat_thresh = config.getint('Ventricular Tachycardia', 'ectopic_beat_thresh')
        
        # the value of the time window in seconds
        self.time_window = 6
        # the value of the sampling frequency of the ecg (Hexoskin in this case) in Hz
        self.sample_freq = 256
        # the max amplitude of the ecg in mV (Hexoskin in this case)
        self.mV_orig = 26.2144

        # the value of the asystole threshold for the asystole detector in mV
        self.asystole_detector_threshold = 0.3
        # the value of the threshold for the asystole classifier in mV
        self.asystolic_classifier_ampl = 0.1

        # mean of ecg data
        self.mean = 0

        # 2^ecg resolution - in this case it is set to 1400 and not (2^12)/2 = 4096/2 = 2048
        self.ecg_resolution = 1400

        # 6 seconds of collected ecg in digital units
        self.ecg = ecg

        if not len(self.ecg) == (self.time_window * self.sample_freq):
            raise ValueError("sufficient ecg data not collected")

        # qrs is deprecated, using only rr
        # 6 seconds of collected rr intervals
        self.rr_intervals = rr_intervals

        if not self.rr_intervals.empty:
            if not (self.rr_intervals['hexoskin_timestamps'][0] >= self.ecg['hexoskin_timestamps'][0] and
                    self.rr_intervals['hexoskin_timestamps'][len(rr_intervals)-1] <= self.ecg['hexoskin_timestamps'][len(ecg)-1]):
                raise ValueError("rr_intervals is not within ecg bounds")

        # 6 seconds of collected qrs detections
        self.rr_interval_status = rr_interval_status

        if not self.rr_interval_status.empty:
            if not (self.rr_interval_status['hexoskin_timestamps'][0] >= self.ecg['hexoskin_timestamps'][0] and
                    self.rr_interval_status['hexoskin_timestamps'][len(rr_interval_status)-1] <= self.ecg['hexoskin_timestamps'][len(ecg)-1]):
                raise ValueError("rr_interval_status is not within ecg bounds")
        
        # setup the rr_interval quality indices
        if not self.rr_interval_status.empty:
            __count_dict = self.rr_interval_status['rr_status'].value_counts()
            self.zero_one_count = 0
            if 1 in __count_dict:
                self.zero_one_count = __count_dict[0] + __count_dict[1]
            elif 0 in __count_dict:
                self.zero_one_count = __count_dict[0]

        # mean amplitude of ecg data
        # get the desired peak or avg amplitude
        self.mean_ampl = get_Ampl(self.ecg)

    def analyze_PVC_Unknown(self):
        # make a local copy
        __ecg = deepcopy(self.ecg)

        # convert ecg data to MIT BIH Arrythmia database standard voltage levels - 11 bit resolution centred around 950 mV
        __ecg['ecg_val'] *= 1.4
        __ecg['ecg_val'] /= 2

        # extend the beats as beat classifier skips first 9 beats and the last beat
        # extend multiple times to bring down error
        __ecg_vals = __ecg['ecg_val'].as_matrix()
        __ecg_vals = __ecg_vals.tolist()
        __ecg_vals.extend(__ecg_vals)
        __ecg_vals.extend(__ecg_vals)
        __ecg_vals.extend(__ecg_vals)
        # call the function from bdac.py which in turn calls the code written in C
        # the prototype is (ecg, gain, bitresolution/2, ipfreq, opfreq) - opfreq is always 200, ipfreq is Hexoskin's ecg freq
        # bitresolution/2 is for a 11 bit resolution of MIT BIH Arrythmia database data
        BDACobj = bdac.BDAC(__ecg_vals)
        __beat_types, __detection_times = BDACobj.AnalyzeBeatTypeSixSecond(200, 1024, 256, 200)
        # print(__beat_types, __detection_times)
        # plt.plot([i for i in xrange(len(__ecg_vals))], __ecg_vals)
        # plt.plot([int(j) for j in __detection_times], [950]*len(__detection_times), 'ro')
        # plt.show()

        # return the truth value of - if number of anomalous beats is greater than 50%
        # try:
        __anomalous_beats = (len(__beat_types) - len([i for i in __beat_types if i == 1]))/len(__beat_types)
        return(__anomalous_beats > 0.5)
        # except:
            # return True

    def analyze_ectopic_beats(self):
        # ensure rr_intervals are of good quality
        if self.zero_one_count >= int(len(self.rr_interval_status)/2):
            __ectopic_count = 0
            if not self.rr_intervals.empty:
                for i in xrange(1,len(self.rr_intervals)):
                    # calc threshold rr_intervals value
                    __thresh_value = (self.ectopic_beat_thresh/100) * self.rr_intervals['rr_int'][i-1]
                    # check if current beat is within given percentage of previous beat
                    if not (self.rr_intervals['rr_int'][i-1] - __thresh_value <= self.rr_intervals['rr_int'][i] <= self.rr_intervals['rr_int'][i-1] + __thresh_value):
                        __ectopic_count += 1
            # return the truth value of - if ectopic beats are greater than 1/5 i.e. 20%
            return ((__ectopic_count/len(self.rr_intervals)) >= 0.2)
        else:
            return True

    def analyze_six_second(self):
        if (self.analyze_PVC_Unknown() or self.analyze_ectopic_beats()):
            return True
        else:
            return False

    def normalize_and_resample(self):
        # remove gain
        self.ecg['ecg_val'] = self.ecg['ecg_val']/self.gain
    
        # center data
        self.ecg['ecg_val'] = self.ecg['ecg_val'] - int(self.ecg_resolution)

        # normalize
        if not self.mV_threshold == self.mV_orig:
            self.ecg['ecg_val'] = self.ecg['ecg_val'] * (self.mV_threshold/self.mV_orig)

        # # subtract mean
        # self.mean = (self.ecg['ecg_val'].sum())/len(self.ecg)
        # self.ecg['ecg_val'] = self.ecg['ecg_val'] - self.mean

        # resample
        if not self.resample_frequency == self.sample_freq:
            new_ecg, new_timestamps = signal.resample(self.ecg['ecg_val'], self.time_window*self.resample_frequency, self.ecg['hexoskin_timestamps'])
            # recreate dataframe
            self.ecg = pd.DataFrame(np.column_stack([new_timestamps, new_ecg]), columns=['hexoskin_timestamps', 'ecg_val'])

        # convert ecg values to mV
        self.ecg['ecg_val'] = self.ecg['ecg_val'] * 0.0064

    def filter(self):
        # plt.plot(self.ecg['hexoskin_timestamps'], self.ecg['ecg_val'])

        # subtract the mean as the high pass filter always shifts the base to zero
        self.mean = (self.ecg['ecg_val'].sum())/len(self.ecg)
        self.ecg['ecg_val'] = self.ecg['ecg_val'] - self.mean

        # implement the 1.4 Hz high-pass forward-backward filter
        __nyq = 0.5 * self.resample_frequency
        __normal_highpass_cutoff = self.highpass_cutoff / __nyq
        __order = 2
        b, a = signal.butter(__order, __normal_highpass_cutoff, btype='high', analog=False)
        self.ecg['ecg_val'] = signal.filtfilt(b, a, self.ecg['ecg_val'])

        # plt.plot(self.ecg['hexoskin_timestamps'], self.ecg['ecg_val'])

        # add back the mean
        self.ecg['ecg_val'] = self.ecg['ecg_val'] + self.mean

        # plt.plot(self.ecg['hexoskin_timestamps'], self.ecg['ecg_val'])

        # implement the 30 Hz low-pass forward-backward filter
        __nyq = 0.5 * self.resample_frequency
        __normal_lowpass_cutoff = self.lowpass_cutoff / __nyq
        __order = 5
        b, a = signal.butter(__order, __normal_lowpass_cutoff, btype='low', analog=False)
        self.ecg['ecg_val'] = signal.filtfilt(b, a, self.ecg['ecg_val'])

        # plt.plot(self.ecg['hexoskin_timestamps'], self.ecg['ecg_val'])
        # plt.plot(self.rr_intervals['hexoskin_timestamps'], [0.01] * len(self.rr_intervals), 'go')

        # plt.show()

    def signal_preprocess(self):
        # self.plot_map(self.ecg)
        self.normalize_and_resample()
        # self.plot_map(self.ecg)
        self.filter()
        # self.plot_map(self.ecg)

    # prev_ampl should be in normalized mV representation
    def DHA_detect(self, prev_ampl):
        # avg peak amplitude - calc average of peaks
        # <http://nbviewer.jupyter.org/github/demotu/BMC/blob/master/notebooks/DetectPeaks.ipynb>
        peak_indices = detect_peaks(np.array(self.ecg['ecg_val']), mph=0, mpd=100)
        trough_indices = detect_peaks(np.array(self.ecg['ecg_val']), mph=0.2, mpd=100, valley=True)
        # plt.plot(self.ecg['hexoskin_timestamps'], self.ecg['ecg_val'])
        # plt.plot([self.ecg['hexoskin_timestamps'][i] for i in peak_indices], [0] * len(peak_indices), 'bo')
        # plt.plot([self.ecg['hexoskin_timestamps'][i] for i in trough_indices], [-0.1] * len(trough_indices), 'go')
        # plt.show()

        self.mean = (self.ecg['ecg_val'].sum())/len(self.ecg)
        __peak_avg_ampl = abs((sum([self.ecg['ecg_val'][i] for i in peak_indices])/len(peak_indices)) - self.mean)
        __trough_avg_ampl = abs((sum([self.ecg['ecg_val'][i] for i in trough_indices])/len(trough_indices)) + self.mean)
        __cur_ampl = (__peak_avg_ampl + __trough_avg_ampl)/2

        # Note that __cur_ampl is in the mV unit as it is needed for further processing
        # But we use self.mean_ampl as it is easier to calulate for a window in general
        if self.mean_ampl > prev_ampl:
            return __cur_ampl, True
        else:
            return __cur_ampl, False

    def asystole_classifier(self):
        # outputs asystole, QRS or VT/VF
        __max_ampl = max(abs(self.ecg['ecg_val']))
        if __max_ampl <= self.asystolic_classifier_ampl:
            # it is an asystolic epoch
            return 'ASYS'

        counter = 0
        for i in xrange(len(self.ecg)):
            if self.ecg['ecg_val'][i] > (max(self.ecg['ecg_val'])/2):
                counter += 1
        # if less than 40% of the signal is above half the maximum value, and the number of QRS detections is below emergency heart rate
        if ((counter/len(self.ecg)) < 0.4) and (((len(self.rr_intervals) * (60/self.time_window)) < self.hr_high) and self.zero_one_count >= int(len(self.rr_intervals)/2)):
            # it is qrs even though amplitude is low
            return 'QRS'

        # else return vtvf
        return 'VT/VF'

    def VT_VF_classifier(self):
        # The paper 'Real time detection of ventricular fibrillation and tachycardia' by Jekova et. al. has a slight variation
        maxAbsAmpl = max(abs(self.ecg['ecg_val']))

        avgAbsAmpl = abs(self.ecg['ecg_val']).sum()/len(self.ecg)

        MD = sum(abs(abs(self.ecg['ecg_val']) - avgAbsAmpl))/len(self.ecg)

        T1, T2, TX = 0, 0, 0

        for i in xrange(len(self.ecg)):
            if (0.8*maxAbsAmpl) <= (self.ecg['ecg_val'][i]) <= maxAbsAmpl:
                T1 += 1
            if (avgAbsAmpl) <= (self.ecg['ecg_val'][i]) <= (0.95*maxAbsAmpl):
                T2 += 1
            if (avgAbsAmpl - MD) <= (self.ecg['ecg_val'][i]) <= (avgAbsAmpl + MD):
                TX += 1

        T3 = (T1 * T2)/TX

        classf = None
        if (T1 < 120 and T2 < 456 and T3 < 100) or ((120 < T1 < 192) and (T2 < 288) and (T3 < 100)):
            classf = 'QRS'
        elif (T1 < 120 and T2 >= 456) or (T1 >= 120 and T3 >= 100) or (T2 >= 528):
            classf = 'VT/VF'
        else:
            classf = 'Unknown'

        return classf

    def asystole_detector(self, cur_ampl):
        vtvfres = None
        # classify with asystole classifier
        if cur_ampl < self.asystole_detector_threshold:
            vtvfres = self.asystole_classifier()
        # classify with VT/VF classifier
        else:
            vtvfres = self.VT_VF_classifier()

        return vtvfres

def main():
    import ConfigParser, os, sys
    config = ConfigParser.RawConfigParser()
    dirname = os.path.dirname(os.path.abspath(sys.argv[0]))
    cfg_filename = os.path.join(dirname, 'anomaly_detector.cfg')
    config.read(cfg_filename)

    """
    a return value of True implies further analysis is needed
    a return value of False implies no further analysis is needed
    a return value of an integer indicates VT episode
    """
    ecg = (pd.read_csv('ecg.txt',
                    sep="\t",
                    nrows=256*6,
                    dtype={"hexoskin_timestamps": np.int64,
                           "ecg_val": np.int32},
                    header=None,
                    names=["hexoskin_timestamps", "ecg_val"]))
    # for testing, ensure that only the relevant timestamped rr_intervals are present in rrinterval.txt
    rr_intervals = (pd.read_csv('rrinterval.txt',
                    sep="\t",
                    nrows=7,
                    dtype={"hexoskin_timestamps": np.int64,
                           "rr_int": np.float64},
                    header=None,
                    names=["hexoskin_timestamps", "rr_int"]))
    # for testing, ensure that only the relevant timestamped rr_status are present in rr_interval_status.txt
    rr_interval_status = (pd.read_csv('rr_interval_status.txt',
                          sep="\t",
                          nrows=7,
                          dtype={"hexoskin_timestamps": np.int64,
                                 "rr_status": np.int32},
                          header=None,
                          names=["hexoskin_timestamps", "rr_status"]))

    VTobj = VentricularTachycardia(ecg, rr_intervals, rr_interval_status, config)
    further_analyze = VTobj.analyze_six_second()
    if not further_analyze:
        return False
    
    print("Doing further analysis")
    VTobj.signal_preprocess()
    cur_ampl, stop_cur = VTobj.DHA_detect(1400)
    
    # to analyze next six second epoch
    if stop_cur == True:
        return True

    # asystole detector
    vtvfres = VTobj.asystole_detector(cur_ampl)

    # to analyze next six second epoch
    if vtvfres == 'VT/VF':
        print(vtvfres)
        return self.zero_one_count
        print(vtvfres)
        return True
    
if __name__ == '__main__':
    main()