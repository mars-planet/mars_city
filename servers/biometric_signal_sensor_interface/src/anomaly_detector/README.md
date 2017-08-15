# Anomaly Detection Algorithms  
NOTE: Please ensure that all the requirements from requirements.txt have been met.  
Refer to [Google Summer of Code blog post links](https://medium.com/@dipankar1995/) for more context.  

1. **Atrial Fibrillation (Heart Related)**  
To execute `python anomaly_detector.py`  
(NOTE: This is the only AD algo for which an `AnomalyDetector` object has to be created)  
One can also uncomment the call to `__plot_map()` in the `get_anomaly()` of `atrial_fibrillation.py`  
*executes the Atrial Fibrillation Anomaly Detection*  
```
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
```
2. **Ventricular Tachycardia (Heart Related)**  
To execute `python vt_helper.py`  
creates a helper object and calls the `VentricularTachycardia` anomaly detection methods  
```
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
```
3. **APC/PVC using Krasteva's paper (Heart Related)**  
To execute `python apc_pvc_helper.py`  
creates a helper object and calls the `APC` anomaly detection methods  
```
Input:
        timestamp:  the first timestamp

Output:
        stores to the results dict of the APC class

Notes:
        based on the following paper:

        'Automatic detection of premature atrial
        contractions in the electrocardiogram'
        by Krasteva et. al.
```
4. **PVC using Hamilton's beat detector (Heart Related)**  
To execute `python pvc_hamilton.py`  
creates a `PVC` object to detect PVCs  
```
Input:
        timestamp:  the first timestamp

Output:
        stores to the results dict of the PVC class

Notes:
        based on:

        'Open Source ECG Analysis Software
        Documentation'
        by Patrick S. Hamilton
```
5. **Respiratory AD (Breathing Related)**  
To execute `python respiration_AD.py`  
creates a `RespiratoryAD` object and performs Respiratory data related Anomaly Detection  
```
Input:
        timestamp:  the first timestamp

Output:
        stores to the results dict of the RespiratoryAD class

Notes:
        based on:

        'http://wps.prenhall.com/wps/media/objects/2791/2858109/toolbox/Box15_1.pdf'
```
6. **Sleep AD (Sleep Related)**  
To execute `python sleep_AD.py`  
creates a `SleepAD` object and performs Sleep data related Anomaly Detection  
```
Input:
        None

Output:
        stores to the anomaly_dict of the SleepAD class

Notes:
        based on:

        'https://www.sleepcycle.com/how-it-works/'
        'http://blog.doctoroz.com/oz-experts/calculating-your-perfect-bedtime-and-sleep-efficiency'
        'https://api.hexoskin.com/docs/resource/sleepphase/'
        'https://api.hexoskin.com/docs/resource/sleepposition/''
        'https://api.hexoskin.com/docs/resource/metric/'
```

**NOTES**  
- `anomaly_detector.cfg` - This is the configuration file which contains various customizable variable values for the various Anomaly Detection algorithms.  
- `bdac.py` - This file is a Python implementation of the file `easytest.c` from the open source ECG analysis software from EPLimited by Patrick Hamilton. Please read the top of the file for more info.  
- `detect_peaks.py` - Detect peaks in data based on their amplitude and other features by Marcos Duarte.  
- `osea.so` - Shared library object file - refer `bdac.py`  
- Some of the text files used are: `breathingrate.txt`, `br_quality.txt`, `ecg.txt`, `expiration.txt`, `heartrate.txt`, `hr_quality.txt`, `inspiration.txt`, `minuteventilation.txt`, `qrs.txt`, `resp.txt`, `rrinterval.txt`, `vt.txt` are the files downloaded directly from Hexoskin.`rrinterval_status.txt`, `sleepphase.txt`, `sleepposition.txt` were created using data extracted from the Hexoskin API.  
- Some of the algorithms will create multiple intermediate files in the working directory.  
- Refer to `../health_monitor/data_model.py` for the DB schema used to store the various anomalies.  

