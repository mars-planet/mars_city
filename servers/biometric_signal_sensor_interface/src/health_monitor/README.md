Health Monitor 
===================
The health monitor package acts as the backbone of the system and an interface for the various components and the device server.
Health monitor contains a a simple file that will be imported by the Tango device server. This file acts as an interface between the device server and all the components. It also contains the database where the anomalies are stored
It is the helper module to the main Tango Device Server.


Project Structure:

```
/mars_city/servers/biometric_signal_sensor_interface/src/health_monitor
|-- anomalies.db
|-- client.py
|-- data_model.py
|-- login_config.cfg
|-- monitor.py
|-- README.md
|-- register_monitor.py

```

## Database Schemas
### AtrFibAlarm
> Atrial Fibrillation Anomaly Database 
```
start_hexo_timestamp - Primary key - Integer
end_hexo_timestamp - Integer
doe - DateTime
num_of_NEC - Integer
data_reliability - Integer
window_size - Integer
```

1) **start_hexo_timestamp** : Starting hexoskin timestamp
2) **end_hexo_timestamp** :  Ending hexoskin timestamp
3) **doe** : Date of Entry
4) **num_of_NEC** :  Number of Non-Zero entries
5) **data_reliability** : Data reliability (Anomaly Measure)
6)  **window_size** : Size of input data-points needed by algorithm

### VenTacAlarms
> Ventricular Tachycardia Anomaly Database 
```
start_hexo_timestamp - Primary key - Integer
end_hexo_timestamp - Integer
doe - DateTime
data_reliability - Integer
```

1) **start_hexo_timestamp** : Starting hexoskin timestamp
2) **end_hexo_timestamp** :  Ending hexoskin timestamp
3) **doe** : Date of Entry
4) **data_reliability** : Data reliability (Anomaly Measure)

### APCAlarms
> APC/PVC & PVC-hamilton Anomaly Database 
```
RRPeak_hexo_timestamp - Primary key - Integer
RR_Quality - Integer
doe - DateTime
PVC_from - Integer
```

1) **RRPeak_hexo_timestamp** : Starting hexoskin timestamp
2) **RR_Quality** :  RR quality signal
3) **doe** : Date of Entry
4) **PVC_from** :

### RespAlarms
> Respiration Based Anomaly Database 
```
Resp_hexo_timestamp - Primary key - Integer
BRstatus_mean - Integer
Anomaly_type - String
doe - DateTime
```

1) **Resp_hexo_timestamp** : Respiration hexoskin timestamp
2) **BRstatus_mean** :  Breathing rate mean
3) **Anomaly_type** : Description of the type of anomaly
4) **doe** : Date of Entry


### SleepAlarms
> Sleep based Anomaly Database
```
start_hexo_timestamp - Primary key - Integer
cycle_time - Integer
doe - DateTime
```

1) **start_hexo_timestamp** : hexoskin timestamp
2) **cycle_time** :  Sleep cycle time
3) **doe** : Date of entry

### Data
> Raw-data database. Consumed by the GUI 
```
hexo_timestamp - Primary key - Integer
data - Integer
datatype - Integer
```

1) **hexo_timestamp** : hexoskin timestamp
2) **data** :  The raw-data corresponding to the datatype
3) **datatype** : Hexoskin Biometric Datatype


# Configurations

Edit the **config.cfg** and add the hexoskin credentials.

# Usage

#### To start the biometric Tango Device Server

- First copy the *osea.so* from the anomaly_detector directory into **/usr/local/lib**
Then, export the environment variable.

```
$ export LD_LIBRARY_PATH='/usr/local/lib'
```

- Create the database and register the device.

```
$ python data_model.py
$ python register_monitor.py
```

- To start the Tango Device Server

```
$ python monitor monitor
```

- This will start the Tango Device Server, which in turn starts all the Anomaly Detection process, except the Sleep Anomaly Detection. To start the sleep detection, start a cron job. Add the following line into the crontab by first running ``` crontab -e ``` and adding the following line

```
0 9 * * * python <path-to-repo>/mars_city/servers/biometric_signal_sensor_interface/src/health_monitor/monitor.py
```

This will run the sleep data collection and sleep anomaly detection procedures every morning at 9. This can be changed based on preference.

> Note: It is important to know that, this ideally works only when the hexoskin is plugged out of shirt and connected to a system for syncing. Ensure that the syncing happens before the time specified in the cron tab.
 
Proceed to starting the Flask server for starting the web dashboard.


