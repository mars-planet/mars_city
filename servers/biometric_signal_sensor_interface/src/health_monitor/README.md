Health Monitor Helper
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


# Usage

#### To start the biometric Tango Device Server

- First copy the osea.so from the anomaly_detector directory into **/usr/local/lib**
Then, export the environment variable.

```
$ export LD_LIBRARY_PATH='/usr/local/lib'
```

- Register the device and create the databases.

```
$ python data_model.py
$ python register_monitor.py
```

- To start the Tango Device Server

```
$ python monitor monitor
```
 
Proceed to starting the Flask server for starting the web dashboard.
