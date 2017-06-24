# Hexoskin Helper Module
=======================


This module acts as helper module to the main Tango Device Server.


Project Structure:

```
/mars_city/servers/biometric_signal_sensor_interface/src/hexoskin_helper
|-- hexoskin
    `-- client.py
    `-- errors.py
    `-- __init__.py
    `-- README.md
|-- anomaly_database_helper.py
|-- __init__.py
|-- resource_helper.py
|-- utility_helper.py
|-- README.md

```

The hexoskin helper package solely deals with polling the smart shirt and getting the data from various sensors on-board the shirt in real-time.
The hexoskin helper is split into 3 parts:

 1. **Utility helper** :- That takes care all non-biometric related stuff like authentication for polling the smart shirt, accessing various records (a live session), etc. Also all the datatypes (the hexoskin biometrics) are defined along with their IDs. This makes it easier to extend the system for more sensors without having a need to dig into the hexoskin documentation, which is excellent.
    The data sample rates are also defined, in 256/samples/second, which makes sure that the data is collected properly from the hexoskin server in batches and with correct sampling rates.
    
   
 2. **Resource helper** :-  This has all the methods that is used for the data collection - previously collected data as well real-time. Real-time data is collected using generator functions in python, that yield the biometric data every, say, 5 seconds.
 3. **Database helper** :-  A small helper file that facilitates the database usage for storing the detected anomalies. 

