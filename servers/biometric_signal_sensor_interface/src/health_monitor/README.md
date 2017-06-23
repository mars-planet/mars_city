Health Monitor Helper
===================

----------


To execute monitor.py
-------------
First copy the osea.so from the anomaly_detector directory into **/usr/local/lib**
Then, export the environment variable.
```
export LD_LIBRARY_PATH='/usr/local/lib'
```
 -  **python monitor.py af** :- This is to run the  Atrial Fribillation 
 -  **python monitor.py vt** :- This is to run the Ventricular Tachycardia
 
 > Note: Run the above in two separate terminals to do the analysis concurrently
