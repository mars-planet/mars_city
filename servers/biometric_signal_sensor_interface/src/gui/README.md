Web based Graphical User Interface (GUI)
===========================

The web-based GUI is developed using [Python Flask](http://flask.pocoo.org/). The frontend is neatened using [Materializecss](http://materializecss.com/). This module serves as the GUI for the Biometric Signal Sensor's Interface providing details regarding the Astronaut and the Biometric signals along with the anomalies detected using plots and tables.

The plots are built using [Plot.ly](http://plot.ly/). 
But instead of having the plot made online and embedding the plots inside website using the URL, I have actually made the plots in the Flask Server using code, and displaying it on the front-end end using [Plotly.js](http://plot.ly/javascript/).

This module is a Python Flask server module.

Project Structure:

```
/mars_city/servers/biometric_signal_sensor_interface/src/gui
|-- app.py
|-- static
|-- templates
|-- README.md

```

# Usage

#### To run the GUI server

```
$ python app.py
```
 > Note: Run the GUI after starting the Tango Device Server.