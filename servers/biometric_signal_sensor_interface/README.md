# Biometric Signal Sensor's Interface

When astronauts travels to Mars, or even other planets, they will be exposed to a number of hazards e.g. radiation, microbes in the spacecraft, planetary surface toxic dust. This project mainly revolves around configuring biometric signal sensors. 
The project is part of the current studies on the simulation of an Astronaut’s Health Monitor system.

Project Structure:

```
/mars_city/servers/biometric_signal_sensor_interface/
|-- docs
    `-- 
|-- src
    `-- anomaly_detector (Anomaly Detection Module)
    `-- gui
    	`-- static
    	`-- templates
    `-- health_monitor (Health Monitor Module)
    `-- hexoskin_helper (Hexoskin Helper Module)
        `-- hexoskin (Hexoskin Python Client)
|-- README.md

```
# Architecture
This project consists of three main modules:

 1. [Anomaly Detector](src/anomaly_detector)
 2. [Web based GUI](src/gui)
 3. [Health Monitor](src/health_monitor)
 4. [Hexoskin Helper](src/hexoskin_helper)

![enter image description here](https://2.bp.blogspot.com/-w0CxZHRuCVo/WUv5OKuWPcI/AAAAAAAADuA/hnY49h5qEN0o1llIUO1cqiH3XcOecSG0wCLcBGAs/s1600/gsoc.jpg)

Device Used :- [Hexoskin Smart Shirt](http://hexoskin.com/)

# Getting started

Refer [this](https://github.com/mars-planet/mars_city/blob/master/servers/body_tracker_v2/src/win/PyTango%20Setup/PyTango%20Installation%20Instructions/Instructions.txt) to set up Tango Controls on your system.


