# Biometric Signal Sensor's Interface

When astronauts travels to Mars, or even other planets, they will be exposed to a number of hazards e.g. radiation, microbes in the spacecraft, planetary surface toxic dust. This project mainly revolves around configuring biometric signal sensors. 
The project is part of the current studies on the simulation of an Astronaut’s Health Monitor system.

Project Structure:

```
/mars_city/servers/biometric_signal_sensor_interface/
|-- src
    `-- anomaly_detector (Anomaly Detection Module)
    `-- gui
    	`-- static
    	`-- templates
    `-- health_monitor (Health Monitor Module)
    `-- hexoskin_helper (Hexoskin Helper Module)
        `-- hexoskin (Hexoskin Python Client)
|-- README.md
|-- requirements.txt

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

### Prerequisities

##### Hardware requirements.

- 64-bit (x64) processor
- 4 GB Memory (or more)
- Physical dual-core 3.1 GHz (2 logical cores per physical) or faster processor
- USB 3.0 controller for synchronising the Hexoskin Smart Shirt
- [Hexoskin Smart Shirt](http://hexoskin.com/)
- A smart phone to run the Hexoskin Android Application 

##### Software requirements.

- [Ubuntu 14.04](http://releases.ubuntu.com/14.04/)
> The required Tango Controls did not install properly on other Ubuntu releases

- [Tango controls](http://www.tango-controls.org/downloads/binary/) (The TANGO control system is a free open source device-oriented controls toolkit for controlling any kind of hardware or software and building SCADA systems)
- [Flask](http://flask.pocoo.org/) (Flask is a microframework for Python based on Werkzeug and Jinja 2. Used for the web application to support Graphical User Interface for the system)
- [Hexoskin Android Application](https://play.google.com/store/apps/details?id=com.hexoskin.hexoskin&hl=en)



### Installing

- Install all the dependencies mentioned above.
- For Tango installation follow [this](http://marscity.readthedocs.io/en/latest/doc/setup.html) link since the default installation guide by tango is outdated and dosen't work with newer MySQL versions.
Once this installation is done sucessfully, JIVE which is an application provided by tango should open sucessfully.
- Install the python dependencies using ``` pip install -r requirements.txt```, present in this directory.

This should install all necessary dependencies and set up the environment for the Biometric Signal Sensor's Interface project.

# Running
#### Follow the instructions in this order.

 - First, wear the Hexoskin Smart Shirt and connect the Hexoskin device with the shirt.
 - Connect the device with a smart phone over blue-tooth to communicate with the application.
> Refer [here](https://www.hexoskin.com/pages/start) for more Hexoskin related instructions for getting started  

 - Start the Tango Device Server to start the data collection and the anomaly detection (Refer [here](https://github.com/mars-planet/mars_city/blob/master/servers/biometric_signal_sensor_interface/src/health_monitor/README.md) for more details)
 - Start the Python Flask Graphical User Interface (Refer [here](https://github.com/mars-planet/mars_city/blob/master/servers/biometric_signal_sensor_interface/src/gui/README.md) for more details)

#### Recommended 

 - Wear the device everyday (during space exploration) and continue wearing it throughout the night to calculate sleep and anomalies based on sleep. Unplug the hexoskin device from the shirt and sync it by connecting it with a system every morning as soon as you wake up. 

## Contributing

Please read [Software Engineering Guidelines](http://marscity.readthedocs.io/en/latest/doc/guidelines.html) for details on our code of conduct, and the process for submitting pull requests to us.

## Versioning

We use [GitHub](http://github.com/) for versioning.

## Authors

* **Abhijith C** - [abhijith0505](https://github.com/abhijith0505) (Tango deivce server and GUI)
* **Dipankar Niranjan** - [Ras-al-Ghul](https://github.com/Ras-al-Ghul) (Anomaly Detection)


## License

This project is licensed under the Mars City License - see the [LICENSE.md](https://github.com/mars-planet/mars_city/blob/master/LICENSE) file for details

## Acknowledgments

* [Antonio](https://github.com/aldebran), [Mario](https://github.com/mtambos) and [Ambar](https://github.com/coder006).



