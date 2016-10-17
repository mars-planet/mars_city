# Kinect V2 with Unity

This project aims to provide a pykinect plugin for unity game engine.  

It in divided into two subsystems, one is a windows based machine which runs the kinect server ans publishes coordinates from the skeleton.
The other one is a linux machine that subscribes to the server and gets a 2D numpyarray with all the coordinates.

## Getting Started

Project structure:

```
/mars_city/servers/unity
|-- docs
|   `-- sad.rst
|-- README.md
|-- requirements.txt
`-- src
    |-- linux
    |   |-- client.py
    |   |-- plotlines.cs
    |   `-- unity.cs
    `-- win
        |-- examples
        |   |-- main.py
        |   |-- PyKinectBodyGame.py
        |   `-- PyKinectInfraRed.py
        |-- newdevice.py
        |-- PowerSupplyDS.py
        |-- tango_pygame.py
        `-- Test.py
```

### Prerequisities

##### Hardware requirements.

- 64-bit (x64) processor
- 4 GB Memory (or more)
- Physical dual-core 3.1 GHz (2 logical cores per physical) or faster processor
- USB 3.0 controller dedicated to the Kinect for Windows v2 sensor*
- DX11 capable graphics adapter**
- A Microsoft Kinect v2 sensor, which includes a power hub and USB cabling
- Discrete Graphic card is a plus.

##### Software requirements.

- [Kinect for Windows SDK 2.0 ](https://www.microsoft.com/en-in/download/details.aspx?id=44561)
- [PyKinect2 for windows](https://pypi.python.org/pypi/pykinect2)
- [Tango controls 9.2.2 64 bit](http://sourceforge.net/projects/tango-cs/files/TangoSetup-9.2.2_win64.exe/download)
- [MySQL for windows 5.7.11 or above](https://dev.mysql.com/downloads/file/?id=464460) (Tested on MySQL Community Server 5.7.11)
- Pygame for windows
- For other python module binaries check [this](http://www.lfd.uci.edu/~gohlke/pythonlibs/) and download the 64 bit binaries if pip won't work. (This tends to happen.)
    

### Installing

- Install all the dependencies mentioned above.
- For Tango installation follow [this](http://shrigsoc.blogspot.in/2016/07/update-on-10-july-tango-installation.html) blog post since the default installation guide by tango is outdated and doesn,t work with newer MySQL versions.
Once this installation is done sucessfully, JIVE which is an application provided by tango should open sucessfully.
- Install the python dependencies using ``` pip install -r requirements.txt``` in Unity server repository.
 


## Running the server and client.

- On windows machine run start-db.sh under %TANGO-ROOT%/bin/
- Run tango_pygame.py with eras1 as parameter. ```python tango_pygame.py eras1```
- On client side run client.py under ./src/linux/ 

TBA

## Contributing

Please read [Software Engineering Guidelines](http://eras.readthedocs.io/en/latest/doc/guidelines.html) for details on our code of conduct, and the process for submitting pull requests to us.

## Versioning

We use [GitHub](http://github.com/) for versioning.

## Authors

* **Shridhar Mishra** - *Initial work* - [shridharmishra4](https://github.com/shridharmishra4)


## License

This project is licensed under the Mars City License - see the [LICENSE.md](https://github.com/mars-planet/mars_city/blob/master/LICENSE) file for details

## Acknowledgments

* Vito and Antonio.
* Pykinect2 team.

