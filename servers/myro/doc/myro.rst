Myro Rover
===========
http://wiki.roboteducation.org/Myro_Reference_Manual

Install Myro 
------------
::

  sudo apt-get install python-tk
  sudo apt-get install idle
  sudo apt-get install python-imaging-tk


get latest release from http://myro.roboteducation.org/download/::

    sudo unzip myro-2.9.1.zip
    cd myro
    sudo python setup.py install


Commands
--------
For the moment only the command::

   move(translate_speed, rotate_speed)

has been implemented  

* translate_speed: 0 to 1 moves forward; 0 to -1 moves backwards 
* rotate_speed: 0 to 1 turns left, 0 to -1 turns right 


